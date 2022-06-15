import argparse
import collections
from enum import Enum
import enum
import logging
import os
import sys
import time
import threading
from typing import List, Optional
import numpy as np
import cv2 as cv

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn import geometry

from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
                                                  DirectoryRegistrationKeepAlive)

from bosdyn.client.fault import FaultClient
from bosdyn.client.server_util import GrpcServiceRunner, populate_response_header
from bosdyn.client.world_object import WorldObjectClient

from bosdyn.client.util import setup_logging

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.api import local_grid_pb2, local_grid_service_pb2_grpc, robot_command_pb2, robot_command_service_pb2

from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *
from bosdyn.api import geometry_pb2 as geom
from bosdyn.api import world_object_pb2
from bosdyn.client.lease import Error as LeaseBaseError

from bosdyn.api.graph_nav import map_pb2, graph_nav_pb2, nav_pb2

import bosdyn.client
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
import bosdyn.client.channel
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotLocalizedToEndError, NotReadyYetError
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
import bosdyn.client.util

from skimage.draw import line

LOGGER = logging.getLogger(__name__)


class AsyncOccupancyGrid(AsyncPeriodicQuery):
    def __init__(self, local_grid_client, grid_types):
        super().__init__("occupancy_grid", local_grid_client, LOGGER,
                                              period_sec=0.5)
        self._grid_types = grid_types

    def _start_query(self):
        return self._client.get_local_grids_async(self._grid_types)


class AsyncFiducialObjects(AsyncPeriodicQuery):
    def __init__(self, world_object_client):
        super().__init__("fiducials", world_object_client, LOGGER, period_sec=0.1)

    def _start_query(self):
        return self._client.list_world_objects_async(object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG])


class AsyncRobotState(AsyncPeriodicQuery):
    def __init__(self, robot_state_client):
        super().__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.1)

    def _start_query(self):
        return self._client.get_robot_state_async()

class AsyncLocalization(AsyncPeriodicQuery):
    def __init__(self, graph_nav_client):
        super().__init__("localization_status", graph_nav_client, LOGGER,
                                              period_sec=0.1)

    def _start_query(self):
        return self._client.get_localization_state_async()


def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.01)


def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed %s: %s" % (desc, err))


def upload_graph_and_snapshots(robot, client, lease, path, disable_alternate_route_finding):
    """Upload the graph and snapshots to the robot"""

    # Load the graph from disk.
    graph_filename = os.path.join(path, 'graph')
    robot.logger.info('Loading graph from ' + graph_filename)

    with open(graph_filename, 'rb') as graph_file:
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)
        robot.logger.info('Loaded graph has {} waypoints and {} edges'.format(
            len(current_graph.waypoints), len(current_graph.edges)))

    if disable_alternate_route_finding:
        for edge in current_graph.edges:
            edge.annotations.disable_alternate_route_finding = True

    # Load the waypoint snapshots from disk.
    current_waypoint_snapshots = dict()
    for waypoint in current_graph.waypoints:
        if len(waypoint.snapshot_id) == 0:
            continue
        snapshot_filename = os.path.join(path, 'waypoint_snapshots', waypoint.snapshot_id)
        robot.logger.info('Loading waypoint snapshot from ' + snapshot_filename)

        with open(snapshot_filename, 'rb') as snapshot_file:
            waypoint_snapshot = map_pb2.WaypointSnapshot()
            waypoint_snapshot.ParseFromString(snapshot_file.read())
            current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

    # Load the edge snapshots from disk.
    current_edge_snapshots = dict()
    for edge in current_graph.edges:
        if len(edge.snapshot_id) == 0:
            continue
        snapshot_filename = os.path.join(path, 'edge_snapshots', edge.snapshot_id)
        robot.logger.info('Loading edge snapshot from ' + snapshot_filename)

        with open(snapshot_filename, 'rb') as snapshot_file:
            edge_snapshot = map_pb2.EdgeSnapshot()
            edge_snapshot.ParseFromString(snapshot_file.read())
            current_edge_snapshots[edge_snapshot.id] = edge_snapshot

    # Upload the graph to the robot.
    robot.logger.info('Uploading the graph and snapshots to the robot...')
    true_if_empty = not len(current_graph.anchoring.anchors)
    response = client.upload_graph(graph=current_graph, lease=lease,
                                   generate_new_anchoring=true_if_empty)
    robot.logger.info('Uploaded graph.')

    # Upload the snapshots to the robot.
    for snapshot_id in response.unknown_waypoint_snapshot_ids:
        waypoint_snapshot = current_waypoint_snapshots[snapshot_id]
        client.upload_waypoint_snapshot(waypoint_snapshot=waypoint_snapshot, lease=lease)
        robot.logger.info('Uploaded {}'.format(waypoint_snapshot.id))

    for snapshot_id in response.unknown_edge_snapshot_ids:
        edge_snapshot = current_edge_snapshots[snapshot_id]
        client.upload_edge_snapshot(edge_snapshot=edge_snapshot, lease=lease)
        robot.logger.info('Uploaded {}'.format(edge_snapshot.id))



class AutonomyEngine:
    class Stage(Enum):
        STARTUP = enum.auto()
        STAND = enum.auto()
        FIDUCIAL_SEARCH = enum.auto()
        BEGIN_GRAPHNAV_RECORD = enum.auto()
        MOVE_TO_AISLE = enum.auto()
        WALK_LOOP = enum.auto()
        STOP_GRAPHNAV_RECORD = enum.auto()
        PROCESS_TOPOLOGY = enum.auto()
        PAUSE = enum.auto()
        EXPORT_PRELIM_RESULTS = enum.auto()
        WALK_SECOND_LOOP = enum.auto()
        PARK = enum.auto()
        EXPORT_FINAL_RESULTS = enum.auto()

    class SCAN(Enum):
        LEFT = enum.auto()
        RIGHT = enum.auto()


    def __init__(self, robot, options) -> None:
        self.robot = robot
        self.scan_interval = options.scan_interval
        self.scan_angles = options.scan_angles
        self.aisle1_se2 = SE2Pose(options.aisel1_se2[0], options.aisel1_se2[1], options.aisel1_se2[2])
        self.aisle_width = options.aisle_width
        self.aisle_scan_standoff = options.aisle_scan_standoff
        self.aisle_strid = options.aisle_stride
        self.map_path = options.map
        self.anchor_tag = options.anchor_tag

        self.occupancy_grid_client = robot.ensure_client('occupancy-grid')
        # self._occupancy_task = AsyncOccupancyGrid(occupancy_grid_client, ['occupancy','edges'])

        self.world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
        # self._fiducial_task = AsyncFiducialObjects(world_object_client)

        self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        # self._robot_state_task = AsyncRobotState(robot_state_client)

        self.graphnav_client = robot.ensure_client(GraphNavClient.default_service_name)
        # self._localization_task = AsyncLocalization(graphnav_client)

        _task_list = [self._occupancy_task, self._fiducial_task, self._robot_state_task, self._localization_task]
        _async_tasks = AsyncTasks(_task_list)

        self._stop_signal = threading.Event()
        self._async_updates_thread = threading.Thread(target=_update_thread, args=[_async_tasks, self._stop_signal])
        self._async_updates_thread.daemon = True
        self._async_updates_thread.start()

    def main_sequence(self):
        self.start()
        # self.sight_fiducial()
        self.enter_first_aisle()
        self.begin_aisle_scan(self.SCAN.RIGHT)
        self.scan_aisle(self.SCAN.RIGHT)
        self.nav_to_second_aisle()
        self.begin_aisle_scan(self.SCAN.LEFT)
        self.scan_aisle(self.SCAN.LEFT)
        self.nav_to_anchor_fiducial()
        self.sight_fiducial()
        self.enter_first_aisle()
        self.begin_aisle_scan(self.SCAN.LEFT)
        self.scan_aisle(self.SCAN.LEFT)
        self.nav_to_second_aisle()
        self.begin_aisle_scan(self.SCAN.RIGHT)
        self.scan_aisle(self.SCAN.RIGHT)
        self.nav_to_anchor_fiducial()
        self.park()
        self.shutdown()


    def start(self):
        self._lease = self._lease_client.acquire()
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.

        cmd = RobotCommandBuilder.stand_command()
        self._dispatch_robot_command(cmd)
        #load map

        self.graphnav_client.clear_graph()
        upload_graph_and_snapshots(robot=self.robot, client=self.graphnav_client, lease=self._lease, path=self.map_path)
        self.sight_fiducial()
        
        localization = nav_pb2.Localization()
        localization_response = self.graph_nav_client.set_localization(initial_guess_localization=localization, 
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC,
            use_fiducial_id=self.anchor_tag)

        if localization_response.status != graph_nav_pb2.SetLocalizationResponse.STATUS_OK:
            LOGGER.warning("could not localize in graph")

    def sight_fiducial(self):
        increment = math.pi*20/180
        cmd = RobotCommandBuilder.stand_command(footprint_R_body=geometry.EulerZXY(yaw=increment))
        self._dispatch_robot_command(cmd)
        cmd = RobotCommandBuilder.stand_command(footprint_R_body=geometry.EulerZXY(yaw=-increment))
        self._dispatch_robot_command(cmd)

        april_tags = self.world_object_client.list_world_objects([world_object_pb2.WORLD_OBJECT_APRILTAG])
        for tag in april_tags:
            if tag.apriltag_properties.tag_id != self.anchor_tag:
                continue
            if tag.apriltag_properties.fiducial_filtered_pose_status != world_object_pb2.AprilTagProperties.STATUS_OK:
                continue
            #tranform from odom to anchor (where is the anchor in odom?)
            self.odom_tform_anchor = get_a_tform_b(tag.transforms_snapshot, ODOM_FRAME_NAME,tag.apriltag_properties.frame_name_fiducial_filtered)
            self.odom_tform_anchor_se2 = get_se2_a_tform_b(tag.transforms_snapshot, ODOM_FRAME_NAME, tag.aprittag_properties.frame_name_fiducial_filtered)
            return
        raise RuntimeError('Could not find fiducial')

    def enter_first_aisle(self):
        #get anchor fiducial in odom frame
        #translate in xy to expected start
        goal = self.odom_tform_anchor_se2 * self.aisle1_se2
        cmd = RobotCommandBuilder.trajectory_command(goal.x, goal.y, goal.angle, ODOM_FRAME_NAME)
        self._dispatch_robot_command(cmd)

    def begin_aisle_scan(self, scan_side):
        #expect to be roughly in the center at the start of the aisle
        self.set_body_roll(0.15)
        self.set_body_roll(-0.15)
        self.set_body_roll(0)
        #look for parallel lines roughly parallel to body frame and roughly an expected distance apart
        grids = self.occupancy_grid_client.get_local_grids(['occupancy_odom'])

        # compute aisle width, orientation, and centerline
        
        
        #prime scan by moving to side move for scan_side view
        state = self.robot_state_client.get_robot_state()
        tforms = state.kinematic_state.transforms_snapshot
        current_se2 = get_se2_a_tform_b(tforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        goal = SE2Pose()
        cmd = RobotCommandBuilder.trajectory_command(goal.x, goal.y, goal.angle, ODOM_FRAME_NAME)
        self._dispatch_robot_command(cmd)
        pass

    def scan_aisle(self, scan_side):
        while self.in_aisle(scan_side):
            for angle in self.capture_angles:
                angle = angle if scan_side == self.SCAN.LEFT else -angle
                self.set_body_roll(angle)
                self.capture_image()
            self.go_to_next_SE2()

    def in_aisle(self, scan_side):
        #grab the line edge map, and see if there is a line at about the expected distance and orientation
        #on the scan side
        grids = self.occupancy_grid_client.get_local_grids(['occupancy_odom'])


        state = self.robot_state_client.get_robot_state()
        tforms = state.kinematic_state.transforms_snapshot
        current_se2 = get_se2_a_tform_b(tforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        return True



    def go_to_next_SE2(self):
        #assume that update_aisle_estimate has been called
        #end points in x,y
        grids = self.occupancy_grid_client.get_local_grids(['occupancy_odom'])


        state = self.robot_state_client.get_robot_state()
        tforms = state.kinematic_state.transforms_snapshot
        current_se2 = get_se2_a_tform_b(tforms, ODOM_FRAME_NAME, BODY_FRAME_NAME)

        if not aisle_detection_valid:
            #force generate from wrt map
            pass


        cmd = RobotCommandBuilder.trajectory_command(next_se2.x, next_se2.y, next_se2.theta, BODY_FRAME_NAME)
        self._dispatch_robot_command(cmd)
    
    
    def nav_to_second_aisle(self):
        self.go_to_junction_midline()
        self.walk_to_expected_next_aisle()
        self.enter_next_aisle()
        self.begin_aisle_scan()



    def set_body_roll(self,angle):
        cmd = RobotCommandBuilder.stand_command(footprint_R_body=geometry.EulerZXY(roll=angle))
        self._dispatch_robot_command(cmd)


    def capture_image(self):
        time.sleep(0.5)



    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info("Shutting down WasdInterface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease:
            _grpc_or_log("returning lease", lambda: self._lease_client.return_lease(self._lease))
            self._lease = None


    def _startup(self):
        pass


    def _locate_home_fiducial(self):
        pass


    def _localize_hallways(self):
        pass



    def _dispatch_robot_command(self, command_proto, end_time_secs=None):
        try:
            return self._robot_command_client.robot_command(lease=None, command=command_proto,
                                                     end_time_secs=end_time_secs)
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message("Failed {}: {}".format(command_proto, err))
            return None
        
        

def add_autnomoy_arguments(parser):
    parser.add_argument('--scan-interval', type=float, help="meters between scans", required=False, default=2)
    parser.add_argument('--scan-angles', type=list, nargs='+', help="body roll when scanning (radians)", required=False, default=[0,0.07,-0.02])
    parser.add_argument('--aisle1-se2', type=list, nargs=3, help="x,y,theta from anchor fiducial to aisle1 entry", required=False, default=[1,1,6,0])
    parser.add_argument('--aisle-width', type=float, help="nominal width of aisle", required=False, default=3)
    parser.add_argument('--aisle-scan-standoff', type=float, help="goal standoff from side of aisle being scanned. must be less than aisel-width", required=False, default=2.4)
    parser.add_argument('--aisle-stride', type=float, help="nominal distance between aisle centerlines", required=False, default=9)
    parser.add_argument('--map', type=str, help="path to GraphNav map")
    parser.add_argument('--anchor-tag', type=int, help="ID of fiducial designating start point and map localization", required=False, default=2)


def main(argv):
    # The last argument should be the IP address of the robot. The app will use the directory to find
    # the velodyne and start getting data from it.
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    bosdyn.client.util.add_payload_credentials_arguments(parser)
    bosdyn.client.util.add_service_endpoint_arguments(parser)
    add_autnomoy_arguments(parser)
    options = parser.parse_args(argv)

    sdk = bosdyn.client.create_standard_sdk('OccupancyGrid')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.sync_with_directory()


    autonomy = AutonomyEngine()

if __name__=='__main__':
    main()