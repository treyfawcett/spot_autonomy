import argparse
import collections
from enum import Enum
import enum
import logging
import sys
import time
import threading
from typing import List, Optional
import numpy as np
import cv2 as cv

import bosdyn
import bosdyn.client
import bosdyn.client.util

from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
                                                  DirectoryRegistrationKeepAlive)

from bosdyn.client.fault import FaultClient
from bosdyn.client.server_util import GrpcServiceRunner, populate_response_header
from bosdyn.client.world_object import WorldObjectClient, make_add_world_object_req

from bosdyn.client.util import setup_logging

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks

from bosdyn.api import local_grid_pb2, local_grid_service_pb2_grpc 

from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *
from bosdyn.api import geometry_pb2 as geom
from bosdyn.api import world_object_pb2


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

        

    def __init__(self, robot) -> None:
        occupancy_grid_client = robot.ensure_client('occupancy-grid')
        self._occupancy_task = AsyncOccupancyGrid(occupancy_grid_client,'occupancy')

        world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
        self._fiducial_task = AsyncFiducialObjects(world_object_client)

        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_state_task = AsyncRobotState(robot_state_client)

        localization_client = robot.ensure_client(GraphNavClient.default_service_name)
        self._localization_task = AsyncLocalization(localization_client)

        _task_list = [self._occupancy_task, self._fiducial_task, self._robot_state_task, self._localization_task]
        _async_tasks = AsyncTasks(_task_list)

        self._stop_signal = threading.Event()
        self._async_updates_thread = threading.Thread(target=_update_thread, args=[_async_tasks, self._stop_signal])
        self._async_updates_thread.daemon = True
        self._async_updates_thread.start()


    def tick(self):
        pass



    def _startup(self):
        pass



    def _locate_home_fiducial(self):
        pass


    def _localize_hallways(self):
        pass




def add_autnomoy_arguments(parser):
    pass

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