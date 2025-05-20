#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from gestelt_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import ExternalShutdownException

from gestelt_commander.scenario import *


def main(args=None):
    try:
        with rclpy.init(args=args):
            mission_mngr = MissionManager()

            executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
            executor.add_node(mission_mngr)

            executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        rclpy.shutdown()
    
    exit(0)

if __name__ == '__main__':
    main()
