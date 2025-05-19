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
from rclpy.duration import Duration

from gestelt_commander.scenario import *


def planAndFollowPath(navigator, ns=''):
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'world'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.5
    initial_pose.pose.orientation.w = 1.0

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(navigator=ns+'/planner_server', localizer='robot_localization')

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'world'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 1.0
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose, planner_id='GridBased', use_start=False)
    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)

    mission_mngr = MissionManager()
    navigator = BasicNavigator(node_name='basic_navigator')

    try: 
        #########
        # Take off 
        #########
        mission_mngr.cmdAllDronesPubGlobal(
            UAVCommand.Request.COMMAND_TAKEOFF, 
            UAVState.IDLE,
            value=mission_mngr.scenario.take_off_height)
        mission_mngr.get_logger().info("Sending commands to TAKE OFF")
        
        #########
        # Wait for Hover
        #########
        if not mission_mngr.waitForReqState(UAVState.HOVERING, max_retries=20):
            raise Exception("Failed to transition to hover mode")
        mission_mngr.get_logger().info("All drones are in HOVER MODE.")
        # Reset occupancy map
        mission_mngr.resetOccMap()

        #########
        # MissionManager mode
        #########
        mission_mngr.cmdAllDronesPubGlobal(
            UAVCommand.Request.COMMAND_START_MISSION, 
            UAVState.HOVERING,
            mode=0)
        mission_mngr.get_logger().info("All drones swtching switching to MISSION MODE")

        #########
        # Wait for mission_mngr
        #########
        if not mission_mngr.waitForReqState(UAVState.MISSION, max_retries=20):
            raise Exception("Failed to transition to mission mode")
        
        mission_mngr.get_logger().info("All drones in MISSION MODE")

        # Send a goal
        mission_mngr.get_logger().info("Requesting planned path.")
        planAndFollowPath(navigator)

        rclpy.spin(mission_mngr)

    except Exception as e:
        print(e)

    mission_mngr.destroy_node()
    # navigator.lifecycleShutdown()
    rclpy.shutdown()

    exit(0)

if __name__ == '__main__':
    main()
