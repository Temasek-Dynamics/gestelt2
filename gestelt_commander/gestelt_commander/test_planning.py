#! /usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 John Tan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
import time

from geometry_msgs.msg import PoseStamped
from gestelt_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

from gestelt_commander.mission_manager import MissionManager

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
    goal_pose.pose.position.x = 3.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 1.0
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose, planner_id='GridBased', use_start=True)
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
        time.sleep(5)
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
