#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 Lin Zhi
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

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class DroneGoalManager(Node):
    def __init__(self):
        super().__init__('drone_goal_manager')

        # Parameters
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('goals', [0.0, 0.0, 1.0])

        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        flat_goals = self.get_parameter('goals').get_parameter_value().double_array_value

        if len(flat_goals) % 3 != 0:
            raise ValueError('Parameter goals must contain x,y,z triples')

        self.goals = []
        for i in range(0, len(flat_goals), 3):
            self.goals.append([
                float(flat_goals[i]),
                float(flat_goals[i + 1]),
                float(flat_goals[i + 2]),
            ])

        self.goal_index = 0
        self.position = None
        self.effective_goal = None
        self.finished = False
        self.publish_flag = True

        # Publisher and subscribers — no namespace, runs locally onboard
        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, sensor_qos
        )

        self.plan_sub = self.create_subscription(
            Path, 'received_global_plan', self.plan_callback, sensor_qos
        )

        self.get_logger().info(f'Loaded {len(self.goals)} goals')

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def odom_callback(self, msg):
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

    def plan_callback(self, msg):
        """Store the last pose of the plan as the effective goal."""
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose.position
            self.effective_goal = [
                last_pose.x,
                last_pose.y,
                last_pose.z,
            ]

    def timer_callback(self):
        if self.finished:
            self.get_logger().info('All goals finished')
            return

        if self.position is None:
            self.get_logger().warn('Received no odom yet')
            return

        # Use the effective goal (end of plan) if available,
        # otherwise we haven't received a plan yet so publish to trigger planning
        if self.effective_goal is None:
            if self.publish_flag:
                self.publish_goal(self.goals[self.goal_index])
                # self.publish_flag = False
            return
        else:
            self.publish_flag = False

        dx = self.effective_goal[0] - self.position[0]
        dy = self.effective_goal[1] - self.position[1]
        dz = self.effective_goal[2] - self.position[2]

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance <= self.goal_tolerance:
            self.get_logger().info(
                f'Reached goal {self.goal_index + 1}/{len(self.goals)} '
                f'(effective goal: {self.effective_goal})'
            )
            self.publish_flag = True
            self.goal_index += 1
            self.effective_goal = None  # Reset for next goal

            if self.goal_index >= len(self.goals):
                self.finished = True
                self.get_logger().info('Finished all goals')
                return

            self.get_logger().info(
                f'Switching to next goal {self.goals[self.goal_index]}'
            )

        if self.publish_flag:
            self.publish_goal(self.goals[self.goal_index])
            self.publish_flag = False

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneGoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
