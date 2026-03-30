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


class MultiDroneGoals(Node):
    def __init__(self):
        super().__init__('drone_goal_manager_sim')

        # Simple parameters
        self.declare_parameter('drone_names', ['d0', 'd1'])
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 2.0)

        drone_names = self.get_parameter('drone_names').get_parameter_value().string_array_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.drones = {}
        self.subscribers = []

        self.publish_flag = []

        for name in drone_names:
            param_name = f'{name}_goals'
            self.declare_parameter(param_name, [0.0, 0.0, 1.0])

            flat_goals = self.get_parameter(param_name).get_parameter_value().double_array_value

            if len(flat_goals) % 3 != 0:
                raise ValueError(f'Parameter {param_name} must contain x,y,z triples')

            goals = []
            for i in range(0, len(flat_goals), 3):
                goals.append([
                    float(flat_goals[i]),
                    float(flat_goals[i + 1]),
                    float(flat_goals[i + 2]),
                ])

            pub = self.create_publisher(
                PoseStamped,
                f'/{name}/goal',
                10
            )

            odom_sub = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, drone_name=name: self.odom_callback(msg, drone_name),
                sensor_qos
            )

            plan_sub = self.create_subscription(
                Path,
                f'/{name}/received_global_plan',
                lambda msg, drone_name=name: self.plan_callback(msg, drone_name),
                sensor_qos
            )

            self.subscribers.append(odom_sub)
            self.subscribers.append(plan_sub)

            self.drones[name] = {
                'goals': goals,
                'goal_index': 0,
                'position': None,
                'effective_goal': None,
                'publisher': pub,
                'finished': False,
            }

            self.publish_flag.append(True)

            self.get_logger().info(f'{name}: loaded {len(goals)} goals')

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def odom_callback(self, msg, drone_name):
        self.drones[drone_name]['position'] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]

    def plan_callback(self, msg, drone_name):
        """Store the last pose of the plan as the effective goal."""
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose.position
            self.drones[drone_name]['effective_goal'] = [
                last_pose.x,
                last_pose.y,
                last_pose.z,
            ]

    def timer_callback(self):
        for drone_name, drone in self.drones.items():
            if drone['finished']:
                self.get_logger().info(f'{drone_name} is finished')
                continue

            if drone['position'] is None:
                self.get_logger().warn(f'Received no odom from {drone_name}')
                continue

            # Use the effective goal (end of plan) if available,
            # otherwise we haven't received a plan yet so skip the check
            effective_goal = drone['effective_goal']
            if effective_goal is None:
                # No plan received yet, publish the goal to trigger planning
                if self.publish_flag[int(drone_name[-1])]:
                    self.publish_goal(drone, drone['goals'][drone['goal_index']])
                    # self.publish_flag[int(drone_name[-1])] = False
                continue
            else:
                self.publish_flag[int(drone_name[-1])] = False

            current_position = drone['position']

            dx = effective_goal[0] - current_position[0]
            dy = effective_goal[1] - current_position[1]
            dz = effective_goal[2] - current_position[2]

            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance <= self.goal_tolerance:
                self.get_logger().info(
                    f'{drone_name}: reached goal {drone["goal_index"] + 1}/{len(drone["goals"])} '
                    f'(effective goal: {effective_goal})'
                )
                self.publish_flag[int(drone_name[-1])] = True

                drone['goal_index'] += 1
                drone['effective_goal'] = None  # Reset for next goal

                if drone['goal_index'] >= len(drone['goals']):
                    drone['finished'] = True
                    self.get_logger().info(f'{drone_name}: finished all goals')
                    continue

                current_goal = drone['goals'][drone['goal_index']]
                self.get_logger().info(
                    f'{drone_name}: switching to next goal {current_goal}'
                )
            if self.publish_flag[int(drone_name[-1])]:
                self.publish_goal(drone, drone['goals'][drone['goal_index']])
                self.publish_flag[int(drone_name[-1])] = False

    def publish_goal(self, drone, goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        drone['publisher'].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneGoals()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()