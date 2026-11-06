#!/usr/bin/env python3
"""
Goal republisher for GCS.

Latches the most recent goal published by RVIZ's "2D Goal Pose" tool and
republishes it at a fixed rate, so a single click keeps driving the goal
topic instead of firing once.
"""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class GoalRepublisher(Node):
    """Republishes the latest goal pose at a fixed rate."""

    def __init__(self):
        super().__init__('goal_republisher')

        self.declare_parameter('input_topic', '/goal_pose')
        self.declare_parameter('output_topic', '/point_goal')
        self.declare_parameter('rate_hz', 10.0)

        input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        rate_hz = self.get_parameter(
            'rate_hz').get_parameter_value().double_value

        if rate_hz <= 0.0:
            raise RuntimeError(f'rate_hz must be positive, got {rate_hz}')

        self.latest_goal = None

        self.subscription = self.create_subscription(
            PoseStamped,
            input_topic,
            self.goal_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            output_topic,
            10
        )

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_goal)

        self.get_logger().info(
            f"Republishing '{input_topic}' to '{output_topic}' "
            f'at {rate_hz:.1f} Hz'
        )

    def goal_callback(self, msg):
        self.latest_goal = msg
        self.get_logger().info('Received new goal')

    def publish_goal(self):
        if self.latest_goal is not None:
            self.latest_goal.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.latest_goal)


def main():
    rclpy.init()
    node = GoalRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
