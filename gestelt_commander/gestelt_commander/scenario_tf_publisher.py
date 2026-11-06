#!/usr/bin/env python3
"""
Static transform publisher for GCS.

Publishes world -> d{id}_map transforms for each drone based on the spawn
positions defined in a single scenario from scenarios.json.

Every transform is sent in a single latched /tf_static message, so late
subscribers (RVIZ, planner, controller) still receive the full set.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from gestelt_commander.scenario import Scenario


def parse_drone_ids(ids_str, num_spawns):
    """Parse the 'drone_ids' parameter into a list of drone IDs.

    An empty string means 0-based indices (0, 1, 2, ...). Otherwise a
    comma-separated list, optionally bracketed, is expected. Useful when drone
    numbering is non-contiguous, e.g. '0,1,4' or '[0, 1, 4]'.
    """
    ids_str = ids_str.strip().strip('[]').strip()

    if not ids_str:
        return list(range(num_spawns))

    return [int(token) for token in ids_str.split(',') if token.strip()]


class ScenarioTfPublisher(Node):
    """Publishes the world -> d{id}_map transforms defined by a scenario."""

    def __init__(self):
        super().__init__('scenario_tf_publisher')

        # Scenario to load from scenarios.json. Its spawns_pos entries define
        # the world->d{id}_map transforms published for each drone.
        self.declare_parameter('scenario_name', 'multi_drone_real_flight')
        self.declare_parameter('global_frame', 'world')
        # Explicit drone IDs, one per spawns_pos entry, in order. Leave empty
        # to use 0-based indices. See parse_drone_ids().
        self.declare_parameter('drone_ids', '')

        scenario_name = self.get_parameter(
            'scenario_name').get_parameter_value().string_value
        global_frame = self.get_parameter(
            'global_frame').get_parameter_value().string_value
        drone_ids_str = self.get_parameter(
            'drone_ids').get_parameter_value().string_value

        scenarios_path = os.path.join(
            get_package_share_directory('gestelt_commander'), 'scenarios.json'
        )
        scenario = Scenario(scenarios_path, scenario_name)

        spawns = scenario.spawns_pos
        if not spawns:
            raise RuntimeError(
                f"Scenario '{scenario_name}' has no spawns_pos entries."
            )

        drone_ids = parse_drone_ids(drone_ids_str, len(spawns))

        if len(drone_ids) != len(spawns):
            raise RuntimeError(
                f'drone_ids length ({len(drone_ids)}) does not match '
                f'spawns_pos count ({len(spawns)}) in scenario '
                f"'{scenario_name}'."
            )

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        transforms = [
            self.make_transform(global_frame, f'd{drone_id}_map', spawn)
            for drone_id, spawn in zip(drone_ids, spawns)
        ]

        # One latched message containing every transform
        self.tf_broadcaster.sendTransform(transforms)

        self.get_logger().info(
            f"Scenario '{scenario_name}': published {len(transforms)} static "
            f"transform(s) from '{global_frame}' to "
            f"[{', '.join(tf.child_frame_id for tf in transforms)}]"
        )

    def make_transform(self, parent_frame, child_frame, spawn):
        """Build a TransformStamped from an [x, y, yaw] spawn entry."""
        x, y, yaw = float(spawn[0]), float(spawn[1]), float(spawn[2])

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0

        # Yaw-only rotation about the Z axis
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)

        return tf


def main():
    rclpy.init()
    node = ScenarioTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
