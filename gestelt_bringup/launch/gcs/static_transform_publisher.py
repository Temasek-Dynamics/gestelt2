#!/usr/bin/env python
"""
Static transform publisher for GCS.

Publishes world -> d{id}_map transforms for each drone based on its
scenario spawn position (x, y, yaw). Also publishes the base_link ->
camera_link identity transform for each drone.

Configure DRONE_SCENARIOS below: list of (drone_id, scenario_name) pairs.
Scenario names must exist in scenarios.json.

Example:
  d0 map frame: scenario "start_1d_zero"  -> spawn [-1.0, -3.0], yaw=0.0
  d1 map frame: scenario "start_1d_one"   -> spawn [ 1.0, -3.0], yaw=0.0
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

# ── Configure here ────────────────────────────────────────────────────────────
# Each tuple: (drone_id, scenario_name)
# The scenario must have at least one entry in spawns_pos.
# spawns_pos format per entry: [x, y, yaw_rad]
DRONE_SCENARIOS = [
    (0, "start_1d_zero"),
    (3, "start_1d_three"),
    (4, "start_1d_four"),
]
# ─────────────────────────────────────────────────────────────────────────────


def load_scenarios(filepath):
    with open(filepath) as f:
        return json.loads(f.read())


def generate_launch_description():
    scenarios_path = os.path.join(
        get_package_share_directory('gestelt_commander'), 'scenarios.json'
    )
    all_scenarios = load_scenarios(scenarios_path)

    ld = LaunchDescription()
    global_frame = "world"

    for drone_id, scenario_name in DRONE_SCENARIOS:
        scenario = all_scenarios.get(scenario_name)
        if scenario is None:
            raise RuntimeError(
                f"Scenario '{scenario_name}' not found in scenarios.json. "
                f"Available: {list(all_scenarios.keys())}"
            )

        spawns = scenario.get("spawns_pos", [])
        if not spawns:
            raise RuntimeError(
                f"Scenario '{scenario_name}' has no spawns_pos entries."
            )

        # spawns_pos[0] = [x, y, yaw_rad]
        spawn = spawns[0]
        x, y, yaw = str(spawn[0]), str(spawn[1]), str(spawn[2])

        ns = f"d{drone_id}"
        map_frame        = f"{ns}_map"
        base_link_frame  = f"{ns}_base_link"
        camera_frame     = f"{ns}_camera_link"

        ld.add_action(
            GroupAction(
                actions=[
                    # world -> d{id}_map  (at spawn x, y, z=0, yaw from scenario)
                    Node(
                        package="tf2_ros",
                        name=f"{ns}_world_to_map_tf",
                        executable="static_transform_publisher",
                        output="own_log",
                        arguments=[x, y, "0", yaw, "0", "0",
                                   global_frame, map_frame],
                    ),
                    # d{id}_base_link -> d{id}_camera_link  (identity)
                    # Node(
                    #     package="tf2_ros",
                    #     name=f"{ns}_base_link_to_cam_tf",
                    #     executable="static_transform_publisher",
                    #     output="own_log",
                    #     arguments=["0.0", "0.0", "0.0",
                    #                "0.0", "0.0", "0.0", "1.0",
                    #                base_link_frame, camera_frame],
                    # ),
                ]
            )
        )

    return ld
