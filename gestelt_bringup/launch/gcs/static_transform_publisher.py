#!/usr/bin/env python3
"""
Static transform publisher for GCS.

Publishes world -> d{id}_map transforms for each drone based on the
spawn positions defined in a single scenario from scenarios.json.

Configure SCENARIO_NAME below. Each entry in spawns_pos is mapped to a
drone in order. Drone IDs default to 0, 1, 2, ... but can be overridden
with DRONE_IDS (useful when drone numbering is non-contiguous, e.g. [0,1,4]).
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

# ── Configure here ────────────────────────────────────────────────────────────
# Scenario to load from scenarios.json. Its spawns_pos entries define the
# world->d{id}_map transforms published for each drone.
SCENARIO_NAME = "multi_drone_real_flight"

# (Optional) Explicit drone IDs, one per spawns_pos entry, in order.
# Leave empty to use 0-based indices (0, 1, 2, ...).
# Example for non-contiguous IDs: DRONE_IDS = [0, 1, 4]
DRONE_IDS = []
# ─────────────────────────────────────────────────────────────────────────────


def load_scenarios(filepath):
    with open(filepath) as f:
        return json.load(f)


def generate_launch_description():
    scenarios_path = os.path.join(
        get_package_share_directory('gestelt_commander'), 'scenarios.json'
    )
    all_scenarios = load_scenarios(scenarios_path)

    scenario = all_scenarios.get(SCENARIO_NAME)
    if scenario is None:
        raise RuntimeError(
            f"Scenario '{SCENARIO_NAME}' not found in scenarios.json. "
            f"Available: {list(all_scenarios.keys())}"
        )

    spawns = scenario.get("spawns_pos", [])
    if not spawns:
        raise RuntimeError(
            f"Scenario '{SCENARIO_NAME}' has no spawns_pos entries."
        )

    drone_ids = DRONE_IDS if DRONE_IDS else list(range(len(spawns)))

    if len(drone_ids) != len(spawns):
        raise RuntimeError(
            f"DRONE_IDS length ({len(drone_ids)}) does not match "
            f"spawns_pos count ({len(spawns)}) in scenario '{SCENARIO_NAME}'."
        )

    ld = LaunchDescription()
    global_frame = "world"

    for drone_id, spawn in zip(drone_ids, spawns):
        x, y, yaw = str(spawn[0]), str(spawn[1]), str(spawn[2])
        ns = f"d{drone_id}"
        map_frame = f"{ns}_map"

        ld.add_action(
            GroupAction(
                actions=[
                    Node(
                        package="tf2_ros",
                        name=f"{ns}_world_to_map_tf",
                        executable="static_transform_publisher",
                        output="own_log",
                        arguments=[x, y, "0", yaw, "0", "0",
                                   global_frame, map_frame],
                    ),
                ]
            )
        )

    return ld
