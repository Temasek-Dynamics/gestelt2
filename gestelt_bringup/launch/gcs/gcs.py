#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess, 
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode

SCENARIO_NAME = "single_drone_test"

class Scenario:
    """Scenario class that contains all the attributes of a scenario, used to start the fake_map
    and define the number of drones and their spawn positions
    """
    def __init__(self, filepath, scenario_name):
        with open(filepath) as f:
            json_dict = json.loads(f.read())

        scenario_dict = json_dict.get(scenario_name, None)

        if scenario_dict == None:
            raise Exception("Specified scenario does not exist!")

        self.name = scenario_name
        self.map = scenario_dict.get("map", None)
        self.spawns_pos = scenario_dict.get("spawns_pos", None )
        self.goals_pos = scenario_dict.get("goals_pos", None )
        self.num_agents = scenario_dict.get("num_agents", None )

        self.checks()

    def checks(self):
        if (len(self.spawns_pos) != self.num_agents):
            raise Exception("Number of spawn positions does not match number of agents!")

        if (len(self.goals_pos) != self.num_agents):
            raise Exception("Number of goal positions does not match number of agents!")

        if self.map == None or self.spawns_pos == None or self.goals_pos == None or self.num_agents == None:
            raise Exception("map_name and/or spawns_pos field does not exist!")

def generate_launch_description():
    scenario = Scenario(
        os.path.join(get_package_share_directory('gestelt_commander'), 'scenarios.json'),
        SCENARIO_NAME
    )

    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'single_drone.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Send single test goal
    # mission_node = Node(
    #     package='gestelt_commander',
    #     executable='test_take_off_goal',
    #     output='screen',
    #     emulate_tty=False,
    #     shell=True,
    #     parameters = [
    #         {'scenario': scenario.name},
    #         {'init_delay': 1},
    #     ]
    # )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)
    # ld.add_action(mission_node)

    return ld