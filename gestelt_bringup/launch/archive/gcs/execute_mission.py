#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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

def launch_setup(context):
    scenario_name = LaunchConfiguration('scenario_name').perform(context)

    scenario = Scenario(os.path.join(
        get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        scenario_name
    )

    # Mission node: Sends goals to agents
    mission_node = Node(
        name='mission_node',
        package='gestelt_mission',
        executable='mission',
        output='screen',
        emulate_tty=False,
        shell=True,
        parameters = [
            {'scenario': scenario.name},
            {'init_delay': 1},
        ]
    )

    return [
        SetEnvironmentVariable(name='PYTHONUNBUFFERED', value='0'),
        mission_node,
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)

    launch_args = [
        DeclareLaunchArgument('scenario_name', default_value='goals_2d_0'),
    ]

    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
