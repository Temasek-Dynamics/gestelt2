#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

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

    drone_id_str = LaunchConfiguration('drone_id').perform(context)
    drone_id_int = int(drone_id_str)
    scenario_name = LaunchConfiguration('scenario_name').perform(context)

    scenario = Scenario(os.path.join(
        get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        scenario_name
    )

    sitl_drone_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch','offboard','include','offboard_nodes.py'
            ])
        ]),
        launch_arguments={
            'drone_id': drone_id_str,
            'init_x':   str(scenario.spawns_pos[drone_id_int][0]),
            'init_y':   str(scenario.spawns_pos[drone_id_int][1]),
            'init_yaw': str(scenario.spawns_pos[drone_id_int][2]),
            'num_drones': str(scenario.num_agents),
        }.items()
    )

    return [sitl_drone_launchfile]


def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)

    launch_args = [
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('scenario_name', default_value='empty2'),
    ]

    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
