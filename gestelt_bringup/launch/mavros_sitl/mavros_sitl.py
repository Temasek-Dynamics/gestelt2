#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushROSNamespace

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

def generateSITLDrone(id, spawn_pos, pcd_filepath, num_drones):

    sitl_drone_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch','mavros_sitl','include','mavros_nodes.py'
            ])
        ]),
        launch_arguments={
            'drone_id': str(id),
            'init_x': str(spawn_pos[0]),
            'init_y': str(spawn_pos[1]),
            'init_yaw': str(spawn_pos[2]),
            'fake_map_pcd_filepath': str(pcd_filepath),
            'num_drones': str(num_drones),
        }.items()
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          sitl_drone_launchfile,
        ]
    )

def launch_setup(context):
    scenario_name = LaunchConfiguration('scenario_name').perform(context)

    scenario = Scenario(os.path.join(
        get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        scenario_name
    )
    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'pcd_maps',
      scenario.map + '.pcd'
    )

    sitl_drone_nodes = []
    for id in range(scenario.num_agents):
        sitl_drone_nodes.append(generateSITLDrone(
            id, scenario.spawns_pos[id], fake_map_pcd_filepath, scenario.num_agents))

    return sitl_drone_nodes

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)

    launch_args = [
        DeclareLaunchArgument('scenario_name', default_value='start_2d_sitl'),
    ]

    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
