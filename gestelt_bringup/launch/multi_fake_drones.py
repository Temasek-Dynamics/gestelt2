#!/usr/bin/env python

"""
Complete launch file to simulate a multi-agent navigation scenario
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# SCENARIO_NAME = "forest_dense_1"
SCENARIO_NAME = "antipodal_swap_8"
# SCENARIO_NAME = "map_test"

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

def generateFakeDrone(id, spawn_pos, pcd_filepath):

    fake_drone_complete_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch',
                'fake_drone.py'
            ])
        ]),
        launch_arguments={
            'drone_id': str(id),
            'init_x': str(spawn_pos[0]),
            'init_y': str(spawn_pos[1]),
            'init_yaw': str(spawn_pos[2]),
            'fake_map_pcd_filepath': str(pcd_filepath),
        }.items()
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          fake_drone_complete_launchfile,
        ]
    )

def generate_launch_description():

    scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        SCENARIO_NAME
    )

    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'pcd_maps',
      scenario.map + '.pcd'
    )

    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    # World to map transformation
    world_to_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "world", "map"])

    # Fake map
    fake_map = Node(
        package='fake_map',
        executable='fake_map_publisher_node',
        output='screen',
        shell=True,
        name='fake_map_publisher_node',
        parameters=[
            {'fake_map.pcd_filepath': fake_map_pcd_filepath},
            {'fake_map.frame_id': "world"},
            {'fake_map.publishing_frequency': 1.0},
        ],
    )

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=True,
        arguments=['-d' + rviz_cfg]
    )

    # Mission node: Sends goals to agents
    mission_node = Node(
        package='gestelt_mission',
        executable='mission',
        output='screen',
        shell=True,
        name='mission_node',
        parameters = [
            {'scenario': scenario.name},
            {'init_delay': 2},
        ]
    )

    # Generate nodes of fake drone simulation instances according to scenario
    fake_drone_nodes = []
    for id in range(scenario.num_agents):
        fake_drone_nodes.append(generateFakeDrone(id, scenario.spawns_pos[id], fake_map_pcd_filepath))

    return LaunchDescription([
        # Central nodes
        world_to_map_tf,
        fake_map,
        rviz_node,
        mission_node,
        # Simulation instances
        *fake_drone_nodes
    ])