#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace

SCENARIO_NAME = "map_test"

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

def generateSITLDrone(id, spawn_pos):

    sitl_drone_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch',
                'sitl_drone.py'
            ])
        ]),
        launch_arguments={
            'drone_id': str(id),
            'init_x': str(spawn_pos[0]),
            'init_y': str(spawn_pos[1]),
            'init_yaw': str(spawn_pos[2]),
        }.items()
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          sitl_drone_launchfile,
        ]
    )

def generate_launch_description():
    scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        SCENARIO_NAME
    )

    '''ROS parameters'''
    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'pcd_maps',
      scenario.map + '.pcd'
    )

    """Directory"""
    px4_dir = os.path.join(
      os.path.expanduser("~"), 'PX4-Autopilot'
    )

    px4_gz = os.path.join(
      px4_dir, "Tools/simulation/gz/simulation-gazebo"
    )

    """Gazebo"""
    gazebo = ExecuteProcess(
        cmd=[
            'python3', px4_gz,
            '--world', 'default',
            # '--headless',
        ],
        name='gazebo',
        shell=True
    )

    # XRCE Agent that will connect to ALL clients
    xrce_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 -p 8888 -v'
        ]],
        name='microxrceagent',
        shell=True
    )

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
            {'init_delay': 5},
        ]
    )

    # Generate nodes of SITL drone instances according to scenario
    sitl_drone_nodes = []
    for id in range(scenario.num_agents):
        sitl_drone_nodes.append(generateSITLDrone(id, scenario.spawns_pos[id]))

    return LaunchDescription([
        # Processes
        gazebo,
        xrce_agent,
        # Nodes
        fake_map,
        mission_node,
        rviz_node,
        # Simulation instances
        *sitl_drone_nodes,
    ])
