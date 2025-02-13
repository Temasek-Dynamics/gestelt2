#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace

SCENARIO_NAME = "empty"
# SCENARIO_NAME = "sutd_2"

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
    scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        SCENARIO_NAME
    )

    '''ROS parameters'''
    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    '''Frames'''
    world_to_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = ["0", "0", "0", "0", "0", "0", 
                                  'world', "map"])

    drone0_origin_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = ["0", "0", "0", "0", "0", "0", 
                                  "map", "drone0_origin"])

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=False,
        arguments=['-d' + rviz_cfg]
    )

    # ROSBag 
    bag_topics = []
    # bag_topics.append("/odom")
    # bag_topics.append("/agent_id_text")
    # bag_topics.append("/d0/occ_map")
    # Subscription to 3d occupancy voxel map
    bag_topics.append("/visbot_itof/point_cloud")

    bag_topics.append("/mavros/local_position/odom")

    # Paths
    bag_topics.append("/d0/fe_plan/viz")
    bag_topics.append("/d0/mpc/traj")
    # Subscription to point clouds
    bag_topics.append("/rosout")
    bag_topics.append("/tf")

    bag_file = os.path.join(
        os.path.expanduser("~"), 'bag_files',
        'bag_' + datetime.now().strftime("%d%m%Y_%H_%M_%S"),
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', 
            '-o', bag_file, *bag_topics],
        output='log'
    )

    return LaunchDescription([
        # rosbag_record,
        rviz_node,
        # world_to_map_tf,
        # drone0_origin_tf,
    ])
