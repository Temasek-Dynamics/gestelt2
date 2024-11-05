#!/usr/bin/env python

"""
Complete launch file to simulate a multi-agent navigation scenario
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch import LaunchDescription

def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]

def generate_launch_description():

    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    # rosbag_player_params = os.path.join(
    #   get_package_share_directory('gestelt_bringup'), 'config',
    #   'rosbag/player.yaml'
    # )

    # rosbag_player = Node(
    #   package='rosbag2_transport',
    #   executable='player',
    #   name='player',
    #   output="screen",
    #   shell=False, # If True, the process will be started in a new shell
    #   # emulate_tty=True, # If False, logs are processed as text only (e.g. removing colors)
    #   parameters=dump_params(rosbag_player_params, 'player'),
    # )

    name = "bag_05112024_16_56_54"

    bag_file = os.path.join(
        os.path.expanduser("~"), 'bag_files',
        name + '/' + name + '_0.mcap',
    )

    rosbag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
             bag_file],
        output='screen'
    )

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=False,
        arguments=['-d' + rviz_cfg]
    )

    return LaunchDescription([
        rosbag_player,
        rviz_node,
    ])