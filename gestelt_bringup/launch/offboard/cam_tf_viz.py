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

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare parameters
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'single_drone.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)

    ns = "d0"
    global_frame = "world" # Fixed
    map_frame = ns + "_map"
    base_link_frame = "base_link"
    camera_link = "camera_link"
    camera_front_frame = "camera_front"
    camera_front_right_frame = "camera_front_right"
    camera_front_left_frame = "camera_front_left"

    ld.add_action(
        GroupAction(
            actions=[
                Node( 
                    package = "tf2_ros", 
                    name=ns+'_world_to_map_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = ["0.0", "0.0", "0.0", 
                                "0.0", "0.0", "0.0", "1.0", 
                                global_frame, map_frame],
                ),
                Node( 
                    package = "tf2_ros", 
                    name=ns+'_world_to_map_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = ["0.0", "0.0", "0.0", 
                                "0.0", "0.0", "0.0", "1.0", 
                                map_frame, base_link_frame],
                ),
                Node(
                    package = "tf2_ros", 
                    name=ns+'_base_link_to_cam_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = [ "-0.085", "0.0", "0.0", 
                                    "0.0993197", "0.0", "-0.9950556", "0.0", #ZYX (-180, 11.4, 0)
                                    # "0.0", "0.0", "1.0", "0.0", #ZYX (-180, 0, 0)
                                    base_link_frame, camera_link],
                ),
                Node(
                    package = "tf2_ros", 
                    name=ns+'_cam_to_front_cam_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = [ "-0.2", "0.0", "0.0", 
                                    "0.5", "0.5", "-0.5", "-0.5", # 90 deg about z then 90 deg about x
                                    camera_link, camera_front_frame],
                ),
                Node(
                    package = "tf2_ros", 
                    name=ns+'_cam_to_left_cam_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = [ "-0.159341", "-0.071877", "0.000139", 
                                    "0.35707", "0.61104", "-0.61057", "-0.35541",
                                    camera_link, camera_front_left_frame],
                ),
                Node(
                    package = "tf2_ros", 
                    name=ns+'_cam_to_right_cam_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = [ "-0.160507", "0.071495", "0.000393", 
                                    "-0.605586", "-0.35930", "0.359366", "0.612390",
                                    camera_link, camera_front_right_frame],
                ),
            ]
        )
    )

    return ld