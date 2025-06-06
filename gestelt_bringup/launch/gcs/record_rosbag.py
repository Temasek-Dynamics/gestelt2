#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime

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

NUM_AGENTS = 1

def generate_launch_description():
    # ROSBag 
    bag_topics = [
        # Pose
        "/odom",
        
        # Mapping
        "/local_map/bounds",
        "/occ_map",

        # Global Plan
        
        # Controller output
        "/sfc",
        "/received_global_plan",
        "/mpc_ref_path",
        "/mpc_traj",
        "/intmd_cmd",

        # Transformations
        "/tf",
        "/tf_static",

        # User commands
        "/uav_command",
        "/global_uav_command",

        # PX4
        "/fmu/out/vehicle_odometry",
        "/fmu/out/vehicle_status",

        "/rosout"
    ]

    bag_file = os.path.join(
        os.path.expanduser("~"), 'bag_files',
        'bag_' + datetime.now().strftime("%d%m%Y_%H_%M_%S"),
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             bag_file,
             *bag_topics],
        output='log'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(rosbag_record)

    return ld