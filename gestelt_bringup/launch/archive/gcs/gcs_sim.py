#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node, PushROSNamespace

def generate_launch_description():

    '''ROS parameters'''
    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'gazebo_sim.rviz'
    )

    '''Frames'''
    # world_to_map_tf = Node(package = "tf2_ros", 
    #                    executable = "static_transform_publisher",
    #                    output="log",
    #                   arguments = ["0", "0", "0", "0", "0", "0", 
    #                               'world', "map"])

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=False,
        arguments=['-d' + rviz_cfg],
    )

    # ROSBag 
    bag_topics = []
    # Position
    bag_topics.append("/d0/odom")
    bag_topics.append("/d0/agent_id_text")
    # Perception
    bag_topics.append("/d0/occ_map")
    bag_topics.append("/d0/visbot_itof/point_cloud")
    # Transforms
    bag_topics.append("/d0/tf")
    bag_topics.append("/d0/tf_static")

    # Paths
    bag_topics.append("/d0/fe_plan_req")
    bag_topics.append("/d0/fe_plan/viz")
    bag_topics.append("/d0/mpc/traj")
    # Subscription to point clouds
    bag_topics.append("/rosout")

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
    ])
