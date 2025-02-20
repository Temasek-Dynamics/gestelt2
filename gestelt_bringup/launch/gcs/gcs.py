#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch.substitutions import FindExecutable

from launch_ros.actions import Node, PushROSNamespace

def generate_launch_description():

    '''ROS parameters'''
    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    # ros2 run topic_tools relay /d0/tf /tf
    # ros2 run topic_tools relay /d1/tf /tf
    # ros2 run topic_tools relay /d0/tf_static /tf_static
    # ros2 run topic_tools relay /d1/tf_static /tf_static

    relay_d0_tf = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 
              'run topic_tools relay /d0/tf /tf'
            ],
        output='screen',
        shell=True
    )

    relay_d1_tf = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 
              'run topic_tools relay /d1/tf /tf'
            ],
        output='screen',
        shell=True
    )

    relay_d0_static_tf = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 
              'run topic_tools relay /d0/static_tf /static_tf'
            ],
        output='screen',
        shell=True
    )

    relay_d1_static_tf = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'), 
              'run topic_tools relay /d1/static_tf /static_tf'
            ],
        output='screen',
        shell=True
    )

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=False,
        arguments=['-d' + rviz_cfg],
        remappings=[
            # ('/tf', ['/d0/tf']),
            # ('/tf_static', ['/d0/tf_static']),
            # ('/tf', ['/d1/tf']),
            # ('/tf_static', ['/d1/tf_static']),
        ],
    )

    # ROSBag 
    bag_topics = []

    for id in range(2):
        prefix = "/d" + str(id) + "/"
        bag_topics.append(prefix + "odom")
        bag_topics.append(prefix + "agent_id_text")
        # Subscription to paths
        bag_topics.append(prefix + "fe_plan_req")
        bag_topics.append(prefix + "fe_plan/viz")
        bag_topics.append(prefix + "sfc")
        bag_topics.append(prefix + "mpc/traj")
        # Subscription to 3d occupancy voxel map
        bag_topics.append(prefix + "occ_map")
        # bag_topics.append(prefix + "visbot_itof/point_cloud")
        # Subscription to maps

    bag_topics.append("/tf")
    bag_topics.append("/tf_static")

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
        rosbag_record,
        rviz_node,
        # relay_d0_tf,
        # relay_d1_tf,
        # relay_d0_static_tf,
        # relay_d1_static_tf,
    ])
