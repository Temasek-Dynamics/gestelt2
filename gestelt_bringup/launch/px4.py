#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id')
    
    drone_id_launch_arg = DeclareLaunchArgument(
      'drone_id',
      default_value='0'
    )

    ''' Get parameter files '''
    px4_pluginlists_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_pluginlists.yaml'
    )

    px4_config_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_config.yaml'
    )

    '''Mavlink/Mavros'''
    fcu_addr =  PythonExpression(['14540 +', drone_id])
    # fcu_port =  PythonExpression(['14580 +', drone_id])
    fcu_port =  PythonExpression(['14557 +', drone_id]) # Used for SITL
    fcu_url = ["udp://:", fcu_addr, "@localhost:", fcu_port] # udp://:14540@localhost:14557
    tgt_system = PythonExpression(['1 +', drone_id])

    mavros_node = Node(
      package='mavros',
      executable='mavros_node',
      output='screen',
      shell=False,
      # name=['mavros_node_', drone_id],
      parameters=[
        {'fcu_url': fcu_url},
        {'gcs_url': ''},
        {'tgt_system': tgt_system},
        {'tgt_component': 1},
        {'fcu_protocol': 'v2.0'},
        # {'local_position.frame_id': map_frame},
        # {'local_position.tf.send': 'true'},
        # {'local_position.tf.frame_id': map_frame},
        # {'local_position.tf.child_frame_id': base_link_frame},
        px4_pluginlists_cfg,
        px4_config_cfg,
      ],

    )

    return LaunchDescription([
        drone_id_launch_arg,
        # Mavlink to ROS bridge
        mavros_node,
        # Drone simulation instance
        # px4_sitl,
    ])
