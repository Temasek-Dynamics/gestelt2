#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

# def render_launch_config(context: LaunchContext, launch_config):
#   launch_config_str = context.perform_substitution(launch_config)
#   # Render xacro

def generate_launch_description():
    ''' Get launch argument values '''
    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    fake_map_pcd_filepath = LaunchConfiguration('fake_map_pcd_filepath')
    num_drones = LaunchConfiguration('num_drones')

    drone_id_launch_arg = DeclareLaunchArgument(
      'drone_id',
      default_value='0'
    )

    init_x_launch_arg = DeclareLaunchArgument(
      'init_x',
      default_value='0.0'
    )
    init_y_launch_arg = DeclareLaunchArgument(
      'init_y',
      default_value='0.0'
    )

    fake_map_pcd_filepath_launch_arg = DeclareLaunchArgument(
      'fake_map_pcd_filepath',
      default_value=''
    )

    num_drones_arg = DeclareLaunchArgument(
      'num_drones',
      default_value='4'
    )

    '''Frames'''
    global_frame = 'map' # Fixed
    map_frame = ['d', drone_id, '_origin'] # Fixed
    base_link_frame = ['d', drone_id, '_base_link'] # Dynamic
    local_map_frame = ['d', drone_id, '_lcl_map'] # Fixed to base_link
    camera_frame = ['d', drone_id, '_camera_link'] # Fixed to base_link

    ''' Get parameter files '''
    navigator_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'navigator.yaml'
    )

    voxel_map_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'voxel_map.yaml'
    )

    ''' Navigator: Planner module '''
    navigator_node = Node(
        package='navigator',
        executable='navigator_node',
        output='screen',
        shell=False,
        name=['navigator_', drone_id],
        parameters=[
            {'drone_id': drone_id},
            {'global_frame': global_frame},
            {'map_frame': map_frame},
            {'local_map_frame': local_map_frame},
            {'base_link_frame': base_link_frame},
            {'camera_frame': camera_frame},
            navigator_cfg,
            {'navigator.num_drones': num_drones},
            voxel_map_cfg,
        ],
      remappings=[
        ('cloud', ['/visbot_itof/point_cloud']),
      ],
    )


    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        fake_map_pcd_filepath_launch_arg,
        num_drones_arg,
        # fake_sensor,
        navigator_node,
    ])
