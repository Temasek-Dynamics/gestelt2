#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
      
    output_json_filepath = os.path.join(
      os.path.expanduser("~"), 'gestelt_ws/src/Gestelt2/gestelt_bringup',
      'map0_a_star_raw.json'
    )

    ''' Get launch argument values '''
    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')

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
    init_yaw_launch_arg = DeclareLaunchArgument(
      'init_yaw',
      default_value='0.0'
    )

    ''' Get parameter files '''
    navigator_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'navigator.yaml'
    )

    voxel_map_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'voxel_map.yaml'
    )

    fake_drone_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'fake_drone.yaml'
    )

    ''' Navigator: Planner module '''
    navigator_node = Node(
        package='voronoi_planner',
        executable='voronoi_planner_node',
        output='screen',
        shell=True,
        name='voronoi_planner',
        parameters=[
            {'navigator.drone_id': drone_id},
            {'navigator.planner.output_json_filepath': output_json_filepath},
            navigator_cfg,
            voxel_map_cfg,
        ],
    )

    ''' Fake drone without dynamics '''
    fake_drone_node = Node(
        package='fake_drone',
        executable='fake_drone_node',
        output='screen',
        shell=True,
        name='fake_drone_node',
        parameters = [
            fake_drone_cfg,
            {'fake_drone.drone_id': drone_id},
            {'fake_drone.init.x': init_x},
            {'fake_drone.init.y': init_y},
            {'fake_drone.init.yaw': init_yaw},
        ]
    )

    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        init_yaw_launch_arg,

        # Planner
        navigator_node,
        # Fake drone 
        fake_drone_node,

    ])
