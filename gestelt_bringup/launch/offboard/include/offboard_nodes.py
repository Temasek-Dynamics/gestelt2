#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

from nav2_common.launch import RewrittenYaml

# def render_launch_config(context: LaunchContext, launch_config):
#   launch_config_str = context.perform_substitution(launch_config)
#   # Render xacro

def generate_launch_description():
    ''' Get launch argument values '''
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

    num_drones_arg = DeclareLaunchArgument(
      'num_drones',
      default_value='4'
    )

    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    num_drones = LaunchConfiguration('num_drones')

    '''Frames'''
    global_frame = 'map' # Fixed
    map_frame = ['d', drone_id, '_origin'] # Fixed
    base_link_frame = ['d', drone_id, '_base_link'] # Dynamic
    local_map_frame = ['d', drone_id, '_lcl_map'] # Fixed to base_link
    camera_frame = ['d', drone_id, '_camera_link'] # Fixed to base_link

    ''' Get parameter files '''
    traj_server_config = os.path.join(
      get_package_share_directory('trajectory_server'),
      'config',
      'trajectory_server.yaml'
    )

    navigator_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'navigator.yaml'
    )

    voxel_map_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'voxel_map.yaml'
    )


    """Nodes"""
    world_to_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = ["0", "0", "0", "0", "0", "0", 
                                  'world', global_frame])

    # Publish TF for map to fixed drone origin
    # This is necessary because PX4 SITL is not able to change it's initial starting position
    drone_origin_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = [init_x, init_y, "0", "0", "0", "0", 
                                  global_frame, map_frame])
    
    # drone base_link to sensor fixed TFx
    camera_link_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = ["0.07535705", 
                                   "0.05172403", 
                                   "-0.03221506", 
                                   "-0.5", 
                                   "0.5", 
                                   "-0.5", 
                                   "0.5", 
                                  base_link_frame, camera_frame])
    
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
        ('odom', ['/mavros/local_position/odom']),
      ],
    )

    ''' Trajectory server for executing trajectories '''
    trajectory_server = Node(
      package='trajectory_server',
      executable='trajectory_server_node',
      output='screen',
      shell=False,
      name=['traj_server_', drone_id],
      parameters=[
        {'drone_id': drone_id},
        {'map_frame': map_frame},
        traj_server_config
      ],
      remappings=[
        ('odom', ['/mavros/local_position/odom']),
        ('mavros/state', ['/mavros/state']),
        ('mavros/setpoint_raw/local', ['/mavros/setpoint_raw/local']),
        ('mavros/cmd/arming', ['/mavros/cmd/arming']),
        ('mavros/set_mode', ['/mavros/set_mode']),
      ],
    )

    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        num_drones_arg,
        # Static transforms
        world_to_map_tf,
        drone_origin_tf,
        camera_link_tf,
        # Nodes
        navigator_node,
        trajectory_server,
        # Set parameters
    ])
