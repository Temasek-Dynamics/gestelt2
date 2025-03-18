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

    init_yaw_launch_arg = DeclareLaunchArgument(
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

    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    fake_map_pcd_filepath = LaunchConfiguration('fake_map_pcd_filepath')
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

    fake_sensor_config = os.path.join(
      get_package_share_directory('fake_sensor'),
      'config',
      'fake_sensor.yaml'
    )

    navigator_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'navigator.yaml'
    )

    voxel_map_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'voxel_map.yaml'
    )

    px4_pluginlists_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_pluginlists.yaml'
    )

    px4_config_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_config.yaml'
    )

    """Nodes"""

    # Publish TF for map to fixed drone origin
    # This is necessary because PX4 SITL is not able to change it's initial starting position
    drone_origin_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = [init_x, init_y, "0", init_yaw, "0", "0", 
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
        ('cloud', ['visbot_itof/point_cloud']),
        ('odom', ['mavros/local_position/odom']),
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
          ('odom', ['mavros/local_position/odom']),
          ('/all_uav_command', ['/d', drone_id, '/all_uav_command'])
        ],
    )

    ''' Fake sensor node: For acting as a simulated depth camera/lidar '''
    fake_sensor = Node(
        package='fake_sensor',
        executable='fake_sensor_node',
        output='screen',
        shell=False,
        name=['fake_sensor_', drone_id],
        parameters=[
          {'drone_id': drone_id},
          {'num_drones': num_drones},
          {'global_frame': global_frame},
          {'map_frame': map_frame},
          {'sensor_frame': camera_frame},
          {'pcd_map.filepath': fake_map_pcd_filepath},
          fake_sensor_config,
        ],
        remappings=[
          ('cloud', ['visbot_itof/point_cloud']),
          ('odom', ['mavros/local_position/odom']),
        ],
    )

    '''Mavlink/Mavros'''
    # Simulation
    # fcu_addr =  PythonExpression(['14540 +', drone_id])
    # fcu_port =  PythonExpression(['14580 +', drone_id])
    # fcu_port =  PythonExpression(['14557 +', drone_id]) # Used for SITL
    # fcu_url = ["udp://:", fcu_addr, "@localhost:", fcu_port] # udp://:14540@localhost:14557
    # tgt_system = PythonExpression(['1 +', drone_id])

    fcu_addr =  PythonExpression(['14540 +', drone_id])
    fcu_port =  PythonExpression(['14580 +', drone_id]) # Used for SITL
    fcu_url = ["udp://:", fcu_addr, "@localhost:", fcu_port] # udp://:14540@localhost:14557
    tgt_system = PythonExpression(['1 +', drone_id])
    # gcs_port = PythonExpression(['14556 +', drone_id])
    # gcs_url = ["udp://:", gcs_port, "@"] 

    px4_config_param_subs = {}
    px4_config_param_subs.update({'/**/local_position.ros__parameters.frame_id': map_frame})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.send': 'true'})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.frame_id': map_frame})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.child_frame_id': base_link_frame})

    new_px4_config_cfg = RewrittenYaml(
        source_file=px4_config_cfg,
        root_key='',
        param_rewrites=px4_config_param_subs,
        convert_types=True)

    mavros_node = Node(
      package='mavros',
      executable='mavros_node',
      output='screen',
      shell=False,
      namespace='mavros',
      parameters=[
        {'fcu_url': fcu_url},
        {'gcs_url': ""},
        {'tgt_system': tgt_system},
        {'tgt_component': 1},
        {'fcu_protocol': 'v2.0'},
        px4_pluginlists_cfg,
        new_px4_config_cfg,
      ],
      # remappings=[
      #   ('local_position/odom', ['odom']),
      # ],
    )

    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        init_yaw_launch_arg,
        fake_map_pcd_filepath_launch_arg,
        num_drones_arg,
        # Static transforms
        drone_origin_tf,
        camera_link_tf,
        # Nodes
        fake_sensor,
        navigator_node,
        trajectory_server,
        # Mavlink to ROS bridge
        mavros_node,
        # Set parameters
    ])
