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
    init_yaw_launch_arg = DeclareLaunchArgument(
      'init_yaw',
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
    # map_frame = ["d", drone_id, "_origin"]
    map_frame = "world"
    local_map_frame = ["d", drone_id, "_lcl_map"]
    base_link_frame = ["d", drone_id, "_base_link"]
    camera_frame = ["d", drone_id, "_camera_link"]

    cloud_topic = ["/d", drone_id, "/cloud"]

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

    fake_drone_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'fake_drone.yaml'
    )

    """Nodes"""
    # Publish TF for map to fixed drone origin
    # This is necessary because PX4 SITL is not able to change it's initial starting position
    # drone_origin_tf = Node(package = "tf2_ros", 
    #                    executable = "static_transform_publisher",
    #                   arguments = [init_x, init_y, "0", "0", "0", "0", 
    #                           "world", map_frame])

    # drone base_link to sensor fixed TF
    camera_link_tf = Node(package = "tf2_ros", 
                          executable = "static_transform_publisher",
                          output = "log",
                          arguments = ["0", "0", "0", "0", "0", "0", base_link_frame, camera_frame])

    ''' Fake drone without dynamics '''
    fake_drone_node = Node(
        package='fake_drone',
        executable='fake_drone_node',
        output='screen',
        shell=False,
        name='fake_drone_node',
        parameters = [
            fake_drone_cfg,
            {'fake_drone.drone_id': drone_id},
            {'fake_drone.init_x': init_x},
            {'fake_drone.init_y': init_y},
            {'fake_drone.init_yaw': init_yaw},
        ]
    )

    ''' Planner module '''
    navigator_node = Node(
        package='navigator',
        executable='navigator_node',
        output='screen',
        shell=False,
        name='navigator',
        parameters=[
            {'drone_id': drone_id},
            {'map_frame': map_frame},
            {'local_map_frame': local_map_frame},
            {'navigator.num_drones': num_drones},
            navigator_cfg,
            voxel_map_cfg,
        ],
    )

    # ''' Octomap mapping module '''
    # octomap_mapping_node = Node(
    #     package='octomap_server',
    #     executable='octomap_server_node',
    #     name='octomap_server',
    #     output='log',
    #     shell=False,
    #     parameters=[
    #         {'resolution': 0.1},
    #         {'frame_id': map_frame},
    #         {'sensor_model.max_range': 5.0},
    #     ],
    #     remappings=[
    #         ('cloud_in', cloud_topic),
    #     ],
    # )

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
          {'base_link_frame': base_link_frame},
          traj_server_config
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
          {'map_frame': map_frame},
          {'local_map_frame': local_map_frame},
          {'sensor_frame': camera_frame},
          {'pcd_map.filepath': fake_map_pcd_filepath},
          fake_sensor_config,
        ],
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
        # drone_origin_tf,
        camera_link_tf,
        # Nodes
        fake_sensor,
        navigator_node,
        trajectory_server,
        # octomap_mapping_node,
        # Drone simulation instance
        fake_drone_node,
    ])
