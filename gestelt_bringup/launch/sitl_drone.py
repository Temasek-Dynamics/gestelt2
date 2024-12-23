#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
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
    map_frame = ["d", drone_id, "_origin"]
    local_map_frame = ["d", drone_id, "_lcl_map"]
    base_link_frame = ["d", drone_id, "_base_link"]
    camera_frame = ["d", drone_id, "_camera_link"]

    ''' Get directories '''

    px4_dir = os.path.join(
      os.path.expanduser("~"), 'PX4-Autopilot'
    )

    px4_build_dir = os.path.join(
      px4_dir, "build/px4_sitl_default"
    )

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
                      arguments = [init_x, init_y, "0", "0", "0", "0", 
                              "world", map_frame])

    # drone base_link to sensor fixed TF
    camera_link_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output="log",
                      arguments = ["0", "0", "0", "0", "0", "0", 
                              base_link_frame, camera_frame])

    ''' PX4 SITL '''
    px4_sitl = ExecuteProcess(
        cmd=[
            # Environment variables
            'PX4_SYS_AUTOSTART=4001',
            'PX4_GZ_WORLD=default',
            'PX4_SIM_MODEL=gz_x500',
            'PX4_GZ_STANDALONE=1',
            ['PX4_GZ_MODEL_POSE="', init_x, ',', init_y, ',0,0,0,', init_yaw, '"'],
            # ['PX4_GZ_MODEL_POSE="0,0,0,0,0,0"'],
            'ROS_DOMAIN_ID=0',
            'PX4_UXRCE_DDS_PORT=8888',
            ['PX4_UXRCE_DDS_NS=d', drone_id],
            # PX4 Executable
            os.path.join(px4_build_dir, 'bin/px4'),      # PX4 executable
            os.path.join(px4_build_dir, 'etc'),          # ?
            '-w', os.path.join(px4_build_dir, 'rootfs'), # Working directory
            '-s', os.path.join(px4_build_dir, 'etc/init.d-posix/rcS'), # Startup file
            '-i', drone_id, # Instance number
            '-d' # Run as daemon (not interactive terminal)
        ],
        name=['px4_sitl_', drone_id],
        shell=True
    )

    ''' Navigator: Planner module '''
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

    mavlink_udp_port =  str(14560 + int([drone_id][0]))
    mavlink_tcp_port =  str(4560 + int([drone_id][0]))
    fcu_addr =  str(14540 + int([drone_id][0]))
    fcu_port =  str(14580 + int([drone_id][0]))
    fcu_url = "udp://:" + fcu_addr + "@localhost:" + fcu_port


    mavros_node = Node(
      package='mavros',
      executable='mavros_node',
      output='screen',
      shell=False,
      name=['mavros_node_', drone_id],

      parameters=[
        {'fcu_url': fcu_url},
        # {'gcs_url': },
        {'tgt_system': str(1 + int([drone_id][0]))},
        {'tgt_component': "1"},
        {'fcu_protocol': "v2.0"},
        px4_pluginlists_cfg,
        px4_config_cfg,
        {'local_position.frame_id', map_frame },
        {'local_position.tf.send', True },
        {'local_position.tf.frame_id', map_frame },
        {'local_position.tf.child_frame_id', base_link_frame },
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
        drone_origin_tf,
        camera_link_tf,
        # Nodes
        fake_sensor,
        navigator_node,
        trajectory_server,
        # Mavlink to ROS bridge
        mavros_node,
        # Drone simulation instance
        px4_sitl,
    ])
