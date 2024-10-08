#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.substitutions import LaunchConfiguration

SCENARIO_NAME = "forest_dense_1"

def generate_launch_description():
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

    px4_dir = os.path.join(
      os.path.expanduser("~"), 'PX4-Autopilot'
    )

    px4_build_dir = os.path.join(
      px4_dir, "build/px4_sitl_default"
    )

    traj_server_config = os.path.join(
      get_package_share_directory('trajectory_server'),
      'config',
      'trajectory_server.yaml'
    )

    """Nodes"""

    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            # Environment variables
            'PX4_SYS_AUTOSTART=4001',
            'PX4_GZ_WORLD=default',
            'PX4_SIM_MODEL=gz_x500',
            'PX4_GZ_STANDALONE=1',
            ['PX4_GZ_MODEL_POSE="', init_x, init_y, '0,0,0', init_yaw, '"'],
            'ROS_DOMAIN_ID=0',
            'PX4_UXRCE_DDS_PORT=8888',
            ['PX4_UXRCE_DDS_NS=d', drone_id],

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

    trajectory_server = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        output='screen',
        shell=True,
        name=['traj_server_', drone_id],
        parameters=[traj_server_config],
    )

    # Publish Transform from world to base_link

    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        init_yaw_launch_arg,
        # Nodes
        px4_sitl,
        trajectory_server,
    ])
