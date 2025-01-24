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

    drone_id_launch_arg = DeclareLaunchArgument(
      'drone_id',
      default_value='0'
    )

    '''Frames'''
    global_frame = 'map' # Fixed
    map_frame = ['d', drone_id, '_origin'] # Fixed
    base_link_frame = ['d', drone_id, '_base_link'] # Dynamic
    local_map_frame = ['d', drone_id, '_lcl_map'] # Fixed to base_link
    camera_frame = ['d', drone_id, '_camera_link'] # Fixed to base_link

    px4_pluginlists_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_pluginlists.yaml'
    )

    px4_config_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_config.yaml'
    )


    '''Mavlink/Mavros'''
    # fcu_addr =  PythonExpression(['14540 +', drone_id])
    # # fcu_port =  PythonExpression(['14580 +', drone_id])
    # fcu_port =  PythonExpression(['14557 +', drone_id]) # Used for SITL
    # # fcu_url = ["udp://:", fcu_addr, "@localhost:", fcu_port] # udp://:14540@localhost:14557
    # tgt_system = PythonExpression(['1 +', drone_id])

    mavros_node = Node(
      package='mavros',
      executable='mavros_node',
      output='screen',
      shell=False,
      namespace='mavros',
      parameters=[
        {'fcu_url': '/dev/ttyS7:921600'},
        {'gcs_url': 'udp://:14556@'},
        {'tgt_system': 36},
        {'tgt_component': 1},
        {'fcu_protocol': 'v2.0'},
        {'startup_px4_usb_quirk': 'true'},
        px4_pluginlists_cfg,
        px4_config_cfg,
        {'local_position.frame_id': map_frame},
        {'local_position.tf.send': 'true'},
        {'local_position.tf.frame_id': map_frame},
        {'local_position.tf.child_frame_id': base_link_frame},
      ],
      remappings=[
        ('local_position/odom', ['/odom']),
      ],
    )



    # ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 15, on_off: true}"
    # ros2 run mavros mav cmd long 511 105 3000 0 0 0 0 0
    # ros2 run mavros mav cmd long 511 32 33333 0 0 0 0 0
    fcu_setup_service_calls = ExecuteProcess(
        cmd=['sleep', '8'],
        log_cmd=True,
        on_exit=[
          ExecuteProcess(
              cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/mavros/set_stream_rate ",
                "mavros_msgs/srv/StreamRate ",
                '"{stream_id: 0, message_rate: 15, on_off: true}"',
            ]],
            shell=True
          ),
          LogInfo(msg='Setting IMU rate to 200'),
          ExecuteProcess(
            cmd=[[
              FindExecutable(name='ros2'),
              " run mavros mav cmd long 511 105 3000 0 0 0 0 0",
            ]],
            shell=True
          ),
          LogInfo(msg='Setting mavros mav cmd 1'),
          ExecuteProcess(
            cmd=[[
              FindExecutable(name='ros2'),
              " run mavros mav cmd long 511 32 33333 0 0 0 0 0",
            ]],
            shell=True
          ),
          LogInfo(msg='Setting mavros mav cmd 2'),
        ]
    )

    return LaunchDescription([
        drone_id_launch_arg,
        mavros_node,
        fcu_setup_service_calls,
        # ros2_broker,
    ])
