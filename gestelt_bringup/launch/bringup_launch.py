# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    
)
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')
    # launch_dir = os.path.join(bringup_dir, 'launch')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    remappings = [
        # (['/', namespace, '/tf'], '/tf'), 
        # (['/', namespace, '/tf_static'], '/tf_static'),
        ('occ_map/cloud_in', '/point_cloud/downsample'), 
    ]

    traj_server_remappings = [
        ('/uav_state', ['/', namespace, '/uav_state']), 
    ]

    global_frame = 'world' # Fixed
    map_frame = [namespace, "_map"]
    # base_link_frame = [namespace, "_base_link"]
    # camera_frame = [namespace, "_camera_link"]
    base_link_frame = 'base_link'
    camera_frame = 'camera_link'

    # Create our own temporary YAML files that include substitutions
    nav_param_substitutions = {
        'autostart': autostart,
        'global_frame': global_frame,
        'map_frame': map_frame,
        'camera_frame': camera_frame,
        'base_link_frame': base_link_frame,
        # 'global_occ_map.ros__parameters.global_frame': global_frame,
        # 'global_occ_map.ros__parameters.map_frame': map_frame,
        # 'global_occ_map.ros__parameters.camera_frame': camera_frame,
        # 'global_occ_map.ros__parameters.base_link_frame': base_link_frame,
    }

    nav_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            # root_key=namespace,
            param_rewrites=nav_param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
    ]

    load_all_nodes  = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, 
                            {'node_names': lifecycle_nodes}],
            ),
            Node(
                package='gestelt_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                # respawn=use_respawn,
                # respawn_delay=2.0,
                parameters=[nav_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='gestelt_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                # respawn=use_respawn,
                # respawn_delay=2.0,
                parameters=[nav_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='trajectory_server',
                executable='trajectory_server_node',
                name='trajectory_server',
                output='screen',
                # respawn=use_respawn,
                # respawn_delay=2.0,
                parameters=[
                    {'namespace': namespace},
                    {'map_frame': map_frame},
                    {'base_link_frame': base_link_frame},
                    {'safety.geofence.min_x': -50.0},
                    {'safety.geofence.min_y': -50.0},
                    {'safety.geofence.min_z': -0.5},
                    {'safety.geofence.max_x': 50.0},
                    {'safety.geofence.max_y': 50.0},
                    {'safety.geofence.max_z': 5.0},
                    {'set_offb_ctrl_freq': 10.0},
                    {'pub_state_freq': 40.0},
                    {'state_machine_tick_freq': 30.0},
                    {'pub_ctrl_freq': 30.0},
                    {'publish_map_to_baselink_tf': True},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=traj_server_remappings,
            ),
        ],

    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_all_nodes)

    return ld
