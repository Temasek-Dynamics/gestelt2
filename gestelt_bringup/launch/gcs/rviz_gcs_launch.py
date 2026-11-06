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

"""
GCS visualisation and goal input for ground testing.

Starts RVIZ, the world->d{id}_map static transforms defined by a scenario,
and the republisher that turns a single RVIZ goal click into a continuous
goal stream.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scenario_name = LaunchConfiguration('scenario_name')
    drone_ids = LaunchConfiguration('drone_ids')
    global_frame = LaunchConfiguration('global_frame')
    use_republisher = LaunchConfiguration('use_republisher')
    goal_input_topic = LaunchConfiguration('goal_input_topic')
    goal_output_topic = LaunchConfiguration('goal_output_topic')
    republish_rate = LaunchConfiguration('republish_rate')

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'five_drones.rviz'),
        # default_value=os.path.join(bringup_dir, 'rviz', 'single_drone_follow.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_scenario_name_cmd = DeclareLaunchArgument(
        'scenario_name',
        default_value='multi_drone_real_flight',
        description=(
            'Scenario from scenarios.json whose spawns_pos entries define the '
            'world->d{id}_map static transforms'
        ),
    )

    declare_drone_ids_cmd = DeclareLaunchArgument(
        'drone_ids',
        default_value='',
        description=(
            'Comma-separated drone IDs, one per spawns_pos entry, in order '
            "(e.g. '0,1,4' for non-contiguous numbering). Leave empty to use "
            '0-based indices'
        ),
    )

    declare_global_frame_cmd = DeclareLaunchArgument(
        'global_frame',
        default_value='world',
        description='Parent frame of the published static transforms',
    )

    declare_use_republisher_cmd = DeclareLaunchArgument(
        'use_republisher',
        default_value='true',
        description='Whether to republish the latest RVIZ goal at a fixed rate',
    )

    declare_goal_input_topic_cmd = DeclareLaunchArgument(
        'goal_input_topic',
        default_value='/goal_pose',
        description="Topic the RVIZ '2D Goal Pose' tool publishes to",
    )

    declare_goal_output_topic_cmd = DeclareLaunchArgument(
        'goal_output_topic',
        default_value='/point_goal',
        description='Topic the latched goal is republished to',
    )

    declare_republish_rate_cmd = DeclareLaunchArgument(
        'republish_rate',
        default_value='10.0',
        description='Rate (Hz) at which the latest goal is republished',
    )

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Publish world->d{id}_map transforms for every drone in the scenario
    start_scenario_tf_cmd = Node(
        package='gestelt_commander',
        executable='scenario_tf_publisher',
        name='scenario_tf_publisher',
        output='screen',
        parameters=[{
            'scenario_name': ParameterValue(scenario_name, value_type=str),
            'drone_ids': ParameterValue(drone_ids, value_type=str),
            'global_frame': ParameterValue(global_frame, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    # Republish the latest RVIZ goal as a continuous stream
    start_goal_republisher_cmd = Node(
        condition=IfCondition(use_republisher),
        package='gestelt_commander',
        executable='goal_republisher',
        name='goal_republisher',
        output='screen',
        parameters=[{
            'input_topic': ParameterValue(goal_input_topic, value_type=str),
            'output_topic': ParameterValue(goal_output_topic, value_type=str),
            'rate_hz': ParameterValue(republish_rate, value_type=float),
            'use_sim_time': use_sim_time,
        }],
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # A bad scenario aborts everything, rather than leaving RVIZ up with no
    # transform tree
    scenario_tf_exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_scenario_tf_cmd,
            on_exit=EmitEvent(
                event=Shutdown(reason='scenario_tf_publisher exited')),
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_scenario_name_cmd)
    ld.add_action(declare_drone_ids_cmd)
    ld.add_action(declare_global_frame_cmd)
    ld.add_action(declare_use_republisher_cmd)
    ld.add_action(declare_goal_input_topic_cmd)
    ld.add_action(declare_goal_output_topic_cmd)
    ld.add_action(declare_republish_rate_cmd)

    # Add the nodes we need
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_scenario_tf_cmd)
    ld.add_action(start_goal_republisher_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)
    ld.add_action(scenario_tf_exit_event_handler)

    return ld
