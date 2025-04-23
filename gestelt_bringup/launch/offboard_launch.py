#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess, 
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode

SCENARIO_NAME = "single_drone_test"

class Scenario:
    """Scenario class that contains all the attributes of a scenario, used to start the fake_map
    and define the number of drones and their spawn positions
    """
    def __init__(self, filepath, scenario_name):
        with open(filepath) as f:
            json_dict = json.loads(f.read())

        scenario_dict = json_dict.get(scenario_name, None)

        if scenario_dict == None:
            raise Exception("Specified scenario does not exist!")

        self.name = scenario_name
        self.map = scenario_dict.get("map", None)
        self.spawns_pos = scenario_dict.get("spawns_pos", None )
        self.goals_pos = scenario_dict.get("goals_pos", None )
        self.num_agents = scenario_dict.get("num_agents", None )

        self.checks()

    def checks(self):
        if (len(self.spawns_pos) != self.num_agents):
            raise Exception("Number of spawn positions does not match number of agents!")

        if (len(self.goals_pos) != self.num_agents):
            raise Exception("Number of goal positions does not match number of agents!")

        if self.map == None or self.spawns_pos == None or self.goals_pos == None or self.num_agents == None:
            raise Exception("map_name and/or spawns_pos field does not exist!")

def generate_launch_description():
    scenario = Scenario(
        os.path.join(get_package_share_directory('gestelt_commander'), 'scenarios.json'),
        SCENARIO_NAME
    )

    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare parameters
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='d0', description='Top-level namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'single_drone.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # XRCE Agent that will connect to ALL XRCE-DDS clients
    xrce_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600'
        ]],
        name='microxrceagent',
        shell=True
    )

    # Send single test goal
    mission_node = Node(
        package='gestelt_commander',
        # executable='test_take_off_goal',
        executable='test_planning',
        output='screen',
        emulate_tty=False,
        shell=True,
        parameters = [
            {'scenario': scenario.name},
            {'init_delay': 1},
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(xrce_agent)
    ld.add_action(start_rviz_cmd)

    ld.add_action(mission_node)

    drone_id = 0
    ns = "d" + str(drone_id)

    global_frame = "world" # Fixed
    map_frame = ns + "_map"
    base_link_frame = ns + "_base_link"
    camera_frame = ns + "_camera_link"

    ld.add_action(
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'bringup_launch.py')),
                    launch_arguments={
                        'namespace': namespace,
                        'use_namespace': use_namespace,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'autostart': 'True',
                        'use_composition': 'False',
                        'use_respawn': 'False',
                    }.items(),
                ),
                Node( 
                    package = "tf2_ros", 
                    name=ns+'_world_to_map_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = [str(scenario.spawns_pos[drone_id][0]), 
                                str(scenario.spawns_pos[drone_id][1]), 
                                "0", 
                                str(scenario.spawns_pos[drone_id][2]), 
                                "0", "0", 
                                global_frame, map_frame],
                ),
                # Transform from base link to camera frame
                Node(
                    package = "tf2_ros", 
                    name=ns+'_base_link_to_cam_tf',
                    executable = "static_transform_publisher",
                    output="own_log",
                    arguments = ["0.12", "0.03", "-0.242", 
                                    "1", "0", "0", "0",
                                    base_link_frame, camera_frame],
                ),
            ]
        )
    )

    return ld