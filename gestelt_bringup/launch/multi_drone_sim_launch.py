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
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode

# from ros_gz_bridge.actions import RosGzBridge

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

    px4_gz = os.path.join(
      os.path.expanduser("~"), 'PX4-Autopilot', 
      "Tools/simulation/gz/simulation-gazebo"
    )

    px4_build_dir = os.path.join(
      os.path.expanduser("~"), 'PX4-Autopilot', 'build/px4_sitl_default'
    )

    # Get the launch directory
    bringup_dir = get_package_share_directory('gestelt_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ros_gz_bridge_params = os.path.join(
        bringup_dir, 'params', 'ros_gz_bridge.yaml')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

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
        default_value='true',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    # start_rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
    #     launch_arguments={
    #         'namespace': namespace,
    #         # 'use_namespace': use_namespace,
    #         'use_sim_time': use_sim_time,
    #         'rviz_config': rviz_config_file,
    #     }.items(),
    # )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # Run python script provided by PX4-Autopilot repo to ease
    # running PX4 custom worlds and models in Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'python3', px4_gz,
            '--world', 'default',
            # '--world', 'default_w_obs',
            # '--interactive',
            # '--headless',
        ],
        name='gazebo',
        shell=False
    )

    # XRCE Agent that will connect to ALL XRCE-DDS clients
    xrce_agent = ExecuteProcess(
        cmd=[[
            'MicroXRCEAgent udp4 -p 8888 -v'
        ]],
        name='microxrceagent',
        shell=True
    )

    ros_gz_bridge_action = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        # arguments=[
        #     "/depth_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        #     "/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        #     "/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        #     "/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        #     # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
        #     "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        # ],
        parameters=[
            {'config_file' : ros_gz_bridge_params},
        ],
        output="screen",
    )

    # ros_gz_bridge_action = RosGzBridge(
    #     bridge_name='ros_gz_bridge',
    #     config_file=ros_gz_bridge_params,
    #     container_name='ros_gz_container',
    #     create_own_container=False,
    #     namespace=namespace,
    #     use_composition=False,
    #     use_respawn=False,
    #     log_level='info',
    #     bridge_params='',
    # )

    # If during startup, gazebo detects that there is another publisher on /clock, 
    # it will only create the fully qualified /world/<worldname>/clock topic. 
    # Gazebo would be the only /clock publisher, the sole source of clock information.
    # Therefore, we should create a unidirectional bridge
    # ros_gz_bridge_unidir_clock = ExecuteProcess(
    #     cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
    #          '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    #     output='log'
    # )

    # Send single test goal
    mission_node = Node(
        package='gestelt_commander',
        executable='test_take_off_goal',
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

    ld.add_action(start_gazebo_cmd)
    ld.add_action(ros_gz_bridge_action)
    # ld.add_action(ros_gz_bridge_unidir_clock)
    ld.add_action(xrce_agent)
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)

    ld.add_action(mission_node)

    # Generate nodes of SITL drone instances according to scenario
    for drone_id in range(scenario.num_agents):

        ns = "d" + str(drone_id)

        global_frame = "world" # Fixed
        map_frame = ns + "_map"
        base_link_frame = ns + "_base_link"
        # camera_frame = ns + "_camera_link"
        camera_frame = "x500_depth_0/OakD-Lite/base_link/StereoOV7251"

        ld.add_action(
            GroupAction(
                actions=[
                    # Transform from global to map frame
                    Node( 
                        package = "tf2_ros", 
                        executable = "static_transform_publisher",
                        output = "log",
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
                        executable = "static_transform_publisher",
                        output="log",
                        arguments = ["0.12", "0.03", "-0.242", 
                                     "1", "0", "0", "0",
                                     base_link_frame, camera_frame],
                    ),
                ]
            )
        )


        ld.add_action(
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
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
                    ExecuteProcess(
                        name=['px4_sitl_', str(drone_id)],
                        cmd=[
                            # Environment variables
                            'PX4_SYS_AUTOSTART=4001',
                            'PX4_GZ_WORLD=default',
                            'PX4_SIM_MODEL=gz_x500_depth',
                            'PX4_GZ_STANDALONE=1',
                            ['PX4_GZ_MODEL_POSE="', 
                                str(scenario.spawns_pos[drone_id][0]), ',', 
                                str(scenario.spawns_pos[drone_id][1]), 
                                ',0,0,0,', str(scenario.spawns_pos[drone_id][2]), '"'],
                            # ['PX4_GZ_MODEL_POSE="0,0,0,0,0,0"'],
                            'ROS_DOMAIN_ID=0',
                            'PX4_UXRCE_DDS_PORT=8888',
                            ['PX4_UXRCE_DDS_NS=', ns],
                            # PX4 Executable
                            os.path.join(px4_build_dir, 'bin/px4'),      # PX4 executable
                            os.path.join(px4_build_dir, 'etc'),          # ?
                            '-w', os.path.join(px4_build_dir, 'rootfs'), # Working directory
                            '-s', os.path.join(px4_build_dir, 'etc/init.d-posix/rcS'), # Startup file
                            '-i', str(drone_id), # Instance number
                            '-d' # Run as daemon (not interactive terminal)
                        ],
                        shell=True
                    )
                ]
            )
        )

    return ld