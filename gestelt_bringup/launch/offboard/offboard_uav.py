#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace, SetParameter

from nav2_common.launch import RewrittenYaml

SCENARIO_NAME = "empty"

# SCENARIO_NAME = "forest_dense_1"
# SCENARIO_NAME = "forest_sparse_1"

# SCENARIO_NAME = "forward_flight_8"
# SCENARIO_NAME = "forward_flight_16"

# SCENARIO_NAME = "antipodal_swap_4_normal"
# SCENARIO_NAME = "antipodal_swap_4_sparse"
# SCENARIO_NAME = "antipodal_swap_4_dense"
# SCENARIO_NAME = "antipodal_swap_4_empty"

# SCENARIO_NAME = "antipodal_swap_8_normal"
# SCENARIO_NAME = "antipodal_swap_8_sparse"

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

def generateSITLDrone(id, spawn_pos, num_drones):

    sitl_drone_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch',
                'offboard',
                'include',
                'offboard_nodes.py'
            ])
        ]),
        launch_arguments={
            'drone_id': str(id),
            'init_x': str(spawn_pos[0]),
            'init_y': str(spawn_pos[1]),
            'init_yaw': str(spawn_pos[2]),
            'num_drones': str(num_drones),
        }.items()
    )

    px4_pluginlists_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_pluginlists.yaml'
    )

    px4_config_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'px4_config.yaml'
    )

    map_frame = ['d0_origin'] # Fixed
    base_link_frame = ['d0_base_link'] # Dynamic

    '''Mavlink/Mavros'''
    fcu_url =  '/dev/ttyS7:921600'
    tgt_system = 36

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
        {'gcs_url': 'udp://:14556@'},
        {'tgt_system': tgt_system},
        {'tgt_component': 1},
        {'fcu_protocol': 'v2.0'},
        {'startup_px4_usb_quirk': 'true'},
        px4_pluginlists_cfg,
        new_px4_config_cfg,
      ],
      # remappings=[
      #   ('imu/data_raw', ['/mavros/imu/data_raw']),
      #   ('vision_pose/pose', ['/mavros/vision_pose/pose']),
      #   ('vision_pose/pose_cov', ['/mavros/vision_pose/pose_cov']),
      #   ('vision_pose/pose_reset', ['/mavros/vision_pose/pose_reset']),
      #   ('vision_speed/speed_twist', ['/mavros/vision_speed/speed_twist']),
      # ],
    )

    # ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 15, on_off: true}"
    # ros2 run mavros mav cmd long 511 105 3000 0 0 0 0 0
    # ros2 run mavros mav cmd long 511 32 33333 0 0 0 0 0
    fcu_setup_service_calls = ExecuteProcess(
        cmd=['sleep', '10'],
        log_cmd=True,
        on_exit=[
          ExecuteProcess(
              cmd=[[
                FindExecutable(name='ros2'),
                " service call",
                " /mavros/set_stream_rate",
                " mavros_msgs/srv/StreamRate ",
                '"{stream_id: 0, message_rate: 15, on_off: true}"',
              ]],
            shell=True
          ),
          ExecuteProcess(
            cmd=[[
              FindExecutable(name='ros2'),
              " run mavros mav cmd long 511 105 3000 0 0 0 0 0",
            ]],
            shell=True
          ),
          ExecuteProcess(
            cmd=[[
              FindExecutable(name='ros2'),
              " run mavros mav cmd long 511 32 33333 0 0 0 0 0",
            ]],
            shell=True
          ),
        ]
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          sitl_drone_launchfile,
          # Mavlink to ROS bridge
          mavros_node,
          fcu_setup_service_calls,
        ]
    )

def generate_launch_description():
    
    scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
      SCENARIO_NAME
    )

    # Generate nodes of SITL drone instances according to scenario
    offboard_nodes = generateSITLDrone(
            0, scenario.spawns_pos[0], scenario.num_agents)
        
    return LaunchDescription([
        offboard_nodes,
    ])
