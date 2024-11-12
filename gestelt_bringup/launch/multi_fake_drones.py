#!/usr/bin/env python

"""
Complete launch file to simulate a multi-agent navigation scenario
"""

import os
from datetime import datetime
import json

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# SCENARIO_NAME = "forest_dense_1"

# SCENARIO_NAME = "antipodal_swap_4_normal"
# SCENARIO_NAME = "antipodal_swap_4_sparse"
# SCENARIO_NAME = "antipodal_swap_4_dense"
# SCENARIO_NAME = "antipodal_swap_4_empty"

SCENARIO_NAME = "antipodal_swap_8_normal"
# SCENARIO_NAME = "antipodal_swap_8_sparse"
# SCENARIO_NAME = "map_test"

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

def generateFakeDrone(id, spawn_pos, pcd_filepath, num_drones):

    fake_drone_complete_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch',
                'fake_drone.py'
            ])
        ]),
        launch_arguments={
            'drone_id': str(id),
            'init_x': str(spawn_pos[0]),
            'init_y': str(spawn_pos[1]),
            'init_yaw': str(spawn_pos[2]),
            'fake_map_pcd_filepath': str(pcd_filepath),
            'num_drones': str(num_drones),
        }.items()
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          fake_drone_complete_launchfile,
        ]
    )

def generate_launch_description():

    scenario = Scenario(os.path.join(get_package_share_directory('gestelt_mission'), 'scenarios.json'),
        SCENARIO_NAME
    )

    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'pcd_maps',
      scenario.map + '.pcd'
    )

    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    # World to map transformation
    world_to_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       output = "log",
                       arguments = ["0", "0", "0", "0", "0", "0", "world", "map"])

    # Fake map
    fake_map = Node(
        package='fake_map',
        executable='fake_map_publisher_node',
        output='log',
        shell=False,
        name='fake_map_publisher_node',
        parameters=[
            {'fake_map.pcd_filepath': fake_map_pcd_filepath},
            {'fake_map.frame_id': "world"},
            {'fake_map.publishing_frequency': 1.0},
        ],
    )


    # Mission node: Sends goals to agents
    swarm_collision_checker = Node(
        package='swarm_collision_checker',
        executable='swarm_collision_checker_node',
        output='screen',
        shell=False,
        name='swarm_collision_checker_node',
        parameters = [
            {'num_drones': scenario.num_agents},
            {'odom_topic': "odom"},
            {'collision_check.frequency': 20.0},
            {'collision_check.warn_radius': 0.225},
            {'collision_check.fatal_radius': 0.14},
        ]
    )

    # RVIZ Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=False,
        arguments=['-d' + rviz_cfg]
    )

    # Mission node: Sends goals to agents
    mission_node = Node(
        package='gestelt_mission',
        executable='mission',
        output='screen',
        shell=False,
        name='mission_node',
        parameters = [
            {'scenario': scenario.name},
            {'init_delay': 2},
        ]
    )

    # Generate nodes of fake drone simulation instances according to scenario
    fake_drone_nodes = []
    for id in range(scenario.num_agents):
        fake_drone_nodes.append(generateFakeDrone(
            id, scenario.spawns_pos[id], fake_map_pcd_filepath, scenario.num_agents))

    # ROSBag 
    bag_topics = []
    for id in range(scenario.num_agents):
        prefix = "/d" + str(id) + "/"
        bag_topics.append(prefix + "odom")
        bag_topics.append(prefix + "static_collisions")
        # Subscription to paths
        bag_topics.append(prefix + "fe_plan/viz")
        bag_topics.append(prefix + "minco_traj_viz")
        # Subscription to 3d occupancy voxel map
        bag_topics.append(prefix + "occ_map")
        # Subscription to maps
        bag_topics.append(prefix + "voro_planning")
        bag_topics.append(prefix + "occ_map_100")
        bag_topics.append(prefix + "occ_map_150")
        bag_topics.append(prefix + "occ_map_200")
        bag_topics.append(prefix + "voro_map_100")
        bag_topics.append(prefix + "voro_map_150")
        bag_topics.append(prefix + "voro_map_200")
    bag_topics.append("/swarm_collision_checker/collisions")
    bag_topics.append("/rosout")
    bag_topics.append("/tf")
    bag_topics.append("/tf_static")
    bag_topics.append("/fake_map")

    bag_file = os.path.join(
        os.path.expanduser("~"), 'bag_files',
        'bag_' + datetime.now().strftime("%d%m%Y_%H_%M_%S"),
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             bag_file,
             *bag_topics],
        output='log'
    )

    # rosbag_record = Node(
    #     package='rosbag2_transport',
    #     executable='recorder',
    #     name='recorder',
    #     output="screen",
    #     parameters=["/path/to/params.yaml"],
    # )

    return LaunchDescription([
        # Central nodes
        world_to_map_tf,
        fake_map,
        swarm_collision_checker,
        rosbag_record,
        # Visualization
        rviz_node,
        # mission
        mission_node,
        # Simulation instances
        *fake_drone_nodes
    ])