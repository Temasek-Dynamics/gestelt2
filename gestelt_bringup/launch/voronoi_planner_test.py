"""
Example to launch a sensor_combined listener node.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    drone_id_ = int(0)

    ''' Get parameter files '''

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

    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'simulation/fake_maps',
      'forest_single.pcd'
    )

    rviz_cfg = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'config',
      'default.rviz'
    )

    world_to_map_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "world", "map"])

    fake_map = Node(
        package='fake_map',
        executable='fake_map_publisher_node',
        output='screen',
        shell=True,
        name='fake_map_publisher_node',
        parameters=[
            {'fake_map.pcd_filepath': fake_map_pcd_filepath},
            {'fake_map.frame_id': "world"},
            {'fake_map.publishing_frequency': 1.0},
        ],
    )

    navigator_node = Node(
        package='voronoi_planner',
        executable='voronoi_planner_node',
        output='screen',
        shell=True,
        name='voronoi_planner',
        parameters=[
            {'navigator.drone_id': 0},
            navigator_cfg,
            voxel_map_cfg,
        ],
    )

    planner_dbg_node = Node(
        package='gestelt_mission',
        executable='planner_dbg',
        output='screen',
        shell=True,
        name='planner_dbg_node',
    )

    fake_drone_node = Node(
        package='fake_drone',
        executable='fake_drone_node',
        output='screen',
        shell=True,
        name='fake_drone_node',
        parameters = [
            fake_drone_cfg,
            {'fake_drone.drone_id': drone_id_},
            {'fake_drone.init.x': -6.0},
            {'fake_drone.init.y': -4.5},
            {'fake_drone.init.z': 1.0},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=True,
        arguments=['-d' + rviz_cfg]
    )

    return LaunchDescription([
        # Planner
        navigator_node,
        # Debugging nodes
        fake_map,
        fake_drone_node,
        # Mission 
        planner_dbg_node,
        # Tools
        world_to_map_tf,
        rviz_node
    ])
