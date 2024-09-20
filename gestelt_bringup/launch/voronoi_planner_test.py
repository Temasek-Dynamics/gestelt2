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

    fake_map_pcd_filepath = os.path.join(
      get_package_share_directory('gestelt_bringup'), 'simulation/fake_maps',
      'forest_single.pcd'
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        fake_map,
        navigator_node,
        rviz_node
    ])
