"""
Example to launch a sensor_combined listener node.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    map_filename = "map_test"
    map_filepath = os.path.join(
      os.path.expanduser("~"), 'gestelt_ws/src/Gestelt2/gestelt_bringup/pcd_maps',
    )

    ''' Get parameter files '''
    pcd_map_gen_cfg = os.path.join(
      get_package_share_directory('pcd_map_generator'), 'config',
      'pcd_map_gen.yaml'
    )

    gen_pcd_map_node = Node(
        package='pcd_map_generator',
        executable='pcd_map_generator',
        output='screen',
        shell=False,
        name='pcd_map_generator',
        parameters=[
            {'map.filename': map_filename},
            {'map.filepath': map_filepath},
            pcd_map_gen_cfg,
        ],
    )

    return LaunchDescription([
        gen_pcd_map_node,
    ])
