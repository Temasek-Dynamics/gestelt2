"""
Example to launch a sensor_combined listener node.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ''' Get launch argument values '''
    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')

    drone_id_launch_arg = DeclareLaunchArgument(
      'drone_id',
      default_value='0'
    )

    init_x_launch_arg = DeclareLaunchArgument(
      'init_x',
      default_value='0.0'
    )
    init_y_launch_arg = DeclareLaunchArgument(
      'init_y',
      default_value='0.0'
    )
    init_z_launch_arg = DeclareLaunchArgument(
      'init_z',
      default_value='0.0'
    )

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

    navigator_node = Node(
        package='voronoi_planner',
        executable='voronoi_planner_node',
        output='screen',
        shell=True,
        name='voronoi_planner',
        parameters=[
            {'navigator.drone_id': drone_id},
            navigator_cfg,
            voxel_map_cfg,
        ],
    )

    fake_drone_node = Node(
        package='fake_drone',
        executable='fake_drone_node',
        output='screen',
        shell=True,
        name='fake_drone_node',
        parameters = [
            fake_drone_cfg,
            {'fake_drone.drone_id': drone_id},
            {'fake_drone.init.x': init_x},
            {'fake_drone.init.y': init_y},
            {'fake_drone.init.z': init_z},
        ]
    )


    return LaunchDescription([
        # Launch arguments
        drone_id_launch_arg,
        init_x_launch_arg,
        init_y_launch_arg,
        init_z_launch_arg,

        # Planner
        navigator_node,
        # Fake drone 
        fake_drone_node,

    ])
