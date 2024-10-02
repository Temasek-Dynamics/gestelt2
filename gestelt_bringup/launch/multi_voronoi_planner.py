import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushROSNamespace

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generateFakeDrone(id, x, y, z):
    voronoi_planner_launchfile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gestelt_bringup'),
                'launch',
                'voronoi_planner.py'
            ])
        ]),
        launch_arguments={
            'drone_id': id,
            'init_x': str(x),
            'init_y': str(y),
            'init_z': str(z),
        }.items()
    )

    return GroupAction(
      actions=[
          PushROSNamespace('d' + str(id)),
          voronoi_planner_launchfile,
        ]
    )

def generate_launch_description():


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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        shell=True,
        arguments=['-d' + rviz_cfg]
    )

    # Mission node
    planner_dbg_node = Node(
        package='gestelt_mission',
        executable='planner_dbg',
        output='screen',
        shell=True,
        name='planner_dbg_node',
    )

    d0 = generateFakeDrone(0, -6.0, -4.5, 1.0)
    d1 = generateFakeDrone(1, 6.0, -4.5, 1.0)
    d2 = generateFakeDrone(2, 6.0, 4.5, 1.0)

    return LaunchDescription([
        # Central nodes
        fake_map,
        world_to_map_tf,
        rviz_node,

        # Mission nodes
        planner_dbg_node,

        # Fake drone unit
        # voronoi_planner_launchfile
        d0,
        d1,
        d2,
    ])