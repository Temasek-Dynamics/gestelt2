import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Common params
    min_dist_arg = DeclareLaunchArgument(
        'min_dist',
        default_value='0.01',
        description='Discard points smaller than min_dist'
    )

    max_dist_arg = DeclareLaunchArgument(
        'max_dist',
        default_value='6.0'
        description='Discard points bigger than max_dist'
    )

    pcl_frame_id_arg = DeclareLaunchArgument(
        'pcl_frame_id',
        default_value='base_link',
        description='Frame to transform PCL to'
    )

    downsample_leaf_size_arg = DeclareLaunchArgument(
        'downsample_leaf_size',
        default_value='0.2',
        description='Output voxel size'
    )

    minimum_points_per_voxel_arg = DeclareLaunchArgument(
        'minimum_points_per_voxel',
        default_value='3',
        description='Minimum number of points to count as a voxel in output'
    )

    min_dist = LaunchConfiguration('min_dist')
    max_dist = LaunchConfiguration('max_dist')
    pcl_frame_id = LaunchConfiguration('pcl_frame_id_arg')
    downsample_leaf_size = LaunchConfiguration('downsample_leaf_size')
    minimum_points_per_voxel = LaunchConfiguration('minimum_points_per_voxel')

    return launch.LaunchDescription([
        # Launch arguments
        min_dist_arg,
        max_dist_arg,
        pcl_frame_id_arg,
        downsample_leaf_size_arg,
        minimum_points_per_voxel_arg,

        # Back left
        launch_ros.actions.Node(
            package='depth2pcl',
            executable='depth2pcl_node',
            output='screen',
            
            # Change below for new node
            name='depth2pcl_front_left',
            remappings=[
                ('/depth/rect', '/front_left/depth/rect'),
                ('/depth/camera_info', '/front_left/depth/camera_info'),
            ],

            parameters=[
                {'min_dist': LaunchConfiguration('min_dist')},
                {'max_dist': LaunchConfiguration('max_dist')},
                {'pcl_frame_id': LaunchConfiguration('pcl_frame_id')},
                {'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size')},
                {'minimum_points_per_voxel': LaunchConfiguration('minimum_points_per_voxel')},
            ]
        ),

        # Back right
        launch_ros.actions.Node(
            package='depth2pcl',
            executable='depth2pcl_node',
            output='screen',
            
            # Change below for new node
            name='depth2pcl_front_right',
            remappings=[
                ('/depth/rect', '/front_right/depth/rect'),
                ('/depth/camera_info', '/front_right/depth/camera_info'),
            ],

            parameters=[
                {'min_dist': LaunchConfiguration('min_dist')},
                {'max_dist': LaunchConfiguration('max_dist')},
                {'pcl_frame_id': LaunchConfiguration('pcl_frame_id')},
                {'downsample_leaf_size': LaunchConfiguration('downsample_leaf_size')},
                {'minimum_points_per_voxel': LaunchConfiguration('minimum_points_per_voxel')},
            ]
        ),
  ])