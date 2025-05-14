import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess, 
    DeclareLaunchArgument
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, SetParameter

def generate_launch_description():

    # Common params
    min_dist_arg = DeclareLaunchArgument(
        'min_dist',
        default_value='0.01',
        description='Discard points smaller than min_dist'
    )

    max_dist_arg = DeclareLaunchArgument(
        'max_dist',
        default_value='6.0',
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
    pcl_frame_id = LaunchConfiguration('pcl_frame_id')
    downsample_leaf_size = LaunchConfiguration('downsample_leaf_size')
    minimum_points_per_voxel = LaunchConfiguration('minimum_points_per_voxel')


    # Vilota depth image bridge
    vilota_bridge_front_left_node = Node(
        package='vilota_bridge',
        executable='vilota_bridge_node',
        output='screen',
        
        name='vilota_bridge_front_left',
        parameters=[
            {'bridge_name': 'front_left'},
            {'disparity_topic': 'S1/stereo1_l/disparity'},
            {'image_topic': 'S1/stereo1_l'},
            {'odom_topic': 'S1/vio_odom'}
        ]
    )

    # Depth map to PCL conversion
    depth2pcl_node = Node(
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
            {'min_dist': min_dist},
            {'max_dist': max_dist},
            {'pcl_frame_id': pcl_frame_id},
            {'downsample_leaf_size': downsample_leaf_size},
            {'minimum_points_per_voxel': minimum_points_per_voxel},
        ]
    )

    # VIO from vilota
    vio_bridge_px4_node = Node(
        package='vision',
        executable='vio_bridge_px4',
        output='screen',
        # Change below for new node
        name='vio_bridge_px4',
    )

    ld = LaunchDescription()

    ld.add_action(min_dist_arg)
    ld.add_action(max_dist_arg)
    ld.add_action(pcl_frame_id_arg)
    ld.add_action(downsample_leaf_size_arg)
    ld.add_action(minimum_points_per_voxel_arg)

    ld.add_action(vilota_bridge_front_left_node)
    ld.add_action(depth2pcl_node)
    ld.add_action(vio_bridge_px4_node)
    
    return ld