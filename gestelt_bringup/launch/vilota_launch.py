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


    # Vilota depth image bridge
    vilota_bridge_front_left_node = Node(
        package='vilota_bridge',
        executable='vilota_bridge_node',
        output='screen',
        
        name='vilota_bridge_front_left',
        parameters=[
            {'bridge_name': 'front_left'},
            {'camera_frame_id': 'camera_front_left'},
            {'disparity_topic': 'S1/stereo1_l/disparity'},
            {'image_topic': 'S1/stereo1_l'},
            # {'odom_topic': 'S1/vio_odom'},
            {'odom_topic': 'S1/vio_odom_ned'},
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
            {'min_dist': 0.01},
            {'max_dist': 0.6},
            {'pcl_frame_id': 'camera_front_left'},
            {'downsample_leaf_size': 0.2},
            {'minimum_points_per_voxel': 3},
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

    ld.add_action(vilota_bridge_front_left_node)
    ld.add_action(depth2pcl_node)
    ld.add_action(vio_bridge_px4_node)
    
    return ld