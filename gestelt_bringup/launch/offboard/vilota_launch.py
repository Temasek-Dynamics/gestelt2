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


drone_id = 1
ns  = "d"+ str(drone_id)

def generate_launch_description():

    # namespace = LaunchConfiguration('namespace', default='px4_offboard')

    # Vilota depth image bridge
    vilota_bridge_front_left_node = Node(
        package='vilota_bridge',
        executable='vilota_bridge_node',
        output='screen',
        namespace= ns,        
        name='vilota_bridge_front_left',
        parameters=[
            {'bridge_name': ns+'/front_left'},
            {'map_frame_id': ns+'_'+'map'},
            {'camera_frame_id': ns+'_'+'camera_front_left'},
            {'disparity_topic': 'S1/stereo1_l/disparity'},
            {'image_topic': 'S1/stereo1_l'},
            # {'odom_topic': 'S1/vio_odom'},
            {'odom_topic': 'S1/vio_odom_ned'},
        ]
    )

    vilota_bridge_front_right_node = Node(
        package='vilota_bridge',
        executable='vilota_bridge_node',
        output='screen',
        namespace=ns,       
        name='vilota_bridge_front_right',
        parameters=[
            {'bridge_name': ns+'/front_right'},
            {'map_frame_id': ns+'_'+'map'},
            {'camera_frame_id': ns+'_'+'camera_front_right'},
            {'disparity_topic': 'S1/stereo2_r/disparity'},
            {'image_topic': 'S1/stereo2_r'},
            # {'odom_topic': 'S1/vio_odom'},
            {'odom_topic': 'S1/vio_odom_ned'},
        ]
    )

    # Depth map to PCL conversion
    depth2pcl_front_left_node = Node(
        package='depth2pcl',
        executable='depth2pcl_node',
        output='screen',
        namespace=ns,
        # Change below for new node
        name='depth2pcl_front_left',
        remappings=[
            ('/depth/rect', '/'+ns+'/front_left/depth/rect'),
            ('/depth/camera_info', '/'+ns+'/front_left/depth/camera_info'),
            ('/point_cloud/downsample', '/'+ns+'/front_left/point_cloud/downsample'),
            ('/point_cloud/full', '/'+ns+'/front_left/point_cloud/full')
        ],
        parameters=[
            {'min_dist': 0.01},
            {'max_dist': 6.0},
            # {'pcl_frame_id': 'camera_front_left'},
            {'pcl_frame_id': ns+'_'+'camera_link'},
            # {'pcl_frame_id': 'base_link'},
            {'downsample_leaf_size': 0.1},
            {'minimum_points_per_voxel': 3},
        ]
    )

    depth2pcl_front_right_node = Node(
        package='depth2pcl',
        executable='depth2pcl_node',
        output='screen',
        namespace=ns,
        # Change below for new node
        name='depth2pcl_front_right',
        remappings=[
            ('/depth/rect', '/'+ns+'/front_right/depth/rect'),
            ('/depth/camera_info', '/'+ns+'/front_right/depth/camera_info'),
            ('/point_cloud/downsample', '/'+ns+'/front_right/point_cloud/downsample'),
            ('/point_cloud/full', '/'+ns+'/front_right/point_cloud/full')
        ],
        parameters=[
            {'min_dist': 0.01},
            {'max_dist': 6.0},
            {'pcl_frame_id': ns+'_'+'camera_link'},
            # {'pcl_frame_id': 'camera_front_right'},
            # {'pcl_frame_id': 'base_link'},
            {'downsample_leaf_size': 0.1},
            {'minimum_points_per_voxel': 3},
        ]
    )

    # Concat PCL topics
    pcl_topics_arg = DeclareLaunchArgument(
        'pcl_topics',
        default_value = "['/d1/front_left/point_cloud/downsample','/d1/front_right/point_cloud/downsample']",
        description='Topics to concatenate'
    )

    concat_pcl_topic_arg = DeclareLaunchArgument(
        'concat_pcl_topic',
        default_value = '/d1/point_cloud/concat',
        description='Concatenate output topic'
    )

    concat_pub_interval_arg = DeclareLaunchArgument(
        'concat_pub_interval',
        default_value = '62', # in ms, ~16 hz
        description='Concatenate output topic publish interval'
    )

    concat_pcl_node = Node(
        package='depth2pcl',
        executable='concat_pcl_node',
        namespace='d1',
        output='screen',
        # prefix=['gdbserver localhost:3000'],

        name='concat_pcl_node',

        parameters=[
            {'pcl_topics': LaunchConfiguration('pcl_topics')},
            {'concat_pcl_topic': LaunchConfiguration('concat_pcl_topic')},
            {'concat_pub_interval': LaunchConfiguration('concat_pub_interval')},
        ]
    )

    # VIO from vilota
    vio_bridge_px4_node = Node(
        package='vision',
        namespace='d1',
        executable='vio_bridge_px4',
        output='screen',
        # Change below for new node
        name='vio_bridge_px4',
    )

    ld = LaunchDescription()

    ld.add_action(vilota_bridge_front_left_node)
    ld.add_action(vilota_bridge_front_right_node)

    ld.add_action(depth2pcl_front_left_node)
    ld.add_action(depth2pcl_front_right_node)

    ld.add_action(pcl_topics_arg)
    ld.add_action(concat_pcl_topic_arg)
    ld.add_action(concat_pub_interval_arg)
    ld.add_action(concat_pcl_node)

    ld.add_action(vio_bridge_px4_node)
    
    return ld
