import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pcl_topics_arg = DeclareLaunchArgument(
        'pcl_topics',
        default_value = "['/front_left/point_cloud/full','/front_right/point_cloud/full']",
        description='Topics to concatenate'
    )

    concat_pcl_topic_arg = DeclareLaunchArgument(
        'concat_pcl_topic',
        default_value = '/point_cloud/concat',
        description='Concatenate output topic'
    )

    concat_pub_interval_arg = DeclareLaunchArgument(
        'concat_pub_interval',
        default_value = '62', # in ms, ~16 hz
        description='Concatenate output topic publish interval'
    )

    return launch.LaunchDescription([
        # Launch arguments
        pcl_topics_arg,
        concat_pcl_topic_arg,
        concat_pub_interval_arg,

        # Front left
        launch_ros.actions.Node(
            package='depth2pcl',
            executable='concat_pcl_node',
            output='screen',
            # prefix=['gdbserver localhost:3000'],

            name='concat_pcl_node',

            parameters=[
                {'pcl_topics': LaunchConfiguration('pcl_topics')},
                {'concat_pcl_topic': LaunchConfiguration('concat_pcl_topic')},
                {'concat_pub_interval': LaunchConfiguration('concat_pub_interval')},
            ]
        )
    ])