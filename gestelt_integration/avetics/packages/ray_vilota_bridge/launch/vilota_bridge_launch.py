import launch
import launch_ros.actions


# from ament_index_python.packages import get_package_share_directory

# print(get_package_share_directory('vilota_bridge'))

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vilota_bridge',
            executable='vilota_bridge_node',
            name='vilota_bridge',
            output='screen'),
  ])