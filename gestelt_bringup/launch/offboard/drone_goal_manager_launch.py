#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gestelt_commander',
            executable='drone_goal_manager',
            name='drone_goal_manager',
            output='screen',
            parameters=[{
                'goal_tolerance': 0.3,
                'frame_id': 'world',
                'publish_rate': 10.0,

                # Goals: [x1, y1, z1, x2, y2, z2, ...]
                'goals': [
                    0.0, 7.0, 1.0,
                    0.0, 0.15, 1.0,
                    0.0, 7.0, 1.0,
                    0.0, 0.15, 1.0,
                    0.0, 7.0, 1.0,
                    0.0, 0.15, 1.0,
                    0.0, 7.0, 1.0,
                    0.0, 0.15, 1.0,
                    0.0, 7.0, 1.0,
                    0.0, 0.15, 1.0,
                ],
            }]
        )
    ])
