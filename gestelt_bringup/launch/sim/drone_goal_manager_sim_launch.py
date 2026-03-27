#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gestelt_commander',
            executable='drone_goal_manager_sim',
            name='drone_goal_manager_sim',
            output='screen',
            parameters=[{
                'drone_names': ['d0', 'd1', 'd2'],
                'goal_tolerance': 0.3,
                'frame_id': 'world', # ignore this. Have something in here as a placeholder
                'publish_rate': 10.0,

                # d0 goals: [x1, y1, z1, x2, y2, z2, ...]
                'd0_goals': [
                    4.0, -4.0, 1.0,
                    0.0, 0.15, 1.0,
                    4.0, -4.0, 1.0,
                    0.0, 0.15, 1.0,
                    # 4.0, -4.0, 1.0,
                    # 0.0, 0.15, 1.0,
                    # 4.0, -4.0, 1.0,
                    # 0.0, 0.15, 1.0,
                    # 4.0, -4.0, 1.0,
                    # 0.0, 0.15, 1.0,
                ],

                # d1 goals
                'd1_goals': [
                    -4.0, -4.0, 1.0,
                    0.0, -0.15, 1.0,
                    -4.0, -4.0, 1.0,
                    0.0, -0.15, 1.0,
                    -4.0, -4.0, 1.0,
                    # 0.0, -0.15, 1.0,
                    # -4.0, -4.0, 1.0,
                    # 0.0, -0.15, 1.0,
                    # -4.0, -4.0, 1.0,
                    # 0.0, -0.15, 1.0,
                ],

                # d2 goals
                'd2_goals': [
                    4.0, 4.0, 1.0,
                    0.15, 0.0, 1.0,
                    4.0, 4.0, 1.0,
                    0.15, 0.0, 1.0,
                    4.0, 4.0, 1.0,
                    # 0.15, 0.0, 1.0,
                    # 4.0, 4.0, 1.0,
                    # 0.15, 0.0, 1.0,
                    # 4.0, 4.0, 1.0,
                    # 0.15, 0.0, 1.0,
                ],
            }]
        )
    ])