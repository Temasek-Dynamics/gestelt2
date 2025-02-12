#!/usr/bin/env python

"""
Complete set of nodes required to simulate an agent (without dynamics)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

from nav2_common.launch import RewrittenYaml

import yaml

def generate_launch_description():
    drone_id = '0'

    '''Frames'''
    global_frame = 'map' # Fixed
    map_frame = ['d', drone_id, '_origin'] # Fixed
    base_link_frame = ['d', drone_id, '_base_link'] # Dynamic
    local_map_frame = ['d', drone_id, '_lcl_map'] # Fixed to base_link
    camera_frame = ['d', drone_id, '_camera_link'] # Fixed to base_link

    px4_config_cfg = os.path.join(
        get_package_share_directory('gestelt_bringup'), 'config',
        'px4_config.yaml'
    )

    context = LaunchContext()

    px4_config_param_subs = {}
    px4_config_param_subs.update({'/**/local_position.ros__parameters.frame_id': map_frame})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.send': 'true'})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.frame_id': map_frame})
    px4_config_param_subs.update({'/**/local_position.ros__parameters.tf.child_frame_id': base_link_frame})

    new_px4_config_cfg = RewrittenYaml(
        source_file=px4_config_cfg,
        root_key='',
        param_rewrites=px4_config_param_subs,
        convert_types=True)

    new_yaml = new_px4_config_cfg.perform(context)

    # print ("This is definitely not a dict: " + str(new_px4_config_cfg))

    with open(px4_config_cfg) as file:
        list = yaml.load(file, Loader=yaml.FullLoader)
        print ("Orig: " + str(list['/**/local_position']['ros__parameters']))

    with open(new_yaml) as file:
        list = yaml.load(file, Loader=yaml.FullLoader)
        print ("New: " + str(list['/**/local_position']['ros__parameters']))
        # print ("New: " + str(list['/**/imu']['ros__parameters']['tf.child_frame_id']))

    return LaunchDescription([
    ])
