#!/usr/bin/env python
"""
Complete set of nodes for trajectory server
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess, 
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer, SetParameter
from launch_ros.descriptions import ComposableNode

NUM_AGENTS = 2

def generate_launch_description():
    # ns = "/d0"

    bag_topics = []

    for ns_index in range(NUM_AGENTS):
    # ROSBag 
        sub_bag_topics = [
            # Pose
            f"/d{str(ns_index)}/odom",
            f"/d{str(ns_index)}/initialpose",

            # # Mapping
            f"/d{str(ns_index)}/local_map/bounds",
            f"/d{str(ns_index)}/local_occ_map/occ_map",
            f"/d{str(ns_index)}/local_occ_map/transition_event",
            f"/d{str(ns_index)}/occ_map",
            f"/d{str(ns_index)}/global_occ_map/occ_map",
            f"/d{str(ns_index)}/global_occ_map/transition_event",
            f"/d{str(ns_index)}/goal",
            f"/d{str(ns_index)}/goals",

            # # Global Plan
            f"/d{str(ns_index)}/planner_server/transition_event",
            f"/d{str(ns_index)}/point_goal",

            # # Controller output
            f"/d{str(ns_index)}/sfc",
            f"/d{str(ns_index)}/received_global_plan",
            f"/d{str(ns_index)}/mpc_ref_path",
            f"/d{str(ns_index)}/mpc_curr_traj",
            f"/d{str(ns_index)}/mpc_traj",
            f"/d{str(ns_index)}/intmd_cmd",
            f"/d{str(ns_index)}/plan",

            f"/d{str(ns_index)}/controller_server/transition_event"

            # # Transformations
            # "/tf",
            # "/tf_static",

            # # User commands
            f"/d{str(ns_index)}/uav_command",
            # "/global_uav_command",

            # PX4
            f"/d{str(ns_index)}/fmu/in/actuator_motors",
            f"/d{str(ns_index)}/fmu/in/actuator_servos",
            f"/d{str(ns_index)}/fmu/in/arming_check_reply",
            f"/d{str(ns_index)}/fmu/in/aux_global_position",
            f"/d{str(ns_index)}/fmu/in/config_control_setpoints",
            f"/d{str(ns_index)}/fmu/in/config_overrides_request",
            f"/d{str(ns_index)}/fmu/in/goto_setpoint",
            f"/d{str(ns_index)}/fmu/in/manual_control_input",
            f"/d{str(ns_index)}/fmu/in/message_format_request",
            f"/d{str(ns_index)}/fmu/in/mode_completed",
            f"/d{str(ns_index)}/fmu/in/obstacle_distance",
            f"/d{str(ns_index)}/fmu/in/offboard_control_mode",
            f"/d{str(ns_index)}/fmu/in/onboard_computer_status",
            f"/d{str(ns_index)}/fmu/in/register_ext_component_request",
            f"/d{str(ns_index)}/fmu/in/sensor_optical_flow",
            f"/d{str(ns_index)}/fmu/in/telemetry_status",
            f"/d{str(ns_index)}/fmu/in/trajectory_setpoint",
            f"/d{str(ns_index)}/fmu/in/unregister_ext_component",
            f"/d{str(ns_index)}/fmu/in/vehicle_attitude_setpoint",
            f"/d{str(ns_index)}/fmu/in/vehicle_command",
            f"/d{str(ns_index)}/fmu/in/vehicle_command_mode_executor",
            f"/d{str(ns_index)}/fmu/in/vehicle_mocap_odometry",
            f"/d{str(ns_index)}/fmu/in/vehicle_rates_setpoint",
            f"/d{str(ns_index)}/fmu/in/vehicle_thrust_setpoint",
            f"/d{str(ns_index)}/fmu/in/vehicle_torque_setpoint",
            f"/d{str(ns_index)}/fmu/in/vehicle_trajectory_bezier",
            f"/d{str(ns_index)}/fmu/in/vehicle_trajectory_waypoint",
            f"/d{str(ns_index)}/fmu/in/vehicle_visual_odometry",
            f"/d{str(ns_index)}/fmu/out/estimator_status_flags",
            f"/d{str(ns_index)}/fmu/out/failsafe_flags",
            f"/d{str(ns_index)}/fmu/out/manual_control_setpoint",
            f"/d{str(ns_index)}/fmu/out/position_setpoint_triplet",
            f"/d{str(ns_index)}/fmu/out/sensor_combined",
            f"/d{str(ns_index)}/fmu/out/timesync_status",
            f"/d{str(ns_index)}/fmu/out/vehicle_attitude",
            f"/d{str(ns_index)}/fmu/out/vehicle_command_ack",
            f"/d{str(ns_index)}/fmu/out/vehicle_control_mode",
            f"/d{str(ns_index)}/fmu/out/vehicle_global_position",
            f"/d{str(ns_index)}/fmu/out/vehicle_gps_position",
            f"/d{str(ns_index)}/fmu/out/vehicle_land_detected",
            f"/d{str(ns_index)}/fmu/out/vehicle_local_position",
            f"/d{str(ns_index)}/fmu/out/vehicle_odometry",
            f"/d{str(ns_index)}/fmu/out/vehicle_status",

            # # # Others
            # "/depth_camera",
            # "/depth_camera/points"
            # "/diagnostics",
            # "/fake_map",
            # "/fe_plan/viz",
            # "/fe_plan/viz_array",
            # "/fe_plan_req",
            # "/fe_plan_req_array",
            # "/initialpose",
            # "/local_map/bounds",
            # "/mpc/traj",
            # "/occ_map/reset_map",
            # "/odom",
            # "/parameter_events",
            # "/point_goal",

            # "/rosout",

            # f"/agent_id_text",
            # f"/agent_id_text_array",
            # f"/amcl_pose",
            # f"/camera",
            # f"/camera_info",
            # f"/clicked_point",
            # f"/clock",
            f"/d{str(ns_index)}/cloud",
            f"/d{str(ns_index)}/cloud_global",
            f"/d{str(ns_index)}/agent_id_text",
            f"/d{str(ns_index)}/agent_id_text_array",
            f"/d{str(ns_index)}/amcl_pose",
            f"/d{str(ns_index)}/bond",
            f"/d{str(ns_index)}/fe_plan/viz",
            f"/d{str(ns_index)}/fe_plan/viz_array",
            f"/d{str(ns_index)}/fe_plan_req",
            f"/d{str(ns_index)}/fe_plan_req_array",
            f"/d{str(ns_index)}/reset_map",
            f"/d{str(ns_index)}/uav_state",
            f"/visbot_itof/point_cloud",
        ]

        for topic in sub_bag_topics:
            bag_topics.append(topic)
    
    extra_topics = [
        # # Others
        "/depth_camera",
        "/depth_camera/points"
        "/diagnostics",
        "/fake_map",
        "/fe_plan/viz",
        "/fe_plan/viz_array",
        "/fe_plan_req",
        "/fe_plan_req_array",
        "/initialpose",
        "/local_map/bounds",
        "/mpc/traj",
        "/occ_map/reset_map",
        "/odom",
        "/parameter_events",
        "/point_goal",

        "/rosout",

        f"/agent_id_text",
        f"/agent_id_text_array",
        f"/amcl_pose",
        f"/camera",
        f"/camera_info",
        f"/clicked_point",
        f"/clock",

        # # User commands
        "/global_uav_command",

        # Transformations
        "/tf",
        "/tf_static",
    ]

    for topic in extra_topics:
        bag_topics.append(topic)

    

    bag_file = os.path.join(
        os.path.expanduser("~"), 'bag_files',
        'bag_' + datetime.now().strftime("%d%m%Y_%H_%M_%S"),
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             bag_file,
             *bag_topics],
        output='log'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(rosbag_record)

    return ld