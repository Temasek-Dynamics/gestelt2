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

NUM_AGENTS = 1

def generate_launch_description():
    ns = "/d0"

    # ROSBag 
    bag_topics = [
        # Pose
        f"{ns}/odom",
        f"{ns}/initialpose",
        
        # # Mapping
        f"{ns}/local_map/bounds",
        f"{ns}/local_occ_map/occ_map",
        f"{ns}/local_occ_map/transition_event",
        f"{ns}/occ_map",
        f"{ns}/global_occ_map/occ_map",
        f"{ns}/global_occ_map/transition_event",
        f"{ns}/goal",
        f"{ns}/goals",

        # # Global Plan
        f"{ns}/planner_server/transition_event",
        f"{ns}/point_goal",
        
        # # Controller output
        f"{ns}/sfc",
        f"{ns}/received_global_plan",
        f"{ns}/mpc_ref_path",
        f"{ns}/mpc_traj",
        f"{ns}/intmd_cmd",
        f"{ns}/plan",

        f"{ns}/controller_server/transition_event"

        # # Transformations
        "/tf",
        "/tf_static",

        # # User commands
        f"{ns}/uav_command",
        "/global_uav_command",

        # PX4
        f"{ns}/fmu/in/actuator_motors",
        f"{ns}/fmu/in/actuator_servos",
        f"{ns}/fmu/in/arming_check_reply",
        f"{ns}/fmu/in/aux_global_position",
        f"{ns}/fmu/in/config_control_setpoints",
        f"{ns}/fmu/in/config_overrides_request",
        f"{ns}/fmu/in/goto_setpoint",
        f"{ns}/fmu/in/manual_control_input",
        f"{ns}/fmu/in/message_format_request",
        f"{ns}/fmu/in/mode_completed",
        f"{ns}/fmu/in/obstacle_distance",
        f"{ns}/fmu/in/offboard_control_mode",
        f"{ns}/fmu/in/onboard_computer_status",
        f"{ns}/fmu/in/register_ext_component_request",
        f"{ns}/fmu/in/sensor_optical_flow",
        f"{ns}/fmu/in/telemetry_status",
        f"{ns}/fmu/in/trajectory_setpoint",
        f"{ns}/fmu/in/unregister_ext_component",
        f"{ns}/fmu/in/vehicle_attitude_setpoint",
        f"{ns}/fmu/in/vehicle_command",
        f"{ns}/fmu/in/vehicle_command_mode_executor",
        f"{ns}/fmu/in/vehicle_mocap_odometry",
        f"{ns}/fmu/in/vehicle_rates_setpoint",
        f"{ns}/fmu/in/vehicle_thrust_setpoint",
        f"{ns}/fmu/in/vehicle_torque_setpoint",
        f"{ns}/fmu/in/vehicle_trajectory_bezier",
        f"{ns}/fmu/in/vehicle_trajectory_waypoint",
        f"{ns}/fmu/in/vehicle_visual_odometry",
        f"{ns}/fmu/out/estimator_status_flags",
        f"{ns}/fmu/out/failsafe_flags",
        f"{ns}/fmu/out/manual_control_setpoint",
        f"{ns}/fmu/out/position_setpoint_triplet",
        f"{ns}/fmu/out/sensor_combined",
        f"{ns}/fmu/out/timesync_status",
        f"{ns}/fmu/out/vehicle_attitude",
        f"{ns}/fmu/out/vehicle_command_ack",
        f"{ns}/fmu/out/vehicle_control_mode",
        f"{ns}/fmu/out/vehicle_global_position",
        f"{ns}/fmu/out/vehicle_gps_position",
        f"{ns}/fmu/out/vehicle_land_detected",
        f"{ns}/fmu/out/vehicle_local_position",
        f"{ns}/fmu/out/vehicle_odometry",
        f"{ns}/fmu/out/vehicle_status",

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
        f"/cloud",
        f"/cloud_global",
        f"{ns}/agent_id_text",
        f"{ns}/agent_id_text_array",
        f"{ns}/amcl_pose",
        f"{ns}/bond",
        f"{ns}/fe_plan/viz",
        f"{ns}/fe_plan/viz_array",
        f"{ns}/fe_plan_req",
        f"{ns}/fe_plan_req_array",
        f"{ns}/reset_map",
        f"{ns}/uav_state",
        f"/visbot_itof/point_cloud",
    ]

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