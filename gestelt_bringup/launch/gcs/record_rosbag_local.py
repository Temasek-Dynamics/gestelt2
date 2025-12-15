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
    ns = os.environ["DRONE_NS"]

    # ROSBag 
    bag_topics = [
        # Pose
        "/odom",
        "/initialpose",
        
        # # Mapping
        "/local_map/bounds",
        "/local_occ_map/occ_map",
        "/local_occ_map/transition_event",
        "/occ_map",
        "/global_occ_map/occ_map",
        "/global_occ_map/transition_event",
        "/goal",
        "/goals",

        # # Global Plan
        "/planner_server/transition_event",
        "/point_goal",
        
        # # Controller output
        "/sfc",
        "/received_global_plan",
        "/mpc_ref_path",
        "/mpc_curr_traj",
        "/mpc_traj",
        "/intmd_cmd",
        "/plan",

        "/controller_server/transition_event"

        # # Transformations
        "/tf",
        "/tf_static",

        # # User commands
        "/uav_command",
        # "/global_uav_command",

        # PX4
        "/fmu/in/actuator_motors",
        "/fmu/in/actuator_servos",
        "/fmu/in/arming_check_reply",
        "/fmu/in/aux_global_position",
        "/fmu/in/config_control_setpoints",
        "/fmu/in/config_overrides_request",
        "/fmu/in/goto_setpoint",
        "/fmu/in/manual_control_input",
        "/fmu/in/message_format_request",
        "/fmu/in/mode_completed",
        "/fmu/in/obstacle_distance",
        "/fmu/in/offboard_control_mode",
        "/fmu/in/onboard_computer_status",
        "/fmu/in/register_ext_component_request",
        "/fmu/in/sensor_optical_flow",
        "/fmu/in/telemetry_status",
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/unregister_ext_component",
        "/fmu/in/vehicle_attitude_setpoint",
        "/fmu/in/vehicle_command",
        "/fmu/in/vehicle_command_mode_executor",
        "/fmu/in/vehicle_mocap_odometry",
        "/fmu/in/vehicle_rates_setpoint",
        "/fmu/in/vehicle_thrust_setpoint",
        "/fmu/in/vehicle_torque_setpoint",
        "/fmu/in/vehicle_trajectory_bezier",
        "/fmu/in/vehicle_trajectory_waypoint",
        "/fmu/in/vehicle_visual_odometry",
        "/fmu/out/estimator_status_flags",
        "/fmu/out/failsafe_flags",
        "/fmu/out/manual_control_setpoint",
        "/fmu/out/position_setpoint_triplet",
        "/fmu/out/sensor_combined",
        "/fmu/out/timesync_status",
        "/fmu/out/vehicle_attitude",
        "/fmu/out/vehicle_command_ack",
        "/fmu/out/vehicle_control_mode",
        "/fmu/out/vehicle_global_position",
        "/fmu/out/vehicle_gps_position",
        "/fmu/out/vehicle_land_detected",
        "/fmu/out/vehicle_local_position",
        "/fmu/out/vehicle_odometry",
        "/fmu/out/vehicle_status",

        # # Others
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
        "/point_goal",

        # "/rosout",

        # f"/agent_id_text",
        # f"/agent_id_text_array",
        # f"/amcl_pose",
        # f"/camera",
        # f"/camera_info",
        # f"/clicked_point",
        # f"/clock",
        # f"/cloud",
        # f"/cloud_global",
        # f"{ns}/agent_id_text",
        # f"{ns}/agent_id_text_array",
        # f"{ns}/amcl_pose",
        # f"{ns}/bond",
        # f"{ns}/fe_plan/viz",
        # f"{ns}/fe_plan/viz_array",
        # f"{ns}/fe_plan_req",
        # f"{ns}/fe_plan_req_array",
        f"{ns}/reset_map",
        f"{ns}/uav_state",
        # f"/visbot_itof/point_cloud",
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