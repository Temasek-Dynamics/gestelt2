# Topics from PX4 

# Input topics

/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/unregister_ext_component

/fmu/in/config_overrides_request
/fmu/in/arming_check_reply
/fmu/in/aux_global_position
    - px4_msgs/msg/VehicleGlobalPosition
/fmu/in/config_control_setpoints
    - [px4_msgs/msg/VehicleControlMode]
/fmu/in/goto_setpoint
    - [px4_msgs/msg/GotoSetpoint]
    - Go to position setpoint with velocity constraints
/fmu/in/manual_control_input
    - [px4_msgs/msg/ManualControlSetpoint]
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance

## Controls

/fmu/in/offboard_control_mode
    - [px4_msgs/msg/OffboardControlMode]
    - Control offboard mode
/fmu/in/vehicle_command
    - [px4_msgs/msg/VehicleCommand]
/fmu/in/vehicle_command_mode_executor
    - [px4_msgs/msg/VehicleCommand]
/fmu/in/trajectory_setpoint
    - [px4_msgs/msg/TrajectorySetpoint]
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/actuator_motors
/fmu/in/actuator_servos

/fmu/in/vehicle_trajectory_bezier
    - [px4_msgs/msg/VehicleTrajectoryBezier]
/fmu/in/vehicle_trajectory_waypoint
    - [px4_msgs/msg/VehicleTrajectoryWaypoint]

/fmu/in/vehicle_visual_odometry
/fmu/in/vehicle_mocap_odometry

# Output topics

/fmu/out/arming_check_request
/fmu/out/battery_status
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/manual_control_setpoint
/fmu/out/message_format_response
/fmu/out/position_setpoint_triplet
/fmu/out/register_ext_component_reply
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status

/parameter_events
/rosout
