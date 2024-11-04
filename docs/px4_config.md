# Configuration of PX4 parameters
- NAV_DLL_ACT: Set to disabled to disable checks for GCS connection
- MAV_MODE_FLAG: needs to be set to 1
- COM_RC_IN_MODE: Set to 4 to disable need for manual controller to arm
- COM_OF_LOSS_T: Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (COM_OBL_RC_ACT)
- COM_OBL_RC_ACT: Flight mode to switch to if offboard control is lost (Values are - 0: Position, 1: Altitude, 2: Manual, 3: *Return, 4: *Land*).
- COM_RC_OVERRIDE: Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to Position mode. This is not enabled for offboard mode by default.

## Important notes
1. All topics that you can use are defined in `dds_topics.yaml`

2. Specifically, nodes should subscribe using the ROS 2 predefined QoS sensor data (from the listener example source code):
```cpp
...
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
...
```

3. Examples of vectors that require rotation are:
- all fields in TrajectorySetpoint message; ENU to NED conversion is required before sending them.
    - first a pi/2 rotation around the Z-axis (up),
    - then a pi rotation around the X-axis (old East/new North).
- all fields in VehicleThrustSetpoint message; FLU to FRD conversion is required before sending them.
    - a pi rotation around the X-axis (front) is sufficient.
