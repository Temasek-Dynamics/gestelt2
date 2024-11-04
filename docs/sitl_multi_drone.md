# Starting the agent
Only need 1 agent for multiple drones

```bash
MicroXRCEAgent udp4 -p 8888 -v
```

# Starting the clients
Start client 

```bash
# Actual hardware
uxrce_dds_client start -t udp -p 8888 -h 192.168.0.100 -n d0
# Simulation
ROS_DOMAIN_ID=$ROS_DOMAIN_ID PX4_UXRCE_DDS_PORT=8888 PX4_UXRCE_DDS_NS=d0 make px4_sitl gz_x500
```

# Starting px4 instance

## Environment variables
- PX4_SYS_AUTOSTART=4001
- PX4_SIM_MODEL=gz_x500
- PX4_GZ_MODEL_POSE="0,1,0,0,0,0"
- PX4_GZ_STANDALONE=1

- PX4_GZ_WORLD=default

[reference](https://docs.px4.io/main/en/sim_gazebo_gz/#usage-configuration-options)

## px4 arguments
-i (instance)
    0
-d (daemon mode)
-s (startupfile)
    $BUILD_DIR/etc/init.d-posix/rcS 
-w (working_directory) 
    $BUILD_DIR/rootfs

```bash
export BUILD_DIR=~/PX4-Autopilot/build/px4_sitl_default

export PX4_GZ_WORLD=default
export PX4_SIM_MODEL=gz_x500
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL_POSE="0,1,0,0,0,0"

export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export PX4_UXRCE_DDS_PORT=8888
export PX4_UXRCE_DDS_NS=d0

$BUILD_DIR/bin/px4 $BUILD_DIR/etc -w $BUILD_DIR/rootfs -s $BUILD_DIR/etc/init.d-posix/rcS -i 0 -d
```

# Quick start

## Initial setup
```bash
# Start gazebo and PX4 SITL
make px4_sitl gz_x500
# Start QGround Control
~/Documents/QGroundControl.AppImage
# Start XRCE DDS agent  
MicroXRCEAgent udp4 -p 8888
```

## Demo 1: Listener example
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
## Demo 2: Offboard control
```bash
ros2 run px4_ros_com offboard_control
```

## Demo 1 example
```bash
# Example
ros2 run example_mode_manual_cpp example_mode_manual

# Check on PX4 shell that the custom mode is registered
commander status
```

## Debugging
```bash
ros2 topic echo /fmu/out/vehicle_status
```

# Input Message types
1. Control input topics
    - **Position, Velocity and Acceleration**
        - [TrajectorySetpoint](https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html): PVA
            - All values are interpreted in NED (Nord, East, Down) coordinate system and the units are [m], [m/s] and [m/s^2] for position, velocity and acceleration, respectively.
    - **Collective thrust, attitude**
        - [VehicleAttitudeSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html) Normalized thrust vector, attitude (quaternion)
            - Normalized thrust command in body FRD frame [-1,1]
            - The quaternion represents the rotation between the drone body FRD (front, right, down) frame and the NED frame. The thrust is in the drone body FRD frame and expressed in normalized [-1, 1] values.
    - Collective thrust, rates
        - [VehicleRatesSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleRatesSetpoint.html)
            - Normalized thrust command in body NED frame [-1,1]
            - All the values are in the drone body FRD frame. The rates are in [rad/s] while thrust_body is normalized in [-1, 1].
    - **Thrust and torque**
        - [VehicleThrustSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html) 
            - Thrust setpoint along X, Y, Z body axis [-1, 1]
        - [VehicleTorqueSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleTorqueSetpoint.html) 
            - Torque setpoint about X, Y, Z body axis (normalized)
            - All the values are in the drone body FRD frame and normalized in [-1, 1].
    - **Individual motors**
        - [ActuatorMotors](https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html) Individual motor values
            - `Topic: "/fmu/in/actuator_motors"`
            - All the values normalized in [-1, 1]. For outputs that do not support negative values, negative entries map to NaN. 
            - NaN maps to disarmed.

2. [VehicleCommand](https://docs.px4.io/main/en/msg_docs/VehicleCommand.html)
    - `Topic: "/fmu/in/vehicle_command"`
    - Used to set to offboard mode

3. [OffboardControlMode](https://docs.px4.io/main/en/msg_docs/OffboardControlMode.html)
    - `Topic: "/fmu/in/offboard_control_mode"`
    - The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
    - The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.

# Output Message types
1. [VehicleOdometry](https://docs.px4.io/main/en/msg_docs/VehicleOdometry.html)
    - `Topic: `
    - Vehicle odometry data. Fits ROS REP 147 for aerial vehicles

    
## Gazebo PX4 targets
```bash
# make px4_sitl 
gz_x500
gz_x500_depth
gz_x500_vision
gz_x500_lidar
gz_standard_vtol
gz_rc_cessna
gz_advanced_plane
gz_r1_rover
gz_rover_ackermann
```

# Reference
1. [Virtual env usage](https://github.com/ros2/ros2/issues/1094)
2. [Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
