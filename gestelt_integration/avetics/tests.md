# tests

# 23/4/25
```bash
# Drone 
ros2 run vision vio_bridge_px4
# 1 (VK180Pro)
# 1 (Forward Facing)
# 0 -10 0 (Roll, Pitch, Yaw)

# Depth to point cloud
ros2 launch depth2pcl depth2pcl.launch 
# Offboard control
ros2 launch gestelt_bringup offboard_launch.py 
# Vilota bridge
ros2 launch vilota_bridge vilota_bridge_launch.py
zenoh-bridge-ros2dds -c /home/nvidia/gestelt_ws/src/gestelt2/gestelt_network/zenoh_d0_cfg.json5
# GCS 
ros2 launch gestelt_bringup gcs.py
ros2 launch gestelt_bringup test_planning.py scenario_name:=single_drone_test
zenoh-bridge-ros2dds -c /home/john/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5
```

# 11/4/25

1. Bridge odom between PX4 and Vilota camera
```bash
# Drone 
ros2 launch gestelt_bringup offboard_launch.py 
zenoh-bridge-ros2dds -c /home/nvidia/gestelt_ws/src/gestelt2/gestelt_network/zenoh_d0_cfg.json5

# GCS 
ros2 launch gestelt_bringup test_takeoff.py scenario_name:=single_drone_test
zenoh-bridge-ros2dds -c /home/john/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5

```

# 4/4/25

1. Test arming/offboard mode from PX4
```bash
# Drone 
ros2 launch gestelt_bringup offboard_launch.py 
zenoh-bridge-ros2dds -c /home/nvidia/gestelt_ws/src/gestelt2/gestelt_network/zenoh_d0_cfg.json5

# GCS 
ros2 launch gestelt_bringup test_takeoff.py scenario_name:=single_drone_test
zenoh-bridge-ros2dds -c /home/john/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5
```

