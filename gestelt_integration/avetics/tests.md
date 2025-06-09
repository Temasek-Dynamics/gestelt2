# tests

# Test Static Planning (3/6/25)

Goals:
    - Test controls and planning with basic obstacle avoidance

```bash
#####
# Drone
#####
~/gestelt_ws/src/gestelt2/gestelt_integration/avetics/startup_scripts/gestelt_startup.sh

#####
# GCS
#####
~/gestelt_ws/src/gestelt2/gestelt_integration/avetics/startup_scripts/gcs_startup.sh
```

# Test Obstacle Avoidance (3/6/25)

Goals:
    - Test controls and planning with basic obstacle avoidance

```bash
#####
# Drone
#####
~/gestelt_ws/src/gestelt2/gestelt_integration/avetics/startup_scripts/gestelt_startup.sh

#####
# GCS
#####
~/gestelt_ws/src/gestelt2/gestelt_integration/avetics/startup_scripts/gcs_startup.sh
```

Result:
1. Drone plans very close to obstacle

# Test take off with point goal (30/5/25)

Goals:
    - Trajectory Server
        - Test orientation with "transform_cmd_from_nwu_to_enu" set to falase
    - Occupancy Map
        - Test with smaller map parameters
    - A star planning
        - See map planning
    - Collect ROSBags
```bash
#####
# Drone
#####
start_vilota
# Launch offboard nodes
ros2 launch gestelt_bringup offboard_launch.py 

#####
# GCS
#####
# Start QGRoundControl

# Start RVIZ Visualization
ros2 launch gestelt_bringup rviz_viz.py

# Take off and start point goal script
ros2 launch gestelt_bringup test_take_off_point_goal.py scenario_name:=single_drone_test

# Land drone
ros2 run gestelt_commander land scenario_name:=single_drone_test
```

# Test take off and planning with perception (26/5/25)

Goals:
    - Trajectory Server
        - Test orientation with "transform_cmd_from_nwu_to_enu" set to falase
    - Occupancy Map
        - Test with smaller map parameters
    - A star planning
        - See map planning
    - Collect ROSBags
```bash
#####
# Drone
#####
start_vilota
# Launch offboard nodes
ros2 launch gestelt_bringup offboard_launch.py 

#####
# GCS
#####
ros2 launch gestelt_bringup rviz_viz.py

ros2 topic pub /reset_map std_msgs/msg/Empty {} -1

ros2 launch gestelt_bringup test_take_off_goal.py scenario_name:=single_drone_test

ros2 run gestelt_commander land scenario_name:=single_drone_test
```

# Test NWU to ENU transformation for command to PX4 (20/5/25)

```bash
#####
# Drone
#####
start_vilota
# Launch offboard nodes
ros2 launch gestelt_bringup offboard_launch.py 

#####
# GCS
#####
ros2 launch gestelt_bringup rviz_viz.py

ros2 topic pub /reset_map std_msgs/msg/Empty {} -1

ros2 launch gestelt_bringup test_take_off_goal.py scenario_name:=single_drone_test

ros2 run gestelt_commander land scenario_name:=single_drone_test
```

# Test Mapping (19/5/25)

```bash
#####
# Drone
#####
ros2 launch gestelt_bringup vilota_launch.py
# Launch offboard nodes
ros2 launch gestelt_bringup offboard_launch.py 

#####
# GCS
#####
ros2 launch gestelt_bringup gcs.py
ros2 launch gestelt_bringup test_take_off_goal.py scenario_name:=single_drone_test
ros2 run gestelt_commander land scenario_name:=single_drone_test
```

# Test Mapping (14/5/25)
```bash
#####
# Drone
#####
ros2 launch gestelt_bringup vilota_launch.py
# Launch offboard nodes
ros2 launch gestelt_bringup offboard_launch.py 

#####
# GCS
#####
ros2 launch gestelt_bringup gcs.py
ros2 launch gestelt_bringup test_take_off_goal.py scenario_name:=single_drone_test
# ros2 launch gestelt_bringup test_planning.py scenario_name:=single_drone_test

ros2 run gestelt_commander land scenario_name:=single_drone_test
```

# Test Planning (23/4/25)
```bash
# Drone 
ros2 run vision vio_bridge_px4
# 1 (VK180Pro)
# 1 (Forward Facing)
# 0 -10 0 (Roll, Pitch, Yaw)
ros2 launch gestelt_bringup offboard_launch.py 

# GCS 
ros2 launch gestelt_bringup gcs.py
ros2 launch gestelt_bringup test_take_off_goal.py scenario_name:=single_drone_test
```

# Test Planning (23/4/25)
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

