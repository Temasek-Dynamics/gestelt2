# tests

# 4/4/25

1. Test arming/offboard mode from PX4
```bash
# Drone 
ros2 launch gestelt_bringup offboard_launch.py 
# On drone
zenoh-bridge-ros2dds -c /home/nvidia/gestelt_ws/src/gestelt2/gestelt_network/zenoh_d0_cfg.json5

# GCS 
ros2 launch gestelt_bringup test_takeoff.py scenario_name:=single_drone_test
# On host
zenoh-bridge-ros2dds -c /home/john/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5
```


# 11/4/25

1. Bridge odom between PX4 and Vilota camera
```bash


```