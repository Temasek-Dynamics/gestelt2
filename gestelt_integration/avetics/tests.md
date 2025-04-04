# tests

# 4/4/25

1. Test arming/offboard mode from PX4w
```bash
# Drone 
ros2 launch gestelt_bringup offboard_launch.py 
# GCS 
ros2 launch gestelt_bringup test_takeoff.py scenario_name:=single_drone_test
```

