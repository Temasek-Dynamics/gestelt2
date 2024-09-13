# Trajectory Server

# Testing use
```bash
colcon build --packages-select trajectory_server && ros2 launch trajectory_server traj_server_test.py

# Send service requests

# Take off 
ros2 run trajectory_server uav_cmd_srv_client 0 1.5 
# Landing 
ros2 run trajectory_server uav_cmd_srv_client 1
# Start Mission
ros2 run trajectory_server uav_cmd_srv_client 2 0
# Stop Mission
ros2 run trajectory_server uav_cmd_srv_client 3
# Emergency Stop
ros2 run trajectory_server uav_cmd_srv_client 4
```




# External libraries
1. [digint/tinyfsm](https://github.com/digint/tinyfsm): Header-only finite state machine library