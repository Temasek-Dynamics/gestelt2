# Update images
```bash
docker pull gestelt/mavoro_arm64:latest
docker pull gestelt/ros_one_broker:latest
```


# Visbot
1. Start up Gestelt container
```bash
# docker run -it --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro:latest
start_gestelt
uav_startup 
```
2. Start up ros one broker
```bash
start_ros_one_broker
```

# GCS
1. Start ground visualization station and mission node
```bash
ros2 launch gestelt_bringup gcs.py
```
2. Execute commands or mission
```bash
# Execute a mission according to a given scenario
ros2 run gestelt_bringup execute_mission.py
# Landing
ros2 run gestelt_mission land
# Takeoff
ros2 run gestelt_mission takeoff
```
