# vistbot_setup (OWL 3)

# First time setup
1. Change hostname
2. Change IP address through network manager for wifi network.
3. Update .bashrc to reflect correct IP address and drone ID
4. Update ~/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch
5. Copy updated "gestelt_startup.sh" script
6. Mount docker images onto removable SSD 
  - Format as EXT4 (For use with Linux systems) 
  - Set custom mount path
7. Git pull and update ROS1 packages and configs 
  - Update "pcl_frame_id" and "minimum_points_per_voxel"
    - vim ~/ros1_ws/src/ros_zmq/launch/ros_zmq.launch
  - catkin_make
8. Update "~/bin/visquad.sh" with correct ip address

# Subsequent setups
1. Make sure hostname is correct
2. Update .bashrc
3. Ensure docker image is up to date
4. Copy startup script "gestelt_startup.sh" over
5. Update and build ROS1 packages

# Install dependencies
1. Install packages
```bash
sudo apt-get install curl gnome-disk-utility
```

2. Uninstall irrelevant packages
```bash
sudo apt-get remove -y thunderbird libreoffice*
```

3. Install docker
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Mount disk using gnome-disk

# Stop docker and update daemon
sudo systemctl stop docker
sudo rm -rf /var/lib/docker
sudo ln -s /mnt/gestelt /var/lib/docker
sudo vim /etc/docker/daemon.json
# Add the following information
{
  "data-root": "/mnt/gestelt"
}
sudo systemctl start docker
```

3. configuration for auto-connect to wifi 

```bash
/config/etc/wifi_cfg.sh
```

4. Scripts to modify

  1. Startup script: /home/visbot/bin/visquad.sh
    - Remove unnecessary nodes like ego_planner and geometric_controller
  2. /config/etc/wifi_cfg.sh
    - Configure auto-connect to WIFI
  3. ~/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch
    - Update `visbot_itof.launch` to make sure that it is setup. The IP address should be manually configured to match the drone's IP address defined in wifi_cfg.sh, and should be reset to 192.168.2.3${drone_id}. The IP address is hardcoded for each drone to expose various messages, especially the depth rostopic /visbot_itof/depth."
  4. ~/ros_ws/src/

5. Install bridge
```bash
sudo apt install libzmqpp4 libzmqpp-dev

https://bitbucket.org/nusuav/ros_zmq/src/master/
https://bitbucket.org/nusuav/depth2octomap/src/master/

ROS2
https://bitbucket.org/nusuav/ros2_zmq/src/master/

```

## Topics of interest

### Odometry
1. `/vins_estimator/camera_pose`
2. `/mavros/local_position/pose`: Ensure this topic is publishing correctly around 20 Hz, which indicates the communication between the onboard computer and the PX4 flight controller established and the VIO system is working.
3. `/mavros/vision_pose/pose`

### Sensor
1. `/vins_estimator/point_cloud`
4. `/visbot_itof/depth`: Only for OWL3. Ensure the depth image is being published around 10 Hz.

## Communications

Drone 6 IP: 

1. GCS connection
    - Look at `VisBot_Drone/visbot_monitor_ws/bringup/scripts/visbot_update/config_update/owl3/owl3_customized/owl3_config.yaml` 
    - Determine what is the GCS UDP port dependending on the drone_id. If drone_id is 6, then set gcs udp port is 14556
    - Add the connection to QGroundControl. More details can be found on the visbot_drone README

2. Mavlink Connection
    - `mavlink status streams`

3. FCU Ports
    - TELEM1: /dev/ttyS6 (FCU) <-> /dev/ttyS7 (OBC)
    - TELEM2: UDP 
    - TELEM3: /dev/ttyS1 (FCU) <->
    - ETHERNET: USB

# Troubleshooting

1. Unable to communicate ROS2 topics between docker container and host  
  - Using --net=host implies both DDS participants believe they are in the same machine and they try to communicate using SharedMemory instead of UDP".
 - [reference](https://robotics.stackexchange.com/questions/98161/ros2-foxy-nodes-cant-communicate-through-docker-container-border)


# Change hostname
```bash
# Change all instances of droneX to the correct hostname
sudo vim /etc/hosts
# Set the hostname to the appropriate droneX
sudo hostnamectl set-hostname droneX
# Restart hostname service
sudo systemctl restart systemd-hostnamed
```


# Change parameters at runtime
```bash

# noise_search_radius
# noise_min_neighbors
# occ_map.prob_miss_log
# occ_map.prob_hit_log
# occ_map.clamp_min_log
# occ_map.clamp_max_log
# occ_map.occupancy_threshold_log
# occ_map.static_inflation

ros2 param set navigator_0 voxel_map.noise_search_radius 0.17
ros2 param set navigator_0 voxel_map.noise_min_neighbors 3

ros2 param set navigator_0 voxel_map.occ_map.prob_miss_log 0.35
ros2 param set navigator_0 voxel_map.occ_map.prob_hit_log 0.78
ros2 param set navigator_0 voxel_map.occ_map.clamp_min_log 0.12
ros2 param set navigator_0 voxel_map.occ_map.clamp_max_log 0.90
ros2 param set navigator_0 voxel_map.occ_map.occupancy_threshold_log 0.65

ros2 param set navigator_0 voxel_map.occ_map.static_inflation 0.1
```

# Mounting for compiling ROS2 ZMQ Bridge
```
docker run --name ros2_container -it --platform linux/arm64 --privileged --ipc=host --network host --mount type=bind,src=/opt/ros/noetic,dst=/opt/ros/noetic gestelt/mavoro_arm64:base
colcon build --symlink-install --packages-select ros2_zmq
```