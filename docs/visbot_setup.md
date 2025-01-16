# vistbot_setup (OWL 3)

## Quick start
1. Start visbot scripts
2. Start Gestelt container
```bash
docker run -it --privileged --network host -e "DRONE_ID=0" gestelt/mavoro:latest
ros2 launch gestelt_bringup offboard.py
```
3. configuration for auto-connect to wifi 
```bash
/config/etc/wifi_cfg.sh
```

## Scripts to modify
1. Startup script: /home/visbot/bin/visquad.sh
2. /config/etc/wifi_cfg.sh
3. ~/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch
4. ~/.bashrc

## Topics of interest

### Odometry
1. `/vins_estimator/camera_pose`
2. `/mavros/local_position/pose`: Ensure this topic is publishing correctly around 20 Hz, which indicates the communication between the onboard computer and the PX4 flight controller established and the VIO system is working.
3. `/mavros/vision_pose/pose`

### Sensor
1. `/vins_estimator/point_cloud`
4. `/visbot_itof/depth`: Only for OWL3. Ensure the depth image is being published around 10 Hz.
  - Update `visbot_itof.launch` to make sure that it is setup. The IP address should be manually configured to match the drone's IP address defined in wifi_cfg.sh, and should be reset to 192.168.2.3${drone_id}. The IP address is hardcoded for each drone to expose various messages, especially the depth rostopic /visbot_itof/depth."

## Restore image on Visbot
- Get a live OS image from (Ubuntu 20.04.3 LTS (Focal Fossa) Daily Build)[https://ftpmirror.your.org/pub/ubuntu/cdimage/focal/daily-live/20211219/HEADER.html]
  - Mount it onto a portable disk

- Identify the partition, an example being `/dev/mmcblk0p7`

- To back-up partition to image
```bash
# Make sure partition is un-mounted first! 
dd if=/dev/mmcblk0p7 of=./IMAGE_NAME.image
```

- To restore image to partition
```bash
dd if=./IMAGE_NAME.image of=/dev/mmcblk0p7
```

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

