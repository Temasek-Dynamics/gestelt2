# vistbot_setup (OWL 3)

# Install dependencies
1. Install packages
```bash
sudo apt-get install curl
```

2. Uninstall irrelevant packages
```bash
sudo apt-get remove thunderbird 
```

3. Install docker
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Stop docker and update daemon
sudo systemctl stop docker
sudo rm -rf /var/lib/docker
sudo ln -s /media/visbot/gestelt/docker /var/lib/docker
sudo vim /etc/docker/daemon.json
# Add the following information
{
  "data-root": "/mnt/gestelt"
}
sudo systemctl start docker
```

# Add to ~/.bashrc
```bash
alias start_gestelt="docker run -it  --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro_arm64:base"
alias start_ros_one_broker="docker run -it --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/ros_one_broker:latest"
```

3. configuration for auto-connect to wifi 

```bash
/config/etc/wifi_cfg.sh
```

## Scripts to modify
1. Startup script: /home/visbot/bin/visquad.sh
  - Remove unnecessary nodes like ego_planner and geometric_controller
2. /config/etc/wifi_cfg.sh
  - Configure auto-connect to WIFI
3. ~/ros_ws/install/share/visbot_itof/launch/visbot_itof.launch
  - Update `visbot_itof.launch` to make sure that it is setup. The IP address should be manually configured to match the drone's IP address defined in wifi_cfg.sh, and should be reset to 192.168.2.3${drone_id}. The IP address is hardcoded for each drone to expose various messages, especially the depth rostopic /visbot_itof/depth."
4. To `~/.bashrc`, add the following
```bash

```

## Topics of interest

### Odometry
1. `/vins_estimator/camera_pose`
2. `/mavros/local_position/pose`: Ensure this topic is publishing correctly around 20 Hz, which indicates the communication between the onboard computer and the PX4 flight controller established and the VIO system is working.
3. `/mavros/vision_pose/pose`

### Sensor
1. `/vins_estimator/point_cloud`
4. `/visbot_itof/depth`: Only for OWL3. Ensure the depth image is being published around 10 Hz.

# Restart NTP 
```bash
sudo service ntp stop
sudo ntpd -gq
sudo service ntp start
```


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

