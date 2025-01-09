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
2. wifi_cfg.sh
2. visbot_itof.launch

## Topics of interest
1. `/vins_estimator/point_cloud`
2. `/mavros/local_position/odom`: Ensure this topic is publishing correctly around 20 Hz, which indicates the communication between the onboard computer and the PX4 flight controller established and the VIO system is working.
3. `/ego_planner_node/grid_map/occupancy`: Verify quality of the depth image obtained from the stereo camera or ToF sensor.
4. `/visbot_itof/depth`: Only for OWL3. Ensure the depth image is being published around 10 Hz.
  - Update `visbot_itof.launch` to make sure that it is setup. The IP address should be manually configured to match the drone's IP address defined in wifi_cfg.sh, and should be reset to 192.168.2.3${drone_id}. The IP address is hardcoded for each drone to expose various messages, especially the depth rostopic /visbot_itof/depth."

## Communications

Drone 6 IP: 

1. GCS connection
    - Look at `VisBot_Drone/visbot_monitor_ws/bringup/scripts/visbot_update/config_update/owl3/owl3_customized/owl3_config.yaml` 
    - Determine what is the GCS UDP port dependending on the drone_id. If drone_id is 6, then set gcs udp port is 14556
    - Add the connection to QGroundControl. More details can be found on the visbot_drone README

2. XRCE Client Connection
    - uxrce_dds_client start -t serial -d /dev/ttyS6 -b 921600
    - uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600
    - [Reference](https://docs.px4.io/main/en/modules/modules_system.html#uxrce-dds-client)

3. Mavlink Connection
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

