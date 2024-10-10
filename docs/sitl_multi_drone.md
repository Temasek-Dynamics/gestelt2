# Starting the agent
Only need 1 agent for multiple drones

```bash
MicroXRCEAgent udp4 -p 8888 -v
```

# Starting the clients
Start client 

```bash
# Actual hardware
uxrce_dds_client start -t udp -p 8888 -h 192.168.0.100 -n d0
# Simulation
ROS_DOMAIN_ID=$ROS_DOMAIN_ID PX4_UXRCE_DDS_PORT=8888 PX4_UXRCE_DDS_NS=d0 make px4_sitl gz_x500
```

# Starting px4 instance

## Environment variables
- PX4_SYS_AUTOSTART=4001
- PX4_SIM_MODEL=gz_x500
- PX4_GZ_MODEL_POSE="0,1,0,0,0,0"
- PX4_GZ_STANDALONE=1

- PX4_GZ_WORLD=default

[reference](https://docs.px4.io/main/en/sim_gazebo_gz/#usage-configuration-options)

## px4 arguments
-i (instance)
    0
-d (daemon mode)
-s (startupfile)
    $BUILD_DIR/etc/init.d-posix/rcS 
-w (working_directory) 
    $BUILD_DIR/rootfs

```bash
export BUILD_DIR=~/PX4-Autopilot/build/px4_sitl_default

export PX4_GZ_WORLD=default
export PX4_SIM_MODEL=gz_x500
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL_POSE="0,1,0,0,0,0"

export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export PX4_UXRCE_DDS_PORT=8888
export PX4_UXRCE_DDS_NS=d0

$BUILD_DIR/bin/px4 $BUILD_DIR/etc -w $BUILD_DIR/rootfs -s $BUILD_DIR/etc/init.d-posix/rcS -i 0 -d
```
