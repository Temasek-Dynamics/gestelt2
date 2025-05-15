#!/bin/bash
set -e

# setup ROS1 environment
# source /opt/ros/noetic/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/root/gestelt_ws/install/setup.bash" --
exec "$@"
~                