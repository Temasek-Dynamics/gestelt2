#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source ~/gestelt_ws/install/setup.bash
exec "$@"