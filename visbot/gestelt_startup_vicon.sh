#!/bin/bash

# Usage: ./gestelt_startup.sh -i <DRONE_ID>

SESSION="gestelt_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Get arguments
#####
# getopts: function to read flags in input
# OPTARG: refers to corresponding values
while getopts i: flag
do
    case "${flag}" in
        i) DRONE_ID=${OPTARG};; 
    esac
done

#####
# Commands
#####
# Start ROS1 nodes
ROS1_NODES=" \
sudo /home/visbot/bin/visquad.sh \
"

# Start ROS1 bridge
ROS1_TO_ROS2_BRIDGE=" \
roslaunch ros_zmq ros_zmq.launch \
"

# Start Gestelt
ROS2_NODES=" \
docker run -e 'DRONE_ID=${DRONE_ID}' --name ros2_container --ipc=host -it --rm --privileged \
--network host --mount type=bind,src=/tmp,dst=/tmp gestelt/mavoro_arm64:devel \
/ros_entrypoint.sh \
ros2 launch gestelt_bringup offboard_uav.py drone_id:=${DRONE_ID} scenario_name:=start_4d_vicon \
"

# Start Zenoh bridge 
ZENOH_BRIDGE=" \
docker exec -it ros2_container /ros_entrypoint.sh ros2 run ros2_zmq ros2_zmq_node \
"

# Start ROS2 Bridge
ROS2_TO_ROS1_BRIDGE=" \
docker exec -it ros2_container /ros_entrypoint.sh zenoh-bridge-ros2dds \
-c /root/gestelt_ws/src/gestelt2/gestelt_network/zenoh_d${DRONE_ID}_cfg.json5 \
"

# Restart VINS estimator
RESTART_VINS=" \
rostopic pub -1 /vins_estimator/vins_restart geometry_msgs/PoseStamped -f /home/visbot/bin/restart.msg \
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$ROS2_NODES" C-m 
    tmux send-keys -t $SESSION:0.1 "sleep 1 && $ZENOH_BRIDGE" C-m
    tmux send-keys -t $SESSION:0.2 "sleep 1 && $ROS2_TO_ROS1_BRIDGE" C-m
    tmux send-keys -t $SESSION:0.3 "sleep 5 && $ROS1_NODES" C-m 
    # ROS1 bridge needs to start a while after ROS1 nodes as ROS1_NODES script will terminate ros1 nodes at the start
    # tmux send-keys -t $SESSION:0.4 "sleep 25 && $ROS1_TO_ROS2_BRIDGE" C-m 
    # tmux send-keys -t $SESSION:0.5 "sleep 16 &&  $RESTART_VINS" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

