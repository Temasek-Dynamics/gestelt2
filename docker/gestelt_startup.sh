#!/bin/bash

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
CMD_0="
sudo /home/visbot/bin/visquad.sh
"

# Start ROS1 bridge
CMD_1="
roslaunch ros_zmq ros_zmq.launch
"

# Start Gestelt
CMD_2="
docker run --name ros2_container --ipc=host -it --rm --privileged --network host --mount type=bind,src=/tmp,dst=/tmp gestelt/mavoro_arm64:devel ros2 launch gestelt_bringup offboard_uav.py
"

# Start Zenoh
CMD_3="
docker exec -it ros2_container /ros_entrypoint.sh ros2 run ros2_zmq ros2_zmq_node
"

# Start ROS2 Bridge
CMD_4="
docker exec -it ros2_container /ros_entrypoint.sh zenoh-bridge-ros2dds -c ~/zenoh_d${DRONE_ID}_cfg.json5
"

# Restart VINS estimator
CMD_5="
rostopic pub -1 /vins_estimator/vins_restart geometry_msgs/PoseStamped -f ~/bin/restart.msg
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h

    # tmux send-keys -t $SESSION:0.0 "$CMD_0" C-m 
    # sleep 25
    # tmux send-keys -t $SESSION:0.1 "$CMD_1" C-m 
    # sleep 20
    # tmux send-keys -t $SESSION:0.2 "$CMD_2" C-m 
    # sleep 2
    # tmux send-keys -t $SESSION:0.3 "$CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

