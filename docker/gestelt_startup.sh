#!/bin/bash

SESSION="gestelt_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Commands
#####
# Start ROS1 bridge
CMD_0="
sudo /home/visbot/bin/visquad.sh
"

# Start Gestelt
CMD_1="
docker run --name ros2_container --ipc=host -it --rm --privileged --network host --mount type=bind,src=/tmp,dst=/tmp gestelt/mavoro_arm64:devel ros2 launch gestelt_bringup offboard_uav.py
"

# Start ROS2 Bridge
CMD_2="
docker exec -it ros2_container /ros_entrypoint.sh ros2 run ros2_zmq ros2_zmq_node
"

# Restart VINS estimator
CMD_3="
rostopic pub -1 /vins_estimator/vins_restart geometry_msgs/PoseStamped -f ~/bin/restart.msg
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    sleep 30
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    sleep 15
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

