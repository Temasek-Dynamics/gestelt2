#!/bin/bash

# Usage: ./gestelt_startup.sh -i <DRONE_ID>

SESSION="relay_tf"
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
relay_tf1=" \
ros2 run topic_tools relay /d0/tf /tf \
"

relay_tf2=" \
ros2 run topic_tools relay /d1/tf /tf \
"

relay_static_tf1=" \
ros2 run topic_tools relay /d0/tf_static /tf_static \
"

# Start Zenoh bridge 
relay_static_tf2=" \
ros2 run topic_tools relay /d1/tf_static /tf_static \
"


if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$relay_tf1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$relay_tf2" C-m 
    sleep 1
    # ROS1 bridge needs to start a while after ROS1 nodes as ROS1_NODES script will terminate ros1 nodes at the start
    tmux send-keys -t $SESSION:0.1 "$relay_static_tf1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$relay_static_tf2" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

