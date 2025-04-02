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
relay_tf0=" \
ros2 run topic_tools relay /d0/tf /tf \
"
relay_tf1=" \
ros2 run topic_tools relay /d1/tf /tf \
"
relay_tf2=" \
ros2 run topic_tools relay /d2/tf /tf \
"
relay_tf3=" \
ros2 run topic_tools relay /d3/tf /tf \
"

relay_static_tf0=" \
ros2 run topic_tools relay /d0/tf_static /tf_static \
"
relay_static_tf1=" \
ros2 run topic_tools relay /d1/tf_static /tf_static \
"
relay_static_tf2=" \
ros2 run topic_tools relay /d2/tf_static /tf_static \
"
relay_static_tf3=" \
ros2 run topic_tools relay /d3/tf_static /tf_static \
"

if [ "$SESSIONEXISTS" = "" ]
then 
    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -v
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.4 -h
    tmux split-window -t $SESSION:0.1 -v
    tmux split-window -t $SESSION:0.0 -v

    tmux send-keys -t $SESSION:0.0 "$relay_tf0" C-m 
    tmux send-keys -t $SESSION:0.1 "$relay_tf1" C-m 
    tmux send-keys -t $SESSION:0.2 "$relay_tf2" C-m 
    tmux send-keys -t $SESSION:0.3 "$relay_tf3" C-m 
    tmux send-keys -t $SESSION:0.4 "$relay_static_tf0" C-m 
    tmux send-keys -t $SESSION:0.5 "$relay_static_tf1" C-m 
    tmux send-keys -t $SESSION:0.6 "$relay_static_tf2" C-m 
    tmux send-keys -t $SESSION:0.7 "$relay_static_tf3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

