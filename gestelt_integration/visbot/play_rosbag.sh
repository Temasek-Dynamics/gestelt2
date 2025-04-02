#!/bin/bash

SESSION="gcs_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Get arguments
#####
# getopts: function to read flags in input
# OPTARG: refers to corresponding values
while getopts n: flag
do
    case "${flag}" in
        n) BAGFILE=${OPTARG};; 
    esac
done


#####
# Commands
#####
RVIZ="ros2 launch gestelt_bringup rosbag_player.py"

ROSBAG_PLAYER="ros2 bag play ~/bag_files/${BAGFILE}/${BAGFILE}.mcap"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h

    tmux send-keys -t $SESSION:0.0 "$RVIZ" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.1 "$ROSBAG_PLAYER" 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

