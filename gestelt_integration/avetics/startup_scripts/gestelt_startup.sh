#!/bin/bash

usage() { echo "Usage: $0 [-s <scenario_name>]" 1>&2; exit 1; }

SESSION="gestelt_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Get arguments
#####
# getopts: function to read flags in input
# OPTARG: refers to corresponding values
while getopts "s:" o; 
do
    case "${o}" in
        s) 
            SCENARIO_NAME=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

#####
# Commands
#####
# Start vilota bridge
VILOTA_BRIDGE="ros2 launch gestelt_bringup vilota_launch.py"

# VILOTA_VIO_ONLY="ros2 run vision vio_bridge_px4"

# Start gestelt nodes
GESTELT_NODE="ros2 launch gestelt_bringup offboard_launch.py "

# Start mission node on OBC
MISSION_NODES="ros2 launch gestelt_bringup test_take_off_point_goal.py scenario_name:=single_drone_test"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h

    tmux send-keys -t $SESSION:0.0 "$VILOTA_BRIDGE" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.1 "$GESTELT_NODE" C-m
    tmux send-keys -t $SESSION:0.2 "$MISSION_NODES"  
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

