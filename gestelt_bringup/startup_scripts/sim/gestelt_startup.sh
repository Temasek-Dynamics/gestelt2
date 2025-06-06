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
MULTI_DRONE_SIM="ros2 launch gestelt_bringup multi_drone_sim_launch.py "

# Start gestelt nodes
MISSION_NODE="ros2 launch gestelt_bringup test_point_goal_sim.py"

QGC="~/Documents/QGroundControl.AppImage"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h

    tmux send-keys -t $SESSION:0.0 "$MULTI_DRONE_SIM" C-m 
    tmux send-keys -t $SESSION:0.1 "$MISSION_NODE" C-m 
    tmux send-keys -t $SESSION:0.2 "$QGC" C-m 
    # tmux send-keys -t $SESSION:0.3 "$QGC" C-m 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

