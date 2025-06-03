#!/bin/bash

usage() { echo "Usage: $0 [-s <scenario_name>]" 1>&2; exit 1; }

SESSION="gcs_startup"
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
# Start QGroundControl 
QGC="~/Documents/QGroundControl.AppImage"

# Start Rviz
RVIZ="ros2 launch gestelt_bringup rviz_viz.py"

# Start mission node
MISSION_NODES="ros2 launch gestelt_bringup test_take_off_point_goal.py scenario_name:=single_drone_test"

# Reset map
RESET_MAP="ros2 topic pub /reset_map std_msgs/msg/Empty {} -5"

# Land
LAND_NODES="ros2 run gestelt_commander land scenario_name:=single_drone_test"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$QGC" C-m 
    tmux send-keys -t $SESSION:0.1 "$RVIZ" C-m 
    tmux send-keys -t $SESSION:0.2 "$MISSION_NODES" 
    tmux send-keys -t $SESSION:0.3 "$RESET_MAP" 
    tmux send-keys -t $SESSION:0.4 "$LAND_NODES" 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

