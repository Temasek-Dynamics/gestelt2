#!/bin/bash

SESSION="gestelt_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Commands
#####
# Start gcs node
GCS_NODES="ros2 launch gestelt_bringup gcs.py"

# Start Zenoh
ZENOH_BRIDGE="zenoh-bridge-ros2dds -c ~/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5"

# Start mission node
MISSION_NODES="ros2 launch gestelt_bringup execute_mission.py scenario_name:=empty2"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h

    tmux send-keys -t $SESSION:0.0 "$GCS_NODES" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$ZENOH_BRIDGE" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.2 "$MISSION_NODES" 
    sleep 2
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

