#!/bin/bash

SESSION="gcs_startup"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Commands
#####
# Start gcs node
GCS_NODES="ros2 launch gestelt_bringup gcs.py"

# Start Zenoh
ZENOH_BRIDGE="zenoh-bridge-ros2dds -c ~/gestelt_ws/src/gestelt2/gestelt_network/zenoh_host_cfg.json5"

# Start mission node
MISSION_NODES="ros2 launch gestelt_bringup execute_mission.py scenario_name:=start_2d"

# Send goals node
SEND_GOAL_NODES="ros2 launch gestelt_bringup execute_send_goals.py scenario_name:=goals_1d_0"

# Reset map 
RESET_MAP="ros2 topic pub /d0/reset_map std_msgs/msg/Empty {} -1 && ros2 topic pub /d2/reset_map std_msgs/msg/Empty {} -1"

# Land
LAND_NODES="ros2 launch gestelt_bringup execute_land.py scenario_name:=start_2d"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$GCS_NODES" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$ZENOH_BRIDGE" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$MISSION_NODES" 
    tmux send-keys -t $SESSION:0.3 "$SEND_GOAL_NODES" 
    tmux send-keys -t $SESSION:0.4 "$RESET_MAP" 
    tmux send-keys -t $SESSION:0.4 "$LAND_NODES" 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

