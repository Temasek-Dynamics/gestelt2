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

# Start gestelt nodes (Add alias into bashrc)
GESTELT_NODE="start_offboard"

# Start mission node
MISSION_NODES="start_takeoff"

# Record bag
RECORD_ROSBAG="ros2 launch gestelt_bringup record_rosbag_local.py"

# Preset goals
AUTO_GOAL="ros2 launch gestelt_bringup drone_goal_manager_launch.py"

# Start zenoh bridge (Add alias into bashrc)
ZENOH_BRIDGE="start_zenoh"

# Start no namespace zenoh bridge (Add alias into bashrc)
ZENOH_NONS="start_zenoh_noNS"

# Start zenoh peer to peer (Add alias into bashrc)
ZENOH_P2P="start_zenoh_p2p"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.2 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.5 -h
    tmux split-window -t $SESSION:0.2 -h

    tmux send-keys -t $SESSION:0.5 "$ZENOH_BRIDGE" C-m
    tmux send-keys -t $SESSION:0.6 "$ZENOH_P2P" C-m
    tmux send-keys -t $SESSION:0.7 "$ZENOH_NONS" C-m
    sleep 5
    tmux send-keys -t $SESSION:0.0 "$VILOTA_BRIDGE" C-m
    sleep 1
    tmux send-keys -t $SESSION:0.1 "$GESTELT_NODE" C-m
    tmux send-keys -t $SESSION:0.2 "$MISSION_NODES"
    tmux send-keys -t $SESSION:0.4 "$RECORD_ROSBAG"
    tmux send-keys -t $SESSION:0.3 "$AUTO_GOAL"
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"

