#!/bin/bash

SESSION="offboard"
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
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."

#####
# Sourcing
#####


#####
# Commands
#####
SSH_TO_DRONE="
sshd${DRONE_ID}
"

# Start up script to send commands
EXEC_MISSION="source ~/.bashrc && ros2 launch gestelt_bringup execute_mission.py"

# Start ground control visualization
START_GCS="source ~/.bashrc && ros2 launch gestelt_bringup gcs.py"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SSH_TO_DRONE" C-m 
    tmux send-keys -t $SESSION:0.1 "$SSH_TO_DRONE" C-m 
    tmux send-keys -t $SESSION:0.2 "$EXEC_MISSION" 
    tmux send-keys -t $SESSION:0.3 "$START_GCS" C-m

    sleep 5

    tmux send-keys -t $SESSION:0.0 "start_gestelt"
    tmux send-keys -t $SESSION:0.1 "start_ros_one_broker"
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
