# .bashrc used by VISBOT

# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
source /opt/ros/noetic/setup.bash
source /home/visbot/ros_ws/install/setup.bash
source /home/visbot/ros_ws/devel/setup.bash
source /home/visbot/ros1_ws/devel/setup.bash

# ROS2
#source /opt/ros/foxy/setup.bash
#export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_config.xml

# ROS1 IP
export MASTER_IP=10.42.0.34
export SELF_IP=10.42.0.34

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

export DRONE_ID=0

alias START_EMPTY_4="./gestelt_startup.sh -i $DRONE_ID -s start_4d_empty"
alias START_OBS_3="./gestelt_startup.sh -i $DRONE_ID -s start_3d_obs"
alias START_OBS_4="./gestelt_startup.sh -i $DRONE_ID -s start_4d_obs"

# Shortcuts
alias killbill="sudo killall roslaunch; sudo killall -9 roscore; sudo killall -9 rosmaster; tmux kill-server;"
alias kill_all_containers="docker stop $(docker ps -a -q)"

# Gestelt commands
alias update_gestelt="docker pull gestelt/mavoro_arm64:devel"
alias enter_gestelt="docker exec -it ros2_container /ros_entrypoint.sh bash"
alias dbg_gestelt="docker run --name ros2_container --ipc=host -it --platform linux/arm64 --privileged --network host --mount type=bind,src=/tmp,dst=/tmp gestelt/mavoro_arm64:devel"
alias compile_bridge_gestelt="docker run --name ros2_container -it --platform linux/arm64 --privileged --ipc=host --network host --mount type=bind,src=/opt/ros/noetic,dst=/opt/ros/noetic gestelt/mavoro_arm64:devel"
alias stop_gestelt="docker stop ros2_container"
alias rm_gestelt="docker rm ros2_container"
alias save_gestelt="docker commit ros2_container gestelt/mavoro_arm64:devel"

# ROS1 Bridge commands
alias start_bridge="roslaunch ros_zmq ros_zmq.launch"
alias ntp_restart="sudo service ntp stop && sudo ntpd -gq && sudo service ntp start"

# visbot commands
alias vins_restart="rostopic pub -1 /vins_estimator/vins_restart geometry_msgs/PoseStamped -f ~/bin/restart.msg"
alias visbot_restart="sudo /home/visbot/bin/visquad.sh"
alias visbot_record="/home/visbot/ros_ws/src/visbot_scripts/data_record/data_record.sh"
alias visbot_record_kill="/home/visbot/ros_ws/src/visbot_scripts/data_record/data_record_kill.sh"
alias visbot_rosbag_record="/home/visbot/ros_ws/src/visbot_scripts/rosbag_record/rosbag_record.sh"
alias visbot_rosbag_record_kill="/home/visbot/ros_ws/src/visbot_scripts/rosbag_record/rosbag_record_kill.sh"
alias visbot_raw_record="/home/visbot/ros_ws/src/visbot_scripts/raw_data_record/vio_raw_data_record.sh"
