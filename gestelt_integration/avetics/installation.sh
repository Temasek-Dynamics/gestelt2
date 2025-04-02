#!/bin/bash

sudo apt-get update && sudo apt-get install --no-install-recommends -y \
    vim \
    curl \
    wget \
    tmux \
    build-essential \
    software-properties-common \
    python3-pip \
    python3-vcstool \
    nlohmann-json3-dev \
    libasio-dev \
    libeigen3-dev \
    ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-message-filters \
    ros-$ROS_DISTRO-geographic-msgs \ 
    ros-$ROS_DISTRO-geometry* \
    ros-$ROS_DISTRO-tf2* \
    ros-$ROS_DISTRO-nav-2d-utils