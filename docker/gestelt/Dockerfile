FROM osrf/ros:jazzy-desktop-full
LABEL maintainer="johntgz@nus.edu.sg"

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    iproute2 \
    python3-vcstool \
    build-essential \
    libeigen3-dev \
    ros-jazzy-pcl-ros \ 
    ros-jazzy-pcl-conversions \
    ros-jazzy-message-filters \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \ 
    && bash ./install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Make directory and clone repository
WORKDIR /gestelt_ws/src/
RUN git clone https://github.com/Temasek-Dynamics/gestelt2 -b master 
WORKDIR /gestelt_ws/src/gestelt2
RUN vcs import < thirdparty.repos --recursive
WORKDIR /gestelt_ws/
RUN source /opt/ros/noetic/setup.bash \
    && colcon build --symlink-install 

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
