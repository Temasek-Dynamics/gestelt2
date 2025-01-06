FROM osrf/ros:jazzy-desktop-full
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    wget \
    iproute2 \
    python3-vcstool \
    build-essential \
    nlohmann-json3-dev \
    libasio-dev \
    libeigen3-dev \
    ros-jazzy-pcl-ros \ 
    ros-jazzy-pcl-conversions \
    ros-jazzy-message-filters \
    ros-jazzy-mavlink \
    ros-jazzy-geographic-msgs \ 
    geographiclib-tools \
    libgeographiclib-dev \
    libgeographiclib26 \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \ 
    && bash ./install_geographiclib_datasets.sh

# Install OSQP and OSQP-eigen for Linear MPC

WORKDIR /root/
RUN git clone https://github.com/osqp/osqp.git \
    && cd osqp \
    && mkdir build \
    && cd build \
    && cmake -G "Unix Makefiles" .. \
    && cmake --build . \
    && cmake --build . --target install

RUN git clone https://github.com/robotology/osqp-eigen.git \
    && cd osqp-eigen \
    && mkdir build \
    && cd build \
    && cmake ../ \
    && make \
    && make install 

# Make directory and clone repository
WORKDIR /gestelt_ws/src/
RUN git clone https://github.com/Temasek-Dynamics/gestelt2 -b mavros_interface 
WORKDIR /gestelt_ws/src/gestelt2
RUN vcs import < thirdparty.repos --recursive
WORKDIR /gestelt_ws/
RUN source /opt/ros/jazzy/setup.bash \
    && export OsqpEigen_DIR==/usr/local/lib/cmake/OsqpEigen/ \
    && colcon build --symlink-install --packages-ignore fake_drone pcd_map_generator swarm_collision_checker

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
