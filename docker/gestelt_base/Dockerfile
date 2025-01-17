# docker buildx build --platform linux/arm64 -t gestelt/mavoro_arm64:base --push .
# docker run -it  --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro_arm64:latest

FROM --platform=$TARGETPLATFORM ros:jazzy-perception-noble
LABEL maintainer="johntgz@nus.edu.sg"
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" 

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    curl \
    wget \
    build-essential \
    software-properties-common \
    python3-pip \
    nlohmann-json3-dev \
    libasio-dev \
    libeigen3-dev \
    ros-jazzy-mavros* \
    ros-jazzy-mavlink \
    ros-jazzy-message-filters \
    ros-jazzy-geographic-msgs \ 
    ros-jazzy-geometry* \
    ros-jazzy-tf2* \
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
    && cmake --build . --target install

RUN git clone https://github.com/robotology/osqp-eigen.git \
    && cd osqp-eigen \
    && mkdir build \
    && cd build \
    && cmake ../ \
    && make \
    && make install 

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y

RUN pip install protobuf --break-system-packages
# RUN source $HOME/.cargo/env && pip install eclipse-zenoh --break-system-packages

# Make directory and clone repository
# WORKDIR /gestelt_ws/src/
# RUN git clone https://github.com/Temasek-Dynamics/gestelt2 -b mavros_interface 
# WORKDIR /gestelt_ws/
# RUN source /opt/ros/jazzy/setup.bash \
#     && export ROS_DOMAIN_ID=0 \
#     && export OsqpEigen_DIR=/usr/local/lib/cmake/OsqpEigen/ \
#     && colcon build --symlink-install --packages-ignore fake_drone pcd_map_generator swarm_collision_checker ikd_tree decomp_ros_utils

# WORKDIR /root
# RUN git clone https://github.com/JohnTGZ/ros_zenoh_exchange.git -b master 

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
# ENTRYPOINT "source /.bashrc && ros2 launch gestelt_bringup offboard_uav.py"

CMD ["bash"]
