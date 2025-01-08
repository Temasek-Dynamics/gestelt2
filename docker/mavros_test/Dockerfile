FROM ros:jazzy-perception-noble
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
    ros-jazzy-eigen-stl-containers \
    ros-jazzy-eigen3-cmake-module \
    ros-jazzy-tf2-eigen \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-geometry* \
    ros-jazzy-tf2* \
    ros-jazzy-mavros* \
    libconsole-bridge-dev \
    geographiclib-tools \
    libgeographiclib-dev \
    libgeographiclib26 \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \ 
    && bash ./install_geographiclib_datasets.sh

# # Install OSQP and OSQP-eigen for Linear MPC

# # Make directory and clone repository
# WORKDIR /ros2_ws/src/
# RUN git clone https://github.com/JohnTGZ/mavros_docker_test.git -b master 
# WORKDIR /ros2_ws/
# RUN source /opt/ros/jazzy/setup.bash \
#     && colcon build --symlink-install 

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
