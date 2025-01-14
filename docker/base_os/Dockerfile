# docker buildx build --platform linux/arm64 -t gestelt/mavoro_arm64:base_os --push .

FROM --platform=linux/arm64 ros:jazzy-perception-noble
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    curl \
    wget \
    iproute2 \
    build-essential \
    software-properties-common \
    python3-pip \
    python3-vcstool \
    nlohmann-json3-dev \
    libasio-dev \
    libeigen3-dev \
    ros-jazzy-mavros* \
    ros-jazzy-message-filters \
    ros-jazzy-mavlink \
    ros-jazzy-geographic-msgs \ 
    ros-jazzy-geometry* \
    ros-jazzy-tf2* \
    geographiclib-tools \
    libgeographiclib-dev \
    libgeographiclib26 \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \ 
    && bash ./install_geographiclib_datasets.sh

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
