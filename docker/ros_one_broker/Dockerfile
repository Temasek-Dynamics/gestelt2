# docker buildx build --platform linux/arm64 -t gestelt/ros_one_broker:latest --push .
# Source: https://github.com/osrf/docker_images/blob/27cc0b68263bbbb10bb58dd814efc0a6b0a01ec7/ros/noetic/ubuntu/focal/ros-core/Dockerfile

# docker run -it --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/ros_one_broker:latest

FROM --platform=$TARGETPLATFORM ros:noetic-ros-core
LABEL maintainer="johntgz@nus.edu.sg"
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" 

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    git \
    wget \
    curl \
    python3-pip \
    build-essential \
    software-properties-common \
    ros-noetic-mavros* \
    && rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y

RUN pip install protobuf
RUN source $HOME/.cargo/env && pip install eclipse-zenoh

WORKDIR /root
RUN git clone https://github.com/JohnTGZ/ros_zenoh_exchange.git -b master 

COPY .bashrc /root/

ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
# ENTRYPOINT "source /.bashrc && cd /ros_zenoh_exchange && python3 ros1_broker.py"

CMD ["bash"]
