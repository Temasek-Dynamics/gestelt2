# docker build --platform linux/arm64 -t gestelt/mavoro_arm64:latest --push .
# docker run -it  --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro_arm64:latest

FROM --platform=$BUILDPLATFORM gestelt/mavoro_arm64:base
LABEL maintainer="johntgz@nus.edu.sg"
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" 

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Make directory and clone repository
WORKDIR /gestelt_ws/src/gestelt2
RUN git pull
WORKDIR /gestelt_ws/
RUN source /opt/ros/jazzy/setup.bash \
    && export OsqpEigen_DIR=/usr/local/lib/cmake/OsqpEigen/ \
    && colcon build --symlink-install --packages-ignore fake_drone pcd_map_generator swarm_collision_checker ikd_tree decomp_ros_utils

WORKDIR /ros_zenoh_exchange
RUN git pull

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
