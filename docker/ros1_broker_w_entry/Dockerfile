# docker buildx build --platform linux/arm64 -t gestelt/ros_one_broker:devel --push .
# Source: https://github.com/osrf/docker_images/blob/27cc0b68263bbbb10bb58dd814efc0a6b0a01ec7/ros/noetic/ubuntu/focal/ros-core/Dockerfile

# docker run -it --platform linux/arm64 --rm --privileged --network host gestelt/ros_one_broker:devel

FROM --platform=$TARGETPLATFORM gestelt/ros_one_broker:base
LABEL maintainer="johntgz@nus.edu.sg"
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" 

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

ENV DRONE_ID=0

COPY ros_entrypoint.sh /
COPY .bashrc /root/

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
