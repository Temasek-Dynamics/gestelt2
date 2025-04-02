# docker build --platform linux/arm64 -t gestelt/mavoro_arm64:latest --push .
# docker run -it  --platform linux/arm64 --rm --privileged --network host  -e "DRONE_ID=0" gestelt/mavoro_arm64:latest

FROM --platform=$BUILDPLATFORM gestelt/mavoro_arm64:base
LABEL maintainer="johntgz@nus.edu.sg"
ARG TARGETPLATFORM
ARG BUILDPLATFORM
RUN echo "I am running on $BUILDPLATFORM, building for $TARGETPLATFORM" 

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 


# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
