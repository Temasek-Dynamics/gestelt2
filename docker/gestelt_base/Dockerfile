FROM --platform=linux/arm64 gestelt/mavoro_arm64:base_os
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

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

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
