FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    clang \
    clang-format \
    curl \
    lsb-release \
    sudo \
    git \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /home/nitic-robot-club

RUN git clone https://github.com/NITIC-Robot-Club/natto_library.git
WORKDIR /home/nitic-robot-club/natto_library

SHELL ["/bin/bash", "-c"]

RUN sudo apt update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install -y --from-paths src --ignore-src && \

RUN source /opt/ros/jazzy/setup.bash && \
    ./colcon_build.sh