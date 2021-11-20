# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Jetpack 4.6
# Docker file for aarch64 based Jetson device
ARG BASE_IMAGE="dustynv/ros:galactic-ros-base-l4t-r32.6.1"
FROM ${BASE_IMAGE}
# Configuration CUDA
ARG CUDA=10.2
ARG L4T=r32.6

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    apt-utils python3-vcstool python3-pip \
    libglew-dev glew-utils libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libglib2.0-dev \
    libjpeg-dev zlib1g-dev

# Error with /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so
RUN git clone https://github.com/dusty-nv/jetson-utils.git /opt/jetson-utils && \
    # Build jetson-utils
    mkdir -p /opt/jetson-utils/build && \
    cd /opt/jetson-utils && \
    git checkout 43c04d6330c3410d9c5f63e311c9653dbbe4e192 && \
    cd /opt/jetson-utils/build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install && \
    ldconfig

################ NANOSAUR PKGS ####################

# Download and build nanosaur_perception
ENV ROS_WS /opt/ros_ws
# Copy wstool isaac_ros.rosinstall
COPY nanosaur_perception/rosinstall/perception.rosinstall perception.rosinstall

RUN mkdir -p ${ROS_WS}/src && \
    vcs import ${ROS_WS}/src < perception.rosinstall && \
    rm -rf /var/lib/apt/lists/*

# Change workdir
WORKDIR $ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ Final enviroment setup ####################

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_perception", "perception.launch.py"]