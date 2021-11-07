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
FROM dustynv/ros:foxy-ros-base-l4t-r32.6.1

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install Git-LFS
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
        apt-get update && apt-get install -y \
        git-lfs \
&& rm -rf /var/lib/apt/lists/*

COPY scripts/cuda_info.txt cuda_info.txt
RUN echo "$(cat cuda_info.txt)" >> /var/lib/dpkg/status

# Add nvidia repo/public key and install VPI libraries
RUN curl https://repo.download.nvidia.com/jetson/jetson-ota-public.asc > /etc/apt/trusted.gpg.d/jetson-ota-public.asc \
    && echo "deb https://repo.download.nvidia.com/jetson/common r32.6 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && echo "deb https://repo.download.nvidia.com/jetson/t194 r32.6 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && apt-get update \
    && apt-get install -y \
        libnvvpi1 \
        vpi1-dev

# Update environment
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-10.2/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"

########### INSTALL ISAAC ROS ###########

# Download and build nanosaur_isaac_ros
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
# Copy wstool isaac_ros.rosinstall
COPY isaac_ros.rosinstall isaac_ros.rosinstall

RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    mkdir -p ${ISAAC_ROS_WS}/src && \
    vcs import ${ISAAC_ROS_WS}/src < isaac_ros.rosinstall && \
    rm -rf /var/lib/apt/lists/*
# Pull LFS files
RUN cd ${ISAAC_ROS_WS}/src/isaac_ros_common && git lfs pull

RUN cd ${ISAAC_ROS_WS} && \
    . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ISAAC_ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_isaac_follower", "isaac_follower.launch.py"]