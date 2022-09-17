# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
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

# Isaac ROS Dev Base
# https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/ros
FROM nvcr.io/nvidia/isaac/ros:aarch64-humble-nav2_35120f31b84976ec36009722e16f1524
# L4T variables
ENV L4T=35.1
ENV L4T_MINOR_VERSION=1.0
# Configuration CUDA
ENV CUDA=10.2

################ INSTALL ISAAC ROS ####################

# Download and build nanosaur_isaac_ros
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
# Copy wstool isaac_ros.rosinstall
COPY nanosaur_perception/rosinstall/isaac_ros.rosinstall isaac_ros.rosinstall

RUN mkdir -p ${ISAAC_ROS_WS}/src && \
    vcs import ${ISAAC_ROS_WS}/src < isaac_ros.rosinstall && \
    rm -rf /var/lib/apt/lists/*
# Pull LFS files
COPY nanosaur_perception/scripts/git_lfs_pull_ws.sh git_lfs_pull_ws.sh
RUN TERM=xterm bash git_lfs_pull_ws.sh ${ISAAC_ROS_WS}/src

# Change workdir
WORKDIR $ISAAC_ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ NANOSAUR PKGS ####################

# Download and build nanosaur_perception
ENV ROS_WS /opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

ADD . $ROS_WS/src/nanosaur_perception

# Change workdir
WORKDIR $ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    . $ISAAC_ROS_WS/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ Final enviroment setup ####################

# Restore using the default Foxy DDS middleware: FastRTPS
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

STOPSIGNAL SIGINT
# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_perception", "perception.launch.py"]