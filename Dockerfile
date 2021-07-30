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

# https://github.com/dusty-nv/jetson-containers

ARG CUDA=10.2
ARG RELEASE=r32.5

FROM dustynv/ros:foxy-ros-base-l4t-r32.5.0

ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Install CUDA
# https://gitlab.com/nvidia/container-images/l4t-base/-/blob/master/Dockerfile.cuda
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 ca-certificates
# COPY jetson-ota-public.key /etc/jetson-ota-public.key
RUN curl https://gitlab.com/nvidia/container-images/l4t-base/-/raw/master/jetson-ota-public.key -o /etc/jetson-ota-public.key
RUN apt-key add /etc/jetson-ota-public.key
RUN echo "deb https://repo.download.nvidia.com/jetson/common $RELEASE main" >> /etc/apt/sources.list

RUN CUDAPKG=$(echo $CUDA | sed 's/\./-/'); \
    apt-get update && apt-get install -y --no-install-recommends \
	cuda-libraries-$CUDAPKG \
	cuda-nvtx-$CUDAPKG \
	cuda-libraries-dev-$CUDAPKG \
	cuda-minimal-build-$CUDAPKG \
	cuda-license-$CUDAPKG \
	cuda-command-line-tools-$CUDAPKG && \
	ln -s /usr/local/cuda-$CUDA /usr/local/cuda && \
	rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

# Install gstream libraries
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y libglew-dev glew-utils libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libglib2.0-dev && \
    rm -rf /var/lib/apt/lists/*
# Install jetson-utils
RUN cd /opt && \
    git clone https://github.com/dusty-nv/jetson-utils.git && \
    mkdir -p jetson-utils/build && cd jetson-utils/build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install && \
    ldconfig
    
RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh
 
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
