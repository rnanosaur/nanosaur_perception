#!/bin/bash
# Copyright (C) 2025, Raffaello Bonghi <raffaello@rnext.it>
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

color_echo() {
    local color_name=$1
    local message=$2
    local color_code

    case $color_name in
        "red") color_code="31" ;;
        "green") color_code="32" ;;
        "yellow") color_code="33" ;;
        *) color_code="0" ;; # default to no color
    esac

    echo -e "\e[${color_code}m${message}\e[0m"
}

LOCAL_DIR=$(pwd)
ISAAC_ROS_BRANCH="release-3.2"
NANOSAUR_DOCKERFILE="nanosaur"
# Get the platform
PLATFORM="$(uname -m)"
ISAAC_ROS_DISTRO="ros2_humble"
# create the base image key
BASE_IMAGE_KEY=$PLATFORM.$ISAAC_ROS_DISTRO.$NANOSAUR_DOCKERFILE

# Check if the file exists
if [ -f "$HOME/.isaac_ros_common-config" ]; then
    color_echo "red" "This script doesn't work. Please cancel ~/.isaac_ros_common-config before to run it."
    exit 1
fi

color_echo "green" "- Clone Isaac ROS common"
# Clone Isaac ROS common
if [ ! -d "isaac_ros_common" ]; then
    git clone -b $ISAAC_ROS_BRANCH https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
    touch isaac_ros_common/COLCON_IGNORE
else
    color_echo "yellow" "isaac_ros_common already exists, skipping clone."
fi

color_echo "green" "- Create .isaac_ros_common-config in isaac_ros_common/scripts"
# Always recreate .isaac_ros_common-config in isaac_ros_common/scripts
cat <<EOL > isaac_ros_common/scripts/.isaac_ros_common-config
CONFIG_IMAGE_KEY="$ISAAC_ROS_DISTRO.$NANOSAUR_DOCKERFILE"
CONFIG_DOCKER_SEARCH_DIRS=(../../)
EOL
color_echo "green" ".isaac_ros_common-config created successfully in isaac_ros_common/scripts."

# Build the image layers and deploy the image with the launch package
cd isaac_ros_common

# Not sure if this is necessary
# ./scripts/build_image_layers.sh --context_dir $LOCAL_DIR --image_key "$BASE_IMAGE_KEY" # --image_name "$PLATFORM-nanosaur"

# Deploy the image with the launch package
./scripts/docker_deploy.sh \
    --base_image_key $BASE_IMAGE_KEY \
    --launch_package "nanosaur_perception" \
    --launch_file "perception.launch.py" \
    --name "nanosaur"

cd $LOCAL_DIR
# EOF
