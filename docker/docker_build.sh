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

# Check if the file exists
if [ -f "$HOME/.isaac_ros_common-config" ]; then
    color_echo "red" "This script doesn't work. Please cancel ~/.isaac_ros_common-config before to run it."
    exit 1
fi

# Main script
main() {
    local LOCAL_DIR=$(pwd)
    local WS_FOLDERS=()
    local ISAAC_ROS_DISTRO="ros2_humble"

    # Load arguments
    while [[ "$#" -gt 0 ]]; do
        case $1 in
            --ws-src)
                WS_FOLDERS+=($2)
                shift ;;
            --image-name)
                IMAGE_NAME=$2
                shift ;;
            *) color_echo "yellow" "Unknown option: $1" ;;
        esac
        shift
    done

    ISAAC_ROS_BRANCH="release-3.2"
    NANOSAUR_DOCKERFILE="nanosaur"
    # Get the platform
    PLATFORM="$(uname -m)"
    
    # Create the base image key
    BASE_IMAGE_KEY=$PLATFORM.$ISAAC_ROS_DISTRO.$NANOSAUR_DOCKERFILE

    # Setup on-exit cleanup tasks
    ON_EXIT=()
    function cleanup {
        for command in "${ON_EXIT[@]}"; do
            $command
        done
    }
    # trap cleanup EXIT

    ON_EXIT+=("cd $LOCAL_DIR")

    # Setup staging temp directory
    TEMP_DIR=$(mktemp -d -t nanosaur_deploy_XXXXXXXX)
    ON_EXIT+=("rm -Rf ${TEMP_DIR}")

    cd $TEMP_DIR
    echo $(pwd)

    # Copy nanosaur workspaces to temp directory
    mkdir -p "$TEMP_DIR/src"
    for folder in "${WS_FOLDERS[@]}"; do
        echo "Copying $folder to $TEMP_DIR/src"
        cp -r "$folder"/* "$TEMP_DIR/src"
    done

    # Find the isaac_ros_common directory in TEMP_DIR/src
    ISAAC_ROS_COMMON_DIR=$(find "$TEMP_DIR/src" -type d -name "isaac_ros_common" | head -n 1)

    if [ -z "$ISAAC_ROS_COMMON_DIR" ]; then
        color_echo "red" "isaac_ros_common directory not found in $TEMP_DIR/src"
        exit 1
    fi

    # Move to the isaac_ros_common directory
    cd "$ISAAC_ROS_COMMON_DIR"

    # Build the image layers and deploy the image with the launch package
    color_echo "green" "- Create .isaac_ros_common-config in isaac_ros_common/scripts"
    cat <<EOL > scripts/.isaac_ros_common-config
CONFIG_IMAGE_KEY="$ISAAC_ROS_DISTRO.$NANOSAUR_DOCKERFILE"
CONFIG_DOCKER_SEARCH_DIRS=(../../nanosaur_perception)
EOL
    color_echo "green" ".isaac_ros_common-config created successfully in isaac_ros_common/scripts."

    ./scripts/build_image_layers.sh --context_dir $TEMP_DIR --image_key "$BASE_IMAGE_KEY" --image_name "$IMAGE_NAME"

    exit 0
}

# Pass arguments to main function
main "$@"