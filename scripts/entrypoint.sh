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


echo "Starting entrypoint.sh"
echo "User: $USER"
echo "Username: $USERNAME"
echo "UID: $HOST_USER_UID"
echo "GID: $HOST_USER_GID"

if [ ! $(getent group ${HOST_USER_GID}) ]; then
    echo "Creating non-root container '${USER}' for host user uid=${HOST_USER_UID}:gid=${HOST_USER_GID}"
    groupadd --gid ${HOST_USER_GID} ${USER} &>/dev/null
else
    echo "Group with GID ${HOST_USER_GID} already exists. Checking group name..."
    CONFLICTING_GROUP_NAME=`getent group ${HOST_USER_GID} | cut -d: -f1`
    groupmod -o --gid ${HOST_USER_GID} -n ${USERNAME} ${CONFLICTING_GROUP_NAME}
fi

if [ ! $(getent passwd ${HOST_USER_UID}) ]; then
    echo "User with UID ${HOST_USER_UID} does not exist. Creating user '${USER}'"
    useradd --no-log-init --uid ${HOST_USER_UID} --gid ${HOST_USER_GID} -m ${USER} &>/dev/null
else
    echo "User with UID ${HOST_USER_UID} already exists. Checking username..."
    CONFLICTING_USER_NAME=`getent passwd ${HOST_USER_UID} | cut -d: -f1`
    usermod -l ${USERNAME} -u ${HOST_USER_UID} -m -d /home/${USERNAME} ${CONFLICTING_USER_NAME} &>/dev/null
    mkdir -p /home/${USERNAME}
    # Wipe files that may create issues for users with large uid numbers.
    rm -f /var/log/lastlog /var/log/faillog
fi

# Create user home directory if it does not exist
if [ ! -d /home/${USERNAME} ]; then
    mkdir -p /home/${USERNAME}
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}
fi

# Update 'admin' user
chown ${USERNAME}:${USERNAME} /home/${USERNAME}
echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME}
chmod 0440 /etc/sudoers.d/${USERNAME}
adduser ${USERNAME} video >/dev/null
adduser ${USERNAME} plugdev >/dev/null
adduser ${USERNAME} sudo  >/dev/null

# If jtop present, give the user access
if [ -S /run/jtop.sock ]; then
    JETSON_STATS_GID="$(stat -c %g /run/jtop.sock)"
    addgroup --gid ${JETSON_STATS_GID} jtop >/dev/null
    adduser ${USERNAME} jtop >/dev/null
fi

# Source ROS workspace if exists
if [[ ! -z "${ROS_WS}" ]]; then
    source ${ROS_WS}/install/setup.bash
    echo "ROS workspace sourced: ${ROS_WS}"
fi

# Source Isaac SDK workspace if exists
if [ "$1" == "bash" ]; then
    shift
    exec bash "$@"
elif [[ ! -z "${PERCEPTION_ARGS}" ]]; then
    echo "Starting perception with arguments: ${PERCEPTION_ARGS}"
    exec gosu ${USERNAME} ros2 launch ${PERCEPTION_PACKAGE:-nanosaur_perception} ${PERCEPTION_LAUNCH_FILE:-perception.launch.py} ${PERCEPTION_ARGS}
else
    echo "Starting perception with commands: $@"
    exec gosu ${USERNAME} ros2 launch ${PERCEPTION_PACKAGE:-nanosaur_perception} ${PERCEPTION_LAUNCH_FILE:-perception.launch.py} $@
fi
