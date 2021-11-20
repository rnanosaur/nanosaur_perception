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

import os
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_config(config):
    if os.path.isfile(config):
        
        with open(config, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    return {}


def generate_launch_description():

    pkg_bringup = FindPackageShare(package='nanosaur_bringup').find('nanosaur_bringup')

    nanosaur_config = os.path.join(pkg_bringup, 'param', 'nanosaur.yml')
    nanosaur_dir = LaunchConfiguration('nanosaur_dir', default=nanosaur_config)

    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_bringup, 'param', 'robot.yml'))
    # Load namespace
    namespace = os.getenv("HOSTNAME") if conf.get("multirobot", False) else ""

    nanosaur_camera_node = Node(
        package='nanosaur_camera',
        namespace=namespace,
        executable='nanosaur_camera',
        name='nanosaur_camera',
        respawn=True,
        respawn_delay=5,
        parameters=[nanosaur_dir] if os.path.isfile(nanosaur_config) else [],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'nanosaur_dir',
            default_value=nanosaur_dir,
            description='Full path to nanosaur parameter file to load'),
        nanosaur_camera_node
        ])
# EOF
