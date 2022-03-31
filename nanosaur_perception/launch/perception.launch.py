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
import sys

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from service import load_config


def generate_launch_description():
    pkg_perception = get_package_share_directory('nanosaur_perception')

    config_common_path = LaunchConfiguration('config_common_path')
    namespace = LaunchConfiguration('namespace')
    cover_type = LaunchConfiguration('cover_type')

    nanosaur_config_path = os.path.join(
        pkg_perception, 'param', 'nanosaur.yml')
    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_perception, 'param', 'robot.yml'))
    # Load namespace from robot.yml
    namespace_conf = os.getenv("HOSTNAME") if conf.get("multirobot", False) else ""
    # Load cover_type
    if "cover_type" in conf:
        cover_type_conf = conf.get("cover_type", 'fisheye')
        print(f"Load cover_type from robot.xml: {cover_type_conf}")
    else:
        cover_type_conf = os.getenv("COVER_TYPE", 'fisheye')
        print(f"Load cover_type from ENV: {cover_type_conf}")

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value=cover_type_conf,
        description='Cover type to use. Options: pi, fisheye, realsense, zedmini.')

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=nanosaur_config_path,
        description='Path to the `nanosaur.yml` file.')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=namespace_conf,
        description='Enable a namespace for multiple robot. This namespace group all nodes and topics.')

    # Nanosaur mipi camera
    mipi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_perception, '/launch/mipi.launch.py']),
        launch_arguments={'cover_type': cover_type}.items())

    # Nanosaur realsense camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_perception, '/launch/realsense.launch.py']))

    ld = LaunchDescription()

    ld.add_action(declare_cover_type_cmd)
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_namespace_cmd)
    
    if cover_type_conf == 'pi' or cover_type_conf == 'fisheye':
        ld.add_action(mipi_launch)
    elif cover_type_conf == 'realsense':
        ld.add_action(realsense_launch)
    else:
        print(f"Cover not in list! Name: {cover_type_conf}")
    
    return ld
# EOF
