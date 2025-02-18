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

import os
import re
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory


PATTERN_COMMAND = re.compile(r'(\w+):=\s*(\[[^\]]+\]|".+?"|\S+)')

def docker_decoder():
    print("[WARNING] Docker environment detected.")
    commands = os.environ['NANOSAUR_COMMANDS']
    return {
        match.group(1): match.group(2)
        for match in PATTERN_COMMAND.finditer(commands)
    }

def launch_perception_setup(context: LaunchContext, support_robot_name, support_camera_type, support_lidar_type, support_engines, support_use_sim_time):
    pkg_perception = get_package_share_directory('nanosaur_perception')
    engines = list(set(context.perform_substitution(support_engines).strip('[]').replace(' ', '').split(',')))

    perception_config = {
        'robot_name': context.perform_substitution(support_robot_name),
        'camera_type': context.perform_substitution(support_camera_type),
        'lidar_type': context.perform_substitution(support_lidar_type),
        'use_sim_time': context.perform_substitution(support_use_sim_time),
    }

    if 'NANOSAUR_COMMANDS' in os.environ:
        perception_config |= docker_decoder()
        if 'engines' in perception_config:
            engines = re.findall(r'\w+', perception_config['engines'])

    # Get engines list from launch file argument 
    nodes_list = []
    # Start all engines
    print(f"Engines: {engines}")
    for engine in engines:
        if engine == 'vslam':
            print("Start VSLAM engine")
            vslam_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_perception, 'launch', 'vslam.launch.py')]),
                launch_arguments=perception_config.items(),
            )
            nodes_list += [vslam_launch]
        else:
            print(f"Engine {engine} not found")

    return nodes_list

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    camera_type = LaunchConfiguration('camera_type')
    lidar_type = LaunchConfiguration('lidar_type')
    engines = LaunchConfiguration('engines')

    robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='nanosaur',
        description='robot name (namespace). If you are working with multiple robot you can change this parameter.')

    declare_camera_type_cmd = DeclareLaunchArgument(
        name='camera_type',
        default_value='empty',
        description='camera type to use. Options: empty, Realsense, zed.')

    declare_lidar_type_cmd = DeclareLaunchArgument(
        name='lidar_type',
        default_value='empty',
        description='Lidar type to use. Options: empty, LD06.')

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_engines_cmd = DeclareLaunchArgument(
        name='engines',
        default_value='',
        description='Load engines. Options: vslam, nvblox, apriltag, yolov4')


    # Define LaunchDescription variable and return it
    ld = LaunchDescription()
    ld.add_action(robot_name_cmd)
    ld.add_action(declare_camera_type_cmd)
    ld.add_action(declare_lidar_type_cmd)
    ld.add_action(declare_engines_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=launch_perception_setup, args=[robot_name, camera_type, lidar_type, engines, use_sim_time]))

    return ld