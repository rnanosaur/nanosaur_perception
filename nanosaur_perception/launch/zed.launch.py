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
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from service import ConfDetector, load_config


def generate_launch_description():
    pkg_perception = get_package_share_directory('nanosaur_perception')
    pkg_zed = get_package_share_directory('zed_wrapper')

    namespace_camera = LaunchConfiguration('namespace_camera')
    config_common_path = LaunchConfiguration('config_common_path')

    nanosaur_config_path = os.path.join(
        pkg_perception, 'param', 'nanosaur.yml')

    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_perception, 'param', 'robot.yml'))
    
    # Load namespace from robot.yml
    namespace_conf = os.getenv("HOSTNAME") if conf.get(
        "multirobot", False) else "nanosaur"

    # Camera model
    # use:
    #  - 'zed' for "ZED" camera
    #  - 'zedm' for "ZED mini" camera
    #  - 'zed2' for "ZED2" camera
    #  - 'zed2i' for "ZED2i" camera
    camera_model = 'zedm'
    camera_name = camera_model
    
    publish_urdf = 'false'  # Publish static frames from camera URDF
    
    # Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
    base_frame = 'base_link'
    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = '0.0'
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = '0.0'
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = '0.0'
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = str(5.0 * math.pi / 180.0) 
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = '0.0'
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = '0.0'

    # ZED Configurations from local config
    config_common_path = os.path.join(pkg_zed, 'config', 'common.yaml')

    if(camera_model != 'zed'):
        config_camera_path = os.path.join(pkg_zed, 'config', camera_model + '.yaml')
    else:
        config_camera_path = ''

    ############# ROS2 DECLARATIONS #############

    declare_namespace_camera_cmd = DeclareLaunchArgument(
        'namespace_camera',
        default_value=namespace_conf,
        description='camera_name_space.')

    ############# ROS2 NODES #############

    # ZED Wrapper node
    # include another launch file in nanosaur namespace
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([pkg_zed, '/launch/include/zed_camera.launch.py']),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_name,
            'node_name': 'zed_node',
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'svo_path': 'live',
            'base_frame': base_frame,
            'cam_pos_x': cam_pos_x,
            'cam_pos_y': cam_pos_y,
            'cam_pos_z': cam_pos_z,
            'cam_roll': cam_roll,
            'cam_pitch': cam_pitch,
            'cam_yaw': cam_yaw
        }.items()
    )

    zed_wrapper_action = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace_camera),
            # ZED wrapper launcher
            zed_wrapper_launch
        ]
    )

    ############################

    ld = LaunchDescription()

    ld.add_action(declare_namespace_camera_cmd)
    ld.add_action(zed_wrapper_action)
    
    return ld