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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    apriltag_exe = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        name='apriltag_exe',
        remappings=[('camera_info', 'resized/camera_info')]
    )

    resize_node = ComposableNode(
        name='isaac_ros_resize',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'scale_height': 0.25,
            'scale_width': 0.25,
        }],
        remappings=[('image', 'image_color')])
    
    rectify_node = ComposableNode(
        name='isaac_ros_rectify',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('image', 'resized/image'), ('camera_info', 'resized/camera_info')])
    argus_camera_mono_node = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        parameters=[{
                'sensor': 5,
                'device': 0,
                'output_encoding': 'rgb8',
                'camera_info_path': "nanosaur_perception/camera_info/camerav2.yml"
        }],
        remappings=[('image_raw', 'image_color')]
    )
    argus_camera_mono_container = ComposableNodeContainer(
        name='argus_camera_mono_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            resize_node,
            rectify_node
        ],
        output='screen'
    )
    
    return LaunchDescription([argus_camera_mono_container, argus_camera_mono_node, apriltag_exe])
# EOF