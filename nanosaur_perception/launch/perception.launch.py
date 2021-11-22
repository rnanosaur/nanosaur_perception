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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# detect all 36h11 tags
cfg_36h11 = {
    'image_transport': 'raw',
    'family': '36h11',
    'size': 0.162
}


def load_config(config):
    if os.path.isfile(config):
        
        with open(config, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    return {}


def generate_launch_description():
    pkg_perception = os.path.join(get_package_share_directory('nanosaur_perception'))
    nanosaur_config = os.path.join(pkg_perception, 'param', 'nanosaur.yml')
    # Load locally the configuration
    loaded_config = load_config(nanosaur_config)
    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_perception, 'param', 'robot.yml'))
    # Load namespace
    namespace = os.getenv("HOSTNAME") + "/" if conf.get("multirobot", False) else ""

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        namespace=namespace + 'camera',
        remappings=[('camera/image_rect', 'image_rect'),
                    ('camera/camera_info', 'resized/camera_info'),
                    ('tf', '/tf'),
                    ('tag_detections', '/tag_detections')],
        parameters=[loaded_config if "isaac_ros_apriltag" in loaded_config else cfg_36h11])

    rectify_node = ComposableNode(
        namespace=namespace + 'camera',
        name='isaac_ros_rectify',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('image', 'resized/image'),
                    ('camera_info', 'resized/camera_info'),
                    ]
    )

    resize_config = {
        'scale_height': 0.25,
        'scale_width': 0.25,
        }

    resize_node = ComposableNode(
        namespace=namespace + 'camera',
        name='isaac_ros_resize',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        parameters=[loaded_config if "isaac_ros_resize" in loaded_config else resize_config],
        remappings=[('image', 'image_color')]
    )

    argus_camera_config = {
        'sensor': 5,
        'device': 0,
        'output_encoding': 'rgb8',
        'camera_info_path': "nanosaur_perception/camera_info/camerav2.yml"
        }

    argus_camera_mono_node = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        namespace=namespace + 'camera',
        parameters=[loaded_config if "isaac_ros_argus_camera_mono" in loaded_config else argus_camera_config],
        remappings=[('/image_raw', 'image_color'),
                    ('/image_raw/compressedDepth', 'image_color/compressedDepth'),
                    ('/image_raw/compressed', 'image_color/compressed'),
                    ('/camera_info', 'camera_info')
                    ]
    )
    
    nodes = [resize_node, rectify_node]
    # apriltag node
    if conf.get("apriltag", True):
        nodes += [apriltag_node]
    # Build Composable node container for intra communication
    argus_camera_mono_container = ComposableNodeContainer(
        name='argus_camera_mono_container',
        namespace=namespace + 'camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen'
    )
    
    return LaunchDescription([
        argus_camera_mono_container,
        argus_camera_mono_node])
# EOF
