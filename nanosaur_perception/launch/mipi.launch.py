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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from service import ConfDetector, load_config

# detect all 36h11 tags
cfg_36h11 = {
    'image_transport': 'raw',
    'family': '36h11',
    'size': 0.162
}


def generate_launch_description():
    pkg_perception = get_package_share_directory('nanosaur_perception')

    namespace_camera = LaunchConfiguration('namespace_camera')
    config_common_path = LaunchConfiguration('config_common_path')
    cover_type = LaunchConfiguration('cover_type')
    apriltag = LaunchConfiguration('apriltag')

    nanosaur_config_path = os.path.join(
        pkg_perception, 'param', 'nanosaur.yml')

    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_perception, 'param', 'robot.yml'))

    # Load cover_type from robot.yml
    cover_type_conf = conf.get("cover_type", 'pi')

    conf_detector = ConfDetector(nanosaur_config_path)

    ############# ROS2 DECLARATIONS #############

    declare_namespace_camera_cmd = DeclareLaunchArgument(
        'namespace_camera',
        default_value='camera',
        description='camera_name_space.')

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=nanosaur_config_path,
        description='Path to the `nanosaur.yml` file.')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value=cover_type_conf,
        description='Cover type to use. Options: pi, fisheye, realsense, zedmini.')

    # Doesn't work now
    declare_apriltag_cmd = DeclareLaunchArgument(
        name='apriltag',
        default_value='True',
        description='Enable or disable Apriltag detector.')

    ############# ROS2 NODES #############

    # Argus camera load
    argus_camera_mono_node = Node(
        package='isaac_ros_argus_camera_mono',
        executable='isaac_ros_argus_camera_mono',
        namespace=namespace_camera,
        parameters=[config_common_path] if conf_detector.is_package('isaac_ros_argus_camera_mono') else [{
                'sensor': 5,
                'device': 0,
                'output_encoding': 'rgb8',
                'camera_info_url': "package://nanosaur_perception/camera_info/camerav2.yml"
        }],
        remappings=[('/image_raw', 'image_color'),
                    ('/image_raw/compressedDepth', 'image_color/compressedDepth'),
                    ('/image_raw/compressed', 'image_color/compressed'),
                    ('/camera_info', 'camera_info')
                    ]
    )

    # Isaac ROS GEMs nodes

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        namespace=namespace_camera,
        remappings=[('camera/image_rect', 'image_rect'),
                    ('camera/camera_info', 'resized/camera_info'),
                    ('tf', '/tf'),
                    ('tag_detections', '/tag_detections')],
        parameters=[config_common_path] if conf_detector.is_package('apriltag') else [cfg_36h11],
    )

    resize_node = ComposableNode(
        namespace=namespace_camera,
        name='isaac_ros_resize',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        parameters=[config_common_path] if conf_detector.is_package('isaac_ros_resize') else [{
            'scale_height': 0.25,
            'scale_width': 0.25,
        }],
        remappings=[('image', 'image_color')]
    )

    remap_pi = 'image_rect' if cover_type_conf == 'pi' else 'image_out'
    
    rectify_node = ComposableNode(
        namespace=namespace_camera,
        name='isaac_ros_rectify',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
        remappings=[('image', 'resized/image'),
                    ('camera_info', 'resized/camera_info'),
                    ('image_rect', remap_pi),
                    ]
    )

    nodes = [resize_node, rectify_node]
    if conf.get("apriltag", False):
        nodes += [apriltag_node]

    # Build Composable node container for intra communication
    argus_camera_mono_container = ComposableNodeContainer(
        name='argus_camera_mono_container',
        namespace=namespace_camera,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen'
    )

    rotate_mage_node = Node(
        package='image_manipulator',
        executable='flipper',
        output='screen',
        namespace=namespace_camera,
        parameters=[{'flip_vertical': True}],
        remappings=[('/image/raw', 'image_rect'),
                    ('/image/flipped', 'image_out')
                    ]
    )

    ############################

    ld = LaunchDescription()

    ld.add_action(declare_namespace_camera_cmd)
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_cover_type_cmd)
    ld.add_action(declare_apriltag_cmd)
    
    ld.add_action(argus_camera_mono_node)
    ld.add_action(argus_camera_mono_container)
    
    if cover_type_conf == 'pi':
        ld.add_action(rotate_mage_node)

    return ld
# EOF
