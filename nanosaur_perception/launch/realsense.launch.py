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

    nanosaur_config_path = os.path.join(
        pkg_perception, 'param', 'nanosaur.yml')

    # Load nanosaur configuration and check if are included extra parameters
    conf = load_config(os.path.join(pkg_perception, 'param', 'robot.yml'))
    # Load namespace from robot.yml
    namespace_conf = os.getenv("HOSTNAME") if conf.get(
        "multirobot", False) else "nanosaur"
    
    conf_detector = ConfDetector(nanosaur_config_path)

    ############# ROS2 DECLARATIONS #############

    declare_namespace_camera_cmd = DeclareLaunchArgument(
        'namespace_camera',
        default_value='camera',
        description='camera_name_space.')
    
    ############# ROS2 NODES #############

    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=namespace_camera,
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'stereo_module.emitter_enabled': 2, #https://github.com/IntelRealSense/realsense-ros/issues/817
                'infra_fps': 90.0,
                'unite_imu_method': 'linear_interpolation' # copy | linear_interpolation
        }],
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

    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        namespace='camera',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        parameters=[{
                    'enable_rectified_pose': False,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': 'camera_infra1_frame',
                    'right_camera_frame': 'camera_infra2_frame',
                    'fixed_frame': 'odom',
                    'imu_frame': 'camera_imu_optical_frame',
                    'current_smooth_frame': 'base_link',
                    'current_rectified_frame': 'base_link_rect'
                    }],
        remappings=[('stereo_camera/left/image', 'infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                    ('stereo_camera/right/image', 'infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                    ('visual_odometry/imu', 'imu')]
    )

    nodes = [visual_odometry_node]
    if conf.get("apriltag", True):
        nodes += [apriltag_node]

    visual_odometry_launch_container = ComposableNodeContainer(
        name='visual_odometry_launch_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen'
    )
    
    ############################

    ld = LaunchDescription()

    ld.add_action(declare_namespace_camera_cmd)
    ld.add_action(realsense_camera_node)
    ld.add_action(visual_odometry_launch_container)
    
    return ld
# EOF
