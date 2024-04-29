# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries. 
# All rights reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera_info_config_file_path = os.path.join(
        get_package_share_directory('qrb_ros_camera'),
        'config', 'camera_info_ov9282.yaml'
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
    ComposableNode(
        package='qrb_ros_vio',
        plugin='qrb_ros_vio::VioComponent',
        name='vio'),
    ComposableNode(
        package='qrb_ros_camera',
        plugin='qrb_ros_camera::CameraNode',
        name='camera',
        parameters=[{
            'camera_info_path': camera_info_config_file_path,
            'fps': 30,
            'width': 640,
            'height': 360,
            'cameraId': 1,
            'publish_latency_type': 1,
            'dump_camera_info_': False,
        }],
        ),
    ComposableNode(
        package='qrb_ros_imu',
        plugin='qrb::ros::ImuComponent',
        name='imu')
    ],
    output='screen',
    )

    return launch.LaunchDescription([container])
