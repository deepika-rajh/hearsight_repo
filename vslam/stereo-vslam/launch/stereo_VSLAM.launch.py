# Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
import launch_ros.descriptions

# Root directory (on target) that holds Configuration/robot.cfg, calibration
# yaml files, etc. Override with sensor_setting:=<dir>.
config_path = '/opt/qcom/qirf-sdk/data/misc/vwslam/'

configurable_parameters = [
    {'name': 'sensor_setting',
     'default': config_path,
     'description': 'directory containing Configuration/robot.cfg and calibration files'},
    {'name': 'alg_setting',
     'default': config_path + 'Configuration/stereoSlam.cfg',
     'description': 'SLAM algorithm .cfg'},
    {'name': 'output_file',
     'default': '/opt/qcom/qirf-sdk/data/vwslam/',
     'description': 'map / log output directory'},
    {'name': 'vslam_config_file',
     'default': 'Configuration/robot.cfg',
     'description': 'robot/sensor cfg (relative to sensor_setting)'},
    {'name': 'camera_yaml',
     'default': '',
     'description': 'camera calibration yaml (absolute, or relative to sensor_setting). '
                    'Empty => use vslam_config_file Stereo/Camera line, then auto-detect '
                    'from /left/camera_info and /right/camera_info'}
]
remap_parameters = [# Defaults point at the D455 infrared stereo pair (Infrared 1 = left,
                    # Infrared 2 = right) under a /camera/camera/... namespace, matching
                    # how this rig's test bags/live camera are set up (see
                    # kLeftImageTopic/kRightImageTopic in InputStereoCameraROS2.cpp for
                    # the literal topic names being remapped from).
                    {'name': 'left_image_topic', 'from': '/left/image_raw',
                     'default': '/camera/camera/infra1/image_rect_raw', 'description': "''"},
                    {'name': 'left_camera_info_topic', 'from': '/left/camera_info',
                     'default': '/camera/camera/infra1/camera_info', 'description': "''"},
                    {'name': 'right_image_topic', 'from': '/right/image_raw',
                     'default': '/camera/camera/infra2/image_rect_raw', 'description': "''"},
                    {'name': 'right_camera_info_topic', 'from': '/right/camera_info',
                     'default': '/camera/camera/infra2/camera_info', 'description': "''"},
                    # Node subscribes to the relative topic "imu" (see InputIMUROS2 in
                    # StereovSLAM.cpp); 'from' must match that literal for the remap to
                    # take effect.
                    {'name': 'input_imu_topic', 'from': 'imu',
                     'default': '/camera/camera/imu', 'description': "''"},
                    {'name': 'raw_pose_pub_topic', 'default': 'vslam_odom_raw',
                     'description': "''"},
                    {'name': 'robot_pose_pub_topic', 'default': 'robot_odom',
                     'description': "''"},
                    {'name': 'imu_pub_topic', 'default': 'sensor_imu',
                     'description': "''"},
                    {'name': 'odom_topic', 'default': 'odom',
                     'description': "''"},
                    {'name': 'labeled_img_pub_topic', 'default': 'vslam/labeled_img',
                     'description': "''"}
                    ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def set_remap_parameters(parameters):
    return [(param.get('from', param['default']), LaunchConfiguration(param['name'])) for param in parameters]

def generate_launch_description():
    stereo_vslam_node = launch_ros.actions.Node(
            package='stereo-vslam', executable='stereo-vslam',
            output='screen',
            parameters=[set_configurable_parameters(configurable_parameters)],
            remappings=set_remap_parameters(remap_parameters))
    
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
                             declare_configurable_parameters(remap_parameters)+[
        stereo_vslam_node])