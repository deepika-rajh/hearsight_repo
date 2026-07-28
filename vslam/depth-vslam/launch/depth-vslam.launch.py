# Copyright (c) 2023 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
import launch_ros.descriptions

# where Configuration folder lay in
# Root directory (on target) that holds Configuration/vslam.cfg, calibration
# yaml files, the target image, etc. Override with sensor_setting:=<dir>.
config_path = '/usr/share/mono-vslam/'

configurable_parameters = [
    {'name': 'sensor_setting',
     'default': config_path,
     'description': 'directory containing Configuration/vslam.cfg and calibration files'},
    {'name': 'alg_setting',
     'default': config_path + 'Configuration/rgbdWSlam.cfg',
     'description': 'SLAM algorithm .cfg'},
    {'name': 'output_file',
     'default': config_path + 'vwslam/',
     'description': 'map / log output directory'},
    {'name': 'camera_yaml',
     'default': '',
     'description': 'camera calibration yaml (absolute, or relative to sensor_setting). '
                    'Empty => use vslam.cfg Camera line, then auto-detect from camera_info'}
]
remap_parameters = [{'name': 'input_rgb_raw_topic', 'default': '/camera/color/image_raw', 'description': "''"},
                    {'name': 'input_camera_info_topic',
                     'default': '/camera/color/camera_info', 'description': "''"},
                    {'name': 'input_depth_topic',
                     'default': '/camera/aligned_depth_to_color/image_raw', 'description': "''"},
                    {'name': 'labeled_img_pub_topic', 'default': 'vslam/labeled_img',
                     'description': "''"},
                    {'name': 'raw_pose_pub_topic', 'default': 'vslam_odom_raw',
                     'description': "''"},
                    {'name': 'robot_pose_pub_topic', 'default': 'robot_odom',
                     'description': "''"},
                    {'name': 'imu_pub_topic', 'default': 'sensor_imu',
                     'description': "''"},
                    # Node subscribes to the relative topic "imu" (see InputIMUROS2 in
                    # DepthvSLAM.cpp); 'from' must match that literal for the remap to
                    # take effect. Defaults to the RealSense D455's combined IMU topic.
                    # The D455 launch must set unite_imu_method:=2 (linear_interpolation)
                    # or 1 (copy) so accel and gyro samples arrive combined on one topic.
                    {'name': 'input_imu_topic', 'from': 'imu', 'default': '/camera/imu',
                     'description': "''"}
                    ]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def set_remap_parameters(parameters):
    return [(param.get('from', param['default']), LaunchConfiguration(param['name'])) for param in parameters]


def generate_launch_description():
    depth_vslam_node = launch_ros.actions.Node(
        package='depth-vslam', executable='depth-vslam',
        output='screen',
        parameters=[set_configurable_parameters(configurable_parameters)],
        remappings=set_remap_parameters(remap_parameters)
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
                             declare_configurable_parameters(remap_parameters)+[
        depth_vslam_node])
