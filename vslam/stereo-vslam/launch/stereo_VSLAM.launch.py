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

# where Configuration folder lay in
config_path = 'Configuration_file_folder/'

configurable_parameters = [
    {'name': 'alg_setting', 'default': config_path +
     'Configuration/rgbdWSlam.cfg', 'description': "''"},
    {'name': 'sensor_setting',
     'default': config_path, 'description': "''"},
    {'name': 'camera_setting',
     'default': 'OAKStereoCamera.yaml', 'description': "''"},
    {'name': 'vslam_config_file',
     'default': 'Configuration/robot.cfg', 'description': "''"},
    {'name': 'output_file', 'default': 'log',
     'description': "''"}
]
remap_parameters = [{'name': 'left_image_topic', 'default': '/left/image_raw', 'description': "''"},
                    {'name': 'left_camera_info_topic',
                     'default': '/left/camera_info', 'description': "''"},
                    {'name': 'right_image_topic', 'default': '/right/image_raw', 'description': "''"},
                    {'name': 'right_camera_info_topic',
                     'default': '/right/camera_info', 'description': "''"},
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
    return [(param['default'], LaunchConfiguration(param['name'])) for param in parameters]

def generate_launch_description():
    stereo_vslam_node = launch_ros.actions.Node(
            package='rvvslam', executable='rvSVSLAM',
            output='screen',
            parameters=[set_configurable_parameters(configurable_parameters)],
            remappings=set_remap_parameters(remap_parameters))
    
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
                             declare_configurable_parameters(remap_parameters)+[
        stereo_vslam_node])