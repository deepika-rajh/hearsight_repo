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


config_path = 'vm_config_path/'

configurable_parameters = [{'name': 'alg_setting', 'default': config_path+'Configuration/rgbdWSlam.cfg', 'description': "''"}
                           ]

remap_parameters = [{'name': 'vslam_odom_raw_topic', 'default': 'vslam_odom_raw', 'description': "''"},
                    {'name': 'vm_state_topic',
                        'default': 'VM_state', 'description': "''"},
                    {'name': 'input_depth_topic',
                     'default': '/camera/aligned_depth_to_color/image_raw', 'description': "''"},
                    {'name': 'occupancy_img_pub_topic', 'default': 'vm/occupancy_img',
                     'description': "''"},
                    {'name': 'gridmap_pub_topic', 'default': 'gridmap',
                     'description': "''"},
                    {'name': 'robot_pose_pub_topic', 'default': 'robot_odom',
                     'description': "''"}
                    ]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def set_remap_parameters(parameters):
    return [(param['default'], LaunchConfiguration(param['name'])) for param in parameters]


def generate_launch_description():

    vm_node = launch_ros.actions.Node(
        package='voxel-map', executable='voxel-map',
        output='screen',
        parameters=[set_configurable_parameters(configurable_parameters)],
        remappings=set_remap_parameters(remap_parameters))

    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) +
        declare_configurable_parameters(remap_parameters) +
        [vm_node])
