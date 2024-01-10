# Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

   mode = LaunchConfiguration('mode', default='2')
   disparity_min = LaunchConfiguration('disparity_min', default='1')
   disparity_level = LaunchConfiguration('disparity_level', default='96')
   enable_color = LaunchConfiguration('enable_color', default='false')

   cmd_mode = DeclareLaunchArgument('mode', default_value=mode, description='0 CVP, 1 CPU, 2 GPU, 4 NORMAL')
   cmd_disparity_min = DeclareLaunchArgument('disparity_min', default_value=disparity_min, description='disparity min value')
   cmd_disparity_level = DeclareLaunchArgument('disparity_level', default_value=disparity_level, description='disparity level')
   cmd_enable_color = DeclareLaunchArgument('enable_color', default_value=enable_color, description='enable RGB image')


   realsense_base = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('realsense2_camera'), 'launch'), '/rs_launch.py']),
      launch_arguments={
         'enable_infra1':'true',
         'enable_infra2':'true',
         'enable_color':enable_color,
         'rgb_camera.profile': '640x480x30',
         'depth_module.profile':'640x480x30'
      }.items()
   )

   dfs = Node(package = "dfs_package", executable = "rv_dfs_ros2", output = 'screen',
              parameters=[{'mode': mode,
                           'disparity_min': disparity_min,
                           'disparity_level': disparity_level,
                           'enable_color': enable_color}]
   )

   return LaunchDescription([
      cmd_mode,
      cmd_disparity_min,
      cmd_disparity_level,
      cmd_enable_color,
      realsense_base,
      dfs
   ])
