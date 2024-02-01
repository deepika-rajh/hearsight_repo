# Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: Qualcomm-Technologies-Inc.-Proprietary

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
   scout_base = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('qti_robot_amr_ctrl'), 'launch'),
         '/qti_robot_amr_ctrl.launch.py'])
      )
   rplidar_ros = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rplidar_ros2'), 'launch'),
         '/rplidar_a3_launch.py'])
      )
   slam_gmapping = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_gmapping'), 'launch'),
         '/slam_gmapping.launch.py']),
	 launch_arguments={'use_sim_time': 'false'}.items()
      )
   laser_tf = Node(package = "tf2_ros",
		executable = "static_transform_publisher",
		arguments = [ "0", "0", "0", "3.1415", "0", "0", "base_link", "laser"])
   nav2_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav2_bringup'), 'launch'),
         '/navigation_launch.py'])
      )
   ae = Node(package = "auto-explore", 
             executable = "auto-explore")


   return LaunchDescription([
      qti_robot_amr_ctrl,
      rplidar_ros2,
      slam_gmapping,
      laser_tf,
      nav2_bringup,
      auto-explore,
   ])
