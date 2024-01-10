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

    dfs_mode = LaunchConfiguration('dfs_mode', default='2')
    disparity_min = LaunchConfiguration('disparity_min', default='1')
    disparity_level = LaunchConfiguration('disparity_level', default='96')
    enable_color = LaunchConfiguration('enable_color', default='false')
    enable_calib_file = LaunchConfiguration('enable_calib_file', default='false')
    enable_stitch_img = LaunchConfiguration('enable_stitch_img', default='false')
    enable_color_point = LaunchConfiguration('enable_color_point', default='true')
    calib_file_path = LaunchConfiguration('calib_file_path', default='/data/config/ov9282/stereo_cal.yml')
    camera_tf = LaunchConfiguration('camera_tf', default='oak_left_camera_frame')

    cmd_dfs_mode = DeclareLaunchArgument('dfs_mode', default_value=dfs_mode, description='0 Coverage, 1 Balance, 2 Speed, 3 CVP')
    cmd_disparity_min = DeclareLaunchArgument('disparity_min', default_value=disparity_min, description='disparity min value')
    cmd_disparity_level = DeclareLaunchArgument('disparity_level', default_value=disparity_level, description='disparity level')
    cmd_enable_color = DeclareLaunchArgument('enable_color', default_value=enable_color, description='enable RGB image')
    cmd_enable_calib_file = DeclareLaunchArgument('enable_calib_file', default_value=enable_calib_file, description='use calibration file instead of use camera_info topic')
    cmd_enable_stitch_img = DeclareLaunchArgument('enable_stitch_img', default_value=enable_stitch_img, description='use one img that stitch left img and right img to one img')
    cmd_enable_color_point = DeclareLaunchArgument('enable_color_point', default_value=enable_color_point, description='false with point xyz, true means point xyzrgb')
    cmd_calib_file_path = DeclareLaunchArgument('calib_file_path', default_value=calib_file_path, description='calibration file path. It works when enable_calib_file=true')
    cmd_camera_tf = DeclareLaunchArgument('camera_tf', default_value=camera_tf, description='camera tf base')

    ## oak stereo_node use mode variable. so we can't use our mode. to rename it dfs_mode.
    # oak_base = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('depthai_examples'), 'launch'), '/stereo.launch.py']),
    #     launch_arguments={
    #         'monoResolution': '400p',
    #         'enableRviz':'false'
    #     }.items()
    # )

    oak_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('depthai_examples'), 'launch'), '/stereo_inertial_node.launch.py']),
        launch_arguments={
            'rectify': 'true',
            'depth_aligned': 'false',
            'monoResolution': '400p',
            'lrcheck': 'false',
            'subpixel': 'false',
            'enableSpatialDetection': 'false',
            'syncNN': 'false',
            'enableRviz':'false'
        }.items()
    )

    dfs_tf = Node(package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = [ "0", "0", "0", "0", "0", "0", "1", camera_tf, "point_cloud"]
    )

    dfs = Node(package = "dfs_package", executable = "rv_dfs_ros2", output = 'screen',
            parameters=[{'dfs_mode': dfs_mode, 
                            'disparity_min': disparity_min,
                            'disparity_level': disparity_level,
                            'enable_color': enable_color,
                            'enable_calib_file':enable_calib_file,
                            'enable_stitch_img':enable_stitch_img,
                            'enable_color_point':enable_color_point,
                            'calib_file_path': calib_file_path}],
            remappings=[('/camera/infra1/camera_info', '/left/camera_info'),
                        ('/camera/infra2/camera_info', '/right/camera_info'),
                        ('/camera/infra1/image_rect_raw', '/left/image_rect'),
                        ('/camera/infra2/image_rect_raw', '/right/image_rect')]
    )


    return LaunchDescription([
        cmd_dfs_mode,
        cmd_disparity_min,
        cmd_disparity_level,
        cmd_enable_color,
        cmd_enable_calib_file,
        cmd_enable_stitch_img,
        cmd_enable_color_point,
        cmd_calib_file_path,
        cmd_camera_tf,
        oak_base,
        dfs_tf,
        dfs
    ])