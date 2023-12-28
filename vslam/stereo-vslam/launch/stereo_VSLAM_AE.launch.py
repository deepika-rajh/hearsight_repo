import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    oak_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_examples'), 'launch'),
            '/stereo_inertial_node.launch.py']),
        launch_arguments={
            'rectify': 'False',
            'depth_aligned': 'False',
            'monoResolution': '400p',
            'lrcheck': 'False',
            'subpixel': 'False',
            'enableSpatialDetection': 'False',
            'syncNN': 'False',
            'enableRviz': 'False',
            'mode': 'disparity',
            'parent_frame': 'base_link',
            'base_frame': 'camera_base'
        }.items()
    )
    scout_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('scout_base'), 'launch'),
            '/scout_mini_base.launch.py'])
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'),
            '/navigation_launch.py'])
    )
    ae = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('aepackage'), 'launch'),
            '/auto_exploration.launch.py'])
    )
    stereo_vslam_vm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rvvslam'), 'launch'),
            '/stereo_VSLAM_VM.launch.py'])
    )
    # delay to wait camera fully initialized
    delay_vslam_vm = TimerAction(period=10.0, actions=[stereo_vslam_vm])
    return LaunchDescription([scout_base,
                              oak_camera,
                              #nav2_bringup,
                              #ae,
                              delay_vslam_vm])
