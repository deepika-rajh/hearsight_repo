import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_path = os.path.join(
            get_package_share_directory('rvvslam'), 'config/stereo/')
    vm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rvvm'), 'launch'),
            '/vm_stereo.launch.py']),
        launch_arguments={
            'alg_setting': config_path+'Configuration/vm.cfg',
            'camera_setting': config_path+'Configuration/OAKStereoCamera.yaml',
            'sensor_setting': config_path
        }.items(),
    )
    stereo_vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rvvslam'), 'launch'),
            '/stereo_VSLAM.launch.py']),
        launch_arguments={
            'alg_setting': config_path+'Configuration/stereoWSlam.cfg',
            'sensor_setting': config_path
        }.items(),
    )
    return LaunchDescription([
                              vm,
                              stereo_vslam])
