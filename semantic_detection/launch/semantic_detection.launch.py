from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('semantic_detection')
    default_params = PathJoinSubstitution([package_share, 'config', 'yolo_params.yaml'])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the yolo_node parameters YAML file',
    )

    yolo_node = Node(
        package='semantic_detection',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        yolo_node,
    ])
