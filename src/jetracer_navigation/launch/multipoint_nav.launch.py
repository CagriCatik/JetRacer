from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'multipoint_nav.yaml',
            ]),
            description='Parameter file for multipoint navigation node.',
        ),
        Node(
            package='jetracer_navigation',
            executable='multipoint_nav.py',
            name='multipoint_navigation',
            output='screen',
            parameters=[config_file],
        ),
    ])
