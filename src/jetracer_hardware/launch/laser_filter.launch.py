from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    launch_lidar = LaunchConfiguration('launch_lidar')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'),
                'config',
                'laser_filter.yaml',
            ]),
        ),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_hardware'),
                    'launch',
                    'lidar.launch.py',
                ])
            ),
            condition=IfCondition(launch_lidar),
        ),
        Node(
            package='jetracer_hardware',
            executable='laser_filter.py',
            name='laser_filter',
            output='screen',
            parameters=[params_file],
        ),
    ])
