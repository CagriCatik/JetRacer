from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'),
                'config',
                'calibrate_linear.yaml',
            ]),
            description='Parameter file for the odometry scale calibration node.',
        ),
        Node(
            package='jetracer_hardware',
            executable='calibrate_linear.py',
            name='calibrate_linear',
            output='screen',
            parameters=[params_file],
        ),
    ])
