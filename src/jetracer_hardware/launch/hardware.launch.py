from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    dry_run = LaunchConfiguration('dry_run')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'),
                'config',
                'hardware.yaml',
            ]),
            description='Hardware parameter file.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='Start hardware node without opening or writing the serial port.',
        ),
        Node(
            package='jetracer_hardware',
            executable='jetracer_serial_node',
            name='jetracer_hardware',
            output='screen',
            parameters=[
                params_file,
                {'dry_run': ParameterValue(dry_run, value_type=bool)},
            ],
        ),
    ])
