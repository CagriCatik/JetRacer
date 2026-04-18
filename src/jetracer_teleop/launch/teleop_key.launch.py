from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.5',
                              description='Linear speed scale (m/s)'),
        DeclareLaunchArgument('turn', default_value='1.0',
                              description='Angular speed scale (rad/s)'),

        Node(
            package='jetracer_teleop',
            executable='teleop_key.py',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'speed': LaunchConfiguration('speed'),
                'turn': LaunchConfiguration('turn'),
            }],
        ),
    ])
