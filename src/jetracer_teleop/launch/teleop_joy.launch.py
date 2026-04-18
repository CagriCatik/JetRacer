from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('x_speed', default_value='0.3',
                              description='Max linear speed (m/s)'),
        DeclareLaunchArgument('w_speed', default_value='1.0',
                              description='Max angular speed (rad/s)'),
        DeclareLaunchArgument('linear_axis', default_value='3',
                              description='Joystick axis index for forward/back'),
        DeclareLaunchArgument('angular_axis', default_value='0',
                              description='Joystick axis index for left/right'),
        DeclareLaunchArgument('deadman_button', default_value='6',
                              description='Button index that must be held to enable motion'),
        DeclareLaunchArgument('hz', default_value='20',
                              description='Publish rate in Hz'),

        # joy_node — reads the physical gamepad and publishes sensor_msgs/Joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),

        # teleop_joy — converts Joy → Twist
        Node(
            package='jetracer_teleop',
            executable='teleop_joy.py',
            name='teleop_joy',
            output='screen',
            parameters=[{
                'x_speed': LaunchConfiguration('x_speed'),
                'w_speed': LaunchConfiguration('w_speed'),
                'linear_axis': LaunchConfiguration('linear_axis'),
                'angular_axis': LaunchConfiguration('angular_axis'),
                'deadman_button': LaunchConfiguration('deadman_button'),
                'hz': LaunchConfiguration('hz'),
            }],
        ),
    ])
