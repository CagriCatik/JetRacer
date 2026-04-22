from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # OLED status display
        Node(
            package='jetracer_hardware',
            executable='oled_node.py',
            name='oled_node',
            output='screen'
        ),
        
        # Controller Relay (Sentinel)
        Node(
            package='jetracer_bringup',
            executable='controller_relay.py',
            name='controller_relay',
            output='screen'
        ),

        # Manual Teleoperation Node
        Node(
            package='jetracer_teleop',
            executable='teleop_joy.py',
            name='teleop_joy',
            parameters=[{
               'x_speed': 0.5,
               'w_speed': 1.0,
               'linear_axis': 4,
               'angular_axis': 0,
               'deadman_button': 6,
            }],
            remappings=[('cmd_vel', 'cmd_vel_teleop')]
        )
    ])
