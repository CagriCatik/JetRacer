from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='csi_cam_0/image_raw/compressed'),
        DeclareLaunchArgument('h_min', default_value='20',
                              description='Hue min (0-180). Default: yellow line.'),
        DeclareLaunchArgument('h_max', default_value='40'),
        DeclareLaunchArgument('s_min', default_value='80'),
        DeclareLaunchArgument('s_max', default_value='255'),
        DeclareLaunchArgument('v_min', default_value='80'),
        DeclareLaunchArgument('v_max', default_value='255'),
        DeclareLaunchArgument('linear_speed', default_value='0.15'),
        DeclareLaunchArgument('kp', default_value='0.005'),
        DeclareLaunchArgument('kd', default_value='0.002'),
        DeclareLaunchArgument('start', default_value='false'),
        Node(
            package='jetracer_perception',
            executable='line_follow.py',
            name='line_follow',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'h_min': LaunchConfiguration('h_min'),
                'h_max': LaunchConfiguration('h_max'),
                's_min': LaunchConfiguration('s_min'),
                's_max': LaunchConfiguration('s_max'),
                'v_min': LaunchConfiguration('v_min'),
                'v_max': LaunchConfiguration('v_max'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'start': LaunchConfiguration('start'),
            }],
        ),
    ])
