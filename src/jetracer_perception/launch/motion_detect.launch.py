from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='csi_cam_0/image_raw/compressed'),
        Node(
            package='jetracer_perception',
            executable='motion_detect.py',
            name='motion_detect',
            output='screen',
            parameters=[{'camera_topic': LaunchConfiguration('camera_topic')}],
        ),
    ])
