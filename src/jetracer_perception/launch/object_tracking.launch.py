from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='csi_cam_0/image_raw/compressed'),
        DeclareLaunchArgument('start', default_value='false'),
        Node(
            package='jetracer_perception',
            executable='object_tracking.py',
            name='object_tracking',
            output='screen',
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'start': LaunchConfiguration('start'),
            }],
        ),
    ])
