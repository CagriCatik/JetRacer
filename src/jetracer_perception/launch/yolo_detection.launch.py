from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspaces/JetRacer-ROS2/models/yolo11n.pt',
            description='Path to trained YOLO11 model (.pt or .engine)'
        ),
        DeclareLaunchArgument(
            'conf_thres',
            default_value='0.5',
            description='Confidence threshold for bounding box detection'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='csi_cam_0/image_raw/compressed',
            description='Input compressed image topic'
        ),
        DeclareLaunchArgument(
            'publish_debug',
            default_value='true',
            description='Whether to publish the annotated bounding-box debug stream'
        ),

        Node(
            package='jetracer_perception',
            executable='yolo_detection.py',
            name='yolo_detection',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf_thres': LaunchConfiguration('conf_thres'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'publish_debug': LaunchConfiguration('publish_debug'),
            }],
        ),
    ])
