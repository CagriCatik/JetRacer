from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[{
                'image_transport': 'compressed',
                'tag_family': 'tag36h11',
                'tag_size': 0.16,  # Standard size in meters
                'publish_tfs': True,
                'camera_frame': 'camera_link_optical',
                'output_frame': 'base_footprint',
            }],
            remappings=[
                ('image_rect', 'csi_cam_0/image_raw'),
                ('camera_info', 'csi_cam_0/camera_info'),
            ]
        )
    ])
