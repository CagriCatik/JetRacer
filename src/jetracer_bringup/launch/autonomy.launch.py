from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    start_camera = LaunchConfiguration('start_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'),
                'config',
                'main_config.yaml',
            ]),
            description='Centralized stack parameter file.',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the CSI camera pipeline required by lane/yolo nodes.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_perception'),
                    'launch',
                    'csi_camera.launch.py',
                ])
            ),
            condition=IfCondition(start_camera),
        ),
        # 1. High-Speed Lane Following Engine
        Node(
            package='jetracer_lane_following',
            executable='lane_following_node.py',
            name='lane_following',
            output='screen',
            parameters=[config_file]
        ),
        
        # 2. YOLO11 Semantic Sensor
        Node(
            package='jetracer_perception',
            executable='yolo_detection.py',
            name='yolo_detection',
            output='screen',
            parameters=[config_file]
        ),
        
        # 3. Behavioral Decision Stack
        Node(
            package='jetracer_behavior',
            executable='semantic_behavior.py',
            name='semantic_behavior',
            output='screen',
            parameters=[config_file]
        ),
        
        # 4. Foxglove Websocket Bridge (Telemetry)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[config_file]
        ),

        # 5. LiDAR Physical Collision Assurance
        Node(
            package='jetracer_behavior',
            executable='collision_assurance.py',
            name='collision_assurance',
            output='screen',
            parameters=[config_file]
        )
    ])
