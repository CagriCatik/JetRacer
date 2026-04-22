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
    start_base = LaunchConfiguration('start_base')
    start_lidar = LaunchConfiguration('start_lidar')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_profile = LaunchConfiguration('rviz_profile')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        DeclareLaunchArgument(
            'start_base',
            default_value='true',
            description='Start the base robot hardware stack (serial, odom, EKF, twist_mux).',
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Start the RPLidar driver.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz for autonomy stack visualization.',
        ),
        DeclareLaunchArgument(
            'rviz_profile',
            default_value='autonomy',
            description='RViz profile: description, navigation, slam, autonomy.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional absolute path to a custom .rviz file.',
        ),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value='odom',
            description='Optional fixed frame override for RViz.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for RViz.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_bringup'),
                    'launch',
                    'jetracer.launch.py',
                ])
            ),
            condition=IfCondition(start_base),
            launch_arguments={
                'config_file': config_file,
                'use_rviz': 'false',  # We handle RViz in autonomy.launch.py
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_hardware'),
                    'launch',
                    'lidar.launch.py',
                ])
            ),
            condition=IfCondition(start_lidar),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_description'),
                    'launch',
                    'rviz.launch.py',
                ])
            ),
            launch_arguments={
                'use_rviz': use_rviz,
                'rviz_profile': rviz_profile,
                'rviz_config': rviz_config,
                'rviz_fixed_frame': rviz_fixed_frame,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # 1. High-Speed Lane Following Engine
        Node(
            package='jetracer_lane_following',
            executable='lane_following_node.py',
            name='lane_following',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('jetracer_lane_following'),
                    'config',
                    'lane_following.yaml'
                ]),
                config_file
            ]
        ),
        
        # 2. YOLO11 Semantic Sensor
        Node(
            package='jetracer_perception',
            executable='yolo_detection.py',
            name='yolo_detection',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('jetracer_perception'),
                    'config',
                    'yolo.yaml'
                ]),
                config_file
            ]
        ),
        
        # 3. Behavioral Decision Stack
        Node(
            package='jetracer_behavior',
            executable='semantic_behavior.py',
            name='semantic_behavior',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('jetracer_behavior'),
                    'config',
                    'behavior.yaml'
                ]),
                config_file
            ]
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
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('jetracer_behavior'),
                    'config',
                    'behavior.yaml'
                ]),
                config_file
            ]
        ),
        
        # 6. Wheel Slip Monitoring (Safety Diagnostic)
        Node(
            package='jetracer_behavior',
            executable='slip_monitor',
            name='slip_monitor',
            output='screen',
            parameters=[config_file]
        )
    ])
