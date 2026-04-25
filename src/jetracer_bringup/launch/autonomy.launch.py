from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    start_camera = LaunchConfiguration('start_camera')
    start_base = LaunchConfiguration('start_base')
    start_lidar = LaunchConfiguration('start_lidar')
    start_yolo = LaunchConfiguration('start_yolo')
    start_rrt = LaunchConfiguration('start_rrt')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_profile = LaunchConfiguration('rviz_profile')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    dry_run = LaunchConfiguration('dry_run')
    safe_mode = LaunchConfiguration('safe_mode')
    debug_mode = LaunchConfiguration('debug_mode')

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
        DeclareLaunchArgument(
            'start_yolo',
            default_value='true',
            description='Start the YOLO perception node. Set false to skip GPU inference.',
        ),
        DeclareLaunchArgument(
            'start_rrt',
            default_value='false',
            description='Start the local RRT* LiDAR planner.',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='Suppress all hardware serial writes. Commands are logged only.',
        ),
        DeclareLaunchArgument(
            'safe_mode',
            default_value='false',
            description='Cap autonomous speed at safe_mode_max_speed_ms and force start=false.',
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug image publishing on all perception nodes.',
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
                'dry_run': dry_run,
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
                config_file,
                # safe_mode overrides: disable autonomous start and cap speed
                {
                    'start': PythonExpression(["'false' if '", safe_mode, "' == 'true' else 'false'"]),
                    'max_speed_ms': PythonExpression(["0.20 if '", safe_mode, "' == 'true' else 0.35"]),
                    'publish_debug_image': PythonExpression(["True if '", debug_mode, "' == 'true' else False"]),
                }
            ]
        ),
        
        # 2. YOLO11 Semantic Sensor (optional — disable to save GPU memory)
        Node(
            package='jetracer_perception',
            executable='yolo_detection.py',
            name='yolo_detection',
            output='screen',
            condition=IfCondition(start_yolo),
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
            executable='semantic_behavior',
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
            executable='collision_assurance',
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
        ),
        
        # 7. Hardware-Aware RRT* Local Path Planner
        Node(
            package='jetracer_navigation',
            executable='rrt_star_planner.py',
            name='rrt_star_planner',
            output='screen',
            condition=IfCondition(start_rrt),
            parameters=[
                config_file,
                {
                    'start': PythonExpression(["'false' if '", safe_mode, "' == 'true' else 'true'"]),
                    'dry_run': PythonExpression(["True if '", dry_run, "' == 'true' else False"]),
                    'safe_mode': PythonExpression(["True if '", safe_mode, "' == 'true' else False"]),
                }
            ]
        )
    ])
