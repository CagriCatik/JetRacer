from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Full lane-following stack: camera + hardware + lane_following node."""
    config_file = LaunchConfiguration('config_file')
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
        DeclareLaunchArgument('start', default_value='false',
                              description='Enable cmd_vel from lane follower. '
                                          'SAFETY: set true only when physically ready.'),
        DeclareLaunchArgument('max_speed_ms', default_value='0.3'),
        DeclareLaunchArgument('kp', default_value='0.8'),
        DeclareLaunchArgument('ki', default_value='0.1'),
        DeclareLaunchArgument('kd', default_value='0.2'),
        DeclareLaunchArgument('integral_windup_limit', default_value='2.0'),
        DeclareLaunchArgument('way_type', default_value='center'),
        DeclareLaunchArgument('lateral_controller_type', default_value='stanley'),
        DeclareLaunchArgument('camera_info_topic', default_value='csi_cam_0/camera_info'),
        DeclareLaunchArgument('use_camera_calibration', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_profile', default_value='autonomy'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_fixed_frame', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── JetRacer base (hardware + RSP + laser) ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'), 'launch', 'jetracer.launch.py',
            ])),
            launch_arguments={
                'config_file': config_file,
                'use_rviz': use_rviz,
                'rviz_profile': rviz_profile,
                'rviz_config': rviz_config,
                'rviz_fixed_frame': rviz_fixed_frame,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # ── CSI camera ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_perception'), 'launch', 'csi_camera.launch.py',
            ])),
        ),

        # ── Lane-following pipeline ───────────────────────────────────────
        Node(
            package='jetracer_lane_following',
            executable='lane_following_node.py',
            name='lane_following',
            output='screen',
            parameters=[
                config_file,
                {
                    'start': LaunchConfiguration('start'),
                    'max_speed_ms': LaunchConfiguration('max_speed_ms'),
                    'kp': LaunchConfiguration('kp'),
                    'ki': LaunchConfiguration('ki'),
                    'kd': LaunchConfiguration('kd'),
                    'integral_windup_limit': LaunchConfiguration('integral_windup_limit'),
                    'way_type': LaunchConfiguration('way_type'),
                    'lateral_controller_type': LaunchConfiguration('lateral_controller_type'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'use_camera_calibration': LaunchConfiguration('use_camera_calibration'),
                },
            ],
        ),
    ])
