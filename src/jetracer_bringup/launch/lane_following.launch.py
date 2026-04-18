from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Full lane-following stack: camera + hardware + lane_following node."""
    config_file = LaunchConfiguration('config_file')

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

        # ── JetRacer base (hardware + RSP + laser) ─────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'), 'launch', 'jetracer.launch.py',
            ])),
            launch_arguments={'config_file': config_file}.items(),
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
                },
            ],
        ),
    ])
