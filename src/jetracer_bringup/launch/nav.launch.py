from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_multipoint_nav = LaunchConfiguration('use_multipoint_nav')
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
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'maps',
                'placeholder_map.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument('use_multipoint_nav', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_profile', default_value='navigation'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_fixed_frame', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_bringup'),
                    'launch',
                    'jetracer.launch.py',
                ])
            ),
            launch_arguments={
                'config_file': config_file,
                'use_rviz': use_rviz,
                'rviz_profile': rviz_profile,
                'rviz_config': rviz_config,
                'rviz_fixed_frame': rviz_fixed_frame,
                'use_sim_time': use_sim_time,
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
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_perception'),
                    'launch',
                    'csi_camera.launch.py',
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_navigation'),
                    'launch',
                    'nav.launch.py',
                ])
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_multipoint_nav': use_multipoint_nav,
                'use_sim_time': use_sim_time,
                'config_file': config_file,
            }.items(),
        ),
    ])
