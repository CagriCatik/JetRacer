from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_backend = LaunchConfiguration('slam_backend')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_multipoint_nav = LaunchConfiguration('use_multipoint_nav')
    config_file = LaunchConfiguration('config_file')

    common_parameters = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'slam_toolbox_online.yaml',
            ]),
        ),
        DeclareLaunchArgument('slam_backend', default_value='slam_toolbox'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_multipoint_nav', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'multipoint_nav.yaml',
            ]),
            description='Parameter file for multipoint navigation node.',
        ),
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
                'use_rviz': 'false',
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
                    FindPackageShare('jetracer_navigation'),
                    'launch',
                    'slam.launch.py',
                ])
            ),
            launch_arguments={
                'slam_backend': slam_backend,
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common_parameters,
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=common_parameters,
        ),

        Node(
            package='jetracer_navigation',
            executable='multipoint_nav.py',
            name='multipoint_navigation',
            output='screen',
            condition=IfCondition(use_multipoint_nav),
            parameters=[
                config_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': ParameterValue(autostart, value_type=bool),
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),
    ])
