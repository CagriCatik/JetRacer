from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    slam_backend = LaunchConfiguration('slam_backend')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    cartographer_basename = LaunchConfiguration('cartographer_basename')

    return LaunchDescription([
        DeclareLaunchArgument('slam_backend', default_value='slam_toolbox'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'slam_toolbox_online.yaml',
            ]),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_navigation'),
                'config',
                'cartographer',
            ]),
        ),
        DeclareLaunchArgument('cartographer_basename', default_value='jetracer.lua'),

        # ── slam_toolbox path ───────────────────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'slam_toolbox'"])),
            parameters=[
                slam_params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),

        # CRITICAL FIX: slam_toolbox is a managed (lifecycle) node.
        # Without this manager it never transitions configure→activate
        # and produces no map output.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'slam_toolbox'"])),
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),

        # ── cartographer path ───────────────────────────────────────────────
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'cartographer'"])),
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_basename,
            ],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'cartographer'"])),
            arguments=['-resolution', '0.05'],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
