from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware_params_file = LaunchConfiguration('hardware_params_file')
    localization_params_file = LaunchConfiguration('localization_params_file')
    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_profile = LaunchConfiguration('rviz_profile')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot_description from xacro at launch time
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('jetracer_description'), 'urdf', 'jetracer.urdf.xacro',
            ]),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'), 'config', 'hardware.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_localization'), 'config', 'ekf.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'), 'config', 'main_config.yaml',
            ]),
            description='Centralized stack parameter file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz for this bringup.',
        ),
        DeclareLaunchArgument(
            'rviz_profile',
            default_value='description',
            description='RViz profile: description, navigation, slam, autonomy.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional absolute path to a custom .rviz file.',
        ),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value='base_footprint',
            description='Optional fixed frame override for RViz.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for RViz.',
        ),

        # CRITICAL FIX: On the physical robot, never launch joint_state_publisher
        # (GUI or otherwise) — it floods /joint_states with zeros and conflicts
        # with any real joint state source.  robot_state_publisher alone is
        # sufficient to broadcast the static TF tree from the URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'), 'launch', 'hardware.launch.py',
            ])),
            launch_arguments={'params_file': hardware_params_file}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_localization'), 'launch', 'localization.launch.py',
            ])),
            launch_arguments={'params_file': localization_params_file}.items(),
        ),

        # Ackermann geometry conversion for Nav2: yaw-rate (rad/s) → steering angle (rad)
        # Nav2 outputs to cmd_vel_nav. We convert it to cmd_vel_nav_steer. 
        Node(
            package='jetracer_navigation',
            executable='cmd_vel_to_steering_node',
            name='cmd_vel_to_steering',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('jetracer_navigation'),
                    'config',
                    'nav_bridge.yaml'
                ]),
                config_file
            ],
        ),

        # ── cmd_vel arbitration ─────────────────────────────────────────────
        # twist_mux selects the highest-priority active source of steering-angle Twists:
        #   teleop (10) > lane (5) > vision (4) > nav2_steer (3)
        # Its output directly feeds the hardware (cmd_vel).
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'), 'config', 'twist_mux.yaml',
            ])],
            remappings=[('cmd_vel_out', 'cmd_vel')],
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
    ])
