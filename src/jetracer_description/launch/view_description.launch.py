from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_profile = LaunchConfiguration('rviz_profile')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetracer_description'),
                'urdf',
                'jetracer.urdf.xacro',
            ]),
            description='Absolute path to the robot description xacro file.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui instead of joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the default configuration.',
        ),
        DeclareLaunchArgument(
            'rviz_profile',
            default_value='description',
            description='RViz profile: description, navigation, slam, autonomy.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Optional absolute path to a custom .rviz file. Overrides rviz_profile when set.',
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
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
