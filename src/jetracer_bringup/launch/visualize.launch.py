from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_profile = LaunchConfiguration('rviz_profile')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz.',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_description'),
                    'launch',
                    'view_description.launch.py',
                ])
            ),
            launch_arguments={
                'use_gui': use_gui,
                'use_rviz': use_rviz,
                'rviz_profile': rviz_profile,
                'rviz_config': rviz_config,
                'rviz_fixed_frame': rviz_fixed_frame,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
