from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context):
    rviz_config_override = LaunchConfiguration('rviz_config').perform(context).strip()
    rviz_profile = LaunchConfiguration('rviz_profile').perform(context).strip().lower()
    rviz_fixed_frame = LaunchConfiguration('rviz_fixed_frame').perform(context).strip()
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))

    rviz_dir = Path(get_package_share_directory('jetracer_description')) / 'rviz'
    profile_map = {
        'description': rviz_dir / 'jetracer.rviz',
        'navigation': rviz_dir / 'navigation.rviz',
        'nav': rviz_dir / 'navigation.rviz',
        'slam': rviz_dir / 'slam.rviz',
        'autonomy': rviz_dir / 'autonomy.rviz',
        'lane_following': rviz_dir / 'autonomy.rviz',
    }

    if rviz_config_override:
        rviz_config_path = Path(rviz_config_override)
    else:
        if rviz_profile not in profile_map:
            valid_profiles = ', '.join(sorted(profile_map.keys()))
            raise RuntimeError(
                f"Unknown rviz_profile '{rviz_profile}'. Valid values: {valid_profiles}.")
        rviz_config_path = profile_map[rviz_profile]

    if not rviz_config_path.exists():
        raise RuntimeError(f'RViz config not found: {rviz_config_path}')

    arguments = ['-d', str(rviz_config_path)]
    if rviz_fixed_frame:
        arguments.extend(['-f', rviz_fixed_frame])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=arguments,
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
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
            description='Optional absolute path to a custom .rviz file. Overrides rviz_profile when set.',
        ),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value='',
            description='Optional RViz fixed frame override. Empty uses the profile default.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for RViz.',
        ),
        OpaqueFunction(
            function=_launch_setup,
            condition=IfCondition(use_rviz),
        ),
    ])
