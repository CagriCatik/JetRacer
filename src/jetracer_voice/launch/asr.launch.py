from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    lang_type = LaunchConfiguration('lang_type')
    runtime_dir = LaunchConfiguration('runtime_dir')

    return LaunchDescription([
        DeclareLaunchArgument('lang_type', default_value='en'),
        DeclareLaunchArgument('runtime_dir', default_value='~/.ros/jetracer_voice'),
        Node(
            package='jetracer_voice',
            executable='vad.py',
            name='voice_asr_en',
            output='screen',
            condition=IfCondition(PythonExpression(["'", lang_type, "' == 'en'"])),
            parameters=[{
                'mode': 'asr_en',
                'runtime_dir': runtime_dir,
            }],
        ),
        Node(
            package='jetracer_voice',
            executable='vad.py',
            name='voice_asr_cn',
            output='screen',
            condition=IfCondition(PythonExpression(["'", lang_type, "' == 'cn'"])),
            parameters=[{
                'mode': 'asr_cn',
                'runtime_dir': runtime_dir,
            }],
        ),
    ])
