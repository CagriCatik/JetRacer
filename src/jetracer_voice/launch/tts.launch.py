from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    lang_type = LaunchConfiguration('lang_type')
    runtime_dir = LaunchConfiguration('runtime_dir')
    player_command = LaunchConfiguration('player_command')
    appid = LaunchConfiguration('appid')
    api_key = LaunchConfiguration('api_key')
    api_secret = LaunchConfiguration('api_secret')

    return LaunchDescription([
        DeclareLaunchArgument('lang_type', default_value='en'),
        DeclareLaunchArgument('runtime_dir', default_value='~/.ros/jetracer_voice'),
        DeclareLaunchArgument('player_command', default_value='play -q'),
        DeclareLaunchArgument('appid', default_value=''),
        DeclareLaunchArgument('api_key', default_value=''),
        DeclareLaunchArgument('api_secret', default_value=''),
        Node(
            package='jetracer_voice',
            executable='tts_en.py',
            name='tts_en',
            output='screen',
            condition=IfCondition(PythonExpression(["'", lang_type, "' == 'en'"])),
            parameters=[{
                'runtime_dir': runtime_dir,
                'player_command': player_command,
            }],
        ),
        Node(
            package='jetracer_voice',
            executable='tts_cn.py',
            name='tts_cn',
            output='screen',
            condition=IfCondition(PythonExpression(["'", lang_type, "' == 'cn'"])),
            parameters=[{
                'runtime_dir': runtime_dir,
                'player_command': player_command,
                'appid': appid,
                'api_key': api_key,
                'api_secret': api_secret,
            }],
        ),
    ])
