from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_audio_common = LaunchConfiguration('use_audio_common')
    runtime_dir = LaunchConfiguration('runtime_dir')
    device = LaunchConfiguration('device')
    bitrate = LaunchConfiguration('bitrate')
    channels = LaunchConfiguration('channels')
    sample_rate = LaunchConfiguration('sample_rate')
    audio_format = LaunchConfiguration('audio_format')
    sample_format = LaunchConfiguration('sample_format')

    return LaunchDescription([
        DeclareLaunchArgument('use_audio_common', default_value='false'),
        DeclareLaunchArgument('runtime_dir', default_value='~/.ros/jetracer_voice'),
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('bitrate', default_value='128'),
        DeclareLaunchArgument('channels', default_value='1'),
        DeclareLaunchArgument('sample_rate', default_value='16000'),
        DeclareLaunchArgument('audio_format', default_value='mp3'),
        DeclareLaunchArgument('sample_format', default_value='S16LE'),
        Node(
            package='jetracer_voice',
            executable='vad.py',
            name='jetracer_voice_play',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_audio_common, "' == 'false'"])),
            parameters=[{
                'mode': 'play',
                'runtime_dir': runtime_dir,
            }],
        ),
        Node(
            package='audio_capture',
            executable='audio_capture',
            name='jetracer_audio_capture',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_audio_common, "' == 'true'"])),
            parameters=[{
                'bitrate': bitrate,
                'device': device,
                'channels': channels,
                'sample_rate': sample_rate,
                'sample_format': sample_format,
                'format': audio_format,
            }],
        ),
        Node(
            package='audio_play',
            executable='audio_play',
            name='jetracer_audio_play',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_audio_common, "' == 'true'"])),
            parameters=[{
                'device': device,
                'channels': channels,
                'sample_rate': sample_rate,
                'sample_format': sample_format,
                'format': audio_format,
            }],
        ),
    ])
