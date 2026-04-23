from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('cam_name', default_value='csi_cam_0'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('image_fps', default_value='20'),
        DeclareLaunchArgument('flip_method', default_value='0'),
        DeclareLaunchArgument('board_size', default_value='5x7'),
        DeclareLaunchArgument('square_size_m', default_value='0.03'),
        DeclareLaunchArgument('startup_delay_sec', default_value='2.0'),
        DeclareLaunchArgument('queue_size', default_value='1'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=[
                TextSubstitution(text='file://'),
                PathJoinSubstitution([
                    FindPackageShare('jetracer_perception'),
                    'config',
                    'cam_640x480.yaml',
                ]),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('jetracer_perception'),
                    'launch',
                    'camera_calibration.launch.py',
                ])
            ),
            launch_arguments={
                'sensor_id': LaunchConfiguration('sensor_id'),
                'cam_name': LaunchConfiguration('cam_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'image_fps': LaunchConfiguration('image_fps'),
                'flip_method': LaunchConfiguration('flip_method'),
                'board_size': LaunchConfiguration('board_size'),
                'square_size_m': LaunchConfiguration('square_size_m'),
                'startup_delay_sec': LaunchConfiguration('startup_delay_sec'),
                'queue_size': LaunchConfiguration('queue_size'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items(),
        ),
    ])
