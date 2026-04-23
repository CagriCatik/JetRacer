from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sensor_id = LaunchConfiguration('sensor_id')
    cam_name = LaunchConfiguration('cam_name')
    frame_id = LaunchConfiguration('frame_id')
    sync_sink = LaunchConfiguration('sync_sink')
    use_gst_timestamps = LaunchConfiguration('use_gst_timestamps')
    camera_info_url = LaunchConfiguration('camera_info_url')
    gscam_config = LaunchConfiguration('gscam_config')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    image_fps = LaunchConfiguration('image_fps')
    flip_method = LaunchConfiguration('flip_method')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_id', default_value='0'),
        DeclareLaunchArgument('cam_name', default_value='csi_cam_0'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('sync_sink', default_value='false'),
        DeclareLaunchArgument('use_gst_timestamps', default_value='false'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('image_fps', default_value='20'),
        DeclareLaunchArgument('flip_method', default_value='0'),
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
        DeclareLaunchArgument(
            'gscam_config',
            default_value=[
                'nvarguscamerasrc sensor-id=', sensor_id,
                ' ! video/x-raw(memory:NVMM), width=(int)', image_width,
                ', height=(int)', image_height,
                ', format=(string)NV12, framerate=(fraction)', image_fps,
                '/1 ! nvvidconv flip-method=', flip_method,
                ' ! videoconvert',
            ],
        ),
        Node(
            package='gscam',
            executable='gscam_node',
            name='gscam',
            namespace=cam_name,
            output='screen',
            parameters=[{
                'camera_name': cam_name,
                'camera_info_url': camera_info_url,
                'frame_id': frame_id,
                'gscam_config': gscam_config,
                'sync_sink': ParameterValue(sync_sink, value_type=bool),
                'use_gst_timestamps': ParameterValue(use_gst_timestamps, value_type=bool),
            }],
            remappings=[
                ('camera/image_raw', 'image_raw'),
                ('camera/camera_info', 'camera_info'),
                ('camera/set_camera_info', 'set_camera_info'),
            ],
        ),
    ])
