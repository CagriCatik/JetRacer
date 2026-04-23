from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'start',
            default_value='false',
            description='Enable cmd_vel output. Safety default is false.',
        ),

        DeclareLaunchArgument('max_speed_ms', default_value='0.3'),
        DeclareLaunchArgument('min_speed_ms', default_value='0.05'),
        DeclareLaunchArgument('kp', default_value='0.8'),
        DeclareLaunchArgument('ki', default_value='0.1'),
        DeclareLaunchArgument('kd', default_value='0.2'),
        DeclareLaunchArgument('integral_windup_limit', default_value='2.0'),

        DeclareLaunchArgument('max_steering_rad', default_value='0.6'),

        DeclareLaunchArgument(
            'lateral_controller_type',
            default_value='stanley',
            description="Lateral controller: 'stanley' or 'mpc'.",
        ),

        DeclareLaunchArgument('gain_constant', default_value='0.025'),
        DeclareLaunchArgument('damping_constant', default_value='0.0125'),

        DeclareLaunchArgument('mpc_horizon', default_value='8'),
        DeclareLaunchArgument('mpc_dt', default_value='0.1'),
        DeclareLaunchArgument('mpc_wheelbase', default_value='0.255'),
        DeclareLaunchArgument('mpc_q_cte', default_value='2.5'),
        DeclareLaunchArgument('mpc_q_heading', default_value='1.5'),
        DeclareLaunchArgument('mpc_q_terminal', default_value='3.0'),
        DeclareLaunchArgument('mpc_r_steer', default_value='0.2'),
        DeclareLaunchArgument('mpc_r_steer_rate', default_value='0.8'),
        DeclareLaunchArgument('mpc_cte_scale_px', default_value='48.0'),
        DeclareLaunchArgument('mpc_speed_scale_ms', default_value='0.35'),
        DeclareLaunchArgument('mpc_min_speed_ms', default_value='0.05'),

        DeclareLaunchArgument('gradient_threshold', default_value='14.0'),
        DeclareLaunchArgument('spline_smoothness', default_value='10.0'),

        DeclareLaunchArgument(
            'way_type',
            default_value='center',
            description="Waypoint mode: 'center' or 'smooth'.",
        ),

        DeclareLaunchArgument(
            'camera_topic',
            default_value='csi_cam_0/image_raw/compressed',
            description='Compressed image topic from the CSI camera.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='csi_cam_0/camera_info',
            description='CameraInfo topic used for online undistortion.',
        ),
        DeclareLaunchArgument(
            'use_camera_calibration',
            default_value='true',
            description='Rectify frames using CameraInfo before BEV processing.',
        ),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),

        Node(
            package='jetracer_lane_following',
            executable='lane_following_node.py',
            name='lane_following',
            output='screen',
            parameters=[{
                'start': LaunchConfiguration('start'),
                'max_speed_ms': LaunchConfiguration('max_speed_ms'),
                'min_speed_ms': LaunchConfiguration('min_speed_ms'),
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'integral_windup_limit': LaunchConfiguration('integral_windup_limit'),
                'max_steering_rad': LaunchConfiguration('max_steering_rad'),
                'lateral_controller_type': LaunchConfiguration('lateral_controller_type'),
                'gain_constant': LaunchConfiguration('gain_constant'),
                'damping_constant': LaunchConfiguration('damping_constant'),
                'mpc_horizon': LaunchConfiguration('mpc_horizon'),
                'mpc_dt': LaunchConfiguration('mpc_dt'),
                'mpc_wheelbase': LaunchConfiguration('mpc_wheelbase'),
                'mpc_q_cte': LaunchConfiguration('mpc_q_cte'),
                'mpc_q_heading': LaunchConfiguration('mpc_q_heading'),
                'mpc_q_terminal': LaunchConfiguration('mpc_q_terminal'),
                'mpc_r_steer': LaunchConfiguration('mpc_r_steer'),
                'mpc_r_steer_rate': LaunchConfiguration('mpc_r_steer_rate'),
                'mpc_cte_scale_px': LaunchConfiguration('mpc_cte_scale_px'),
                'mpc_speed_scale_ms': LaunchConfiguration('mpc_speed_scale_ms'),
                'mpc_min_speed_ms': LaunchConfiguration('mpc_min_speed_ms'),
                'gradient_threshold': LaunchConfiguration('gradient_threshold'),
                'spline_smoothness': LaunchConfiguration('spline_smoothness'),
                'way_type': LaunchConfiguration('way_type'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'use_camera_calibration': LaunchConfiguration('use_camera_calibration'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }],
        ),
    ])
