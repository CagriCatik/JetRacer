from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── Safety defaults ─────────────────────────────────────────────────
        # start=false: the car will not move until explicitly enabled.
        # Enable with:  ros2 param set /lane_following start true
        # Disable with: ros2 param set /lane_following start false
        DeclareLaunchArgument('start', default_value='false',
                              description='Enable cmd_vel output. SAFETY: default false.'),

        # ── Speed limits ─────────────────────────────────────────────────────
        DeclareLaunchArgument('max_speed_ms', default_value='0.3',
                              description='Maximum forward speed in m/s.'),
        DeclareLaunchArgument('min_speed_ms', default_value='0.05',
                              description='Minimum (cornering) speed in m/s.'),
        DeclareLaunchArgument('kp', default_value='0.8',
                              description='Longitudinal PID proportional gain.'),
        DeclareLaunchArgument('ki', default_value='0.1',
                              description='Longitudinal PID integral gain.'),
        DeclareLaunchArgument('kd', default_value='0.2',
                              description='Longitudinal PID derivative gain.'),
        DeclareLaunchArgument('integral_windup_limit', default_value='2.0',
                              description='Longitudinal PID integral anti-windup clamp.'),

        # ── Steering ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('max_steering_rad', default_value='0.6',
                              description='Physical steering range in rad (JetRacer max ~0.6).'),
        DeclareLaunchArgument('gain_constant', default_value='0.025',
                              description='Stanley cross-track error gain.'),
        DeclareLaunchArgument('damping_constant', default_value='0.0125',
                              description='First-order steering damping.'),

        # ── Lane detector ────────────────────────────────────────────────────
        DeclareLaunchArgument('gradient_threshold', default_value='14.0',
                              description='Gradient magnitude threshold for edge detection.'),
        DeclareLaunchArgument('spline_smoothness', default_value='10.0',
                              description='B-spline smoothness (higher = smoother lanes).'),

        # ── Planner ──────────────────────────────────────────────────────────
        # "center" is faster on Jetson Nano; "smooth" adds ~5 ms L-BFGS-B optimisation.
        DeclareLaunchArgument('way_type', default_value='center',
                              description='Waypoint mode: "center" or "smooth".'),

        # ── Camera ───────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'camera_topic',
            default_value='csi_cam_0/image_raw/compressed',
            description='Compressed image topic from the CSI camera.',
        ),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),

        # ── Node ─────────────────────────────────────────────────────────────
        Node(
            package='jetracer_lane_following',
            executable='lane_following_node.py',
            name='lane_following',
            output='screen',
            parameters=[{
                'start':               LaunchConfiguration('start'),
                'max_speed_ms':        LaunchConfiguration('max_speed_ms'),
                'min_speed_ms':        LaunchConfiguration('min_speed_ms'),
                'kp':                  LaunchConfiguration('kp'),
                'ki':                  LaunchConfiguration('ki'),
                'kd':                  LaunchConfiguration('kd'),
                'integral_windup_limit': LaunchConfiguration('integral_windup_limit'),
                'max_steering_rad':    LaunchConfiguration('max_steering_rad'),
                'gain_constant':       LaunchConfiguration('gain_constant'),
                'damping_constant':    LaunchConfiguration('damping_constant'),
                'gradient_threshold':  LaunchConfiguration('gradient_threshold'),
                'spline_smoothness':   LaunchConfiguration('spline_smoothness'),
                'way_type':            LaunchConfiguration('way_type'),
                'camera_topic':        LaunchConfiguration('camera_topic'),
                'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            }],
        ),
    ])
