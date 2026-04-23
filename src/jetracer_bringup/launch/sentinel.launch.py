from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Build robot_description from xacro at launch time
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('jetracer_description'), 'urdf', 'jetracer.urdf.xacro',
            ]),
        ]),
        value_type=str,
    )

    hardware_params_file = PathJoinSubstitution([
        FindPackageShare('jetracer_hardware'), 'config', 'hardware.yaml',
    ])

    return LaunchDescription([
        # Broadcast URDF and TF tree (required for all modes)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Hardware bridge: serial communication with RP2040 controller
        # Publishes: odom, imu, battery_state; subscribes to cmd_vel
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('jetracer_hardware'), 'launch', 'hardware.launch.py',
            ])),
            launch_arguments={'params_file': str(hardware_params_file)}.items(),
        ),

        # cmd_vel arbitration: ensures highest-priority control always wins
        # Priorities: safety (255) > teleop (10) > others (lower)
        # Essential for safe manual override of any background automation
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'), 'config', 'twist_mux.yaml',
            ])],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),

        # Joy input device driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # OLED status display
        Node(
            package='jetracer_hardware',
            executable='oled_node.py',
            name='oled_node',
            output='screen'
        ),
        
        # Controller Relay (Sentinel state machine)
        Node(
            package='jetracer_bringup',
            executable='controller_relay.py',
            name='controller_relay',
            output='screen'
        ),

        # Manual Teleoperation: converts gamepad to cmd_vel_teleop
        Node(
            package='jetracer_teleop',
            executable='teleop_joy.py',
            name='teleop_joy',
            parameters=[{
               'x_speed': 0.5,
               'w_speed': 1.0,
               'linear_axis': 4,
               'angular_axis': 0,
               'deadman_button': 6,
            }],
            remappings=[('cmd_vel', 'cmd_vel_teleop')]
        ),

        # Pre-arm validator: gates mission launch until all hardware verified
        # Monitors: IMU stable, serial link, LiDAR, camera, battery, CPU temp
        Node(
            package='jetracer_bringup',
            executable='sentinel_validator.py',
            name='sentinel_validator',
            output='screen',
            parameters=[{
                'imu_stable_duration': 2.0,
                'imu_accel_variance_threshold': 0.1,
                'lidar_timeout_sec': 5.0,
                'camera_min_hz': 5.0,
                'battery_min_voltage': 10.0,
                'battery_max_current': 3500.0,
                'cpu_max_temp': 75.0,
                'startup_timeout_sec': 30.0,
            }],
        ),
    ])
