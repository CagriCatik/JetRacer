"""
simulation.launch.py
====================
JetRacer Gazebo Classic 11 simulation launcher for ROS 2 Foxy.

Starts the full simulation stack:
  1. gzserver       — Gazebo physics engine (headless)
  2. gzclient       — Gazebo GUI (optional)
  3. robot_state_publisher   — publishes TF from URDF + joint_states
  4. spawn_model    — spawns the robot URDF into Gazebo via ROS service
  5. Gazebo ROS bridges      — publishes Gazebo topics to ROS 2
  6. image_republisher       — converts raw Image to CompressedImage
                               so lane_following_node gets its expected topic
  7. RViz2 (optional)        — visualization

Arguments
---------
  world            : SDF world file path (default: jetracer_track.sdf)
  robot_name       : Gazebo model name (default: jetracer)
  spawn_x          : Spawn X coordinate (default: 0.0)
  spawn_y          : Spawn Y coordinate (default: 0.0)
  spawn_z          : Spawn Z (default: 0.05, slightly above ground)
  spawn_yaw        : Spawn heading in radians (default: 0.0)
  gui              : Show Gazebo GUI (default: true)
  use_rviz         : Launch RViz2 (default: false)
  use_sim_time     : All nodes use Gazebo clock (default: true)
  debug            : Enable verbose gazebo logging (default: false)

Interface contract
------------------
All topic names on the ROS 2 side match the real hardware stack:
  /cmd_vel               ← twist_mux output
  /odom                  → lane_following, EKF (if running)
  /scan                  → collision_assurance, SLAM
  /imu/data              → EKF, safety_supervisor
  /csi_cam_0/image_raw             → (raw)
  /csi_cam_0/image_raw/compressed  → lane_following, yolo_detection
  /csi_cam_0/camera_info           → lane_following
  /joint_states          → robot_state_publisher

Safety
------
This launch file NEVER starts jetracer_hardware (the real motor driver).
Hardware drivers are intentionally excluded to prevent accidental motor
commands when the simulation and hardware share a ROS_DOMAIN_ID.

Note
----
Gazebo Classic 11 publishes to /gazebo/* topics. Manual ROS 2 publishers
in the simulation node bridge these to ROS 2 standard namespaces.
"""

import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context, *args, **kwargs):
    # ── Resolve launch arguments ──────────────────────────────────────────────
    gz_pkg = get_package_share_directory('jetracer_gazebo')
    desc_pkg = get_package_share_directory('jetracer_description')

    world = LaunchConfiguration('world').perform(context).strip()
    if not world:
        world = str(Path(gz_pkg) / 'worlds' / 'jetracer_track.sdf')

    robot_name = LaunchConfiguration('robot_name').perform(context).strip()
    spawn_x = LaunchConfiguration('spawn_x').perform(context).strip()
    spawn_y = LaunchConfiguration('spawn_y').perform(context).strip()
    spawn_z = LaunchConfiguration('spawn_z').perform(context).strip()
    spawn_yaw = LaunchConfiguration('spawn_yaw').perform(context).strip()
    gui = _to_bool(LaunchConfiguration('gui').perform(context))
    use_rviz = _to_bool(LaunchConfiguration('use_rviz').perform(context))
    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    debug = _to_bool(LaunchConfiguration('debug').perform(context))

    # ── Build combined URDF (base + Gazebo plugins) ───────────────────────────
    base_xacro = Path(desc_pkg) / 'urdf' / 'jetracer.urdf.xacro'
    sim_xacro = Path(gz_pkg) / 'urdf' / 'jetracer_sim.urdf.xacro'

    # We compose both xacros: start from the base and include the sim additions.
    # The sim additions are applied as a separate xacro that is merged in.
    # Strategy: process base URDF first, then append sim plugin XML.
    try:
        base_result = subprocess.run(
            ['xacro', str(base_xacro)],
            capture_output=True, text=True, check=True,
        )
        base_urdf = base_result.stdout

        sim_result = subprocess.run(
            ['xacro', str(sim_xacro)],
            capture_output=True, text=True, check=True,
        )
        sim_xml = sim_result.stdout

        # Merge: strip the outer <robot>...</robot> wrapper from the sim xacro,
        # then inject the plugin content just before </robot> in the base URDF.
        import re
        # Extract content between <robot ...> and </robot> from sim_xml
        inner_match = re.search(
            r'<robot[^>]*>(.*?)</robot>', sim_xml, re.DOTALL)
        if inner_match:
            sim_inner = inner_match.group(1)
        else:
            sim_inner = ''

        # Inject into base URDF before closing </robot>
        robot_description = base_urdf.replace(
            '</robot>', sim_inner + '\n</robot>', 1)

    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f"xacro failed: {exc.stderr}\n"
            "Ensure xacro is installed: sudo apt install ros-$ROS_DISTRO-xacro"
        ) from exc

    rviz_config = str(Path(desc_pkg) / 'rviz' / 'autonomy.rviz')

    nodes = []

    # ── 1. Gazebo Classic 11 Server (headless physics) ──────────────────────
    gzserver_cmd = ['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    
    if debug:
        gzserver_cmd.insert(1, '--verbose')
    
    gzserver_cmd.append(world)
    
    gzserver_env = {'GAZEBO_RESOURCE_PATH': str(Path(gz_pkg) / 'worlds')}
    
    nodes.append(ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        additional_env=gzserver_env,
    ))
    nodes.append(LogInfo(msg='[jetracer_gazebo] Gazebo Classic 11 server started'))

    # ── 2. Gazebo Client (GUI) ────────────────────────────────────────────────
    if gui:
        nodes.append(ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
        ))
        nodes.append(LogInfo(msg='[jetracer_gazebo] Gazebo GUI started'))

    # ── 3. robot_state_publisher ──────────────────────────────────────────────
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    ))

    # ── 4. Spawn entity into Gazebo ───────────────────────────────────────────
    # Delay so Gazebo has time to start before the spawn request.
    # gazebo_ros spawn_model ROS service is the standard way in Classic.
    spawn_model_cmd = [
        'ros2', 'service', 'call',
        '/spawn_entity',
        'gazebo_msgs/srv/SpawnEntity',
        f'{{name: {robot_name}, xml: "{robot_description.replace('"', '\\"')}", '
        f'initial_pose: {{position: {{x: {spawn_x}, y: {spawn_y}, z: {spawn_z}}}, '
        f'orientation: {{z: {spawn_yaw}}}}}}}',
    ]
    
    # Simplified: use a Python node that calls the spawn service
    nodes.append(TimerAction(
        period=2.0,
        actions=[
            Node(
                package='jetracer_gazebo',
                executable='spawn_model.py',
                name='spawn_jetracer',
                output='screen',
                arguments=[
                    '--name', robot_name,
                    '--x', spawn_x,
                    '--y', spawn_y,
                    '--z', spawn_z,
                    '--yaw', spawn_yaw,
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            LogInfo(msg=f'[jetracer_gazebo] Spawning {robot_name} into Gazebo'),
        ],
    ))

    # ── 5. Gazebo ROS Topic Bridge Node ───────────────────────────────────────
    # This node subscribes to Gazebo's /gazebo/* topics and republishes to ROS 2 standard names
    nodes.append(Node(
        package='jetracer_gazebo',
        executable='gazebo_bridge.py',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # ── 6. Image republisher: raw → compressed ────────────────────────────────
    # lane_following_node and yolo_detection subscribe to
    # /csi_cam_0/image_raw/compressed (CompressedImage, JPEG).
    # Gazebo provides /csi_cam_0/image_raw (raw Image).
    # image_transport republisher converts between them.
    nodes.append(Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in',  '/csi_cam_0/image_raw'),
            ('out', '/csi_cam_0/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # ── 7. RViz2 (optional) ───────────────────────────────────────────────────
    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────────

        DeclareLaunchArgument(
            'world',
            default_value='',
            description=(
                'Absolute path to a .sdf world file. '
                'Defaults to jetracer_gazebo/worlds/jetracer_track.sdf.'
            ),
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='jetracer',
            description='Gazebo entity name for the spawned robot.',
        ),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot spawn X coordinate in the world frame.',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot spawn Y coordinate in the world frame.',
        ),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.05',
            description='Robot spawn Z coordinate (slightly above ground).',
        ),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot spawn heading in radians.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Show Gazebo GUI. Set false for headless/CI use.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2 alongside Gazebo.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description=(
                'All ROS 2 nodes use the Gazebo simulation clock. '
                'Must be true for bag replay compatibility.'
            ),
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable verbose Gazebo logging (-v 4).',
        ),

        OpaqueFunction(function=_launch_setup),
    ])
