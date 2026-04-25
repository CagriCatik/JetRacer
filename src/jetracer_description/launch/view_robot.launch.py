"""
view_robot.launch.py
====================
Stand-alone robot model inspection launcher.

Starts robot_state_publisher with the JetRacer URDF and optionally:
  - joint_state_publisher_gui  (interactive sliders — good for checking URDF geometry)
  - jetracer_joint_state_publisher (cmd_vel-driven — good for live/replay animation)
  - RViz2 with the 'description' profile

This launch file does NOT start any hardware drivers, autonomous stack nodes,
or sensor pipelines. It is safe to run on any machine (Jetson or laptop).

Typical usage
-------------
  # Inspect the URDF with interactive sliders (laptop):
  ros2 launch jetracer_description view_robot.launch.py use_rviz:=true use_joint_state_gui:=true

  # Animate the model from a running stack (on same machine or same ROS_DOMAIN_ID):
  ros2 launch jetracer_description view_robot.launch.py use_rviz:=true use_cmd_vel_joints:=true

  # Headless — just robot_state_publisher + TF (for testing TF without RViz):
  ros2 launch jetracer_description view_robot.launch.py

Migration note
--------------
Replaces the ROS 1 `cytron_jetracer/launch/rviz.launch` which used:
  - `$(find xacro)/xacro --inorder '$(find cytron_jetracer)/urdf/jetracer.xacro'`
  - `node pkg=rviz type=rviz`
  - `node pkg=cytron_jetracer type=joint_state.py`
All updated to ROS 2 Python launch API.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _to_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _launch_setup(context, *args, **kwargs):
    pkg_dir = Path(get_package_share_directory('jetracer_description'))
    urdf_path = pkg_dir / 'urdf' / 'jetracer.urdf.xacro'
    rviz_config = pkg_dir / 'rviz' / 'jetracer.rviz'

    use_sim_time = _to_bool(LaunchConfiguration('use_sim_time').perform(context))
    use_rviz = _to_bool(LaunchConfiguration('use_rviz').perform(context))
    use_joint_state_gui = _to_bool(
        LaunchConfiguration('use_joint_state_gui').perform(context))
    use_cmd_vel_joints = _to_bool(
        LaunchConfiguration('use_cmd_vel_joints').perform(context))
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context).strip()

    # Xacro → robot_description string
    import subprocess
    try:
        result = subprocess.run(
            ['xacro', str(urdf_path)],
            capture_output=True, text=True, check=True,
        )
        robot_description = result.stdout
    except Exception as exc:
        raise RuntimeError(
            f"Failed to process URDF with xacro: {exc}\n"
            f"Ensure xacro is installed: sudo apt install ros-$ROS_DISTRO-xacro"
        ) from exc

    nodes = []

    # ── robot_state_publisher ─────────────────────────────────────────────────
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

    # ── joint_state_publisher_gui (interactive sliders) ───────────────────────
    # Mutually exclusive with use_cmd_vel_joints.
    if use_joint_state_gui and not use_cmd_vel_joints:
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    elif not use_cmd_vel_joints:
        # Neither mode: publish zero joint states so robot_state_publisher is happy
        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    # ── cmd_vel-driven joint state publisher (visualization from live stack) ──
    if use_cmd_vel_joints:
        nodes.append(Node(
            package='jetracer_description',
            executable='joint_state_publisher.py',
            name='jetracer_joint_state_publisher',
            output='screen',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'use_sim_time': use_sim_time,
            }],
        ))

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    if use_rviz:
        if not rviz_config.exists():
            raise RuntimeError(f'RViz config not found: {rviz_config}')
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (bag replay) clock.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz2 for model visualisation.',
        ),
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='false',
            description=(
                'Start joint_state_publisher_gui with interactive sliders. '
                'Good for inspecting the URDF geometry. '
                'Mutually exclusive with use_cmd_vel_joints.'
            ),
        ),
        DeclareLaunchArgument(
            'use_cmd_vel_joints',
            default_value='false',
            description=(
                'Start the cmd_vel-driven joint state publisher to animate the '
                'robot model from a running autonomous stack or bag replay. '
                'Mutually exclusive with use_joint_state_gui.'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='cmd_vel topic to drive joint animation from.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
