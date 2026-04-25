"""
Launch integration test for the JetRacer autonomy stack.

Starts the full autonomy.launch.py (camera off, hardware on) and verifies:
  1. Critical nodes log their initialisation messages within a timeout.
  2. Core safety and control topics are advertised.

Run with:
    colcon test --packages-select jetracer_bringup
    colcon test-result --verbose
"""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Launch the autonomy stack in a hardware-free test mode:
      - start_camera=false   skip CSI camera (not available in CI)
      - start_base=true      start serial hardware node (uses dry_run)
      - start_lidar=false    no RPLidar in CI
      - dry_run=true         suppress all serial writes
    """
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'),
                'launch',
                'autonomy.launch.py',
            ])
        ),
        launch_arguments={
            'start_camera': 'false',
            'start_lidar': 'false',
            'start_yolo': 'false',
            'dry_run': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        launch_description,
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutonomyBringup(unittest.TestCase):

    def test_node_startup(self, proc_output):
        """
        Wait for the stack to start and verify that core nodes log their
        initialisation messages within the timeout.
        """
        proc_output.assertWaitFor('Safety supervisor initialized.', timeout=20)
        proc_output.assertWaitFor('Startup speed limit active', timeout=20)

    def test_topic_advertising(self):
        """
        Verify that critical safety and control topics are advertised by the stack.
        Retries up to 10 seconds to allow nodes time to start.
        """
        rclpy.init()

        class _Probe(rclpy.node.Node):
            def __init__(self):
                super().__init__('autonomy_test_probe')

        probe = _Probe()
        critical_topics = {
            'cmd_vel_safety':   'geometry_msgs/msg/Twist',
            'cmd_vel_lane':     'geometry_msgs/msg/Twist',
        }

        try:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                advertised = {
                    topic: types[0] if types else 'unknown'
                    for topic, types in probe.get_topic_names_and_types()
                }
                if all(t in advertised for t in critical_topics):
                    break
                rclpy.spin_once(probe, timeout_sec=0.5)

            advertised = {
                topic: types[0] if types else 'unknown'
                for topic, types in probe.get_topic_names_and_types()
            }
            for topic in critical_topics:
                self.assertIn(
                    topic, advertised,
                    f"Critical topic '{topic}' not advertised by autonomy stack",
                )
        finally:
            probe.destroy_node()
            rclpy.try_shutdown()
