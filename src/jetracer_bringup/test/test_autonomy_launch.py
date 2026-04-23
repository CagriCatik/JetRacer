import os
import unittest
from typing import List

import launch
import launch_ros
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Launches the full autonomy stack in a test sandbox.
    """
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('jetracer_bringup'),
                'launch',
                'autonomy.launch.py'
            ])
        ),
        launch_arguments={'start_camera': 'false'}.items(), # Skip camera in CI if possible
    )

    return launch.LaunchDescription([
        launch_description,
        launch_testing.actions.ReadyToTest(),
    ])

class TestAutonomyBringup(unittest.TestCase):

    def test_node_startup(self, proc_output):
        """
        Wait for the stack to start and verify that nodes are initialized.
        """
        # We expect to see the initialization log from at least one core node
        proc_output.assertWaitFor('LiDAR Collision Assurance initialized.', timeout=10)
        proc_output.assertWaitFor('Initialized C++ Ackermann Bridge', timeout=10)

    def test_topic_advertising(self):
        """
        Verify that critical safety and control topics are advertised by the stack.
        This checks that core autonomy nodes are actually publishing expected outputs.
        """
        # Initialize ROS 2 context for topic discovery
        rclpy.init()
        test_node = Node('autonomy_test_monitor')
        
        try:
            # Wait for critical topics to be advertised
            critical_topics = {
                'cmd_vel_lane': 'geometry_msgs/msg/Twist',  # Lane following output
                'cmd_vel_safety': 'geometry_msgs/msg/Twist',  # Safety arbitration output
                'lane_following/debug_image/compressed': 'sensor_msgs/msg/CompressedImage',  # Vision feedback
                'detections': 'vision_msgs/msg/Detection2DArray',  # YOLO output
            }
            
            for attempt in range(10):  # Retry up to 10 times (10 seconds total)
                advertised = test_node.get_topic_names_and_types()
                advertised_dict = {topic: types[0] if types else 'unknown' 
                                 for topic, types in advertised}
                
                all_found = all(topic in advertised_dict for topic in critical_topics.keys())
                if all_found:
                    break
                
                if attempt < 9:
                    rclpy.spin_once(test_node, timeout_sec=1.0)
            
            # Assert that all critical topics are now advertised
            advertised = {topic: types[0] if types else 'unknown' 
                         for topic, types in test_node.get_topic_names_and_types()}
            
            for topic, expected_type in critical_topics.items():
                self.assertIn(topic, advertised, 
                            f"Critical topic '{topic}' not advertised by autonomy stack")
                
        finally:
            test_node.destroy_node()
            rclpy.shutdown()

