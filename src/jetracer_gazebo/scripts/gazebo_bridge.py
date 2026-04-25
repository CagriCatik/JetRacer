#!/usr/bin/env python3
"""
Gazebo Classic ↔ ROS 2 Topic Bridge

Bridges key topics from Gazebo Classic to ROS 2 standard namespaces:
  /gazebo/model_states         → /odom (odometry)
  /gazebo/<model>/imu/data     → /imu/data
  /gazebo/<model>/camera/image → /csi_cam_0/image_raw
  /gazebo/<model>/camera/info  → /csi_cam_0/camera_info
  /gazebo/<model>/scan        → /scan
  
  ROS 2 /cmd_vel              → /gazebo/<model>/cmd_vel (for velocity commands)

This replaces ros_gz_bridge functionality for Gazebo Classic 11 compatibility.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster
import math


class GazeboBridge(Node):
    def __init__(self):
        super().__init__('gazebo_bridge')
        
        # Subscribe to Gazebo topics
        self.gazebo_odom_sub = self.create_subscription(
            Odometry,
            '/gazebo/model_states',
            self.gazebo_odom_callback,
            10
        )
        
        self.gazebo_imu_sub = self.create_subscription(
            Imu,
            '/gazebo/jetracer/imu/data',
            self.gazebo_imu_callback,
            10
        )
        
        self.gazebo_image_sub = self.create_subscription(
            Image,
            '/gazebo/jetracer/camera/image_raw',
            self.gazebo_image_callback,
            10
        )
        
        self.gazebo_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/gazebo/jetracer/camera/camera_info',
            self.gazebo_camera_info_callback,
            10
        )
        
        self.gazebo_scan_sub = self.create_subscription(
            LaserScan,
            '/gazebo/jetracer/scan',
            self.gazebo_scan_callback,
            10
        )
        
        # Subscribe to ROS 2 standard commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to ROS 2 standard topics
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.image_pub = self.create_publisher(Image, '/csi_cam_0/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/csi_cam_0/camera_info', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Publish commands to Gazebo
        self.gazebo_cmd_vel_pub = self.create_publisher(Twist, '/gazebo/jetracer/cmd_vel', 10)
        
        # TF broadcaster for odom frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Gazebo Bridge initialized')
    
    def gazebo_odom_callback(self, msg: Odometry):
        """Forward Gazebo odometry to ROS 2 /odom topic."""
        self.odom_pub.publish(msg)
        
        # Also publish TF (base_link → odom)
        try:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warn(f'TF broadcast failed: {e}')
    
    def gazebo_imu_callback(self, msg: Imu):
        """Forward Gazebo IMU to ROS 2 /imu/data topic."""
        self.imu_pub.publish(msg)
    
    def gazebo_image_callback(self, msg: Image):
        """Forward Gazebo camera image to ROS 2 /csi_cam_0/image_raw topic."""
        self.image_pub.publish(msg)
    
    def gazebo_camera_info_callback(self, msg: CameraInfo):
        """Forward Gazebo camera info to ROS 2 /csi_cam_0/camera_info topic."""
        self.camera_info_pub.publish(msg)
    
    def gazebo_scan_callback(self, msg: LaserScan):
        """Forward Gazebo LaserScan to ROS 2 /scan topic."""
        self.scan_pub.publish(msg)
    
    def cmd_vel_callback(self, msg: Twist):
        """Forward ROS 2 /cmd_vel commands to Gazebo."""
        self.gazebo_cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = GazeboBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
