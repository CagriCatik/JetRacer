#!/usr/bin/env python3
"""
Spawn a robot model into Gazebo Classic via ROS 2 service.

This script calls the /spawn_model ROS 2 service provided by gazebo_ros.
It is used as a replacement for ros_gz_sim's spawn functionality in Gazebo Harmonic,
to maintain compatibility with Gazebo Classic 11 in ROS 2 Foxy.
"""

import argparse
import sys
import time

import rclpy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


def main():
    parser = argparse.ArgumentParser(description='Spawn a robot model into Gazebo.')
    parser.add_argument('--name', type=str, default='jetracer', help='Model name')
    parser.add_argument('--x', type=float, default=0.0, help='X coordinate')
    parser.add_argument('--y', type=float, default=0.0, help='Y coordinate')
    parser.add_argument('--z', type=float, default=0.05, help='Z coordinate')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle in radians')
    parser.add_argument('--urdf', type=str, default='', help='URDF file path')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('spawn_model')
    
    # Create service client for /spawn_model
    cli = node.create_client(SpawnModel, '/spawn_model')
    
    # Wait for the service to be available
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn_model service...')
        time.sleep(1)
    
    # Get robot_description from parameter server
    robot_description = node.declare_parameter(
        'robot_description',
        rclpy.Parameter.Type.STRING
    ).value
    
    if not robot_description:
        node.get_logger().error('robot_description parameter not set')
        return 1
    
    # Build the spawn request
    req = SpawnModel.Request()
    req.model_name = args.name
    req.model_xml = robot_description
    req.initial_pose = Pose(
        position=Point(x=float(args.x), y=float(args.y), z=float(args.z)),
        orientation=Quaternion(x=0.0, y=0.0, z=sys.float_info.epsilon, w=1.0),
    )
    # Note: Gazebo Classic uses a simple Pose; yaw is encoded in orientation.
    # For now, we use a simple identity quaternion. The actual yaw should be
    # handled separately or by generating the correct quaternion.
    
    node.get_logger().info(f'Spawning {args.name} at ({args.x}, {args.y}, {args.z})')
    
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info(f'Model {args.name} spawned successfully')
        result = future.result()
        if result.success:
            node.get_logger().info(result.status_message)
            return 0
        else:
            node.get_logger().error(result.status_message)
            return 1
    else:
        node.get_logger().error('Service call failed')
        return 1


if __name__ == '__main__':
    sys.exit(main())
