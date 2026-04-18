#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomPoseToOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_pose_to_odometry')
        self.declare_parameter('input_topic', 'odom_combined')
        self.declare_parameter('output_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)

        self.publisher = self.create_publisher(Odometry, output_topic, 5)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, input_topic, self.callback, 5)

    def callback(self, msg: PoseWithCovarianceStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        # Note: This is an architectural limitation. This node acts as a pose-only bridge.
        # It copies the 'pose' field but leaves the 'twist' (velocity) field unpopulated 
        # because PoseWithCovarianceStamped does not contain velocity information.
        odom.pose = msg.pose
        self.publisher.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPoseToOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()