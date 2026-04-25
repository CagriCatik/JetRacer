#!/usr/bin/env python3

from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class MultipointNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__('multipoint_navigation')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('marker_topic', 'path_point')
        self.declare_parameter('clicked_point_topic', 'clicked_point')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('forward_initialpose_topic', 'rtabmap/initialpose')
        self.declare_parameter('loop_forever', True)
        self.declare_parameter('retry_failed_once', True)

        self.goal_frame = str(self.get_parameter('goal_frame').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        clicked_point_topic = str(self.get_parameter('clicked_point_topic').value)
        initial_pose_topic = str(self.get_parameter('initial_pose_topic').value)
        forward_initialpose_topic = str(self.get_parameter('forward_initialpose_topic').value)
        self.loop_forever = bool(self.get_parameter('loop_forever').value)
        self.retry_failed_once = bool(self.get_parameter('retry_failed_once').value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, qos)
        self.clicked_sub = self.create_subscription(PointStamped, clicked_point_topic, self.click_callback, 10)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, initial_pose_topic, self.initialpose_callback, 10)
        self.forward_initialpose_pub = None
        if forward_initialpose_topic and forward_initialpose_topic != initial_pose_topic:
            self.forward_initialpose_pub = self.create_publisher(
                PoseWithCovarianceStamped, forward_initialpose_topic, 10)

        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.waypoints: list[PointStamped] = []
        self.active_index: Optional[int] = None
        self.retry_available = True
        self.goal_handle = None
        self.create_timer(1.0, self.publish_markers)

    def destroy_node(self) -> None:
        self.cancel_active_goal()
        super().destroy_node()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.cancel_active_goal()
        self.waypoints.clear()
        self.active_index = None
        self.retry_available = True
        self.publish_markers(clear=True)
        if self.forward_initialpose_pub is not None:
            self.forward_initialpose_pub.publish(msg)

    def click_callback(self, msg: PointStamped) -> None:
        self.waypoints.append(msg)
        self.get_logger().info(f'Added target point {len(self.waypoints) - 1}.')
        self.publish_markers()
        if self.active_index is None and len(self.waypoints) == 1:
            self.dispatch_goal(0)

    def publish_markers(self, clear: bool = False) -> None:
        marker_array = MarkerArray()
        if clear:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        for index, point in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = self.goal_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'jetracer_multipoint_nav'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.6
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = point.point.x
            marker.pose.position.y = point.point.y
            marker.pose.position.z = point.point.z
            marker.pose.orientation.w = 1.0
            marker.text = str(index)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def cancel_active_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def dispatch_goal(self, index: int) -> None:
        if index < 0 or index >= len(self.waypoints):
            return
        if not self.navigator.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Nav2 navigate_to_pose action server.')
            return

        target = self.waypoints[index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target.point.x
        goal.pose.pose.position.y = target.point.y
        goal.pose.pose.position.z = target.point.z
        goal.pose.pose.orientation.w = 1.0

        self.active_index = index
        send_future = self.navigator.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send Nav2 goal: {exc}')
            self.handle_result(GoalStatus.STATUS_ABORTED)
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the patrol goal.')
            self.handle_result(GoalStatus.STATUS_ABORTED)
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal result failed: {exc}')
            self.goal_handle = None
            self.handle_result(GoalStatus.STATUS_ABORTED)
            return
        self.goal_handle = None
        self.handle_result(result.status)

    def handle_result(self, status: int) -> None:
        if self.active_index is None or not self.waypoints:
            return

        current_index = self.active_index
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached target point {current_index}.')
            self.retry_available = True
            next_index = current_index + 1
            if next_index >= len(self.waypoints):
                if self.loop_forever:
                    next_index = 0
                else:
                    self.active_index = None
                    return
            self.dispatch_goal(next_index)
            return

        self.get_logger().warn(f'Unable to reach target point {current_index}.')
        if self.retry_failed_once and self.retry_available:
            self.retry_available = False
            self.get_logger().warn(f'Retrying target point {current_index}.')
            self.dispatch_goal(current_index)
            return

        self.retry_available = True
        next_index = current_index + 1
        if next_index >= len(self.waypoints):
            if self.loop_forever:
                next_index = 0
            else:
                self.active_index = None
                return
        self.dispatch_goal(next_index)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultipointNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
