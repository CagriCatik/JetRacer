#!/usr/bin/env python3
# Line-following node for JetRacer (ROS 2)
# Detects a coloured line in the bottom half of the camera image and steers
# toward its centroid using a PD controller on lateral error.
# All thresholds and gains are live-tunable via ROS 2 parameters.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class LineFollowNode(Node):
    def __init__(self) -> None:
        super().__init__('line_follow')

        # HSV bounds — default: yellow line
        self.declare_parameter('h_min', 20)
        self.declare_parameter('h_max', 40)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('v_max', 255)

        # Control
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('kd', 0.002)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('min_blob_area', 300.0)
        # Fraction of image height at which to split ROI (upper half ignored)
        self.declare_parameter('roi_top_fraction', 0.5)
        self.declare_parameter('start', False)

        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'line_follow/compressed')

        self._bridge = CvBridge()
        self._last_err = 0.0
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        camera_topic = str(self.get_parameter('camera_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, 1)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_vision', 1)

        self.get_logger().info('line_follow ready (start=false by default).')

    def _load_params(self) -> None:
        self._lower = np.array([
            int(self.get_parameter('h_min').value),
            int(self.get_parameter('s_min').value),
            int(self.get_parameter('v_min').value),
        ])
        self._upper = np.array([
            int(self.get_parameter('h_max').value),
            int(self.get_parameter('s_max').value),
            int(self.get_parameter('v_max').value),
        ])
        self._kp = float(self.get_parameter('kp').value)
        self._kd = float(self.get_parameter('kd').value)
        self._linear = float(self.get_parameter('linear_speed').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
        self._min_area = float(self.get_parameter('min_blob_area').value)
        self._roi_top = float(self.get_parameter('roi_top_fraction').value)
        self._start = bool(self.get_parameter('start').value)

    def _param_callback(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        h, w = frame.shape[:2]
        roi_start = int(h * self._roi_top)

        roi = frame.copy()
        roi[:roi_start, :] = 0   # blank upper half

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        gray = cv2.bitwise_and(
            cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY),
            cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY),
            mask=mask,
        )
        _, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cmd = Twist()
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) >= self._min_area:
                (cx, cy), _ = cv2.minEnclosingCircle(cnt)

                # Lateral error normalised by remaining distance to bottom
                err = float(w / 2 - cx) / float(max(h - cy, 1))
                angular = (self._kp * err + self._kd * (err - self._last_err))
                angular = max(-self._max_ang, min(self._max_ang, angular))
                self._last_err = err

                if self._start:
                    cmd.linear.x = self._linear
                    cmd.angular.z = angular

                # Annotate
                cv2.drawContours(
                    frame,
                    [cv2.boxPoints(cv2.minAreaRect(cnt)).astype(int)],
                    0, (0, 255, 0), 2,
                )
                cv2.line(frame, (w // 2, h), (int(cx), int(cy)), (255, 0, 0), 2)

        self._cmd_pub.publish(cmd)
        try:
            self._img_pub.publish(self._bridge.cv2_to_compressed_imgmsg(frame))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
