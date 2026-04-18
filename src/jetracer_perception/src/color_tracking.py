#!/usr/bin/env python3
# Color tracking node for JetRacer (ROS 2)
# Subscribes to a compressed camera image, tracks a colour blob via HSV masking,
# and publishes cmd_vel Twist commands to keep the blob centred.
# HSV thresholds and PD gains are live-tunable via ROS 2 parameters.

import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ColorTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('color_tracking')

        # HSV thresholds — default tracks blue
        self.declare_parameter('h_min', 100)
        self.declare_parameter('h_max', 140)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('v_max', 255)

        # Control gains
        self.declare_parameter('kp', 0.003)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('min_blob_area', 200.0)
        self.declare_parameter('start', False)

        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'color_tracking/compressed')

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self._bridge = CvBridge()
        self._last_err = 0.0

        camera_topic = str(self.get_parameter('camera_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, 1)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_vision', 1)

        self.get_logger().info('color_tracking ready (start=false by default).')

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
        self._start = bool(self.get_parameter('start').value)

    def _param_callback(self, params):
        for p in params:
            try:
                setattr(self, f'_{p.name}', p.value)
            except AttributeError:
                pass
        self._load_params()
        return SetParametersResult(successful=True)

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        h, w = frame.shape[:2]

        # Only look in the bottom half of the image
        roi = frame.copy()
        roi[: h // 2, :] = 0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        result = cv2.bitwise_and(roi, roi, mask=mask)

        contours, _ = cv2.findContours(
            cv2.cvtColor(result, cv2.COLOR_BGR2GRAY),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        cmd = Twist()
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) >= self._min_area:
                (cx, cy), _ = cv2.minEnclosingCircle(cnt)
                err = float(w / 2 - cx) / float(max(h - cy, 1))
                angular = self._kp * err + self._kd * (err - self._last_err)
                angular = max(-self._max_ang, min(self._max_ang, angular))
                self._last_err = err

                if self._start:
                    cmd.linear.x = self._linear
                    cmd.angular.z = angular

                # Annotate frame
                cv2.drawContours(frame, [cv2.boxPoints(cv2.minAreaRect(cnt)).astype(int)], 0, (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        self._cmd_pub.publish(cmd)

        # Publish annotated image
        try:
            self._img_pub.publish(self._bridge.cv2_to_compressed_imgmsg(frame))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorTrackingNode()
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
