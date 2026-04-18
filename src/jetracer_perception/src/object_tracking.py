#!/usr/bin/env python3
# Object tracking node for JetRacer (ROS 2)
# Uses OpenCV CSRT tracker (robust to partial occlusion) to follow an object
# selected by the user at startup (largest blob in the first frame).
# Publishes cmd_vel to keep the object centred horizontally.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class ObjectTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('object_tracking')

        # Initial selection colour (HSV) — used to bootstrap the tracker
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 180)
        self.declare_parameter('s_min', 50)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)

        self.declare_parameter('kp', 0.002)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('min_blob_area', 500.0)
        self.declare_parameter('start', False)
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'object_tracking/compressed')

        self._bridge = CvBridge()
        self._tracker = None
        self._tracking = False
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        camera_topic = str(self.get_parameter('camera_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, qos_profile_sensor_data)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_vision', 1)

        self.get_logger().info('object_tracking ready.')

    def _create_tracker(self):
        """Create a robust OpenCV tracker across OpenCV API variants."""
        if hasattr(cv2, 'TrackerCSRT_create'):
            return cv2.TrackerCSRT_create()
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
            return cv2.legacy.TrackerCSRT_create()
        if hasattr(cv2, 'TrackerKCF_create'):
            self.get_logger().warn('CSRT tracker unavailable; falling back to KCF tracker.')
            return cv2.TrackerKCF_create()
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerKCF_create'):
            self.get_logger().warn('CSRT tracker unavailable; falling back to KCF tracker.')
            return cv2.legacy.TrackerKCF_create()
        raise RuntimeError('No supported OpenCV tracker factory found (CSRT/KCF).')

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
        self._linear = float(self.get_parameter('linear_speed').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
        self._min_area = float(self.get_parameter('min_blob_area').value)
        self._start = bool(self.get_parameter('start').value)
        # Reset tracker when parameters change
        self._tracker = None
        self._tracking = False

    def _param_callback(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def _init_tracker(self, frame: np.ndarray) -> bool:
        """Bootstrap the CSRT tracker using the largest blob of the target colour."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < self._min_area:
            return False
        x, y, w, h = cv2.boundingRect(cnt)
        try:
            self._tracker = self._create_tracker()
            self._tracker.init(frame, (x, y, w, h))
        except Exception as exc:
            self.get_logger().error(f'Failed to initialize tracker backend: {exc}')
            self._tracker = None
            self._tracking = False
            return False
        self._tracking = True
        self.get_logger().info(f'Tracker initialised at ({x},{y}) {w}×{h}')
        return True

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        h, w = frame.shape[:2]
        cmd = Twist()

        if not self._tracking:
            if self._start:
                self._init_tracker(frame)
        else:
            ok, bbox = self._tracker.update(frame)
            if ok:
                bx, by, bw, bh = [int(v) for v in bbox]
                cx = bx + bw // 2
                err = float(w // 2 - cx)
                angular = self._kp * err
                angular = max(-self._max_ang, min(self._max_ang, angular))

                if self._start:
                    cmd.linear.x = self._linear
                    cmd.angular.z = angular

                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.line(frame, (w // 2, h), (cx, by + bh // 2), (255, 0, 0), 2)
            else:
                self.get_logger().warn('Tracker lost — reinitialising.')
                self._tracking = False

        self._cmd_pub.publish(cmd)
        try:
            self._img_pub.publish(self._bridge.cv2_to_compressed_imgmsg(frame))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectTrackingNode()
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
