#!/usr/bin/env python3
# Motion detection node for JetRacer (ROS 2)
# Uses frame differencing to detect moving regions in the camera image.
# Publishes a boolean String ("motion_detected: true/false") and an
# annotated CompressedImage showing detected motion areas.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class MotionDetectNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_detect')

        self.declare_parameter('min_area', 500.0)
        self.declare_parameter('blur_size', 21)
        self.declare_parameter('threshold', 25)
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'motion_detect/compressed')
        self.declare_parameter('status_topic', 'motion_detect/status')

        self._bridge = CvBridge()
        self._prev_gray = None
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        camera_topic = str(self.get_parameter('camera_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, qos_profile_sensor_data)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._status_pub = self.create_publisher(String, status_topic, 10)

        self.get_logger().info('motion_detect ready.')

    def _load_params(self) -> None:
        self._min_area = float(self.get_parameter('min_area').value)
        blur = int(self.get_parameter('blur_size').value)
        self._blur_size = blur if blur % 2 == 1 else blur + 1   # must be odd
        self._threshold = int(self.get_parameter('threshold').value)

    def _param_callback(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self._blur_size, self._blur_size), 0)

        motion_detected = False
        if self._prev_gray is not None:
            diff = cv2.absdiff(self._prev_gray, gray)
            _, thresh = cv2.threshold(diff, self._threshold, 255, cv2.THRESH_BINARY)
            thresh = cv2.dilate(thresh, None, iterations=2)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) >= self._min_area:
                    motion_detected = True
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        self._prev_gray = gray

        status = String()
        status.data = f'motion_detected: {str(motion_detected).lower()}'
        self._status_pub.publish(status)

        label = 'MOTION' if motion_detected else 'NO MOTION'
        colour = (0, 0, 255) if motion_detected else (0, 255, 0)
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

        try:
            self._img_pub.publish(self._bridge.cv2_to_compressed_imgmsg(frame))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
