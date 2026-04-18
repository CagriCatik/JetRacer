#!/usr/bin/env python3
# Face detection node for JetRacer (ROS 2)
# Uses OpenCV's Haar cascade (frontal face, bundled with opencv) to detect faces
# and optionally steers the robot to keep a detected face centred.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


class FaceDetectNode(Node):
    # Path to the bundled OpenCV Haar cascade
    _CASCADE_PATH = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'

    def __init__(self) -> None:
        super().__init__('face_detect')

        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbours', 5)
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('linear_speed', 0.0)   # 0 = don't drive, just detect
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('start', False)
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'face_detect/compressed')

        self._bridge = CvBridge()
        self._cascade = cv2.CascadeClassifier(self._CASCADE_PATH)
        if self._cascade.empty():
            raise RuntimeError(f'Failed to load cascade from {self._CASCADE_PATH}')
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        camera_topic = str(self.get_parameter('camera_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, 1)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_vision', 1)
        self._count_pub = self.create_publisher(Int32, 'face_detect/count', 10)

        self.get_logger().info('face_detect ready.')

    def _load_params(self) -> None:
        self._scale = float(self.get_parameter('scale_factor').value)
        self._neighbours = int(self.get_parameter('min_neighbours').value)
        self._min_size = int(self.get_parameter('min_face_size').value)
        self._kp = float(self.get_parameter('kp').value)
        self._linear = float(self.get_parameter('linear_speed').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=self._scale,
            minNeighbors=self._neighbours,
            minSize=(self._min_size, self._min_size),
        )

        count_msg = Int32()
        count_msg.data = len(faces) if len(faces) else 0
        self._count_pub.publish(count_msg)

        cmd = Twist()
        if len(faces):
            # Track the largest face
            x, y, fw, fh = max(faces, key=lambda f: f[2] * f[3])
            cx = x + fw // 2
            err = float(w // 2 - cx)
            angular = self._kp * err
            angular = max(-self._max_ang, min(self._max_ang, angular))

            if self._start:
                cmd.linear.x = self._linear
                cmd.angular.z = angular

            for (fx, fy, ffx, ffy) in faces:
                cv2.rectangle(frame, (fx, fy), (fx + ffx, fy + ffy), (0, 255, 0), 2)
            cv2.putText(frame, f'Faces: {len(faces)}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        self._cmd_pub.publish(cmd)
        try:
            self._img_pub.publish(self._bridge.cv2_to_compressed_imgmsg(frame))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceDetectNode()
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
