#!/usr/bin/env python3
"""
YOLO11 Semantic Object Detection Node

This module bridges the Ultralytics deep-learning library directly into the ROS 2 space.
It subscribes to a heavily compressed PyTorch video feed, performs GPU-accelerated inference
(via TensorRT/CUDA), and translates proprietary YOLO bounds into standard ROS 2
vision_msgs primitives. This allows node-agnostic behavior mapping.
"""

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Graceful degradation logic for standalone builds missing heavy ML dependencies
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloDetectionNode(Node):
    """
    Subscribes to image streams, computes deep-learning bounding boxes, and publishes
    Detection2D arrays for system-wide consumption.
    """

    def __init__(self) -> None:
        """Initializes thresholds, networking, and pre-warms the PyTorch model."""
        super().__init__('yolo_detection')

        if YOLO is None:
            self.get_logger().error("Ultralytics package not found. Run: pip install ultralytics")
            raise RuntimeError("Missing Core Dependency: ultralytics")

        self.declare_parameter('model_path', 'yolov11n.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'perception/yolo_debug/compressed')
        self.declare_parameter('detection_topic', 'perception/yolo_detections')
        self.declare_parameter('publish_debug', True)

        self._bridge = CvBridge()
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self.get_logger().info(f"Mapping YOLO Weights: {self._model_path}")
        self._model = YOLO(self._model_path)
        
        self._warmup_model()
        self.get_logger().info("YOLO Model Architecture Loaded & Warmed Up.")

        # Networking
        camera_topic: str = str(self.get_parameter('camera_topic').value)
        output_topic: str = str(self.get_parameter('output_topic').value)
        detection_topic: str = str(self.get_parameter('detection_topic').value)

        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, 1)
        
        self._det_pub = self.create_publisher(Detection2DArray, detection_topic, 10)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)

        self.get_logger().info('YOLO Semantic Parser Online.')

    def _load_params(self) -> None:
        self._model_path: str = str(self.get_parameter('model_path').value)
        self._conf_thres: float = float(self.get_parameter('conf_thres').value)
        self._device: str = str(self.get_parameter('device').value)
        self._publish_debug: bool = bool(self.get_parameter('publish_debug').value)

    def _param_callback(self, params) -> SetParametersResult:
        previous_model_path = self._model_path
        previous_device = self._device
        previous_model = self._model
        self._load_params()
        if self._model_path != previous_model_path:
            requested_model_path = self._model_path
            try:
                self._model = YOLO(self._model_path)
                self._warmup_model()
                self.get_logger().info(f"Loaded new YOLO model: {self._model_path}")
            except Exception as exc:
                self._model_path = previous_model_path
                self._device = previous_device
                self._model = previous_model
                self.get_logger().error(f"Failed to load model '{requested_model_path}': {exc}")
                return SetParametersResult(successful=False, reason='failed to load model_path')
        return SetParametersResult(successful=True)

    def _warmup_model(self) -> None:
        # GPU graph warm-up prevents a heavy latency spike on the first frame.
        dummy_img: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            self._model.predict(source=dummy_img, conf=self._conf_thres, verbose=False, device=self._device)
        except Exception as exc:
            if self._device.lower() != 'cpu':
                self.get_logger().warn(
                    f"YOLO warm-up failed on '{self._device}' ({exc}); falling back to CPU."
                )
                self._device = 'cpu'
                self._model.predict(source=dummy_img, conf=self._conf_thres, verbose=False, device=self._device)
            else:
                raise

    def _predict(self, frame: np.ndarray):
        try:
            return self._model.predict(
                source=frame,
                conf=self._conf_thres,
                verbose=False,
                device=self._device,
            )
        except Exception as exc:
            if self._device.lower() != 'cpu':
                self.get_logger().warn(
                    f"YOLO inference failed on '{self._device}' ({exc}); retrying on CPU."
                )
                self._device = 'cpu'
                return self._model.predict(
                    source=frame,
                    conf=self._conf_thres,
                    verbose=False,
                    device=self._device,
                )
            raise

    def _image_callback(self, msg: CompressedImage) -> None:
        """
        Translates compressed ROS images to CV2 matrices, infers tensors,
        and packs the output back into ROS network matrices.
        """
        try:
            frame: np.ndarray = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f"Image translation exception (cv_bridge): {exc}")
            return

        # Core Inference Call -> Forces GPU to avoid CPU lockout
        try:
            results = self._predict(frame)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        if len(results) > 0:
            result = results[0]
            boxes = result.boxes
            
            for box in boxes:
                # Extract PyTorch unrolled coordinate vectors
                x, y, w, h = box.xywh[0].tolist()
                conf: float = float(box.conf[0])
                cls_id: int = int(box.cls[0])
                cls_name: str = str(result.names[cls_id])

                # Architect the formal vision message
                detection = Detection2D()
                detection.header = msg.header
                
                detection.bbox.center.position.x = float(x)
                detection.bbox.center.position.y = float(y)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = cls_name
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)

                det_msg.detections.append(detection)

            # Optional telemetry output for RViz or Foxglove Studio
            if self._publish_debug and len(boxes) > 0:
                annotated_frame: np.ndarray = result.plot()
                try:
                    debug_msg = self._bridge.cv2_to_compressed_imgmsg(annotated_frame)
                    debug_msg.header = msg.header
                    self._img_pub.publish(debug_msg)
                except Exception as exc:
                    self.get_logger().error(f"Failed to generate output topology map: {exc}")

        # Always publish the root array. If it's empty, listeners know their path is physically clear.
        self._det_pub.publish(det_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except RuntimeError as re:
        print(f"Node execution aborted due to missing dependencies: {re}")
    except Exception as e:
        print(f"Exception during semantic processing: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
