#!/usr/bin/env python3
"""
YOLO11 Semantic Object Detection Node

This module bridges the Ultralytics deep-learning library directly into the ROS 2 space.
It subscribes to a heavily compressed PyTorch video feed, performs GPU-accelerated inference
(via TensorRT/CUDA), and translates proprietary YOLO bounds into standard ROS 2
vision_msgs primitives. This allows node-agnostic behavior mapping.

Reliability improvements:
  - 3-second post-init delay before subscribing to the camera. This gives the
    GStreamer / nvarguscamerasrc CSI pipeline time to stabilize before YOLO
    demands the first frames, preventing a burst of stale-frame skips on startup.
  - Frame age check: frames older than max_frame_age_sec are skipped with a
    throttled warning. An empty detection array is still published so downstream
    consumers (semantic_behavior) know YOLO is alive but degraded.
  - Per-frame FPS tracking published as perception/yolo_fps (std_msgs/Float32)
    and reported to the diagnostic_updater.
"""

import os
import time
from ament_index_python.packages import get_package_share_directory
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from diagnostic_updater import Updater, FunctionDiagnosticTask
    from diagnostic_msgs.msg import DiagnosticStatus
    _HAS_DIAG = True
except ImportError:
    _HAS_DIAG = False

# Graceful degradation logic for standalone builds missing heavy ML dependencies
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class YoloDetectionNode(Node):
    def __init__(self) -> None:
        """Initializes thresholds, networking, and pre-warms the PyTorch model."""
        super().__init__('yolo_detection')

        if YOLO is None:
            self.get_logger().error("Ultralytics package not found. Run: pip install ultralytics")
            raise RuntimeError("Missing Core Dependency: ultralytics")

        # Resolve package models directory
        package_share_dir = get_package_share_directory('jetracer_perception')
        default_model = os.path.join(package_share_dir, 'models', 'yolo11n-best.pt')

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')
        self.declare_parameter('output_topic', 'perception/yolo_debug/compressed')
        self.declare_parameter('detection_topic', 'perception/yolo_detections')
        self.declare_parameter('publish_debug', True)
        # Frame age: skip frames older than this many seconds (stale under load)
        self.declare_parameter('max_frame_age_sec', 0.15)
        # Post-init delay: wait before subscribing to camera so CSI pipeline stabilises
        self.declare_parameter('camera_start_delay_sec', 3.0)

        self._bridge = CvBridge()
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # Logic for path portability
        model_path = self._model_path
        if not os.path.isabs(model_path):
            model_path = os.path.join(package_share_dir, 'models', model_path)

        self._model_path = model_path
        self.get_logger().info(f"Mapping YOLO Weights: {self._model_path}")
        self._model = YOLO(self._model_path)

        self._warmup_model()
        self.get_logger().info("YOLO Model Architecture Loaded & Warmed Up.")

        # FPS tracking
        self._last_callback_wall: float = 0.0
        self._fps_ema: float = 0.0          # Exponential moving average
        self._fps_ema_alpha: float = 0.1    # Smoothing factor

        # Networking
        camera_topic: str = str(self.get_parameter('camera_topic').value)
        output_topic: str = str(self.get_parameter('output_topic').value)
        detection_topic: str = str(self.get_parameter('detection_topic').value)

        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._det_pub = self.create_publisher(Detection2DArray, detection_topic, 10)
        self._img_pub = self.create_publisher(CompressedImage, output_topic, 1)
        self._fps_pub = self.create_publisher(Float32, 'perception/yolo_fps', 10)

        # Diagnostics
        if _HAS_DIAG:
            self._diag_updater = Updater(self)
            self._diag_updater.setHardwareID('YOLO_Detection_Node')
            self._diag_updater.add(
                FunctionDiagnosticTask('YOLO Inference FPS', self._diag_fps))

        # Delayed camera subscription: let the CSI pipeline stabilise first
        delay = float(self.get_parameter('camera_start_delay_sec').value)
        self._camera_topic = camera_topic
        if delay > 0.0:
            self.get_logger().info(
                f"Camera subscription deferred by {delay:.1f} s to allow CSI pipeline to stabilise.")
            self._startup_timer = self.create_timer(delay, self._subscribe_to_camera)
        else:
            self._subscribe_to_camera()

        self.get_logger().info('YOLO Semantic Parser Online.')

    # ── Subscription ───────────────────────────────────────────────────────

    def _subscribe_to_camera(self) -> None:
        """Called once after the startup delay to attach the camera subscription."""
        if hasattr(self, '_startup_timer') and self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        self._img_sub = self.create_subscription(
            CompressedImage,
            self._camera_topic,
            self._image_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.get_logger().info(f"Camera subscription active on '{self._camera_topic}'.")

    # ── Parameters ─────────────────────────────────────────────────────────

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        self._model_path = str(fetch('model_path'))
        self._conf_thres = float(fetch('conf_thres'))
        self._device = str(fetch('device'))
        self._publish_debug = bool(fetch('publish_debug'))
        self._max_frame_age_sec = float(fetch('max_frame_age_sec'))

    def _param_callback(self, params) -> SetParametersResult:
        updated = {p.name: p.value for p in params}
        previous_model_path = self._model_path

        if 'model_path' in updated and str(updated['model_path']) != previous_model_path:
            self.get_logger().warn(
                "Expert Policy: Blocking dynamic model swapping to prevent CUDA thread locks.")
            return SetParametersResult(successful=False, reason="model_path is read-only")

        self._load_params(updates=updated)
        return SetParametersResult(successful=True)

    # ── Model management ───────────────────────────────────────────────────

    def _warmup_model(self) -> None:
        dummy_img: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            self._model.predict(source=dummy_img, conf=self._conf_thres, verbose=False,
                                device=self._device)
        except Exception as exc:
            if self._device.lower() != 'cpu':
                self.get_logger().warn(
                    f"YOLO warm-up failed on '{self._device}' ({exc}); falling back to CPU.")
                self._device = 'cpu'
                self._model.predict(source=dummy_img, conf=self._conf_thres, verbose=False,
                                    device=self._device)
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
                    f"YOLO inference failed on '{self._device}' ({exc}); retrying on CPU.")
                self._device = 'cpu'
                return self._model.predict(
                    source=frame,
                    conf=self._conf_thres,
                    verbose=False,
                    device=self._device,
                )
            raise

    # ── Diagnostics ────────────────────────────────────────────────────────

    def _diag_fps(self, stat) -> None:
        fps = self._fps_ema
        if fps <= 0.0:
            stat.summary(DiagnosticStatus.WARN, "No frames received yet")
        elif fps < 2.0:
            stat.summary(DiagnosticStatus.ERROR, f"FPS critically low: {fps:.1f}")
        elif fps < 5.0:
            stat.summary(DiagnosticStatus.WARN, f"FPS degraded: {fps:.1f}")
        else:
            stat.summary(DiagnosticStatus.OK, f"Running: {fps:.1f} Hz")
        stat.add("Inference FPS (EMA)", f"{fps:.2f}")
        stat.add("Max Frame Age (s)", f"{self._max_frame_age_sec:.3f}")

    def _update_fps(self) -> None:
        """Update EMA FPS and publish to the monitoring topic."""
        now = time.monotonic()
        if self._last_callback_wall > 0.0:
            dt = now - self._last_callback_wall
            if 0.0 < dt < 5.0:
                inst_fps = 1.0 / dt
                if self._fps_ema <= 0.0:
                    self._fps_ema = inst_fps
                else:
                    self._fps_ema = (self._fps_ema_alpha * inst_fps
                                     + (1.0 - self._fps_ema_alpha) * self._fps_ema)
        self._last_callback_wall = now
        fps_msg = Float32()
        fps_msg.data = float(self._fps_ema)
        self._fps_pub.publish(fps_msg)

        if _HAS_DIAG and hasattr(self, '_diag_updater'):
            self._diag_updater.update()

    # ── Main callback ──────────────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage) -> None:
        """
        Translates compressed ROS images to CV2 matrices, infers tensors,
        and packs the output back into ROS network matrices.
        """
        self._update_fps()

        # Frame age check: skip stale frames that have queued under inference load
        try:
            frame_stamp_sec = (float(msg.header.stamp.sec)
                               + float(msg.header.stamp.nanosec) * 1e-9)
            if frame_stamp_sec > 0.0:
                now_sec = (self.get_clock().now().nanoseconds * 1e-9)
                age = now_sec - frame_stamp_sec
                if age > self._max_frame_age_sec:
                    self.get_logger().warn(
                        f"Skipping stale YOLO frame: age={age:.3f}s > "
                        f"max={self._max_frame_age_sec:.3f}s",
                        throttle_duration_sec=2.0)
                    # Publish empty array so downstream knows YOLO is alive
                    empty = Detection2DArray()
                    empty.header = msg.header
                    self._det_pub.publish(empty)
                    return
        except Exception:
            pass  # Non-fatal: proceed with inference on clock errors

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
                x, y, w, h = box.xywh[0].tolist()
                conf: float = float(box.conf[0])
                cls_id: int = int(box.cls[0])
                cls_name: str = str(result.names[cls_id])

                detection = Detection2D()
                detection.header = msg.header

                if hasattr(detection.bbox.center, 'position'):
                    detection.bbox.center.position.x = float(x)
                    detection.bbox.center.position.y = float(y)
                else:
                    detection.bbox.center.x = float(x)
                    detection.bbox.center.y = float(y)
                    detection.bbox.center.theta = 0.0
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = cls_name
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)

                det_msg.detections.append(detection)

            if self._publish_debug and len(boxes) > 0:
                annotated_frame: np.ndarray = result.plot()
                try:
                    debug_msg = self._bridge.cv2_to_compressed_imgmsg(annotated_frame)
                    debug_msg.header = msg.header
                    self._img_pub.publish(debug_msg)
                except Exception as exc:
                    self.get_logger().error(f"Failed to generate output topology map: {exc}")

        # Always publish the root array so listeners know the path is clear
        self._det_pub.publish(det_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = YoloDetectionNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except RuntimeError as re:
        print(f"Node execution aborted due to missing dependencies: {re}")
    except Exception as e:
        print(f"Exception during semantic processing: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
