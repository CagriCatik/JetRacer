#!/usr/bin/env python3
"""
JetRacer lane-following node for ROS 2.

Pipeline per frame:
  1. Decode the compressed camera image.
  2. Optionally rectify from CameraInfo.
  3. Resize to the fixed 320x240 lane-processing space.
  4. Detect lane boundaries in bird's-eye view (BEV).
  5. Build a centerline in BEV, vehicle, and camera coordinates.
  6. Run the selected lateral controller on vehicle-frame waypoints.
  7. Run PID longitudinal control in m/s.
  8. Publish Ackermann drive plus a legacy steering-angle Twist.
  9. Publish metric waypoints, compatibility camera waypoints, status, and debug.
"""

from __future__ import annotations

import os
import sys

# Make the install-side lib directory importable.
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:  # pragma: no cover - optional runtime dependency on non-ROS hosts
    AckermannDriveStamped = None

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, String

from lane_detection import LaneDetection, LaneDetectionResult
from lateral_control import LateralController
from longitudinal_control import LongitudinalController
from mpc_control import MPCController
from waypoint_prediction import WaypointPlan, target_speed_prediction, waypoint_prediction


class LaneFollowingNode(Node):
    """ROS 2 node that drives JetRacer along a detected lane."""

    def __init__(self) -> None:
        super().__init__("lane_following")

        # Safety + longitudinal control
        self.declare_parameter("start", False)
        self.declare_parameter("max_speed_ms", 0.3)
        self.declare_parameter("min_speed_ms", 0.05)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("ki", 0.1)
        self.declare_parameter("kd", 0.2)
        self.declare_parameter("integral_windup_limit", 2.0)

        # Shared steering limits
        self.declare_parameter("max_steering_rad", 0.6)

        # Lateral controller selection
        self.declare_parameter("lateral_controller_type", "stanley")

        # Stanley tuning
        self.declare_parameter("gain_constant", 1.8)
        self.declare_parameter("damping_constant", 0.25)  # Deprecated alias.
        self.declare_parameter("steering_smoothing", -1.0)
        self.declare_parameter("controller_lookahead_m", 0.35)
        self.declare_parameter("controller_heading_points", 4)

        # MPC tuning
        self.declare_parameter("mpc_horizon", 8)
        self.declare_parameter("mpc_dt", 0.1)
        self.declare_parameter("mpc_wheelbase", 0.255)
        self.declare_parameter("mpc_q_cte", 2.5)
        self.declare_parameter("mpc_q_heading", 1.5)
        self.declare_parameter("mpc_q_terminal", 3.0)
        self.declare_parameter("mpc_r_steer", 0.2)
        self.declare_parameter("mpc_r_steer_rate", 0.8)
        self.declare_parameter("mpc_cte_scale_px", 48.0)  # Deprecated compatibility.
        self.declare_parameter("mpc_speed_scale_ms", 0.35)  # Deprecated compatibility.
        self.declare_parameter("mpc_min_speed_ms", 0.05)
        self.declare_parameter("max_steering_rate_radps", 3.0)

        # Lane detector
        self.declare_parameter("luminance_threshold", 180.0)
        self.declare_parameter("roi_top_y", 55.0)
        self.declare_parameter("roi_top_width", 40.0)

        # Waypoint planner and BEV scaling
        self.declare_parameter("num_waypoints", 6)
        self.declare_parameter("way_type", "center")
        self.declare_parameter("smoothing_beta", 30.0)
        self.declare_parameter("fallback_lane_width_px", 220.0)
        self.declare_parameter("lane_width_ema_alpha", 0.25)
        self.declare_parameter("lane_width_min_px", 160.0)
        self.declare_parameter("lane_width_max_px", 280.0)
        self.declare_parameter("bev_vehicle_center_x_px", 160.0)
        self.declare_parameter("bev_vehicle_origin_y_px", 239.0)
        self.declare_parameter("bev_lateral_m_per_px", 0.0022)
        self.declare_parameter("bev_forward_m_per_px", 0.0030)
        self.declare_parameter("speed_curvature_gain", 2.5)
        self.declare_parameter("speed_confidence_floor", 0.35)
        self.declare_parameter("path_frame_id", "base_link")

        # I/O
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("camera_topic", "csi_cam_0/image_raw/compressed")
        self.declare_parameter("camera_info_topic", "csi_cam_0/camera_info")
        self.declare_parameter("use_camera_calibration", True)
        self.declare_parameter("publish_ackermann_drive", True)
        self.declare_parameter("publish_legacy_twist", True)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self._detector = LaneDetection(
            luminance_threshold=self._luminance_threshold,
            roi_top_y=self._roi_top_y,
            roi_top_width=self._roi_top_width,
        )

        self._lateral_stanley = LateralController(
            gain_constant=self._gain_constant,
            steering_smoothing=self._steering_smoothing,
            lookahead_distance_m=self._controller_lookahead_m,
            heading_fit_points=self._controller_heading_points,
            steering_limit=self._max_steering_rad,
        )
        self._lateral_mpc = MPCController(
            horizon=self._mpc_horizon,
            dt=self._mpc_dt,
            wheelbase=self._mpc_wheelbase,
            q_cte=self._mpc_q_cte,
            q_heading=self._mpc_q_heading,
            q_terminal=self._mpc_q_terminal,
            r_steer=self._mpc_r_steer,
            r_steer_rate=self._mpc_r_steer_rate,
            min_speed_ms=self._mpc_min_speed_ms,
            lookahead_distance_m=self._controller_lookahead_m,
            heading_fit_points=self._controller_heading_points,
            max_steering_rate_radps=self._max_steering_rate_radps,
            steering_limit=self._max_steering_rad,
        )
        self._longitudinal = LongitudinalController(
            KP=self._kp,
            KI=self._ki,
            KD=self._kd,
            integral_windup_limit=self._integral_windup_limit,
            max_output_ms=self._max_speed_ms,
        )

        self._bridge = CvBridge()
        self._current_speed_ms: float = 0.0
        self._last_control_stamp_sec: float | None = None
        self._lane_width_estimate_px = float(self._fallback_lane_width_px)
        self._tracking_status = "LOST"
        self._tracking_confidence = 0.0
        self._camera_info_size: tuple[int, int] | None = None
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._rectification_matrix: np.ndarray | None = None
        self._projection_matrix: np.ndarray | None = None
        self._undistort_map1: np.ndarray | None = None
        self._undistort_map2: np.ndarray | None = None
        self._undistort_size: tuple[int, int] | None = None
        self._warned_missing_camera_info = False

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel_lane", 1)
        self._drive_pub = None
        if AckermannDriveStamped is not None:
            self._drive_pub = self.create_publisher(AckermannDriveStamped, "drive_lane", 1)
        elif self._publish_ackermann_drive:
            self.get_logger().warn(
                "ackermann_msgs is not available; drive_lane publishing is disabled."
            )

        self._debug_img_pub = self.create_publisher(
            CompressedImage, "lane_following/debug_image/compressed", 1
        )
        self._waypoints_pub = self.create_publisher(
            Float32MultiArray, "lane_following/waypoints", 1
        )
        self._waypoints_cam_pub = self.create_publisher(
            Float32MultiArray, "lane_following/waypoints_camera", 1
        )
        self._viz_pub = self.create_publisher(PoseArray, "lane_following/path_viz", 1)
        self._status_pub = self.create_publisher(String, "lane_following/status", 1)
        self._confidence_pub = self.create_publisher(Float32, "lane_following/confidence", 1)

        self._camera_cb_group = MutuallyExclusiveCallbackGroup()
        self._odom_cb_group = MutuallyExclusiveCallbackGroup()

        camera_topic = str(self.get_parameter("camera_topic").value)
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._img_sub = self.create_subscription(
            CompressedImage,
            camera_topic,
            self._image_callback,
            img_qos,
            callback_group=self._camera_cb_group,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            img_qos,
            callback_group=self._camera_cb_group,
        )
        self._odom_sub = self.create_subscription(
            Odometry, "odom", self._odom_callback, 10, callback_group=self._odom_cb_group
        )

        self.get_logger().info(
            "lane_following ready -> "
            f"controller={self._lateral_controller_type} "
            f"start={self._start} "
            f"max_speed={self._max_speed_ms:.2f} m/s "
            f"max_steer={self._max_steering_rad:.2f} rad "
            f"lookahead={self._controller_lookahead_m:.2f} m "
            f"calibration={'on' if self._use_camera_calibration else 'off'} "
            f"ackermann={'on' if self._drive_pub is not None and self._publish_ackermann_drive else 'off'}"
        )

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        self._start = bool(fetch("start"))
        self._max_speed_ms = float(fetch("max_speed_ms"))
        self._min_speed_ms = float(fetch("min_speed_ms"))
        self._kp = float(fetch("kp"))
        self._ki = float(fetch("ki"))
        self._kd = float(fetch("kd"))
        self._integral_windup_limit = float(fetch("integral_windup_limit"))

        self._max_steering_rad = float(fetch("max_steering_rad"))
        self._lateral_controller_type = str(fetch("lateral_controller_type")).strip().lower()

        self._gain_constant = float(fetch("gain_constant"))
        self._damping_constant = float(fetch("damping_constant"))
        steering_smoothing = float(fetch("steering_smoothing"))
        if steering_smoothing < 0.0:
            self._steering_smoothing = float(np.clip(self._damping_constant, 0.0, 1.0))
        else:
            self._steering_smoothing = float(steering_smoothing)
        self._controller_lookahead_m = float(fetch("controller_lookahead_m"))
        self._controller_heading_points = int(fetch("controller_heading_points"))

        self._mpc_horizon = int(fetch("mpc_horizon"))
        self._mpc_dt = float(fetch("mpc_dt"))
        self._mpc_wheelbase = float(fetch("mpc_wheelbase"))
        self._mpc_q_cte = float(fetch("mpc_q_cte"))
        self._mpc_q_heading = float(fetch("mpc_q_heading"))
        self._mpc_q_terminal = float(fetch("mpc_q_terminal"))
        self._mpc_r_steer = float(fetch("mpc_r_steer"))
        self._mpc_r_steer_rate = float(fetch("mpc_r_steer_rate"))
        self._mpc_cte_scale_px = float(fetch("mpc_cte_scale_px"))
        self._mpc_speed_scale_ms = float(fetch("mpc_speed_scale_ms"))
        self._mpc_min_speed_ms = float(fetch("mpc_min_speed_ms"))
        self._max_steering_rate_radps = float(fetch("max_steering_rate_radps"))

        self._luminance_threshold = float(fetch("luminance_threshold"))
        self._roi_top_y = float(fetch("roi_top_y"))
        self._roi_top_width = float(fetch("roi_top_width"))

        self._num_waypoints = int(fetch("num_waypoints"))
        self._way_type = str(fetch("way_type"))
        self._smoothing_beta = float(fetch("smoothing_beta"))
        self._fallback_lane_width_px = float(fetch("fallback_lane_width_px"))
        self._lane_width_ema_alpha = float(fetch("lane_width_ema_alpha"))
        self._lane_width_min_px = float(fetch("lane_width_min_px"))
        self._lane_width_max_px = float(fetch("lane_width_max_px"))
        self._bev_vehicle_center_x_px = float(fetch("bev_vehicle_center_x_px"))
        self._bev_vehicle_origin_y_px = float(fetch("bev_vehicle_origin_y_px"))
        self._bev_lateral_m_per_px = float(fetch("bev_lateral_m_per_px"))
        self._bev_forward_m_per_px = float(fetch("bev_forward_m_per_px"))
        self._speed_curvature_gain = float(fetch("speed_curvature_gain"))
        self._speed_confidence_floor = float(fetch("speed_confidence_floor"))
        self._path_frame_id = str(fetch("path_frame_id")).strip() or "base_link"

        self._publish_debug = bool(fetch("publish_debug_image"))
        self._camera_info_topic = str(fetch("camera_info_topic"))
        self._use_camera_calibration = bool(fetch("use_camera_calibration"))
        self._publish_ackermann_drive = bool(fetch("publish_ackermann_drive"))
        self._publish_legacy_twist = bool(fetch("publish_legacy_twist"))

        self._integral_windup_limit = max(0.0, self._integral_windup_limit)
        self._max_speed_ms = max(0.0, self._max_speed_ms)
        self._min_speed_ms = max(0.0, self._min_speed_ms)
        self._max_steering_rad = max(0.0, self._max_steering_rad)
        self._mpc_min_speed_ms = max(0.0, self._mpc_min_speed_ms)
        self._controller_lookahead_m = max(0.0, self._controller_lookahead_m)
        self._controller_heading_points = max(2, self._controller_heading_points)
        self._lane_width_ema_alpha = float(np.clip(self._lane_width_ema_alpha, 0.0, 1.0))
        self._speed_confidence_floor = float(np.clip(self._speed_confidence_floor, 0.0, 1.0))
        self._steering_smoothing = float(np.clip(self._steering_smoothing, 0.0, 1.0))
        self._fallback_lane_width_px = max(1.0, self._fallback_lane_width_px)
        self._lane_width_min_px = max(1.0, self._lane_width_min_px)
        self._lane_width_max_px = max(self._lane_width_min_px, self._lane_width_max_px)
        self._bev_lateral_m_per_px = max(1e-4, self._bev_lateral_m_per_px)
        self._bev_forward_m_per_px = max(1e-4, self._bev_forward_m_per_px)
        self._max_steering_rate_radps = max(0.0, self._max_steering_rate_radps)

        if self._min_speed_ms > self._max_speed_ms:
            self._min_speed_ms = self._max_speed_ms
        if self._way_type not in {"center", "smooth"}:
            self._way_type = "center"
        if self._lateral_controller_type not in {"stanley", "mpc"}:
            self._lateral_controller_type = "stanley"

    @staticmethod
    def _is_nonnegative(value: object) -> bool:
        try:
            return float(value) >= 0.0
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _is_positive(value: object) -> bool:
        try:
            return float(value) > 0.0
        except (TypeError, ValueError):
            return False

    def _validate_param_update(self, updated: dict[str, object]) -> SetParametersResult:
        for speed_key in ("max_speed_ms", "min_speed_ms", "mpc_min_speed_ms"):
            if speed_key in updated and not self._is_nonnegative(updated[speed_key]):
                return SetParametersResult(
                    successful=False,
                    reason=f"{speed_key} must be >= 0.0",
                )

        if "way_type" in updated and str(updated["way_type"]) not in {"center", "smooth"}:
            return SetParametersResult(
                successful=False,
                reason="way_type must be either 'center' or 'smooth'",
            )

        if "lateral_controller_type" in updated:
            controller = str(updated["lateral_controller_type"]).strip().lower()
            if controller not in {"stanley", "mpc"}:
                return SetParametersResult(
                    successful=False,
                    reason="lateral_controller_type must be 'stanley' or 'mpc'",
                )

        if "integral_windup_limit" in updated and not self._is_nonnegative(
            updated["integral_windup_limit"]
        ):
            return SetParametersResult(
                successful=False,
                reason="integral_windup_limit must be >= 0.0",
            )

        positive_float_keys = {
            "max_steering_rad",
            "mpc_dt",
            "mpc_wheelbase",
            "mpc_cte_scale_px",
            "mpc_speed_scale_ms",
            "controller_lookahead_m",
            "fallback_lane_width_px",
            "lane_width_min_px",
            "lane_width_max_px",
            "bev_lateral_m_per_px",
            "bev_forward_m_per_px",
        }
        for key in positive_float_keys:
            if key in updated and not self._is_positive(updated[key]):
                return SetParametersResult(successful=False, reason=f"{key} must be > 0.0")

        nonnegative_float_keys = {
            "gain_constant",
            "damping_constant",
            "mpc_q_cte",
            "mpc_q_heading",
            "mpc_q_terminal",
            "mpc_r_steer",
            "mpc_r_steer_rate",
            "max_steering_rate_radps",
            "speed_curvature_gain",
        }
        for key in nonnegative_float_keys:
            if key in updated and not self._is_nonnegative(updated[key]):
                return SetParametersResult(successful=False, reason=f"{key} must be >= 0.0")

        bounded_keys = ("lane_width_ema_alpha", "speed_confidence_floor")
        for key in bounded_keys:
            if key in updated:
                try:
                    value = float(updated[key])
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f"{key} must be between 0.0 and 1.0")
                if not 0.0 <= value <= 1.0:
                    return SetParametersResult(successful=False, reason=f"{key} must be between 0.0 and 1.0")

        if "steering_smoothing" in updated:
            try:
                value = float(updated["steering_smoothing"])
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason="steering_smoothing must be between 0.0 and 1.0")
            if value < 0.0 or value > 1.0:
                return SetParametersResult(successful=False, reason="steering_smoothing must be between 0.0 and 1.0")

        integer_keys = {
            "mpc_horizon": 2,
            "controller_heading_points": 2,
            "num_waypoints": 2,
        }
        for key, minimum in integer_keys.items():
            if key in updated:
                try:
                    value = int(updated[key])
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f"{key} must be an integer >= {minimum}")
                if value < minimum:
                    return SetParametersResult(successful=False, reason=f"{key} must be >= {minimum}")

        if "lane_width_min_px" in updated or "lane_width_max_px" in updated:
            try:
                lane_width_min = float(updated.get("lane_width_min_px", self._lane_width_min_px))
                lane_width_max = float(updated.get("lane_width_max_px", self._lane_width_max_px))
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason="lane_width_min_px and lane_width_max_px must be numeric")
            if lane_width_max < lane_width_min:
                return SetParametersResult(successful=False, reason="lane_width_max_px must be >= lane_width_min_px")

        return SetParametersResult(successful=True)

    def _configure_controllers(self) -> None:
        self._lateral_stanley.gain_constant = self._gain_constant
        self._lateral_stanley.steering_smoothing = self._steering_smoothing
        self._lateral_stanley.lookahead_distance_m = self._controller_lookahead_m
        self._lateral_stanley.heading_fit_points = self._controller_heading_points
        self._lateral_stanley.steering_limit = self._max_steering_rad

        self._lateral_mpc.update_config(
            horizon=self._mpc_horizon,
            dt=self._mpc_dt,
            wheelbase=self._mpc_wheelbase,
            q_cte=self._mpc_q_cte,
            q_heading=self._mpc_q_heading,
            q_terminal=self._mpc_q_terminal,
            r_steer=self._mpc_r_steer,
            r_steer_rate=self._mpc_r_steer_rate,
            min_speed_ms=self._mpc_min_speed_ms,
            lookahead_distance_m=self._controller_lookahead_m,
            heading_fit_points=self._controller_heading_points,
            max_steering_rate_radps=self._max_steering_rate_radps,
            steering_limit=self._max_steering_rad,
        )

        self._longitudinal.KP = self._kp
        self._longitudinal.KI = self._ki
        self._longitudinal.KD = self._kd
        self._longitudinal.integral_windup_limit = self._integral_windup_limit
        self._longitudinal.max_output_ms = self._max_speed_ms

    @staticmethod
    def _scale_intrinsic_matrix(
        matrix: np.ndarray,
        src_size: tuple[int, int],
        dst_size: tuple[int, int],
    ) -> np.ndarray:
        scaled = matrix.astype(np.float32, copy=True)
        src_w, src_h = src_size
        dst_w, dst_h = dst_size
        if src_w <= 0 or src_h <= 0:
            return scaled

        scale_x = float(dst_w) / float(src_w)
        scale_y = float(dst_h) / float(src_h)
        scaled[0, 0] *= scale_x
        scaled[0, 2] *= scale_x
        scaled[1, 1] *= scale_y
        scaled[1, 2] *= scale_y
        scaled[2, 2] = 1.0
        return scaled

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9 or msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return

        self._camera_info_size = (int(msg.width), int(msg.height))
        self._camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        if len(msg.d) > 0:
            self._dist_coeffs = np.array(msg.d, dtype=np.float32)
        else:
            self._dist_coeffs = np.zeros(5, dtype=np.float32)

        if len(msg.r) == 9:
            rectification = np.array(msg.r, dtype=np.float32).reshape(3, 3)
        else:
            rectification = np.eye(3, dtype=np.float32)
        if not np.isfinite(rectification).all() or not np.any(rectification):
            rectification = np.eye(3, dtype=np.float32)
        self._rectification_matrix = rectification

        if len(msg.p) == 12:
            projection = np.array(msg.p, dtype=np.float32).reshape(3, 4)[:, :3]
            if (
                np.isfinite(projection).all()
                and projection[0, 0] > 0.0
                and projection[1, 1] > 0.0
            ):
                self._projection_matrix = projection
            else:
                self._projection_matrix = None
        else:
            self._projection_matrix = None

        self._undistort_map1 = None
        self._undistort_map2 = None
        self._undistort_size = None
        if self._warned_missing_camera_info:
            self.get_logger().info(
                f"camera calibration received from {self._camera_info_topic}"
            )
            self._warned_missing_camera_info = False

    def _rectify_frame(self, frame_bgr: np.ndarray) -> np.ndarray:
        if not self._use_camera_calibration:
            return frame_bgr

        if (
            self._camera_matrix is None
            or self._dist_coeffs is None
            or self._rectification_matrix is None
            or self._camera_info_size is None
        ):
            if not self._warned_missing_camera_info:
                self.get_logger().warn(
                    "camera calibration enabled but no CameraInfo received on "
                    f"{self._camera_info_topic}; using raw frames for now."
                )
                self._warned_missing_camera_info = True
            return frame_bgr

        height, width = frame_bgr.shape[:2]
        current_size = (width, height)
        if self._undistort_size != current_size:
            scaled_k = self._scale_intrinsic_matrix(
                self._camera_matrix, self._camera_info_size, current_size
            )
            if (
                self._projection_matrix is not None
                and np.isfinite(self._projection_matrix).all()
                and self._projection_matrix[0, 0] > 0.0
                and self._projection_matrix[1, 1] > 0.0
            ):
                target_k = self._scale_intrinsic_matrix(
                    self._projection_matrix, self._camera_info_size, current_size
                )
            else:
                target_k = scaled_k

            self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                scaled_k,
                self._dist_coeffs,
                self._rectification_matrix,
                target_k,
                current_size,
                cv2.CV_16SC2,
            )
            self._undistort_size = current_size

        return cv2.remap(
            frame_bgr,
            self._undistort_map1,
            self._undistort_map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def _param_callback(self, params) -> SetParametersResult:
        updated = {p.name: p.value for p in params}
        validation = self._validate_param_update(updated)
        if not validation.successful:
            return validation

        previous_controller = self._lateral_controller_type

        self._load_params(updates=updated)
        self._configure_controllers()

        if (
            not self._start
            or previous_controller != self._lateral_controller_type
        ):
            self._lateral_stanley.reset()
            self._lateral_mpc.reset()
            self._longitudinal.reset()
            self._last_control_stamp_sec = None

        if "fallback_lane_width_px" in updated and self._lane_width_estimate_px <= 0.0:
            self._lane_width_estimate_px = float(self._fallback_lane_width_px)

        if previous_controller != self._lateral_controller_type:
            self.get_logger().info(
                f"lateral controller switched to {self._lateral_controller_type}"
            )

        return SetParametersResult(successful=True)

    def _odom_callback(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed_ms = float(np.sqrt(vx**2 + vy**2))

    def _compute_steering(self, waypoints_vehicle: np.ndarray) -> float:
        if self._lateral_controller_type == "mpc":
            return self._lateral_mpc.mpc(waypoints_vehicle, self._current_speed_ms)
        return self._lateral_stanley.stanley(waypoints_vehicle, self._current_speed_ms)

    def _update_lane_width_estimate(self, lane_width_px: float | None) -> None:
        if lane_width_px is None or not np.isfinite(lane_width_px) or lane_width_px <= 1.0:
            return

        alpha = self._lane_width_ema_alpha
        self._lane_width_estimate_px = float(
            alpha * lane_width_px + (1.0 - alpha) * self._lane_width_estimate_px
        )

    def _compute_tracking_state(
        self,
        detection: LaneDetectionResult,
        plan: WaypointPlan,
    ) -> tuple[str, float]:
        detected_count = int(detection.left_detected) + int(detection.right_detected)
        lane_width_px = detection.lane_width_px
        if lane_width_px is None:
            lane_width_px = plan.lane_width_px
        if lane_width_px is None:
            lane_width_px = self._lane_width_estimate_px

        width_valid = True
        if lane_width_px is not None and np.isfinite(lane_width_px):
            width_valid = self._lane_width_min_px <= lane_width_px <= self._lane_width_max_px

        if detected_count == 2 and width_valid:
            return "GOOD", 1.0
        if detected_count == 2:
            return "WEAK", 0.65
        if detected_count == 1 and width_valid:
            return "WEAK", 0.60
        if detected_count == 1:
            return "WEAK", 0.45
        if detection.left_fit is not None or detection.right_fit is not None:
            return "WEAK", 0.30
        return "LOST", 0.0

    def _publish_drive_commands(self, header, linear_x: float, steering_angle: float) -> None:
        if self._publish_legacy_twist:
            cmd = Twist()
            if self._start:
                cmd.linear.x = float(linear_x)
                cmd.angular.z = float(steering_angle)
            self._cmd_pub.publish(cmd)

        if self._publish_ackermann_drive and self._drive_pub is not None:
            drive_msg = AckermannDriveStamped()
            drive_msg.header = header
            if self._start:
                drive_msg.drive.speed = float(linear_x)
                drive_msg.drive.steering_angle = float(steering_angle)
            self._drive_pub.publish(drive_msg)

    def _publish_waypoints(self, header, plan: WaypointPlan) -> None:
        wp_vehicle = Float32MultiArray()
        wp_vehicle.data = plan.waypoints_vehicle.flatten().tolist()
        self._waypoints_pub.publish(wp_vehicle)

        wp_camera = Float32MultiArray()
        wp_camera.data = plan.waypoints_camera.flatten().tolist()
        self._waypoints_cam_pub.publish(wp_camera)

        viz_msg = PoseArray()
        viz_msg.header.stamp = header.stamp
        viz_msg.header.frame_id = self._path_frame_id
        for x_forward, y_left in zip(plan.waypoints_vehicle[0], plan.waypoints_vehicle[1]):
            pose = Pose()
            pose.position.x = float(x_forward)
            pose.position.y = float(y_left)
            viz_msg.poses.append(pose)
        self._viz_pub.publish(viz_msg)

    def _publish_tracking_state(self) -> None:
        status_msg = String()
        status_msg.data = self._tracking_status
        self._status_pub.publish(status_msg)

        confidence_msg = Float32()
        confidence_msg.data = float(self._tracking_confidence)
        self._confidence_pub.publish(confidence_msg)

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame_bgr = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge decode error: {exc}")
            return

        working_frame = self._rectify_frame(frame_bgr)
        frame_cv = cv2.resize(working_frame, (320, 240), interpolation=cv2.INTER_LINEAR)

        detection = self._detector.lane_detection(frame_cv)
        self._update_lane_width_estimate(detection.lane_width_px)

        plan = waypoint_prediction(
            detection.left_fit,
            detection.right_fit,
            detection.Minv,
            num_waypoints=self._num_waypoints,
            way_type=self._way_type,
            smoothing_beta=self._smoothing_beta,
            fallback_lane_width_px=self._lane_width_estimate_px,
            bev_vehicle_center_x_px=self._bev_vehicle_center_x_px,
            bev_vehicle_origin_y_px=self._bev_vehicle_origin_y_px,
            bev_lateral_m_per_px=self._bev_lateral_m_per_px,
            bev_forward_m_per_px=self._bev_forward_m_per_px,
        )

        previous_status = self._tracking_status
        status, confidence = self._compute_tracking_state(detection, plan)
        self._tracking_status = status
        self._tracking_confidence = confidence

        if previous_status != status:
            level = self.get_logger().warn if status == "LOST" else self.get_logger().info
            level(f"lane tracking status -> {status} ({confidence:.2f})")
            if status == "LOST":
                self._lateral_stanley.reset()
                self._lateral_mpc.reset()
                self._longitudinal.reset()
                self._last_control_stamp_sec = None

        target_speed_ms = 0.0
        steering_angle = 0.0
        if status != "LOST":
            target_speed_ms = target_speed_prediction(
                plan.waypoints_vehicle,
                max_speed=self._max_speed_ms,
                min_speed=self._min_speed_ms,
                K_v=self._speed_curvature_gain,
                confidence=confidence,
                confidence_floor=self._speed_confidence_floor,
            )
            steering_angle = self._compute_steering(plan.waypoints_vehicle)

        dt = None
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            if self._last_control_stamp_sec is not None:
                dt_measured = stamp_sec - self._last_control_stamp_sec
                if 0.0 < dt_measured <= 1.0:
                    dt = dt_measured
            self._last_control_stamp_sec = stamp_sec

        linear_x = 0.0
        if status != "LOST":
            linear_x = self._longitudinal.target_linear_x(
                self._current_speed_ms,
                target_speed_ms,
                dt=dt,
            )

        self._publish_drive_commands(msg.header, linear_x, steering_angle)
        self._publish_waypoints(msg.header, plan)
        self._publish_tracking_state()

        if self._publish_debug:
            self._publish_debug_frame(
                working_frame,
                plan.waypoints_camera,
                target_speed_ms,
                steering_angle,
            )

    def _publish_debug_frame(
        self,
        frame_bgr: np.ndarray,
        waypoints_camera: np.ndarray,
        target_speed_ms: float,
        steering_angle: float,
    ) -> None:
        annotated = self._detector.draw_splines(frame_bgr, waypoints_camera)

        cv2.putText(
            annotated,
            f"speed: {self._current_speed_ms:.2f} -> {target_speed_ms:.2f} m/s",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            annotated,
            f"ctrl: {self._lateral_controller_type}  steer: {steering_angle:+.3f} rad",
            (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
        )
        cv2.putText(
            annotated,
            f"track: {self._tracking_status}  conf: {self._tracking_confidence:.2f}",
            (8, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 200, 0),
            1,
        )
        cv2.putText(
            annotated,
            f"{'ACTIVE' if self._start else 'STOPPED'}",
            (8, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0) if self._start else (0, 0, 255),
            1,
        )
        cv2.putText(
            annotated,
            f"cal: {'RECTIFIED' if self._use_camera_calibration and self._camera_matrix is not None else 'RAW'}",
            (8, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 200, 0),
            1,
        )

        try:
            img_msg = self._bridge.cv2_to_compressed_imgmsg(annotated)
            self._debug_img_pub.publish(img_msg)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFollowingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._cmd_pub.publish(Twist())
            if node._drive_pub is not None and AckermannDriveStamped is not None:
                node._drive_pub.publish(AckermannDriveStamped())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
