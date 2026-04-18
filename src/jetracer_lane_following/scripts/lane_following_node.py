#!/usr/bin/env python3
"""
JetRacer Lane Following Node (ROS 2)

This node integrates the full lane-following pipeline into the JetRacer
ROS 2 stack:

  CSI Camera → LaneDetection → WaypointPrediction → Stanley + PID → cmd_vel

Pipeline:
  1. Subscribe to compressed camera image.
  2. Resize frame to 96×96 (LaneDetection's native resolution).
  3. Detect left/right lane boundary B-splines.
  4. Predict 6 centre-lane waypoints (optionally smoothed).
  5. Predict target speed from path curvature.
  6. Stanley controller → normalised steering → angular.z (rad/s).
  7. PID controller → linear.x (m/s).
  8. Publish geometry_msgs/Twist to cmd_vel.
  9. Optionally publish an annotated debug image.

All safety-critical default values (start=false, max_speed_ms=0.3) ensure
the car does not move until deliberately enabled.
"""

from __future__ import annotations

import os
import sys

# ── Make the lib/ directory importable ────────────────────────────────────────
# Regardless of whether we run from the install tree or the source tree,
# the library modules are always in the same directory as this script.
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from lane_detection import LaneDetection
from lateral_control import LateralController
from longitudinal_control import LongitudinalController
from waypoint_prediction import target_speed_prediction, waypoint_prediction


class LaneFollowingNode(Node):
    """ROS 2 node that drives JetRacer along a detected lane."""

    # Simulator speed scale — target_speed_prediction returns [min_speed, max_speed]
    # in CarRacing-v2 units.  We map this range onto [min_speed_ms, max_speed_ms].
    _SIM_MAX_SPEED = 30.0
    _SIM_MIN_SPEED = 15.0

    def __init__(self) -> None:
        super().__init__('lane_following')

        # ── Declare parameters ──────────────────────────────────────────────
        # Safety: start=false means the node never publishes cmd_vel until
        # explicitly enabled via: ros2 param set /lane_following start true
        self.declare_parameter('start', False)

        # Speed limits (m/s) — tune for your environment
        self.declare_parameter('max_speed_ms', 0.3)
        self.declare_parameter('min_speed_ms', 0.05)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_windup_limit', 2.0)

        # Steering
        self.declare_parameter('max_steering_rad', 0.6)   # JetRacer physical max
        self.declare_parameter('gain_constant', 0.025)    # Stanley k
        self.declare_parameter('damping_constant', 0.0125)

        # Lane detector
        self.declare_parameter('cut_size', 68)
        self.declare_parameter('spline_smoothness', 10.0)
        self.declare_parameter('gradient_threshold', 14.0)
        self.declare_parameter('distance_maxima_gradient', 3)

        # Waypoint planner
        self.declare_parameter('num_waypoints', 6)
        # "center" is faster on Nano; "smooth" is better tracking quality
        self.declare_parameter('way_type', 'center')
        self.declare_parameter('smoothing_beta', 30.0)

        # Debug output
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')

        # ── Load parameter values ───────────────────────────────────────────
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ── Instantiate pipeline components ────────────────────────────────
        self._detector = LaneDetection(
            cut_size=self._cut_size,
            spline_smoothness=self._spline_smoothness,
            gradient_threshold=self._gradient_threshold,
            distance_maxima_gradient=self._distance_maxima_gradient,
        )
        self._lateral = LateralController(
            gain_constant=self._gain_constant,
            damping_constant=self._damping_constant,
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
        self._current_speed_ms: float = 0.0   # updated from /odom
        self._last_control_stamp_sec: float | None = None

        # ── Publishers ─────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_lane', 1)
        self._debug_img_pub = self.create_publisher(
            CompressedImage, 'lane_following/debug_image/compressed', 1)
        self._waypoints_pub = self.create_publisher(
            Float32MultiArray, 'lane_following/waypoints', 1)

        # ── Subscribers ────────────────────────────────────────────────────
        camera_topic = str(self.get_parameter('camera_topic').value)
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, img_qos)

        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10)

        self.get_logger().info(
            f'lane_following ready → publishing to cmd_vel_lane (→ twist_mux → cmd_vel_to_steering → cmd_vel). '
            f'start={self._start}  max_speed={self._max_speed_ms:.2f} m/s  '
            f'max_steer={self._max_steering_rad:.2f} rad  way_type={self._way_type}'
        )

    # ── Parameter management ────────────────────────────────────────────────

    def _load_params(self) -> None:
        self._start = bool(self.get_parameter('start').value)
        self._max_speed_ms = float(self.get_parameter('max_speed_ms').value)
        self._min_speed_ms = float(self.get_parameter('min_speed_ms').value)
        self._kp = float(self.get_parameter('kp').value)
        self._ki = float(self.get_parameter('ki').value)
        self._kd = float(self.get_parameter('kd').value)
        self._integral_windup_limit = float(self.get_parameter('integral_windup_limit').value)
        self._max_steering_rad = float(self.get_parameter('max_steering_rad').value)
        self._gain_constant = float(self.get_parameter('gain_constant').value)
        self._damping_constant = float(self.get_parameter('damping_constant').value)
        self._cut_size = int(self.get_parameter('cut_size').value)
        self._spline_smoothness = float(self.get_parameter('spline_smoothness').value)
        self._gradient_threshold = float(self.get_parameter('gradient_threshold').value)
        self._distance_maxima_gradient = int(
            self.get_parameter('distance_maxima_gradient').value)
        self._num_waypoints = int(self.get_parameter('num_waypoints').value)
        self._way_type = str(self.get_parameter('way_type').value)
        self._smoothing_beta = float(self.get_parameter('smoothing_beta').value)
        self._publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self._integral_windup_limit = max(0.0, self._integral_windup_limit)
        if self._min_speed_ms > self._max_speed_ms:
            self._min_speed_ms = self._max_speed_ms
        if self._way_type not in {'center', 'smooth'}:
            self._way_type = 'center'

    def _param_callback(self, params) -> SetParametersResult:
        try:
            updated = {p.name: p.value for p in params}
            for speed_key in ('max_speed_ms', 'min_speed_ms'):
                if speed_key in updated and float(updated[speed_key]) < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{speed_key} must be >= 0.0',
                    )
            if 'way_type' in updated and str(updated['way_type']) not in {'center', 'smooth'}:
                return SetParametersResult(
                    successful=False,
                    reason="way_type must be either 'center' or 'smooth'",
                )
            if 'integral_windup_limit' in updated and float(updated['integral_windup_limit']) < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason='integral_windup_limit must be >= 0.0',
                )
        except (TypeError, ValueError):
            return SetParametersResult(successful=False, reason='invalid parameter type/value')

        self._load_params()
        if self._min_speed_ms > self._max_speed_ms:
            self.get_logger().warn(
                f'min_speed_ms ({self._min_speed_ms:.3f}) > max_speed_ms '
                f'({self._max_speed_ms:.3f}); clamping min_speed_ms to max_speed_ms.'
            )
            self._min_speed_ms = self._max_speed_ms
        # Propagate tunable gains into controller instances at runtime
        self._lateral.gain_constant = self._gain_constant
        self._lateral.damping_constant = self._damping_constant
        self._lateral.steering_limit = self._max_steering_rad
        self._longitudinal.KP = self._kp
        self._longitudinal.KI = self._ki
        self._longitudinal.KD = self._kd
        self._longitudinal.integral_windup_limit = self._integral_windup_limit
        self._longitudinal.max_output_ms = self._max_speed_ms
        # Reset integral on speed limit change to avoid wind-up artefact
        self._longitudinal.reset()
        if not self._start:
            # Safety: reset steering memory when disabled
            self._lateral.reset()
            self._last_control_stamp_sec = None
        return SetParametersResult(successful=True)

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """Track current vehicle speed from wheel odometry."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed_ms = float(np.sqrt(vx ** 2 + vy ** 2))

    def _image_callback(self, msg: CompressedImage) -> None:
        """Main pipeline callback — runs once per camera frame."""
        try:
            frame_bgr = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge decode error: {exc}')
            return

        # ── 1. Resize to 96×96 (LaneDetection's native pixel space) ────────
        frame_96 = cv2.resize(frame_bgr, (96, 96), interpolation=cv2.INTER_LINEAR)

        # ── 2. Lane detection ────────────────────────────────────────────────
        lb1, lb2 = self._detector.lane_detection(frame_96)

        # ── 3. Waypoint prediction ───────────────────────────────────────────
        waypoints = waypoint_prediction(
            lb1, lb2,
            num_waypoints=self._num_waypoints,
            way_type=self._way_type,
            smoothing_beta=self._smoothing_beta,
        )

        # ── 4. Target speed from curvature ───────────────────────────────────
        sim_speed = target_speed_prediction(
            waypoints,
            max_speed=self._SIM_MAX_SPEED,
            min_speed=self._SIM_MIN_SPEED,
        )
        # Convert simulator speed units → m/s
        speed_range_ms = self._max_speed_ms - self._min_speed_ms
        speed_range_sim = self._SIM_MAX_SPEED - self._SIM_MIN_SPEED
        target_speed_ms = (
            (sim_speed - self._SIM_MIN_SPEED) / speed_range_sim * speed_range_ms
            + self._min_speed_ms
        )

        # ── 5. Stanley lateral control ───────────────────────────────────────
        steer_norm = self._lateral.stanley(waypoints, self._current_speed_ms)
        angular_z = steer_norm * self._max_steering_rad   # rad/s equivalent

        # ── 6. PID longitudinal control ──────────────────────────────────────
        dt = None
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            if self._last_control_stamp_sec is not None:
                dt_measured = stamp_sec - self._last_control_stamp_sec
                # Reject bad deltas caused by clock jumps or stale headers.
                if 0.0 < dt_measured <= 1.0:
                    dt = dt_measured
            self._last_control_stamp_sec = stamp_sec

        linear_x = self._longitudinal.target_linear_x(
            self._current_speed_ms,
            target_speed_ms,
            dt=dt,
        )

        # ── 7. Publish Twist ─────────────────────────────────────────────────
        cmd = Twist()
        if self._start:
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
        # If not started, publish zero Twist (explicit stop)
        self._cmd_pub.publish(cmd)

        # ── 8. Publish waypoints for RViz / debugging ─────────────────────────
        wp_msg = Float32MultiArray()
        wp_msg.data = waypoints.flatten().tolist()
        self._waypoints_pub.publish(wp_msg)

        # ── 9. Debug image ────────────────────────────────────────────────────
        if self._publish_debug:
            self._publish_debug_frame(frame_bgr, waypoints, target_speed_ms, steer_norm)

    def _publish_debug_frame(
        self,
        frame_bgr: np.ndarray,
        waypoints: np.ndarray,
        target_speed_ms: float,
        steer_norm: float,
    ) -> None:
        """Overlay pipeline state onto the full-resolution camera frame and publish."""
        annotated = self._detector.draw_splines(frame_bgr, waypoints)
        h, w = annotated.shape[:2]

        # Speed and steering HUD
        cv2.putText(annotated,
                    f'speed: {self._current_speed_ms:.2f} m/s → {target_speed_ms:.2f} m/s',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(annotated,
                    f'steer: {steer_norm:+.3f}  {"ACTIVE" if self._start else "STOPPED"}',
                    (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if self._start else (0, 0, 255), 1)

        try:
            img_msg = self._bridge.cv2_to_compressed_imgmsg(annotated)
            self._debug_img_pub.publish(img_msg)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())   # Safety stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
