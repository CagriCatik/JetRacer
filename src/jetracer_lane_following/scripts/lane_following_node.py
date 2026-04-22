#!/usr/bin/env python3
"""
JetRacer Lane Following Node (ROS 2).

Pipeline:
  1) Subscribe to compressed camera image.
  2) Resize frame to 96x96.
  3) Detect lane boundaries.
  4) Predict center-lane waypoints.
  5) Predict target speed from path curvature.
  6) Run selected lateral controller (Stanley or MPC).
  7) Run PID longitudinal controller.
  8) Publish Twist to cmd_vel_lane.
  9) Optionally publish annotated debug image.
"""

from __future__ import annotations

import os
import sys

# Make the install-side lib directory importable.
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from lane_detection import LaneDetection
from lateral_control import LateralController
from longitudinal_control import LongitudinalController
from mpc_control import MPCController
from waypoint_prediction import target_speed_prediction, waypoint_prediction


class LaneFollowingNode(Node):
    """ROS 2 node that drives JetRacer along a detected lane."""

    # target_speed_prediction() returns values in this simulator scale.
    _SIM_MAX_SPEED = 30.0
    _SIM_MIN_SPEED = 15.0

    def __init__(self) -> None:
        super().__init__('lane_following')

        # Safety + longitudinal control
        self.declare_parameter('start', False)
        self.declare_parameter('max_speed_ms', 0.3)
        self.declare_parameter('min_speed_ms', 0.05)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('integral_windup_limit', 2.0)

        # Shared steering limits
        self.declare_parameter('max_steering_rad', 0.6)

        # Lateral controller selection
        self.declare_parameter('lateral_controller_type', 'stanley')

        # Stanley tuning
        self.declare_parameter('gain_constant', 0.025)
        self.declare_parameter('damping_constant', 0.0125)

        # MPC tuning
        self.declare_parameter('mpc_horizon', 8)
        self.declare_parameter('mpc_dt', 0.1)
        self.declare_parameter('mpc_wheelbase', 0.255)
        self.declare_parameter('mpc_q_cte', 2.5)
        self.declare_parameter('mpc_q_heading', 1.5)
        self.declare_parameter('mpc_q_terminal', 3.0)
        self.declare_parameter('mpc_r_steer', 0.2)
        self.declare_parameter('mpc_r_steer_rate', 0.8)
        self.declare_parameter('mpc_cte_scale_px', 160.0)
        self.declare_parameter('mpc_speed_scale_ms', 0.35)
        self.declare_parameter('mpc_min_speed_ms', 0.05)

        # Lane detector (Hardware Conformant)
        self.declare_parameter('luminance_threshold', 180.0)
        self.declare_parameter('roi_top_y', 55.0)
        self.declare_parameter('roi_top_width', 40.0)

        # Waypoint planner
        self.declare_parameter('num_waypoints', 6)
        self.declare_parameter('way_type', 'center')
        self.declare_parameter('smoothing_beta', 30.0)

        # I/O
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('camera_topic', 'csi_cam_0/image_raw/compressed')

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self._detector = LaneDetection(
            luminance_threshold=self._luminance_threshold,
            roi_top_y=self._roi_top_y,
            roi_top_width=self._roi_top_width,
        )

        self._lateral_stanley = LateralController(
            gain_constant=self._gain_constant,
            damping_constant=self._damping_constant,
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
            cte_scale_px=self._mpc_cte_scale_px,
            speed_scale_ms=self._mpc_speed_scale_ms,
            min_speed_ms=self._mpc_min_speed_ms,
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

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_lane', 1)
        self._debug_img_pub = self.create_publisher(
            CompressedImage, 'lane_following/debug_image/compressed', 1
        )
        self._waypoints_pub = self.create_publisher(
            Float32MultiArray, 'lane_following/waypoints', 1
        )
        self._viz_pub = self.create_publisher(
            PoseArray, 'lane_following/path_viz', 1
        )

        self._camera_cb_group = MutuallyExclusiveCallbackGroup()
        self._odom_cb_group = MutuallyExclusiveCallbackGroup()

        camera_topic = str(self.get_parameter('camera_topic').value)
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._img_sub = self.create_subscription(
            CompressedImage, camera_topic, self._image_callback, img_qos, callback_group=self._camera_cb_group
        )
        self._odom_sub = self.create_subscription(Odometry, 'odom', self._odom_callback, 10, callback_group=self._odom_cb_group)

        self.get_logger().info(
            f'lane_following ready -> controller={self._lateral_controller_type} '
            f'start={self._start} max_speed={self._max_speed_ms:.2f} m/s '
            f'max_steer={self._max_steering_rad:.2f} rad way_type={self._way_type}'
        )

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        self._start = bool(fetch('start'))
        self._max_speed_ms = float(fetch('max_speed_ms'))
        self._min_speed_ms = float(fetch('min_speed_ms'))
        self._kp = float(fetch('kp'))
        self._ki = float(fetch('ki'))
        self._kd = float(fetch('kd'))
        self._integral_windup_limit = float(fetch('integral_windup_limit'))

        self._max_steering_rad = float(fetch('max_steering_rad'))
        self._lateral_controller_type = str(
            fetch('lateral_controller_type')
        ).strip().lower()

        self._gain_constant = float(fetch('gain_constant'))
        self._damping_constant = float(fetch('damping_constant'))

        self._mpc_horizon = int(fetch('mpc_horizon'))
        self._mpc_dt = float(fetch('mpc_dt'))
        self._mpc_wheelbase = float(fetch('mpc_wheelbase'))
        self._mpc_q_cte = float(fetch('mpc_q_cte'))
        self._mpc_q_heading = float(fetch('mpc_q_heading'))
        self._mpc_q_terminal = float(fetch('mpc_q_terminal'))
        self._mpc_r_steer = float(fetch('mpc_r_steer'))
        self._mpc_r_steer_rate = float(fetch('mpc_r_steer_rate'))
        self._mpc_cte_scale_px = float(fetch('mpc_cte_scale_px'))
        self._mpc_speed_scale_ms = float(fetch('mpc_speed_scale_ms'))
        self._mpc_min_speed_ms = float(fetch('mpc_min_speed_ms'))

        self._luminance_threshold = float(fetch('luminance_threshold'))
        self._roi_top_y = float(fetch('roi_top_y'))
        self._roi_top_width = float(fetch('roi_top_width'))

        self._num_waypoints = int(fetch('num_waypoints'))
        self._way_type = str(fetch('way_type'))
        self._smoothing_beta = float(fetch('smoothing_beta'))

        self._publish_debug = bool(fetch('publish_debug_image'))

        self._integral_windup_limit = max(0.0, self._integral_windup_limit)
        self._max_steering_rad = max(0.0, self._max_steering_rad)
        self._mpc_min_speed_ms = max(0.0, self._mpc_min_speed_ms)
        if self._min_speed_ms > self._max_speed_ms:
            self._min_speed_ms = self._max_speed_ms
        if self._way_type not in {'center', 'smooth'}:
            self._way_type = 'center'
        if self._lateral_controller_type not in {'stanley', 'mpc'}:
            self._lateral_controller_type = 'stanley'

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
        for speed_key in ('max_speed_ms', 'min_speed_ms', 'mpc_min_speed_ms'):
            if speed_key in updated and not self._is_nonnegative(updated[speed_key]):
                return SetParametersResult(
                    successful=False,
                    reason=f'{speed_key} must be >= 0.0',
                )

        if 'way_type' in updated and str(updated['way_type']) not in {'center', 'smooth'}:
            return SetParametersResult(
                successful=False,
                reason="way_type must be either 'center' or 'smooth'",
            )

        if 'lateral_controller_type' in updated:
            controller = str(updated['lateral_controller_type']).strip().lower()
            if controller not in {'stanley', 'mpc'}:
                return SetParametersResult(
                    successful=False,
                    reason="lateral_controller_type must be 'stanley' or 'mpc'",
                )

        if 'integral_windup_limit' in updated and not self._is_nonnegative(
            updated['integral_windup_limit']
        ):
            return SetParametersResult(
                successful=False,
                reason='integral_windup_limit must be >= 0.0',
            )

        positive_float_keys = {
            'max_steering_rad',
            'mpc_dt',
            'mpc_wheelbase',
            'mpc_cte_scale_px',
            'mpc_speed_scale_ms',
        }
        for key in positive_float_keys:
            if key in updated and not self._is_positive(updated[key]):
                return SetParametersResult(successful=False, reason=f'{key} must be > 0.0')

        nonnegative_float_keys = {
            'gain_constant',
            'damping_constant',
            'mpc_q_cte',
            'mpc_q_heading',
            'mpc_q_terminal',
            'mpc_r_steer',
            'mpc_r_steer_rate',
        }
        for key in nonnegative_float_keys:
            if key in updated and not self._is_nonnegative(updated[key]):
                return SetParametersResult(successful=False, reason=f'{key} must be >= 0.0')

        if 'mpc_horizon' in updated:
            try:
                horizon = int(updated['mpc_horizon'])
            except (TypeError, ValueError):
                return SetParametersResult(successful=False, reason='mpc_horizon must be an integer >= 2')
            if horizon < 2:
                return SetParametersResult(successful=False, reason='mpc_horizon must be >= 2')

        return SetParametersResult(successful=True)

    def _configure_controllers(self) -> None:
        self._lateral_stanley.gain_constant = self._gain_constant
        self._lateral_stanley.damping_constant = self._damping_constant
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
            cte_scale_px=self._mpc_cte_scale_px,
            speed_scale_ms=self._mpc_speed_scale_ms,
            min_speed_ms=self._mpc_min_speed_ms,
            steering_limit=self._max_steering_rad,
        )

        self._longitudinal.KP = self._kp
        self._longitudinal.KI = self._ki
        self._longitudinal.KD = self._kd
        self._longitudinal.integral_windup_limit = self._integral_windup_limit
        self._longitudinal.max_output_ms = self._max_speed_ms

    def _param_callback(self, params) -> SetParametersResult:
        updated = {p.name: p.value for p in params}
        validation = self._validate_param_update(updated)
        if not validation.successful:
            return validation

        previous_controller = self._lateral_controller_type

        self._load_params(updates=updated)
        if self._min_speed_ms > self._max_speed_ms:
            self.get_logger().warn(
                f'min_speed_ms ({self._min_speed_ms:.3f}) > max_speed_ms '
                f'({self._max_speed_ms:.3f}); clamping min_speed_ms to max_speed_ms.'
            )
            self._min_speed_ms = self._max_speed_ms

        self._configure_controllers()
        self._longitudinal.reset()

        if (
            not self._start
            or previous_controller != self._lateral_controller_type
        ):
            self._lateral_stanley.reset()
            self._lateral_mpc.reset()
            self._last_control_stamp_sec = None

        if previous_controller != self._lateral_controller_type:
            self.get_logger().info(
                f'lateral controller switched to {self._lateral_controller_type}'
            )

        return SetParametersResult(successful=True)

    def _odom_callback(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._current_speed_ms = float(np.sqrt(vx ** 2 + vy ** 2))

    def _compute_steering(self, waypoints: np.ndarray) -> float:
        if self._lateral_controller_type == 'mpc':
            return self._lateral_mpc.mpc(waypoints, self._current_speed_ms)
        return self._lateral_stanley.stanley(waypoints, self._current_speed_ms)

    def _image_callback(self, msg: CompressedImage) -> None:
        try:
            frame_bgr = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge decode error: {exc}')
            return

        frame_cv = cv2.resize(frame_bgr, (320, 240), interpolation=cv2.INTER_LINEAR)

        left_fit, right_fit, Minv = self._detector.lane_detection(frame_cv)
        waypoints = waypoint_prediction(
            left_fit,
            right_fit,
            Minv,
            num_waypoints=self._num_waypoints,
            way_type=self._way_type,
            smoothing_beta=self._smoothing_beta,
        )

        sim_speed = target_speed_prediction(
            waypoints,
            max_speed=self._SIM_MAX_SPEED,
            min_speed=self._SIM_MIN_SPEED,
        )
        speed_range_ms = self._max_speed_ms - self._min_speed_ms
        speed_range_sim = self._SIM_MAX_SPEED - self._SIM_MIN_SPEED
        target_speed_ms = (
            (sim_speed - self._SIM_MIN_SPEED) / speed_range_sim * speed_range_ms
            + self._min_speed_ms
        )

        steer_norm = self._compute_steering(waypoints)
        angular_z = steer_norm * self._max_steering_rad

        dt = None
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            if self._last_control_stamp_sec is not None:
                dt_measured = stamp_sec - self._last_control_stamp_sec
                if 0.0 < dt_measured <= 1.0:
                    dt = dt_measured
            self._last_control_stamp_sec = stamp_sec

        linear_x = self._longitudinal.target_linear_x(
            self._current_speed_ms,
            target_speed_ms,
            dt=dt,
        )

        cmd = Twist()
        if self._start:
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
        self._cmd_pub.publish(cmd)

        wp_msg = Float32MultiArray()
        wp_msg.data = waypoints.flatten().tolist()
        self._waypoints_pub.publish(wp_msg)

        # RViz Path Visualization
        viz_msg = PoseArray()
        viz_msg.header = msg.header
        for i in range(len(waypoints)):
            p = Pose()
            p.position.x = float(waypoints[i, 0])
            p.position.y = float(waypoints[i, 1])
            viz_msg.poses.append(p)
        self._viz_pub.publish(viz_msg)

        if self._publish_debug:
            self._publish_debug_frame(frame_bgr, waypoints, target_speed_ms, steer_norm)

    def _publish_debug_frame(
        self,
        frame_bgr: np.ndarray,
        waypoints: np.ndarray,
        target_speed_ms: float,
        steer_norm: float,
    ) -> None:
        annotated = self._detector.draw_splines(frame_bgr, waypoints)

        cv2.putText(
            annotated,
            f'speed: {self._current_speed_ms:.2f} m/s -> {target_speed_ms:.2f} m/s',
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        cv2.putText(
            annotated,
            f'ctrl: {self._lateral_controller_type}  steer: {steer_norm:+.3f}',
            (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
        )
        cv2.putText(
            annotated,
            f'{"ACTIVE" if self._start else "STOPPED"}',
            (8, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0) if self._start else (0, 0, 255),
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
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
