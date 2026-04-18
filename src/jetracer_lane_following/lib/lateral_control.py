"""
Lateral (steering) controller — adapted from dev/lateral_control.py for JetRacer ROS 2.

Changes vs simulator original:
  - Default steering_limit raised from 0.4 → 0.6 rad (JetRacer physical limit).
  - Docstrings updated: output is angular.z in rad/s for geometry_msgs/Twist.
  - No other logic changes — Stanley algorithm is hardware-agnostic.
"""

from __future__ import annotations

import numpy as np

__all__ = ["LateralController"]


class LateralController:
    """Stanley-based lateral controller with damping.

    The waypoints live in the **96×96 pixel image space** produced by
    :class:`LaneDetection`.  Speed should be supplied in **m/s** (from odometry).
    The gain ``k`` may need re-tuning on the physical robot; start with the
    default and adjust until tracking is stable.

    Args:
        gain_constant: Cross-track error gain ``k`` in the Stanley law.
        damping_constant: First-order damping coefficient to reduce steering jitter.
        vehicle_center_x: Column (in 96-pixel space) that represents the car centre.
        steering_limit: Physical steering saturation in **radians**.
            JetRacer max is ~0.6 rad; the output is scaled to [-1, 1] against
            this limit, then the caller multiplies back by ``steering_limit``
            to get the rad/s to put in ``cmd_vel.angular.z``.
    """

    def __init__(
        self,
        gain_constant: float = 0.025,
        damping_constant: float = 0.0125,
        vehicle_center_x: float = 48.0,
        steering_limit: float = 0.6,   # updated from 0.4 (sim) → 0.6 (JetRacer)
    ) -> None:
        self.gain_constant = float(gain_constant)
        self.damping_constant = float(damping_constant)
        self.vehicle_center_x = float(vehicle_center_x)
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6
        self.previous_steering_angle: float = 0.0

    def reset(self) -> None:
        """Clear controller state (e.g. at lane re-acquisition)."""
        self.previous_steering_angle = 0.0

    def _valid_waypoints(self, waypoints: np.ndarray) -> bool:
        return (
            isinstance(waypoints, np.ndarray)
            and waypoints.ndim == 2
            and waypoints.shape[0] == 2
            and waypoints.shape[1] >= 1
            and np.isfinite(waypoints).all()
        )

    def _to_vehicle_frame(self, waypoints: np.ndarray) -> np.ndarray:
        """Convert image-space waypoints to the vehicle frame.

        Image space: axis-0 = column (left/right), axis-1 = row (closer/further).
        Vehicle frame: +x forward, +y left.
        """
        x_image = waypoints[0].astype(np.float32, copy=False)
        y_image = waypoints[1].astype(np.float32, copy=False)
        x_vehicle = y_image
        y_vehicle = self.vehicle_center_x - x_image
        return np.vstack([x_vehicle, y_vehicle])

    def stanley(self, waypoints: np.ndarray, speed_ms: float) -> float:
        """Compute one Stanley control step.

        Args:
            waypoints: ``(2, N)`` array in 96-pixel image coordinates.
            speed_ms: Current vehicle speed in **m/s** (from ``/odom``).

        Returns:
            Normalised steering command in ``[-1, 1]``.
            Multiply by ``steering_limit`` to obtain radians for
            ``cmd_vel.angular.z``.
        """
        limit = self.steering_limit

        if not self._valid_waypoints(waypoints):
            prev = float(self.previous_steering_angle)
            return float(np.clip(prev, -limit, limit) / limit)

        epsilon = 1e-6
        v = max(float(speed_ms) if np.isfinite(speed_ms) else 0.0, 0.0)

        vf = self._to_vehicle_frame(waypoints)

        if vf.shape[1] >= 2:
            dx = float(vf[0, 1] - vf[0, 0])
            dy = float(vf[1, 1] - vf[1, 0])
            path_heading = float(np.arctan2(dy, dx)) if (abs(dx) + abs(dy)) > 0.0 else 0.0
        else:
            path_heading = 0.0

        psi_t = (path_heading + np.pi) % (2.0 * np.pi) - np.pi
        d_t = float(vf[1, 0])

        delta_sc = psi_t + np.arctan2(self.gain_constant * d_t, v + epsilon)
        delta = delta_sc - self.damping_constant * (delta_sc - self.previous_steering_angle)
        self.previous_steering_angle = float(delta)
        return float(np.clip(delta, -limit, limit) / limit)
