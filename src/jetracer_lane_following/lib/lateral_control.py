"""
Stanley lateral controller for JetRacer lane following.

The controller operates on vehicle-frame waypoints in meters:
  - x: forward
  - y: left

It returns a steering angle in radians, ready for Ackermann-style interfaces.
"""

from __future__ import annotations

import numpy as np

__all__ = ["LateralController"]


class LateralController:
    """Stanley controller with lookahead-based error evaluation."""

    def __init__(
        self,
        gain_constant: float = 1.8,
        steering_smoothing: float = 0.25,
        lookahead_distance_m: float = 0.35,
        heading_fit_points: int = 4,
        steering_limit: float = 0.6,
    ) -> None:
        self.gain_constant = float(gain_constant)
        self.steering_smoothing = float(steering_smoothing)
        self.lookahead_distance_m = float(max(0.0, lookahead_distance_m))
        self.heading_fit_points = int(max(2, heading_fit_points))
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6
        self.previous_steering_angle: float = 0.0

    def reset(self) -> None:
        self.previous_steering_angle = 0.0

    def _valid_waypoints(self, waypoints: np.ndarray) -> bool:
        return (
            isinstance(waypoints, np.ndarray)
            and waypoints.ndim == 2
            and waypoints.shape[0] == 2
            and waypoints.shape[1] >= 2
            and np.isfinite(waypoints).all()
        )

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _select_lookahead_index(self, waypoints: np.ndarray) -> int:
        distances = np.abs(waypoints[0] - self.lookahead_distance_m)
        return int(np.argmin(distances))

    def _path_heading(self, waypoints: np.ndarray, idx: int) -> float:
        start = max(0, idx - 1)
        end = min(waypoints.shape[1], idx + self.heading_fit_points)
        window = waypoints[:, start:end]

        if window.shape[1] >= 2 and float(np.ptp(window[0])) > 1e-6:
            try:
                slope, _ = np.polyfit(window[0], window[1], 1)
                return float(np.arctan(float(slope)))
            except np.linalg.LinAlgError:
                pass

        if idx + 1 < waypoints.shape[1]:
            dx = float(waypoints[0, idx + 1] - waypoints[0, idx])
            dy = float(waypoints[1, idx + 1] - waypoints[1, idx])
        elif idx > 0:
            dx = float(waypoints[0, idx] - waypoints[0, idx - 1])
            dy = float(waypoints[1, idx] - waypoints[1, idx - 1])
        else:
            dx = 1.0
            dy = 0.0

        if abs(dx) + abs(dy) <= 1e-6:
            return 0.0
        return float(np.arctan2(dy, dx))

    def stanley(self, waypoints: np.ndarray, speed_ms: float) -> float:
        """Return a steering angle in radians."""
        limit = self.steering_limit
        if limit <= 0.0:
            return 0.0

        if not self._valid_waypoints(waypoints):
            return float(np.clip(self.previous_steering_angle, -limit, limit))

        epsilon = 1e-6
        v = max(float(speed_ms) if np.isfinite(speed_ms) else 0.0, 0.0)
        idx = self._select_lookahead_index(waypoints)
        path_heading = self._path_heading(waypoints, idx)

        psi_t = self._wrap_pi(path_heading)
        d_t = float(waypoints[1, idx])

        delta_cmd = psi_t + np.arctan2(self.gain_constant * d_t, v + epsilon)
        alpha = float(np.clip(self.steering_smoothing, 0.0, 1.0))
        delta = alpha * float(delta_cmd) + (1.0 - alpha) * float(self.previous_steering_angle)
        delta = float(np.clip(delta, -limit, limit))
        self.previous_steering_angle = delta
        return delta
