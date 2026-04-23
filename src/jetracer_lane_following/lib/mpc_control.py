"""
Lightweight linear MPC lateral controller for JetRacer.

The controller consumes vehicle-frame waypoints in meters and returns a
steering angle in radians.
"""

from __future__ import annotations

import numpy as np

__all__ = ["MPCController"]


class MPCController:
    """Finite-horizon lateral MPC around a small-angle kinematic bicycle model."""

    def __init__(
        self,
        horizon: int = 8,
        dt: float = 0.1,
        wheelbase: float = 0.255,
        q_cte: float = 2.5,
        q_heading: float = 1.5,
        q_terminal: float = 3.0,
        r_steer: float = 0.2,
        r_steer_rate: float = 0.8,
        min_speed_ms: float = 0.05,
        lookahead_distance_m: float = 0.35,
        heading_fit_points: int = 4,
        max_steering_rate_radps: float = 3.0,
        steering_limit: float = 0.6,
    ) -> None:
        self.horizon = int(max(2, horizon))
        self.dt = float(max(1e-3, dt))
        self.wheelbase = float(max(1e-3, wheelbase))
        self.q_cte = float(max(0.0, q_cte))
        self.q_heading = float(max(0.0, q_heading))
        self.q_terminal = float(max(0.0, q_terminal))
        self.r_steer = float(max(0.0, r_steer))
        self.r_steer_rate = float(max(0.0, r_steer_rate))
        self.min_speed_ms = float(max(0.0, min_speed_ms))
        self.lookahead_distance_m = float(max(0.0, lookahead_distance_m))
        self.heading_fit_points = int(max(2, heading_fit_points))
        self.max_steering_rate_radps = float(max(0.0, max_steering_rate_radps))
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6
        self.previous_steering_angle: float = 0.0
        self.last_saturation_ratio: float = 0.0

    def update_config(
        self,
        *,
        horizon: int,
        dt: float,
        wheelbase: float,
        q_cte: float,
        q_heading: float,
        q_terminal: float,
        r_steer: float,
        r_steer_rate: float,
        min_speed_ms: float,
        lookahead_distance_m: float,
        heading_fit_points: int,
        max_steering_rate_radps: float,
        steering_limit: float,
    ) -> None:
        self.horizon = int(max(2, horizon))
        self.dt = float(max(1e-3, dt))
        self.wheelbase = float(max(1e-3, wheelbase))
        self.q_cte = float(max(0.0, q_cte))
        self.q_heading = float(max(0.0, q_heading))
        self.q_terminal = float(max(0.0, q_terminal))
        self.r_steer = float(max(0.0, r_steer))
        self.r_steer_rate = float(max(0.0, r_steer_rate))
        self.min_speed_ms = float(max(0.0, min_speed_ms))
        self.lookahead_distance_m = float(max(0.0, lookahead_distance_m))
        self.heading_fit_points = int(max(2, heading_fit_points))
        self.max_steering_rate_radps = float(max(0.0, max_steering_rate_radps))
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6

    def reset(self) -> None:
        self.previous_steering_angle = 0.0
        self.last_saturation_ratio = 0.0

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

    def _build_prediction_matrices(self, speed_ms: float) -> tuple[np.ndarray, np.ndarray]:
        n = self.horizon
        dt = self.dt
        wheelbase = self.wheelbase

        a = np.array([[1.0, dt * speed_ms], [0.0, 1.0]], dtype=np.float64)
        b = np.array([[0.0], [dt * speed_ms / wheelbase]], dtype=np.float64)

        nx = 2
        sx = np.zeros((n * nx, nx), dtype=np.float64)
        su = np.zeros((n * nx, n), dtype=np.float64)

        for i in range(n):
            a_power = np.linalg.matrix_power(a, i + 1)
            sx[i * nx:(i + 1) * nx, :] = a_power
            for j in range(i + 1):
                su[i * nx:(i + 1) * nx, j] = (
                    np.linalg.matrix_power(a, i - j) @ b
                ).reshape(nx)

        return sx, su

    def mpc(self, waypoints: np.ndarray, speed_ms: float) -> float:
        """Run one MPC step and return a steering angle in radians."""
        limit = self.steering_limit
        if limit <= 0.0:
            return 0.0

        if not self._valid_waypoints(waypoints):
            return float(np.clip(self.previous_steering_angle, -limit, limit))

        idx = self._select_lookahead_index(waypoints)
        cte = float(waypoints[1, idx])
        heading_err = self._wrap_pi(self._path_heading(waypoints, idx))
        x0 = np.array([cte, heading_err], dtype=np.float64)

        v = max(float(speed_ms) if np.isfinite(speed_ms) else 0.0, self.min_speed_ms)
        sx, su = self._build_prediction_matrices(v)

        n = self.horizon
        nx = 2
        q_step = np.diag([self.q_cte, self.q_heading]).astype(np.float64)
        q_terminal = self.q_terminal * q_step
        q_bar = np.kron(np.eye(n, dtype=np.float64), q_step)
        q_bar[(n - 1) * nx:n * nx, (n - 1) * nx:n * nx] = q_terminal

        r_bar = self.r_steer * np.eye(n, dtype=np.float64)

        diff = np.zeros((n, n), dtype=np.float64)
        diff[0, 0] = 1.0
        for i in range(1, n):
            diff[i, i] = 1.0
            diff[i, i - 1] = -1.0

        rd = self.r_steer_rate * np.eye(n, dtype=np.float64)
        d_ref = np.zeros((n,), dtype=np.float64)
        d_ref[0] = float(self.previous_steering_angle)

        h = su.T @ q_bar @ su + r_bar + diff.T @ rd @ diff
        f = su.T @ q_bar @ (sx @ x0) - diff.T @ rd @ d_ref

        reg = 1e-6
        h_reg = h + reg * np.eye(n, dtype=np.float64)
        try:
            u = -np.linalg.solve(h_reg, f)
        except np.linalg.LinAlgError:
            u = -np.linalg.lstsq(h_reg, f, rcond=None)[0]

        delta = float(u[0]) if u.size > 0 and np.isfinite(u[0]) else 0.0

        if self.max_steering_rate_radps > 0.0:
            max_delta_step = self.max_steering_rate_radps * self.dt
            lower = self.previous_steering_angle - max_delta_step
            upper = self.previous_steering_angle + max_delta_step
            delta = float(np.clip(delta, lower, upper))

        unclipped_delta = delta
        delta = float(np.clip(delta, -limit, limit))
        self.last_saturation_ratio = float(min(1.0, abs(unclipped_delta) / max(limit, 1e-6)))
        self.previous_steering_angle = delta
        return delta
