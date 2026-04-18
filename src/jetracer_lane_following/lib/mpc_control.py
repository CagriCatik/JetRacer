"""
Model Predictive Control (MPC) lateral controller for JetRacer lane following.

This controller consumes the same waypoint format as `LateralController`:
  - waypoints: (2, N) in 320x240 image-space coordinates
  - speed_ms: current speed from /odom in m/s

Output contract:
  - normalized steering command in [-1, 1]
  - caller multiplies by `steering_limit` to get radians
"""

from __future__ import annotations

import numpy as np

__all__ = ["MPCController"]


class MPCController:
    """Lightweight linear MPC for lateral control.

    The internal model uses a small-angle kinematic bicycle approximation over
    a finite horizon. To keep units stable with image-space waypoints, the
    cross-track error and speed are normalized by configurable scales.
    """

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
        cte_scale_px: float = 160.0,
        speed_scale_ms: float = 0.35,
        min_speed_ms: float = 0.05,
        vehicle_center_x: float = 160.0,
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
        self.cte_scale_px = float(max(1e-3, cte_scale_px))
        self.speed_scale_ms = float(max(1e-3, speed_scale_ms))
        self.min_speed_ms = float(max(0.0, min_speed_ms))
        self.vehicle_center_x = float(vehicle_center_x)
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6
        self.previous_steering_angle: float = 0.0

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
        cte_scale_px: float,
        speed_scale_ms: float,
        min_speed_ms: float,
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
        self.cte_scale_px = float(max(1e-3, cte_scale_px))
        self.speed_scale_ms = float(max(1e-3, speed_scale_ms))
        self.min_speed_ms = float(max(0.0, min_speed_ms))
        self.steering_limit = float(abs(steering_limit)) if steering_limit != 0 else 0.6

    def reset(self) -> None:
        """Clear controller memory."""
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
        x_image = waypoints[0].astype(np.float32, copy=False)
        y_image = waypoints[1].astype(np.float32, copy=False)
        x_vehicle = y_image
        y_vehicle = self.vehicle_center_x - x_image
        return np.vstack([x_vehicle, y_vehicle])

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _build_prediction_matrices(self, v_norm: float) -> tuple[np.ndarray, np.ndarray]:
        n = self.horizon
        dt = self.dt
        l = self.wheelbase

        # Small-angle linearized kinematic bicycle error model:
        # e_y[k+1]   = e_y[k] + dt * v * e_psi[k]
        # e_psi[k+1] = e_psi[k] + dt * v / L * delta[k]
        a = np.array([[1.0, dt * v_norm], [0.0, 1.0]], dtype=np.float64)
        b = np.array([[0.0], [dt * v_norm / l]], dtype=np.float64)

        nx = 2
        sx = np.zeros((n * nx, nx), dtype=np.float64)
        su = np.zeros((n * nx, n), dtype=np.float64)

        a_power = np.eye(nx, dtype=np.float64)
        for i in range(n):
            a_power = a_power @ a
            sx[i * nx:(i + 1) * nx, :] = a_power
            a_j = np.eye(nx, dtype=np.float64)
            for j in range(i, -1, -1):
                su[i * nx:(i + 1) * nx, j] = (a_j @ b).reshape(nx)
                a_j = a_j @ a
        return sx, su

    def mpc(self, waypoints: np.ndarray, speed_ms: float) -> float:
        """Run one MPC step and return normalized steering in [-1, 1]."""
        limit = self.steering_limit
        if limit <= 0.0:
            return 0.0

        if not self._valid_waypoints(waypoints):
            prev = float(np.clip(self.previous_steering_angle, -limit, limit))
            return float(prev / limit)

        vf = self._to_vehicle_frame(waypoints)
        if vf.shape[1] >= 2:
            dx = float(vf[0, 1] - vf[0, 0])
            dy = float(vf[1, 1] - vf[1, 0])
            heading = float(np.arctan2(dy, dx)) if (abs(dx) + abs(dy)) > 0.0 else 0.0
        else:
            heading = 0.0

        # Normalize state so MPC weights are stable with image-space inputs.
        cte_norm = float(vf[1, 0]) / self.cte_scale_px
        heading_err = self._wrap_pi(heading)
        x0 = np.array([cte_norm, heading_err], dtype=np.float64)

        v = max(float(speed_ms) if np.isfinite(speed_ms) else 0.0, self.min_speed_ms)
        v_norm = max(v / self.speed_scale_ms, 1e-3)

        sx, su = self._build_prediction_matrices(v_norm)

        n = self.horizon
        nx = 2
        q_step = np.diag([self.q_cte, self.q_heading]).astype(np.float64)
        q_terminal = self.q_terminal * q_step
        q_bar = np.kron(np.eye(n, dtype=np.float64), q_step)
        q_bar[(n - 1) * nx:n * nx, (n - 1) * nx:n * nx] = q_terminal

        r_bar = self.r_steer * np.eye(n, dtype=np.float64)

        d = np.zeros((n, n), dtype=np.float64)
        d[0, 0] = 1.0
        for i in range(1, n):
            d[i, i] = 1.0
            d[i, i - 1] = -1.0
        rd = self.r_steer_rate * np.eye(n, dtype=np.float64)
        d_ref = np.zeros((n,), dtype=np.float64)
        d_ref[0] = float(self.previous_steering_angle)

        h = su.T @ q_bar @ su + r_bar + d.T @ rd @ d
        f = su.T @ q_bar @ (sx @ x0) - d.T @ rd @ d_ref

        reg = 1e-6
        h_reg = h + reg * np.eye(n, dtype=np.float64)
        try:
            u = -np.linalg.solve(h_reg, f)
        except np.linalg.LinAlgError:
            u = -np.linalg.lstsq(h_reg, f, rcond=None)[0]

        delta = float(u[0]) if u.size > 0 and np.isfinite(u[0]) else 0.0
        delta = float(np.clip(delta, -limit, limit))
        self.previous_steering_angle = delta
        return float(delta / limit)
