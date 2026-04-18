"""
Longitudinal (speed) controller — adapted from dev/longitudinal_control.py for JetRacer ROS 2.

Changes vs simulator original:
  - Removed matplotlib plot_speed() (no display needed on Jetson).
  - Added direct speed_ms output method: :meth:`target_linear_x`.
  - All control logic unchanged.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

__all__ = ["LongitudinalController"]


class LongitudinalController:
    """PID throttle/brake controller.

    The controller keeps integral and derivative state across calls.
    Call :meth:`reset` when starting a new lane-following episode.

    Default gains are tuned for the JetRacer at indoor speeds (0–0.5 m/s).
    Adjust ``KP``, ``KI``, ``KD`` via ROS 2 parameters on the running node.
    """

    def __init__(
        self,
        KP: float = 0.8,
        KI: float = 0.1,
        KD: float = 0.2,
        integral_windup_limit: float = 2.0,
        max_output_ms: float = 0.5,     # m/s — JetRacer indoor limit
        default_dt: float = 1.0 / 20.0, # 20 Hz camera callback rate
    ) -> None:
        self.KP = float(KP)
        self.KI = float(KI)
        self.KD = float(KD)
        self.integral_windup_limit = float(integral_windup_limit)
        self.max_output_ms = float(max_output_ms)
        self.default_dt = float(default_dt)

        self.last_error: float = 0.0
        self.sum_error: float = 0.0

    def reset(self) -> None:
        """Clear PID state."""
        self.last_error = 0.0
        self.sum_error = 0.0

    def PID_step(self, speed: float, target_speed: float, dt: Optional[float] = None) -> float:
        """Compute raw PID control effort (positive = accelerate, negative = brake).

        Args:
            speed: Current speed in **m/s**.
            target_speed: Desired speed in **m/s**.
            dt: Integration step in seconds.

        Returns:
            Control effort in m/s.
        """
        dt_val = self.default_dt if dt is None else float(dt)
        dt_val = max(dt_val, 1e-3)

        error = float(target_speed) - float(speed)
        p_term = self.KP * error

        self.sum_error += error * dt_val
        self.sum_error = float(
            np.clip(self.sum_error, -self.integral_windup_limit, self.integral_windup_limit)
        )
        i_term = self.KI * self.sum_error
        d_term = self.KD * (error - self.last_error) / dt_val

        self.last_error = error
        return float(p_term + i_term + d_term)

    def target_linear_x(
        self,
        speed: float,
        target_speed: float,
        dt: Optional[float] = None,
    ) -> float:
        """Return the ``cmd_vel.linear.x`` setpoint in **m/s**.

        This is the primary output method for ROS 2 hardware integration.
        Replaces the simulator-style ``(gas, brake)`` tuple output.

        Args:
            speed: Current speed from ``/odom`` (m/s).
            target_speed: Desired speed from planner (m/s).
            dt: Time since last call (seconds).

        Returns:
            Clamped linear velocity command in ``[0, max_output_ms]`` m/s.
        """
        effort = self.PID_step(speed, target_speed, dt=dt)
        return float(np.clip(effort, 0.0, self.max_output_ms))
