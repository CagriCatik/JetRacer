"""
Unit tests for LateralController (Stanley) and MPCController.

These tests run without any hardware or ROS 2 runtime.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

import numpy as np
import pytest
from lateral_control import LateralController
from mpc_control import MPCController


def _straight_path(n: int = 6) -> np.ndarray:
    """Straight path directly ahead: x = 0..0.5, y = 0."""
    x = np.linspace(0.1, 0.5, n, dtype=np.float32)
    y = np.zeros(n, dtype=np.float32)
    return np.vstack([x, y])


def _left_curve_path(n: int = 6) -> np.ndarray:
    """Gentle left curve."""
    x = np.linspace(0.1, 0.5, n, dtype=np.float32)
    y = np.linspace(0.0, 0.15, n, dtype=np.float32)
    return np.vstack([x, y])


# ── Stanley controller ────────────────────────────────────────────────────────

class TestLateralControllerStanley:

    def _make(self, **kw):
        defaults = dict(gain_constant=1.8, steering_smoothing=0.0,
                        lookahead_distance_m=0.35, heading_fit_points=4,
                        steering_limit=0.6)
        defaults.update(kw)
        return LateralController(**defaults)

    def test_straight_path_zero_steer(self):
        ctrl = self._make()
        out = ctrl.stanley(_straight_path(), speed_ms=0.3)
        assert abs(out) < 0.15, "Straight ahead path should produce near-zero steering"

    def test_left_curve_positive_steer(self):
        ctrl = self._make()
        out = ctrl.stanley(_left_curve_path(), speed_ms=0.3)
        # Left deviation should produce positive (left) steering
        assert out > 0.0

    def test_steering_limit_enforced(self):
        ctrl = self._make(steering_limit=0.3)
        # Large lateral error path
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.ones(6, dtype=np.float32) * 10.0  # extreme deviation
        path = np.vstack([x, y])
        out = ctrl.stanley(path, speed_ms=0.3)
        assert abs(out) <= 0.3 + 1e-6

    def test_invalid_waypoints_returns_previous(self):
        ctrl = self._make()
        # Set a known previous angle
        ctrl.previous_steering_angle = 0.2
        out = ctrl.stanley(np.zeros((2, 1)), speed_ms=0.3)  # too few cols
        assert abs(out) <= 0.6 + 1e-6  # must be clamped, not crash

    def test_reset_clears_state(self):
        ctrl = self._make()
        ctrl.stanley(_left_curve_path(), speed_ms=0.3)
        ctrl.reset()
        assert ctrl.previous_steering_angle == 0.0

    def test_zero_speed_does_not_crash(self):
        ctrl = self._make()
        out = ctrl.stanley(_straight_path(), speed_ms=0.0)
        assert np.isfinite(out)


# ── MPC controller ────────────────────────────────────────────────────────────

class TestMPCController:

    def _make(self, **kw):
        defaults = dict(horizon=4, dt=0.05, wheelbase=0.255,
                        q_cte=2.5, q_heading=1.5, q_terminal=3.0,
                        r_steer=0.2, r_steer_rate=0.8, min_speed_ms=0.05,
                        lookahead_distance_m=0.35, heading_fit_points=4,
                        max_steering_rate_radps=3.0, steering_limit=0.6)
        defaults.update(kw)
        return MPCController(**defaults)

    def test_straight_path_small_steer(self):
        ctrl = self._make()
        out = ctrl.mpc(_straight_path(), speed_ms=0.3)
        assert abs(out) < 0.2

    def test_steering_limit_enforced(self):
        ctrl = self._make(steering_limit=0.25)
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.ones(6, dtype=np.float32) * 5.0
        path = np.vstack([x, y])
        out = ctrl.mpc(path, speed_ms=0.3)
        assert abs(out) <= 0.25 + 1e-6

    def test_steering_rate_limit(self):
        ctrl = self._make(max_steering_rate_radps=1.0, dt=0.05)
        # Max allowed step = 1.0 * 0.05 = 0.05 rad
        ctrl.previous_steering_angle = 0.0
        path = _left_curve_path()
        out = ctrl.mpc(path, speed_ms=0.3)
        assert abs(out - 0.0) <= 0.05 + 1e-6

    def test_invalid_waypoints_returns_previous(self):
        ctrl = self._make()
        ctrl.previous_steering_angle = 0.15
        out = ctrl.mpc(np.zeros((2, 0)), speed_ms=0.3)
        assert abs(out) <= 0.6 + 1e-6

    def test_reset_clears_state(self):
        ctrl = self._make()
        ctrl.mpc(_left_curve_path(), speed_ms=0.3)
        ctrl.reset()
        assert ctrl.previous_steering_angle == 0.0
        assert ctrl.last_saturation_ratio == 0.0

    def test_finite_output_always(self):
        ctrl = self._make()
        for _ in range(10):
            out = ctrl.mpc(_left_curve_path(), speed_ms=0.1)
            assert np.isfinite(out)
