"""
Unit tests for LongitudinalController.

These tests run without any hardware or ROS 2 runtime.
"""

import sys
import os

# Make the lib directory importable from the test runner
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

import pytest
from longitudinal_control import LongitudinalController


class TestLongitudinalController:

    def _make(self, **kw):
        defaults = dict(KP=1.0, KI=0.1, KD=0.0, integral_windup_limit=2.0, max_output_ms=0.5)
        defaults.update(kw)
        return LongitudinalController(**defaults)

    # ── Basic output range ────────────────────────────────────────────────────

    def test_output_clamped_above_zero(self):
        ctrl = self._make()
        out = ctrl.target_linear_x(speed=0.0, target_speed=10.0)
        assert 0.0 <= out <= 0.5, "Output must be within [0, max_output_ms]"

    def test_output_not_negative(self):
        ctrl = self._make()
        out = ctrl.target_linear_x(speed=1.0, target_speed=0.0)
        assert out >= 0.0, "Longitudinal command must not be negative (no reversing)"

    def test_max_speed_respected(self):
        ctrl = self._make(max_output_ms=0.3)
        out = ctrl.target_linear_x(speed=0.0, target_speed=5.0)
        assert out <= 0.3 + 1e-6

    # ── PID windup ────────────────────────────────────────────────────────────

    def test_integral_windup_clamped(self):
        ctrl = self._make(KI=1.0, integral_windup_limit=1.0)
        for _ in range(200):
            ctrl.PID_step(speed=0.0, target_speed=1.0, dt=0.05)
        # After many steps with large error, integral must be clamped
        assert abs(ctrl.sum_error) <= 1.0 + 1e-6

    # ── Reset ─────────────────────────────────────────────────────────────────

    def test_reset_clears_state(self):
        ctrl = self._make(KI=1.0)
        for _ in range(50):
            ctrl.PID_step(speed=0.0, target_speed=1.0, dt=0.05)
        ctrl.reset()
        assert ctrl.sum_error == 0.0
        assert ctrl.last_error == 0.0

    # ── dt guard ──────────────────────────────────────────────────────────────

    def test_zero_dt_does_not_crash(self):
        ctrl = self._make()
        # dt=0 should fall back to minimum internal value (1e-3)
        out = ctrl.target_linear_x(speed=0.0, target_speed=0.2, dt=0.0)
        assert out >= 0.0

    # ── Target equals current ─────────────────────────────────────────────────

    def test_at_target_speed_no_large_correction(self):
        ctrl = self._make()
        # Small residual allowed; should be near zero
        out = ctrl.PID_step(speed=0.3, target_speed=0.3, dt=0.05)
        assert abs(out) < 0.05
