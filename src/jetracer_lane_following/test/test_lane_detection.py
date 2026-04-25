"""
Unit tests for LaneDetection and target_speed_prediction.

These tests run without any hardware or ROS 2 runtime.
They generate synthetic binary images to exercise the sliding-window algorithm
and test the fit-age expiry logic.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

import numpy as np
import cv2
import pytest
from lane_detection import LaneDetection, LaneDetectionResult
from waypoint_prediction import target_speed_prediction, waypoint_prediction


def _make_synthetic_frame(left_x: int = 90, right_x: int = 230) -> np.ndarray:
    """
    Create a synthetic 320x240 BGR frame with two vertical white stripes
    simulating lane markings on a black road.
    """
    frame = np.zeros((240, 320, 3), dtype=np.uint8)
    # Draw white stripes (10 px wide)
    frame[:, max(0, left_x - 5):left_x + 5] = 255
    frame[:, max(0, right_x - 5):right_x + 5] = 255
    return frame


def _make_blank_frame() -> np.ndarray:
    """A completely black frame — no lane markings."""
    return np.zeros((240, 320, 3), dtype=np.uint8)


class TestLaneDetection:

    def _make(self, **kw):
        return LaneDetection(
            luminance_threshold=180.0,
            roi_top_y=55.0,
            roi_top_width=40.0,
            **kw
        )

    def test_detects_both_lanes_on_clear_frame(self):
        det = self._make()
        result = det.lane_detection(_make_synthetic_frame())
        assert result.left_detected, "Left lane should be detected on clear frame"
        assert result.right_detected, "Right lane should be detected on clear frame"

    def test_returns_none_fits_on_blank_frame(self):
        det = self._make()
        result = det.lane_detection(_make_blank_frame())
        # With no prior history, both fits should be None
        assert result.left_fit is None or not result.left_detected
        assert result.right_fit is None or not result.right_detected

    def test_fit_age_expiry(self):
        """After max_fit_age_frames consecutive blank frames the cached fit is evicted."""
        max_age = 3
        det = self._make(max_fit_age_frames=max_age)

        # Prime the cache with a good frame
        det.lane_detection(_make_synthetic_frame())
        assert det.left_fit_old is not None

        # Feed blank frames up to the limit — cache should still be available
        for i in range(max_age):
            result = det.lane_detection(_make_blank_frame())
        # After exactly max_age misses the fit age equals max_age, still within limit
        # Feed one more — now age > max_age, fit should be evicted
        result = det.lane_detection(_make_blank_frame())
        assert result.left_fit is None, (
            "Stale fit should be evicted after max_fit_age_frames + 1 consecutive misses")

    def test_lane_width_computed_when_both_detected(self):
        det = self._make()
        result = det.lane_detection(_make_synthetic_frame(left_x=70, right_x=250))
        if result.left_detected and result.right_detected:
            assert result.lane_width_px is not None
            assert result.lane_width_px > 0.0

    def test_minpix_parameter_accepted(self):
        det = self._make(sliding_window_minpix=5)
        assert det.sliding_window_minpix == 5

    def test_result_has_valid_minv(self):
        det = self._make()
        result = det.lane_detection(_make_synthetic_frame())
        assert result.Minv is not None
        assert result.Minv.shape == (3, 3)
        assert np.isfinite(result.Minv).all()


class TestTargetSpeedPrediction:

    def test_straight_path_near_max_speed(self):
        # Straight path → low curvature → speed close to max
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.zeros(6, dtype=np.float32)
        wp = np.vstack([x, y])
        speed = target_speed_prediction(wp, max_speed=0.35, min_speed=0.05, K_v=2.5,
                                        confidence=1.0, confidence_floor=0.35)
        assert speed >= 0.25, "Straight path speed should be close to max"

    def test_curved_path_lower_speed(self):
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.linspace(0.0, 0.4, 6, dtype=np.float32)  # sharp curve
        wp = np.vstack([x, y])
        speed = target_speed_prediction(wp, max_speed=0.35, min_speed=0.05, K_v=2.5,
                                        confidence=1.0, confidence_floor=0.35)
        assert speed < 0.30, "Sharp curve should reduce speed"

    def test_low_confidence_reduces_speed(self):
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.zeros(6, dtype=np.float32)
        wp = np.vstack([x, y])
        speed_hi = target_speed_prediction(wp, confidence=1.0, confidence_floor=0.35)
        speed_lo = target_speed_prediction(wp, confidence=0.0, confidence_floor=0.35)
        assert speed_lo < speed_hi

    def test_output_in_bounds(self):
        x = np.linspace(0.1, 0.5, 6, dtype=np.float32)
        y = np.random.uniform(-0.2, 0.2, 6).astype(np.float32)
        wp = np.vstack([x, y])
        speed = target_speed_prediction(wp, max_speed=0.35, min_speed=0.05)
        assert 0.05 - 1e-6 <= speed <= 0.35 + 1e-6

    def test_invalid_waypoints_returns_min_speed(self):
        out = target_speed_prediction(None, max_speed=0.35, min_speed=0.05)
        assert out == pytest.approx(0.05)

    def test_single_column_does_not_crash(self):
        wp = np.array([[0.3], [0.0]], dtype=np.float32)
        out = target_speed_prediction(wp, max_speed=0.35, min_speed=0.05)
        assert 0.05 - 1e-6 <= out <= 0.35 + 1e-6
