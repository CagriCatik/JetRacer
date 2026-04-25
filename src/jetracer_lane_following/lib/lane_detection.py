#!/usr/bin/env python3
"""
Advanced JetRacer Lane Detection (ROS 2)

Upgraded from legacy point-growing splines to a robust 
Bird's-Eye View Sliding-Window Polynomial fit, heavily tailored 
for high-contrast "white lines on black track" physical robotics.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

__all__ = ["LaneDetection", "LaneDetectionResult"]


@dataclass(slots=True)
class LaneDetectionResult:
    left_fit: np.ndarray | None
    right_fit: np.ndarray | None
    Minv: np.ndarray
    left_detected: bool
    right_detected: bool
    lane_width_px: float | None


class LaneDetection:
    """
    Lane boundary detection using Perspetive Warps and Sliding Windows.

    Operates entirely within a high-density 320x240 pixel space, heavily exploiting ARM CPU architecture natively.
    Expects perfectly cropped bounds dynamically matching physical hardware metrics.
    """

    def __init__(
        self,
        luminance_threshold: float = 180.0,
        roi_top_y: float = 140.0,
        roi_top_width: float = 120.0,
        sliding_window_minpix: int = 15,
        max_fit_age_frames: int = 5,
    ) -> None:
        """
        Initializes perspective transform geometries.

        Args:
            luminance_threshold: Drops background textures below this brightness (0-255).
            roi_top_y: The height (Y) where the road vanishes into the horizon.
            roi_top_width: The width of the road trapezoid at the vanishing point.
            sliding_window_minpix: Minimum white pixels in a window to recenter it.
            max_fit_age_frames: Consecutive missed detections before the cached fit is
                discarded. After this many misses the node reports LOST instead of
                steering toward an obsolete polynomial.
        """
        self.lum_thresh = int(luminance_threshold)
        self.sliding_window_minpix = int(max(1, sliding_window_minpix))
        self.max_fit_age_frames = int(max(1, max_fit_age_frames))

        # Standard 320x240 camera projection.
        # Define the trapezoid covering the physical path the car is travelling towards:
        tl_x = (320.0 - roi_top_width) / 2.0
        tr_x = 320.0 - tl_x
        
        src_points = np.float32([
            [tl_x, roi_top_y],    # Top-Left
            [tr_x, roi_top_y],    # Top-Right
            [320.0, 240.0],         # Bottom-Right
            [0.0, 240.0]           # Bottom-Left
        ])

        # Warp the trapezoid out to perfect rectangular tracks.
        dst_points = np.float32([
            [50.0, 0.0],
            [270.0, 0.0],
            [270.0, 240.0],
            [50.0, 240.0]
        ])

        self.M = cv2.getPerspectiveTransform(src_points, dst_points)
        self.Minv = cv2.getPerspectiveTransform(dst_points, src_points)

        self.left_fit_old = None
        self.right_fit_old = None
        # Age counters: incremented each frame a fit is NOT refreshed.
        # When age exceeds max_fit_age_frames the cached fit is evicted.
        self._left_fit_age: int = 0
        self._right_fit_age: int = 0

    def lane_detection(self, state_image_full: np.ndarray) -> LaneDetectionResult:
        """
        Calculates polynomial boundaries.
        Args:
            state_image_full: (96, 96, 3) Image Frame.
        Returns:
            LaneDetectionResult with current-fit validity and fallback-adjusted fits.
        """
        # 1. Warp to Bird's Eye View
        warped = cv2.warpPerspective(state_image_full, self.M, (320, 240), flags=cv2.INTER_LINEAR)

        # 2. Binary Thresholding (White Lines on Black Ground)
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.lum_thresh, 255, cv2.THRESH_BINARY)

        # 3. Sliding Window Initialization via Histogram
        histogram = np.sum(binary[120:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(240 / nwindows)
        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 35
        # Use parameterized minimum pixels threshold
        minpix = self.sliding_window_minpix

        left_lane_inds = []
        right_lane_inds = []

        # 4. Extract track pixels driving upwards
        for window in range(nwindows):
            win_y_low = 240 - (window + 1) * window_height
            win_y_high = 240 - window * window_height
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > self.sliding_window_minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.sliding_window_minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = None
        right_fit = None
        left_detected = False
        right_detected = False
        lane_width_px = None

        # 5. Parabola fitting (x = Ay^2 + By + C)
        if len(leftx) > 10:
            left_fit = np.polyfit(lefty, leftx, 2)
            left_detected = True
        if len(rightx) > 10:
            right_fit = np.polyfit(righty, rightx, 2)
            right_detected = True

        if left_fit is not None and right_fit is not None:
            sample_y = np.linspace(80.0, 235.0, 6, dtype=np.float32)
            width_samples = (
                right_fit[0] * sample_y**2 + right_fit[1] * sample_y + right_fit[2]
            ) - (
                left_fit[0] * sample_y**2 + left_fit[1] * sample_y + left_fit[2]
            )
            valid_widths = width_samples[width_samples > 1.0]
            if valid_widths.size > 0:
                lane_width_px = float(np.median(valid_widths))

        # Update cached fits and their age counters.
        # A fit is only used from cache if it is younger than max_fit_age_frames.
        if left_fit is not None:
            self.left_fit_old = left_fit
            self._left_fit_age = 0
        else:
            self._left_fit_age += 1
            if self._left_fit_age <= self.max_fit_age_frames:
                left_fit = self.left_fit_old  # still fresh enough
            else:
                left_fit = None  # expired — do not use

        if right_fit is not None:
            self.right_fit_old = right_fit
            self._right_fit_age = 0
        else:
            self._right_fit_age += 1
            if self._right_fit_age <= self.max_fit_age_frames:
                right_fit = self.right_fit_old  # still fresh enough
            else:
                right_fit = None  # expired — do not use

        return LaneDetectionResult(
            left_fit=left_fit,
            right_fit=right_fit,
            Minv=self.Minv,
            left_detected=left_detected,
            right_detected=right_detected,
            lane_width_px=lane_width_px,
        )

    def draw_splines(self, image: np.ndarray, waypoints: np.ndarray | None = None) -> np.ndarray:
        """Draw camera-space waypoint overlays onto an image."""
        annotated = image.copy()
        h, w = image.shape[:2]

        # Waypoints are expressed in the detector's 320x240 camera space. Scale
        # them to the debug frame size used for visualization.
        if waypoints is not None and waypoints.ndim == 2 and waypoints.shape[0] == 2:
            col = (waypoints[0] / 320.0 * w).astype(int)
            row = (waypoints[1] / 240.0 * h).astype(int)
            for cx, cy in zip(col, row):
                cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)

        return annotated
