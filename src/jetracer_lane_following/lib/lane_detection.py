#!/usr/bin/env python3
"""
Advanced JetRacer Lane Detection (ROS 2)

Upgraded from legacy point-growing splines to a robust 
Bird's-Eye View Sliding-Window Polynomial fit, heavily tailored 
for high-contrast "white lines on black track" physical robotics.
"""

from __future__ import annotations

import cv2
import numpy as np

__all__ = ["LaneDetection"]


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
    ) -> None:
        """
        Initializes perspective transform geometries.
        
        Args:
            luminance_threshold: Drops background textures below this brightness (0-255).
            roi_top_y: The height (Y) where the road vanishes into the horizon.
            roi_top_width: The width of the road trapezoid at the vanishing point.
        """
        self.lum_thresh = int(luminance_threshold)

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

    def lane_detection(self, state_image_full: np.ndarray):
        """
        Calculates polynomial boundaries.
        Args:
            state_image_full: (96, 96, 3) Image Frame.
        Returns:
            left_fit, right_fit, Minv
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
        minpix = 15

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
            
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = None
        right_fit = None

        # 5. Parabola fitting (x = Ay^2 + By + C)
        if len(leftx) > 10:
            left_fit = np.polyfit(lefty, leftx, 2)
        if len(rightx) > 10:
            right_fit = np.polyfit(righty, rightx, 2)

        if left_fit is None:
            left_fit = self.left_fit_old
        if right_fit is None:
            right_fit = self.right_fit_old

        self.left_fit_old = left_fit
        self.right_fit_old = right_fit

        return left_fit, right_fit, self.Minv

    def draw_splines(self, image: np.ndarray, waypoints: np.ndarray | None = None) -> np.ndarray:
        """
        Native projection matrix to map real time output overlays.
        """
        annotated = image.copy()
        h, w = image.shape[:2]

        # Draw physical waypoints backwards-mapped through perspective
        if waypoints is not None and waypoints.ndim == 2 and waypoints.shape[0] == 2:
            col = (waypoints[0] / 320.0 * w).astype(int)
            row = (waypoints[1] / 240.0 * h).astype(int)
            for cx, cy in zip(col, row):
                cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)

        return annotated
