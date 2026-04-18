#!/usr/bin/env python3
"""
Lane Detection — adapted from dev/lane_detection.py for JetRacer ROS 2.

Changes vs simulator original:
  - Removed matplotlib import and plot_state_lane() (no display on Jetson)
  - Removed unused `time` import
  - Added __all__ for clean namespace
  - Docstrings clarified for real-robot context
  - Accepts any image size (caller is responsible for resizing to 96×96 before
    calling lane_detection(); this keeps the pixel-space constants unchanged)
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.signal import find_peaks

__all__ = ["LaneDetection"]


class LaneDetection:
    """Lane boundary detection using gradient edge detection and B-spline fitting.

    The detector works in a **96×96 pixel** image space.  When using the
    JetRacer's 640×480 CSI camera, resize the frame to 96×96 before calling
    :meth:`lane_detection`.

    Args:
        cut_size: Row index at which to cut the image (bottom of crop = car bonnet).
            Lower values = less road visible ahead. Default 68.
        spline_smoothness: Spline fitting smoothness (``s`` parameter in
            :func:`scipy.interpolate.splprep`). Higher = smoother but less accurate.
        gradient_threshold: Minimum gradient magnitude to keep as an edge candidate.
        distance_maxima_gradient: Minimum distance between gradient peaks in a row.
    """

    def __init__(
        self,
        cut_size: int = 68,
        spline_smoothness: float = 10.0,
        gradient_threshold: float = 14.0,
        distance_maxima_gradient: int = 3,
    ) -> None:
        self.car_position = np.array([48, 0])
        self.cut_size = int(cut_size)
        self.spline_smoothness = float(spline_smoothness)
        self.gradient_threshold = float(gradient_threshold)
        self.distance_maxima_gradient = int(distance_maxima_gradient)
        self.lane_boundary1_old = None
        self.lane_boundary2_old = None

    def cut_gray(self, state_image_full: np.ndarray) -> np.ndarray:
        """Crop image to the road region ahead and convert to grayscale.

        Args:
            state_image_full: ``(96, 96, 3)`` uint8 BGR or RGB image.

        Returns:
            ``(cut_size, 96, 1)`` float64 grayscale array, flipped vertically
            so row 0 is closest to the car.
        """
        gray = state_image_full[: self.cut_size]
        gray = np.dot(gray[..., :3], [0.299, 0.587, 0.114])
        gray = np.expand_dims(gray, axis=2)
        return gray[::-1]

    def edge_detection(self, gray_image: np.ndarray) -> np.ndarray:
        """Compute thresholded gradient magnitude.

        Args:
            gray_image: ``(cut_size, 96, 1)`` float64 grayscale array.

        Returns:
            ``(cut_size, 96, 1)`` gradient magnitude array, zeroed below threshold.
        """
        img = gray_image.squeeze()
        grad_y, grad_x = np.gradient(img)
        gradient_sum = np.abs(grad_x) + np.abs(grad_y)
        gradient_sum[gradient_sum < self.gradient_threshold] = 0.0
        return np.expand_dims(gradient_sum, axis=2)

    def find_maxima_gradient_rowwise(self, gradient_sum: np.ndarray) -> np.ndarray:
        """Find local column maxima in each row of the gradient image.

        Returns:
            ``(2, M)`` array of ``[col, row]`` maxima coordinates, or empty array.
        """
        maxima_list = []
        for row_index in range(gradient_sum.shape[0]):
            row = gradient_sum[row_index, :, 0]
            peaks, _ = find_peaks(row, distance=self.distance_maxima_gradient)
            for col_index in peaks:
                maxima_list.append([col_index, row_index])
        if not maxima_list:
            return np.empty((2, 0), dtype=np.float64)
        return np.array(maxima_list).T

    def find_first_lane_point(self, gradient_sum: np.ndarray):
        """Locate the seed points for the two lane boundaries nearest the car.

        Returns:
            Tuple ``(lb1_start, lb2_start, lanes_found)`` where start points are
            shape ``(1, 2)`` arrays of ``[col, row]``.
        """
        lanes_found = False
        row = 0
        lane_boundary1_startpoint = np.array([[0, 0]])
        lane_boundary2_startpoint = np.array([[0, 0]])

        while not lanes_found and row < self.cut_size:
            peaks = find_peaks(gradient_sum[row, :, 0], distance=3)[0]
            n = peaks.shape[0]

            if n == 1:
                lane_boundary1_startpoint = np.array([[peaks[0], row]])
                lane_boundary2_startpoint = np.array(
                    [[0 if peaks[0] >= 48 else 95, row]]
                )
                lanes_found = True

            elif n == 2:
                lane_boundary1_startpoint = np.array([[peaks[0], row]])
                lane_boundary2_startpoint = np.array([[peaks[1], row]])
                lanes_found = True

            elif n > 2:
                order = np.argsort((peaks - self.car_position[0]) ** 2)
                lane_boundary1_startpoint = np.array([[peaks[order[0]], row]])
                lane_boundary2_startpoint = np.array([[peaks[order[1]], row]])
                lanes_found = True

            row += 1

        return lane_boundary1_startpoint, lane_boundary2_startpoint, lanes_found

    def lane_detection(self, state_image_full: np.ndarray):
        """Detect lane boundaries and fit B-splines.

        Args:
            state_image_full: ``(96, 96, 3)`` uint8 image.  The JetRacer
                camera frame must be resized to ``(96, 96)`` before calling.

        Returns:
            ``(lane_boundary1, lane_boundary2)`` — each is a scipy spline
            ``(t, c, k)`` tuple, or ``None`` / the previous frame's spline if
            detection fails.
        """
        gray_state = self.cut_gray(state_image_full)
        gradient_sum = self.edge_detection(gray_state)
        maxima = self.find_maxima_gradient_rowwise(gradient_sum)

        lb1_start, lb2_start, lane_found = self.find_first_lane_point(gradient_sum)
        lane_boundary1, lane_boundary2 = self.lane_boundary1_old, self.lane_boundary2_old

        if lane_found and maxima.size > 0:
            maxima_list = maxima.T.tolist()

            lb1_pts = [lb1_start[0].tolist()]
            lb2_pts = [lb2_start[0].tolist()]

            for seed in [lb1_start[0], lb2_start[0]]:
                seed_l = seed.tolist()
                if seed_l in maxima_list:
                    maxima_list.remove(seed_l)

            def _grow(lane_pts):
                while True:
                    last = lane_pts[-1]
                    next_row = last[1] + 1
                    if next_row >= self.cut_size:
                        break
                    row_pts = [p for p in maxima_list if p[1] == next_row]
                    if not row_pts:
                        break
                    dists = [abs(p[0] - last[0]) for p in row_pts]
                    best_d = min(dists)
                    if best_d >= 100:
                        break
                    best_pt = row_pts[dists.index(best_d)]
                    lane_pts.append(best_pt)
                    maxima_list.remove(best_pt)
                return lane_pts

            lb1_pts = np.array(_grow(lb1_pts))
            lb2_pts = np.array(_grow(lb2_pts))

            if lb1_pts.shape[0] > 4 and lb2_pts.shape[0] > 4:
                tck1, _ = splprep(
                    [lb1_pts[:, 0], lb1_pts[:, 1]], s=self.spline_smoothness
                )
                tck2, _ = splprep(
                    [lb2_pts[:, 0], lb2_pts[:, 1]], s=self.spline_smoothness
                )
                lane_boundary1 = tck1
                lane_boundary2 = tck2

        self.lane_boundary1_old = lane_boundary1
        self.lane_boundary2_old = lane_boundary2
        return lane_boundary1, lane_boundary2

    def draw_splines(
        self, image: np.ndarray, waypoints: np.ndarray | None = None
    ) -> np.ndarray:
        """Overlay lane splines (and optional waypoints) onto an image.

        Works on any resolution — uses the current lane boundaries stored after
        the last :meth:`lane_detection` call.  Call after ``lane_detection``.

        Args:
            image: ``(H, W, 3)`` BGR image (e.g. original 640×480 camera frame).
            waypoints: Optional ``(2, N)`` waypoints in 96×96 pixel space.

        Returns:
            Annotated copy of *image*.
        """
        import cv2  # local import — keeps the class importable without cv2

        annotated = image.copy()
        h, w = image.shape[:2]

        def _to_hw(pts_96, axis0, axis1):
            """Scale 96-pixel coordinates to (h, w) image space."""
            col = (pts_96[axis0] / 96.0 * w).astype(int)
            cut = max(float(self.cut_size), 1.0)
            row = (pts_96[axis1] / cut * h).astype(int)
            return col, row

        t = np.linspace(0, 1, 20)
        for spline, colour in [
            (self.lane_boundary1_old, (0, 165, 255)),   # orange
            (self.lane_boundary2_old, (0, 165, 255)),
        ]:
            if spline is None:
                continue
            try:
                pts = np.array(splev(t, spline))  # [2, 20] in 96px space
                col, row = _to_hw(pts, 0, 1)
                for i in range(len(col) - 1):
                    cv2.line(annotated, (col[i], row[i]), (col[i + 1], row[i + 1]), colour, 2)
            except Exception:
                pass

        if waypoints is not None and waypoints.ndim == 2 and waypoints.shape[0] == 2:
            col, row = _to_hw(waypoints, 0, 1)
            for cx, cy in zip(col, row):
                cv2.circle(annotated, (cx, cy), 4, (255, 255, 255), -1)

        return annotated
