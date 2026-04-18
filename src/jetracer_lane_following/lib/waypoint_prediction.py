#!/usr/bin/env python3
"""
Advanced JetRacer Waypoint Prediction

Converts Parabolic Bird's-Eye View Polynomials into actionable 
96x96 driving trajectories via Inverse Perspective Transformations.
"""

from __future__ import annotations
import numpy as np
import cv2

__all__ = ["waypoint_prediction", "target_speed_prediction"]

def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v, axis=0)
    norm = np.where(norm == 0.0, 1.0, norm)
    return v / norm

def _fallback_waypoints(num_waypoints: int = 6) -> np.ndarray:
    """Straight-ahead fallback."""
    x = np.full(num_waypoints, 160.0, dtype=np.float32)
    y = np.linspace(20.0, 220.0, num_waypoints, dtype=np.float32)
    return np.vstack([x, y])

def waypoint_prediction(
    left_fit: np.ndarray | None,
    right_fit: np.ndarray | None,
    Minv: np.ndarray,
    num_waypoints: int = 6,
    way_type: str = "center",
    smoothing_beta: float = 30.0,
) -> np.ndarray:
    """
    Predicts driving line waypoints by evaluating polynomial intercepts and un-warping vectors.
    """
    if left_fit is None and right_fit is None:
        return _fallback_waypoints(num_waypoints)

    ploty = np.linspace(0, 235, num_waypoints)

    # 1. Evaluate polynomials 
    if left_fit is not None and right_fit is not None:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        center_fitx = (left_fitx + right_fitx) / 2.0
    elif left_fit is not None:
        # Standard lane BEV width approximate ~220px (between 50 and 270) -> Half is 110.0
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        center_fitx = left_fitx + 110.0 
    else:
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        center_fitx = right_fitx - 110.0

    # 2. Warp Back to Camera View Space (Perspective Trajectories)
    pts_bev = np.vstack([center_fitx, ploty]).T.reshape(-1, 1, 2)
    pts_cam = cv2.perspectiveTransform(pts_bev, Minv)
    
    x_cam = pts_cam[:, 0, 0]
    y_cam = pts_cam[:, 0, 1]

    waypoints = np.vstack([x_cam, y_cam]).astype(np.float32)
    return waypoints

def target_speed_prediction(
    waypoints: np.ndarray,
    num_waypoints_used: int = 4,
    max_speed: float = 30.0,
    min_speed: float = 15.0,
    K_v: float = 2.5,
) -> float:
    """Predict a scalar target speed from path curvature."""
    if waypoints is None or waypoints.shape[1] < 2:
        return float(min_speed)

    n = min(num_waypoints_used, waypoints.shape[1])
    w = waypoints[:, :n]
    delta = w[:, 1:] - w[:, :-1]
    delta_n = normalize(delta)

    if delta_n.shape[1] < 2:
        curvature_term = 0.0
    else:
        dots = np.sum(delta_n[:, :-1] * delta_n[:, 1:], axis=0)
        curvature_term = float(np.sum(1.0 - dots))

    speed = (max_speed - min_speed) * np.exp(-K_v * curvature_term) + min_speed
    return float(max(speed, 0.0))
