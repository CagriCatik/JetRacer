#!/usr/bin/env python3
"""
Waypoint planning helpers for JetRacer lane following.

The lane detector solves the path in a 320x240 bird's-eye view (BEV). This
module keeps that BEV representation intact, derives an approximate
vehicle-frame path in meters, and only projects back to camera space for
visualization.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

__all__ = ["WaypointPlan", "target_speed_prediction", "waypoint_prediction"]


@dataclass(slots=True)
class WaypointPlan:
    """Container for the planned centerline in multiple coordinate systems."""

    waypoints_bev: np.ndarray
    waypoints_vehicle: np.ndarray
    waypoints_camera: np.ndarray
    lane_width_px: float | None


def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v, axis=0)
    norm = np.where(norm == 0.0, 1.0, norm)
    return v / norm


def _sample_ploty(num_waypoints: int) -> np.ndarray:
    """Bias waypoint samples toward the near field where control is sensitive."""
    count = max(2, int(num_waypoints))
    near_y = 235.0
    far_y = 70.0
    t = np.linspace(0.0, 1.0, count, dtype=np.float32)
    return (near_y - (near_y - far_y) * np.power(t, 1.35)).astype(np.float32)


def _project_to_camera(waypoints_bev: np.ndarray, Minv: np.ndarray | None) -> np.ndarray:
    if waypoints_bev.ndim != 2 or waypoints_bev.shape[0] != 2:
        return np.zeros((2, 0), dtype=np.float32)

    if Minv is None or not np.isfinite(Minv).all():
        return waypoints_bev.astype(np.float32, copy=True)

    pts_bev = waypoints_bev.T.reshape(-1, 1, 2).astype(np.float32)
    pts_cam = cv2.perspectiveTransform(pts_bev, Minv)
    return pts_cam[:, 0, :].T.astype(np.float32)


def _bev_to_vehicle(
    waypoints_bev: np.ndarray,
    *,
    bev_vehicle_center_x_px: float,
    bev_vehicle_origin_y_px: float,
    bev_lateral_m_per_px: float,
    bev_forward_m_per_px: float,
) -> np.ndarray:
    x_bev = waypoints_bev[0].astype(np.float32, copy=False)
    y_bev = waypoints_bev[1].astype(np.float32, copy=False)

    x_forward = (float(bev_vehicle_origin_y_px) - y_bev) * float(bev_forward_m_per_px)
    y_left = (float(bev_vehicle_center_x_px) - x_bev) * float(bev_lateral_m_per_px)
    return np.vstack([x_forward, y_left]).astype(np.float32)


def _fallback_waypoints(
    *,
    num_waypoints: int,
    Minv: np.ndarray | None,
    bev_vehicle_center_x_px: float,
    bev_vehicle_origin_y_px: float,
    bev_lateral_m_per_px: float,
    bev_forward_m_per_px: float,
) -> WaypointPlan:
    x_bev = np.full(max(2, int(num_waypoints)), float(bev_vehicle_center_x_px), dtype=np.float32)
    y_bev = _sample_ploty(num_waypoints)
    waypoints_bev = np.vstack([x_bev, y_bev]).astype(np.float32)
    return WaypointPlan(
        waypoints_bev=waypoints_bev,
        waypoints_vehicle=_bev_to_vehicle(
            waypoints_bev,
            bev_vehicle_center_x_px=bev_vehicle_center_x_px,
            bev_vehicle_origin_y_px=bev_vehicle_origin_y_px,
            bev_lateral_m_per_px=bev_lateral_m_per_px,
            bev_forward_m_per_px=bev_forward_m_per_px,
        ),
        waypoints_camera=_project_to_camera(waypoints_bev, Minv),
        lane_width_px=None,
    )


def waypoint_prediction(
    left_fit: np.ndarray | None,
    right_fit: np.ndarray | None,
    Minv: np.ndarray | None,
    *,
    num_waypoints: int = 6,
    way_type: str = "center",
    smoothing_beta: float = 30.0,
    fallback_lane_width_px: float = 220.0,
    bev_vehicle_center_x_px: float = 160.0,
    bev_vehicle_origin_y_px: float = 239.0,
    bev_lateral_m_per_px: float = 0.0022,
    bev_forward_m_per_px: float = 0.0030,
) -> WaypointPlan:
    """
    Predict a centerline path in BEV, vehicle, and camera coordinates.

    The returned vehicle-frame path uses a pseudo-metric BEV scale and is the
    representation intended for control.
    """
    if left_fit is None and right_fit is None:
        return _fallback_waypoints(
            num_waypoints=num_waypoints,
            Minv=Minv,
            bev_vehicle_center_x_px=bev_vehicle_center_x_px,
            bev_vehicle_origin_y_px=bev_vehicle_origin_y_px,
            bev_lateral_m_per_px=bev_lateral_m_per_px,
            bev_forward_m_per_px=bev_forward_m_per_px,
        )

    ploty = _sample_ploty(num_waypoints)
    lane_width_px: float | None = None
    fallback_width = float(max(1.0, fallback_lane_width_px))

    if left_fit is not None and right_fit is not None:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        width_samples = right_fitx - left_fitx
        positive_widths = width_samples[width_samples > 1.0]
        if positive_widths.size > 0:
            lane_width_px = float(np.median(positive_widths))
            fallback_width = lane_width_px
        center_fitx = (left_fitx + right_fitx) / 2.0
    elif left_fit is not None:
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        center_fitx = left_fitx + 0.5 * fallback_width
    else:
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        center_fitx = right_fitx - 0.5 * fallback_width

    center_fitx = np.clip(center_fitx, 0.0, 319.0)

    if way_type == "smooth" and center_fitx.size >= 3:
        blend = float(np.clip(smoothing_beta / 100.0, 0.0, 1.0))
        smooth_poly = np.polyfit(ploty, center_fitx, 2)
        smooth_fitx = smooth_poly[0] * ploty**2 + smooth_poly[1] * ploty + smooth_poly[2]
        center_fitx = (1.0 - blend) * center_fitx + blend * smooth_fitx

    waypoints_bev = np.vstack([center_fitx.astype(np.float32), ploty]).astype(np.float32)
    return WaypointPlan(
        waypoints_bev=waypoints_bev,
        waypoints_vehicle=_bev_to_vehicle(
            waypoints_bev,
            bev_vehicle_center_x_px=bev_vehicle_center_x_px,
            bev_vehicle_origin_y_px=bev_vehicle_origin_y_px,
            bev_lateral_m_per_px=bev_lateral_m_per_px,
            bev_forward_m_per_px=bev_forward_m_per_px,
        ),
        waypoints_camera=_project_to_camera(waypoints_bev, Minv),
        lane_width_px=lane_width_px,
    )


def target_speed_prediction(
    waypoints: np.ndarray,
    *,
    num_waypoints_used: int = 4,
    max_speed: float = 0.35,
    min_speed: float = 0.05,
    K_v: float = 2.5,
    confidence: float = 1.0,
    confidence_floor: float = 0.35,
) -> float:
    """Predict a target speed in m/s from local path curvature and confidence."""
    if waypoints is None or waypoints.ndim != 2 or waypoints.shape[0] != 2 or waypoints.shape[1] < 2:
        return float(max(0.0, min_speed))

    n = min(max(2, int(num_waypoints_used)), waypoints.shape[1])
    w = waypoints[:, :n].astype(np.float32, copy=False)
    delta = w[:, 1:] - w[:, :-1]

    if delta.shape[1] == 0:
        return float(np.clip(max_speed, min_speed, max_speed))

    delta_n = normalize(delta)
    if delta_n.shape[1] < 2:
        curvature_term = 0.0
    else:
        dots = np.sum(delta_n[:, :-1] * delta_n[:, 1:], axis=0)
        dots = np.clip(dots, -1.0, 1.0)
        heading_steps = np.arccos(dots)
        forward_extent = max(float(w[0, -1] - w[0, 0]), 1e-3)
        curvature_term = float(np.sum(np.abs(heading_steps)) / forward_extent)

    heading_error = float(np.arctan2(w[1, -1] - w[1, 0], max(float(w[0, -1] - w[0, 0]), 1e-3)))
    curvature_term += abs(heading_error)

    base_speed = (float(max_speed) - float(min_speed)) * np.exp(-float(K_v) * curvature_term) + float(min_speed)
    base_speed = float(np.clip(base_speed, min_speed, max_speed))

    confidence_clamped = float(np.clip(confidence, 0.0, 1.0))
    floor = float(np.clip(confidence_floor, 0.0, 1.0))
    confidence_scale = floor + (1.0 - floor) * confidence_clamped
    speed = float(min_speed) + (base_speed - float(min_speed)) * confidence_scale
    return float(np.clip(speed, min_speed, max_speed))
