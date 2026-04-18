"""
Waypoint prediction — copied verbatim from dev/waypoint_prediction.py.

No changes required: all logic is pure NumPy/SciPy math that is
hardware-agnostic. Speed unit conversion happens in lane_following_node.py.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from scipy.interpolate import splev
from scipy.optimize import minimize

__all__ = ["waypoint_prediction", "target_speed_prediction"]


# ─── Utilities ────────────────────────────────────────────────────────────────

def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v, axis=0)
    norm = np.where(norm == 0.0, 1.0, norm)
    return v / norm


def curvature(waypoints: np.ndarray) -> float:
    """Curvature proxy for path smoothing and speed prediction.

    Args:
        waypoints: ``(2, N)`` array.

    Returns:
        Scalar curvature proxy (sum of cosines of successive direction changes).
    """
    if waypoints.ndim != 2 or waypoints.shape[0] != 2 or waypoints.shape[1] < 3:
        return 0.0
    delta = waypoints[:, 1:] - waypoints[:, :-1]
    delta_n = normalize(delta)
    if delta_n.shape[1] < 2:
        return 0.0
    dots = np.sum(delta_n[:, :-1] * delta_n[:, 1:], axis=0)
    return float(np.sum(dots))


def _is_valid_tck(tck: Optional[Tuple]) -> bool:
    if tck is None or not isinstance(tck, (tuple, list)) or len(tck) != 3:
        return False
    _, c, k = tck
    try:
        int(k)
    except Exception:
        return False
    try:
        tc = np.asarray(c)
        if tc.ndim < 1 or tc.size == 0:
            return False
    except Exception:
        return False
    return True


def _fallback_waypoints(num_waypoints: int = 6) -> np.ndarray:
    """Straight-ahead centred fallback in 96-pixel image space."""
    x = np.full(num_waypoints, 48.0, dtype=np.float32)
    y = np.linspace(5.0, 60.0, num_waypoints, dtype=np.float32)
    return np.vstack([x, y])


# ─── Smoothing objective ───────────────────────────────────────────────────────

def smoothing_objective(
    waypoints_flat: np.ndarray,
    waypoints_center_flat: np.ndarray,
    beta: float = 30.0,
) -> float:
    if waypoints_center_flat.ndim != 1 or waypoints_flat.ndim != 1:
        return 0.0
    if waypoints_center_flat.shape[0] != waypoints_flat.shape[0]:
        return 0.0
    n = waypoints_center_flat.shape[0] // 2
    wps = waypoints_flat.reshape(2, n)
    wps_c = waypoints_center_flat.reshape(2, n)
    return float(np.sum((wps - wps_c) ** 2)) - beta * curvature(wps)


# ─── Public API ───────────────────────────────────────────────────────────────

def waypoint_prediction(
    roadside1_spline: Optional[Tuple],
    roadside2_spline: Optional[Tuple],
    num_waypoints: int = 6,
    way_type: str = "smooth",
    smoothing_beta: float = 30.0,
) -> np.ndarray:
    """Predict the centre-lane waypoints from two boundary splines.

    Args:
        roadside1_spline: Left boundary spline ``(t, c, k)`` from LaneDetection.
        roadside2_spline: Right boundary spline ``(t, c, k)`` from LaneDetection.
        num_waypoints: Number of waypoints to sample.
        way_type: ``"center"`` (simple midpoint) or ``"smooth"`` (L-BFGS-B
            smoothed to maximise path curvature continuity).  Use ``"center"``
            on resource-constrained Jetson Nano at high frame rates.
        smoothing_beta: Curvature weighting in the smoothing objective.

    Returns:
        ``(2, num_waypoints)`` float32 array in 96-pixel image space.
        Falls back to a straight-ahead path on any failure.
    """
    if not (_is_valid_tck(roadside1_spline) and _is_valid_tck(roadside2_spline)):
        return _fallback_waypoints(num_waypoints)

    u = np.linspace(0.0, 1.0, num_waypoints)
    try:
        r1 = np.array(splev(u, roadside1_spline))
        r2 = np.array(splev(u, roadside2_spline))
    except Exception:
        return _fallback_waypoints(num_waypoints)

    if r1.shape != (2, num_waypoints) or r2.shape != (2, num_waypoints):
        return _fallback_waypoints(num_waypoints)
    if not (np.isfinite(r1).all() and np.isfinite(r2).all()):
        return _fallback_waypoints(num_waypoints)

    waypoints_center = (r1 + r2) / 2.0

    if way_type == "center":
        return waypoints_center.astype(np.float32, copy=False)

    if way_type == "smooth":
        w0 = waypoints_center.flatten()
        try:
            res = minimize(
                smoothing_objective,
                w0,
                args=(w0, smoothing_beta),
                method="L-BFGS-B",
                options={"maxiter": 200, "ftol": 1e-6},
            )
            w_opt = res.x if res.success and np.isfinite(res.x).all() else w0
        except Exception:
            w_opt = w0
        return w_opt.reshape(2, num_waypoints).astype(np.float32, copy=False)

    return waypoints_center.astype(np.float32, copy=False)


def target_speed_prediction(
    waypoints: np.ndarray,
    num_waypoints_used: int = 4,
    max_speed: float = 30.0,
    min_speed: float = 15.0,
    K_v: float = 2.5,
) -> float:
    """Predict a scalar target speed from path curvature.

    Returns a value in the **simulator speed scale** ``[min_speed, max_speed]``.
    The caller (``lane_following_node.py``) must convert to m/s by scaling with
    ``max_speed_ms / max_speed``.

    The curvature term is 0 on a straight road → returns ``max_speed``;
    larger curves reduce speed toward ``min_speed``.
    """
    if (
        waypoints is None
        or waypoints.ndim != 2
        or waypoints.shape[0] != 2
        or waypoints.shape[1] < 2
    ):
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
