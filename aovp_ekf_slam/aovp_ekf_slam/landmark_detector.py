"""Extract circular landmarks from a 2-D laser scan."""

from typing import Iterable, List, Tuple

import numpy as np


def _split_segments(points: np.ndarray, indices: np.ndarray) -> List[np.ndarray]:
    if points.size == 0:
        return []
    segments = []
    start = 0
    for i in range(1, points.shape[0]):
        range_scale = max(np.linalg.norm(points[i - 1]), np.linalg.norm(points[i]))
        allowed_gap = 0.045 + 0.018 * range_scale
        missing_beam = indices[i] != indices[i - 1] + 1
        if missing_beam or np.linalg.norm(points[i] - points[i - 1]) > allowed_gap:
            segments.append(points[start:i])
            start = i
    segments.append(points[start:])
    return segments


def _fit_circle(points: np.ndarray) -> Tuple[np.ndarray, float, float]:
    # Algebraic least squares: x^2+y^2 = 2*cx*x + 2*cy*y + k.
    matrix = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1],
                              np.ones(points.shape[0])))
    rhs = np.sum(points * points, axis=1)
    solution, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    center = solution[:2]
    radius_sq = solution[2] + center @ center
    radius = float(np.sqrt(max(radius_sq, 0.0)))
    residual = float(np.sqrt(np.mean(
        (np.linalg.norm(points - center, axis=1) - radius) ** 2
    )))
    return center, radius, residual


def detect_circular_landmarks(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    sensor_x_offset: float = 0.20,
    radius_min: float = 0.07,
    radius_max: float = 0.18,
    residual_max: float = 0.018,
) -> List[Tuple[float, float]]:
    """Return landmark centers as (range, bearing) in the base frame."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(ranges.size) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    valid_indices = np.flatnonzero(valid)
    scan_points = np.column_stack((
        ranges[valid] * np.cos(angles[valid]),
        ranges[valid] * np.sin(angles[valid]),
    ))

    observations = []
    for segment in _split_segments(scan_points, valid_indices):
        if segment.shape[0] < 5 or segment.shape[0] > 90:
            continue
        chord = np.linalg.norm(segment[-1] - segment[0])
        if chord < 0.035 or chord > 0.38:
            continue
        center, radius, residual = _fit_circle(segment)
        if not radius_min <= radius <= radius_max or residual > residual_max:
            continue
        # Convert lidar coordinates to the base_footprint origin.
        center[0] += sensor_x_offset
        distance = float(np.linalg.norm(center))
        bearing = float(np.arctan2(center[1], center[0]))
        if 0.20 < distance < range_max:
            observations.append((distance, bearing))
    return observations
