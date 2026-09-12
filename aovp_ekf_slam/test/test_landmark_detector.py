import math
import unittest

import numpy as np

from aovp_ekf_slam.landmark_detector import detect_circular_landmarks


def _ray_circle_range(angle, center_x, center_y, radius):
    direction = np.array([math.cos(angle), math.sin(angle)])
    center = np.array([center_x, center_y])
    projection = direction @ center
    discriminant = projection * projection - (center @ center - radius * radius)
    if discriminant < 0.0:
        return math.inf
    distance = projection - math.sqrt(discriminant)
    return distance if distance > 0.0 else math.inf


class TestLandmarkDetector(unittest.TestCase):
    def test_detects_twelve_centimeter_cylinder(self):
        angle_min = -1.0
        angle_increment = 0.0025
        angles = angle_min + np.arange(801) * angle_increment
        # Circle coordinates are in the lidar frame. The detector then adds
        # the x offset to report the center in base_footprint.
        ranges = [_ray_circle_range(a, 2.0, 0.30, 0.12) for a in angles]
        result = detect_circular_landmarks(
            ranges, angle_min, angle_increment, 0.12, 10.0,
            sensor_x_offset=0.20,
        )
        self.assertEqual(len(result), 1)
        expected_range = math.hypot(2.20, 0.30)
        expected_bearing = math.atan2(0.30, 2.20)
        self.assertTrue(math.isclose(
            result[0][0], expected_range, abs_tol=0.03))
        self.assertTrue(math.isclose(
            result[0][1], expected_bearing, abs_tol=0.015))

    def test_ignores_flat_wall_segment(self):
        angle_min = -0.5
        angle_increment = 0.0025
        angles = angle_min + np.arange(401) * angle_increment
        ranges = np.full(angles.size, math.inf)
        mask = np.abs(angles) < 0.12
        ranges[mask] = 2.0 / np.cos(angles[mask])
        result = detect_circular_landmarks(
            ranges, angle_min, angle_increment, 0.12, 10.0)
        self.assertEqual(result, [])


if __name__ == '__main__':
    unittest.main()
