import math
import unittest

import numpy as np

from aovp_ekf_slam.ekf import EkfSlam, wrap_angle


class TestEkfSlam(unittest.TestCase):
    def test_angle_wrapping(self):
        self.assertTrue(math.isclose(wrap_angle(3.0 * math.pi), -math.pi))
        self.assertTrue(math.isclose(wrap_angle(-3.0 * math.pi), -math.pi))

    def test_prediction_moves_robot_forward(self):
        slam = EkfSlam()
        slam.predict(1.0, 0.0)
        np.testing.assert_allclose(
            slam.state[:3], [1.0, 0.0, 0.0], atol=1e-12)

    def test_prediction_includes_front_drive_side_slip(self):
        slam = EkfSlam()
        slam.predict(1.0, 0.0, lateral_distance=0.25)
        np.testing.assert_allclose(
            slam.state[:3], [1.0, 0.25, 0.0], atol=1e-12)

    def test_new_landmark_and_reobservation(self):
        slam = EkfSlam(range_std=0.05, bearing_std=0.02)
        matched, added = slam.observe([(2.0, 0.0)])
        self.assertEqual((matched, added), (0, 1))
        np.testing.assert_allclose(
            slam.state[3:5], [2.0, 0.0], atol=1e-12)
        matched, added = slam.observe([(2.01, 0.005)])
        self.assertEqual((matched, added), (1, 0))
        self.assertEqual(slam.landmark_count, 1)

    def test_covariance_stays_symmetric_positive_semidefinite(self):
        slam = EkfSlam()
        slam.observe([(2.0, 0.2), (3.0, -0.4)])
        for _ in range(20):
            slam.predict(0.10, 0.01)
            slam.observe([(1.9, 0.19), (2.9, -0.42)])
        np.testing.assert_allclose(
            slam.covariance, slam.covariance.T, atol=1e-10)
        self.assertGreater(np.linalg.eigvalsh(slam.covariance).min(), -1e-9)


if __name__ == '__main__':
    unittest.main()
