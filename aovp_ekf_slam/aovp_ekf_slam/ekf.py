"""Small, deliberately classical EKF-SLAM implementation.

The state vector is [robot_x, robot_y, robot_yaw, landmark_1_x,
landmark_1_y, ...].  Odometry increments drive the prediction and anonymous
range-bearing observations are associated using a Mahalanobis gate.
"""

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Association:
    landmark: Optional[int]
    distance: float


class EkfSlam:
    """EKF-SLAM with unknown, stationary point landmarks."""

    def __init__(
        self,
        initial_pose_covariance: Iterable[float] = (0.02, 0.02, 0.01),
        range_std: float = 0.045,
        bearing_std: float = 0.025,
        gate: float = 5.991,
        max_landmarks: int = 30,
    ) -> None:
        self.state = np.zeros(3, dtype=float)
        self.covariance = np.diag(np.asarray(initial_pose_covariance, dtype=float))
        self.measurement_noise = np.diag([range_std ** 2, bearing_std ** 2])
        self.gate = gate
        self.max_landmarks = max_landmarks

    @property
    def landmark_count(self) -> int:
        return (self.state.size - 3) // 2

    def predict(
        self,
        distance: float,
        delta_yaw: float,
        distance_std: float = 0.015,
        yaw_std: float = 0.012,
        lateral_distance: float = 0.0,
        lateral_std: float = 0.020,
    ) -> None:
        """Apply a midpoint skid-steer motion model.

        ``lateral_distance`` is normally zero, but a front-driven cart without
        steering develops measurable side slip during braking turns.  Treating
        that displacement as a noisy control input keeps the classical EKF
        motion model consistent with the simulated chassis.
        """
        theta = self.state[2]
        heading = theta + 0.5 * delta_yaw
        c = np.cos(heading)
        s = np.sin(heading)

        self.state[0] += distance * c - lateral_distance * s
        self.state[1] += distance * s + lateral_distance * c
        self.state[2] = wrap_angle(theta + delta_yaw)

        size = self.state.size
        jacobian = np.eye(size)
        jacobian[0, 2] = -distance * s - lateral_distance * c
        jacobian[1, 2] = distance * c - lateral_distance * s

        control_jacobian = np.zeros((size, 3))
        control_jacobian[0, 0] = c
        control_jacobian[1, 0] = s
        control_jacobian[0, 1] = -s
        control_jacobian[1, 1] = c
        control_jacobian[0, 2] = 0.5 * jacobian[0, 2]
        control_jacobian[1, 2] = 0.5 * jacobian[1, 2]
        control_jacobian[2, 2] = 1.0

        # A small floor prevents unjustified certainty while the robot is idle.
        motion_noise = np.diag([
            distance_std ** 2 + 0.01 * abs(distance),
            lateral_std ** 2 + 0.03 * abs(lateral_distance),
            yaw_std ** 2 + 0.01 * abs(delta_yaw),
        ])
        self.covariance = (
            jacobian @ self.covariance @ jacobian.T
            + control_jacobian @ motion_noise @ control_jacobian.T
        )
        self._symmetrize()

    def expected_observation(self, landmark: int) -> Tuple[np.ndarray, np.ndarray]:
        """Return expected [range, bearing] and its full-state Jacobian."""
        offset = 3 + 2 * landmark
        dx = self.state[offset] - self.state[0]
        dy = self.state[offset + 1] - self.state[1]
        q = max(dx * dx + dy * dy, 1.0e-12)
        distance = np.sqrt(q)

        expected = np.array([
            distance,
            wrap_angle(np.arctan2(dy, dx) - self.state[2]),
        ])
        h = np.zeros((2, self.state.size))
        h[:, :3] = np.array([
            [-dx / distance, -dy / distance, 0.0],
            [dy / q, -dx / q, -1.0],
        ])
        h[:, offset:offset + 2] = np.array([
            [dx / distance, dy / distance],
            [-dy / q, dx / q],
        ])
        return expected, h

    def associate(
        self,
        observation: Iterable[float],
        excluded: Optional[set] = None,
    ) -> Association:
        """Find the most likely landmark inside the chi-square gate."""
        z = np.asarray(observation, dtype=float)
        excluded = excluded or set()
        best_index = None
        best_distance = np.inf
        for index in range(self.landmark_count):
            if index in excluded:
                continue
            expected, h = self.expected_observation(index)
            innovation = z - expected
            innovation[1] = wrap_angle(innovation[1])
            innovation_cov = h @ self.covariance @ h.T + self.measurement_noise
            mahalanobis = float(
                innovation.T @ np.linalg.solve(innovation_cov, innovation)
            )
            if mahalanobis < best_distance:
                best_index = index
                best_distance = mahalanobis
        if best_distance > self.gate:
            best_index = None
        return Association(best_index, best_distance)

    def observe(self, observations: Iterable[Iterable[float]]) -> Tuple[int, int]:
        """Associate and update one scan; return (matched, newly_added)."""
        matched = 0
        added = 0
        used = set()
        for raw_observation in observations:
            observation = np.asarray(raw_observation, dtype=float)
            if observation[0] <= 0.0 or not np.all(np.isfinite(observation)):
                continue
            association = self.associate(observation, used)
            if association.landmark is None:
                if self.landmark_count < self.max_landmarks:
                    used.add(self._add_landmark(observation))
                    added += 1
            else:
                self._update_landmark(association.landmark, observation)
                used.add(association.landmark)
                matched += 1
        return matched, added

    def _add_landmark(self, observation: np.ndarray) -> int:
        distance, bearing = observation
        angle = self.state[2] + bearing
        c = np.cos(angle)
        s = np.sin(angle)
        landmark = np.array([
            self.state[0] + distance * c,
            self.state[1] + distance * s,
        ])

        old_size = self.state.size
        old_covariance = self.covariance
        pose_jacobian = np.array([
            [1.0, 0.0, -distance * s],
            [0.0, 1.0, distance * c],
        ])
        observation_jacobian = np.array([
            [c, -distance * s],
            [s, distance * c],
        ])

        cross_covariance = pose_jacobian @ old_covariance[:3, :]
        landmark_covariance = (
            pose_jacobian @ old_covariance[:3, :3] @ pose_jacobian.T
            + observation_jacobian
            @ self.measurement_noise
            @ observation_jacobian.T
        )

        expanded = np.zeros((old_size + 2, old_size + 2))
        expanded[:old_size, :old_size] = old_covariance
        expanded[old_size:, :old_size] = cross_covariance
        expanded[:old_size, old_size:] = cross_covariance.T
        expanded[old_size:, old_size:] = landmark_covariance
        self.state = np.concatenate((self.state, landmark))
        self.covariance = expanded
        self._symmetrize()
        return self.landmark_count - 1

    def _update_landmark(self, landmark: int, observation: np.ndarray) -> None:
        expected, h = self.expected_observation(landmark)
        innovation = observation - expected
        innovation[1] = wrap_angle(innovation[1])
        innovation_cov = h @ self.covariance @ h.T + self.measurement_noise
        gain = self.covariance @ h.T @ np.linalg.inv(innovation_cov)
        self.state += gain @ innovation
        self.state[2] = wrap_angle(self.state[2])

        identity = np.eye(self.state.size)
        # Joseph form is more robust to floating-point roundoff.
        residual = identity - gain @ h
        self.covariance = (
            residual @ self.covariance @ residual.T
            + gain @ self.measurement_noise @ gain.T
        )
        self._symmetrize()

    def _symmetrize(self) -> None:
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
