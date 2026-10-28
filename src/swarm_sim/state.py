"""Backend-independent telemetry returned by a swarm simulator."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class SwarmState:
    """Snapshot of all drones in world coordinates and SI units.

    Array shapes are ``(number_of_drones, 3)``, except ``motor_rpm`` which is
    ``(number_of_drones, 4)``. Angles and angular velocities are in radians.
    """

    time: float
    positions: FloatArray
    velocities: FloatArray
    orientations_rpy: FloatArray
    angular_velocities: FloatArray
    motor_rpm: FloatArray

    @classmethod
    def from_aviary_observation(cls, observation: NDArray, time: float) -> "SwarmState":
        """Convert a gym-pybullet-drones ``(N, 20)`` observation."""

        obs = np.asarray(observation, dtype=np.float64)
        if obs.ndim != 2 or obs.shape[1] != 20:
            raise ValueError(f"expected an observation shaped (N, 20), got {obs.shape}")
        return cls(
            time=float(time),
            positions=obs[:, 0:3].copy(),
            orientations_rpy=obs[:, 7:10].copy(),
            velocities=obs[:, 10:13].copy(),
            angular_velocities=obs[:, 13:16].copy(),
            motor_rpm=obs[:, 16:20].copy(),
        )

    @property
    def count(self) -> int:
        return int(self.positions.shape[0])

    def pairwise_distances(self, dimensions: int = 3) -> FloatArray:
        """Return the NxN Euclidean distance matrix."""

        if dimensions not in (2, 3):
            raise ValueError("dimensions must be 2 or 3")
        delta = self.positions[:, None, :dimensions] - self.positions[None, :, :dimensions]
        return np.linalg.norm(delta, axis=2)

    def adjacency(self, radius: float, dimensions: int = 3) -> NDArray[np.bool_]:
        """Return an NxN neighbour mask, excluding each drone itself."""

        if radius <= 0:
            raise ValueError("radius must be positive")
        distances = self.pairwise_distances(dimensions)
        return (distances <= radius) & ~np.eye(self.count, dtype=bool)
