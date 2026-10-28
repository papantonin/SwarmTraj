"""Small reference algorithms intended to be replaced by user code."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .state import SwarmState


def regular_polygon(
    count: int,
    radius: float,
    center: ArrayLike = (0.0, 0.0, 1.0),
    phase: float = 0.0,
) -> NDArray[np.float64]:
    """Return 3D target positions equally spaced around a horizontal circle."""

    if count < 1:
        raise ValueError("count must be at least one")
    center_array = np.asarray(center, dtype=float)
    if center_array.shape != (3,):
        raise ValueError("center must have three coordinates")
    angles = phase + np.arange(count) * 2.0 * np.pi / count
    offsets = np.column_stack(
        (radius * np.cos(angles), radius * np.sin(angles), np.zeros(count))
    )
    return center_array + offsets


def flocking_velocity(
    state: SwarmState,
    perception_radius: float = 1.0,
    separation_radius: float = 0.25,
    cohesion_gain: float = 0.8,
    alignment_gain: float = 0.4,
    separation_gain: float = 0.08,
    max_speed: float = 0.6,
    dimensions: int = 3,
) -> NDArray[np.float64]:
    """Compute a Reynolds-style velocity command for every drone.

    This is deliberately compact reference code. It uses only local neighbours
    and demonstrates how an algorithm consumes :class:`SwarmState` telemetry.
    """

    if dimensions not in (2, 3):
        raise ValueError("dimensions must be 2 or 3")
    if separation_radius <= 0 or perception_radius < separation_radius:
        raise ValueError("require 0 < separation_radius <= perception_radius")

    positions = state.positions[:, :dimensions]
    velocities = state.velocities[:, :dimensions]
    distances = state.pairwise_distances(dimensions)
    commands = np.zeros_like(positions)

    for drone in range(state.count):
        neighbours = (distances[drone] > 0) & (distances[drone] <= perception_radius)
        if not np.any(neighbours):
            commands[drone] = velocities[drone]
            continue

        cohesion = positions[neighbours].mean(axis=0) - positions[drone]
        alignment = velocities[neighbours].mean(axis=0) - velocities[drone]

        close = (distances[drone] > 0) & (distances[drone] < separation_radius)
        separation = np.zeros(dimensions)
        if np.any(close):
            away = positions[drone] - positions[close]
            separation = np.sum(away / distances[drone, close][:, None] ** 2, axis=0)

        commands[drone] = (
            velocities[drone]
            + cohesion_gain * cohesion
            + alignment_gain * alignment
            + separation_gain * separation
        )

    speeds = np.linalg.norm(commands, axis=1)
    too_fast = speeds > max_speed
    commands[too_fast] *= (max_speed / speeds[too_fast])[:, None]

    result = np.zeros((state.count, 3))
    result[:, :dimensions] = commands
    return result
