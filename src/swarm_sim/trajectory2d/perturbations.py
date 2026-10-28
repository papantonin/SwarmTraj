"""Deterministic simulator-only wind and bounded position-measurement errors."""

import numpy as np


def wind_at(time, drone_id, magnitude, model="rotating", heading_deg=0):
    angle = np.deg2rad(heading_deg)
    if model == "rotating":
        angle += 0.17 * time + 1.7 * drone_id
        amplitude = magnitude
    elif model == "constant":
        amplitude = magnitude
    elif model == "gust":
        amplitude = magnitude * (0.5 + 0.5 * np.sin(0.73 * time + 1.7 * drone_id))
        angle += 0.45 * np.sin(0.19 * time)
    else:
        raise ValueError("wind_model must be rotating, constant or gust")
    return amplitude * np.array([np.cos(angle), np.sin(angle), 0.])


def position_bias_vectors(drone_ids, magnitude, dimensions):
    if not np.isfinite(magnitude) or magnitude < 0:
        raise ValueError("position bias magnitude must be finite and nonnegative")
    result = []
    for drone_id in drone_ids:
        rng = np.random.default_rng(np.random.SeedSequence([int(drone_id), 8821]))
        vector = rng.normal(size=dimensions)
        result.append(magnitude * vector / np.linalg.norm(vector))
    return np.array(result)


class PositionSensor:
    def __init__(self, drone_id, dimensions, bias, noise_bound, seed):
        self.dimensions = dimensions
        self.bias = np.asarray(bias, dtype=float)
        self.noise_bound = noise_bound
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, drone_id, 9277]))

    def error(self):
        if self.noise_bound == 0:
            return self.bias.copy()
        direction = self.rng.normal(size=self.dimensions)
        radius = self.noise_bound * self.rng.random()**(1 / self.dimensions)
        return self.bias + radius * direction / np.linalg.norm(direction)
