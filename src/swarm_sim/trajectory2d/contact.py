"""Shared numerical first-entry events; no certified ODE/interval error bound."""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Contact:
    time: float
    state: np.ndarray
    bracket_width_s: float


class ContactResolutionError(RuntimeError):
    """Numerical event search exceeded its bounded work allowance."""


def first_entry(times, states, sample, center, radius, speed_bound, start_time=0., tolerance=1e-6):
    """Localize the earliest sphere/disk entry by reintegrating, not interpolating.

    Speed bounds prune intervals; recursive midpoint tests also detect passages
    with both endpoints outside. Excursions shorter than tolerance and grazing
    contacts at floating-point resolution remain numerical limitations.
    """
    times, states, center = np.asarray(times), np.asarray(states), np.asarray(center)
    if (len(times) < 2 or states.shape[0] != len(times) or np.any(np.diff(times) <= 0)
            or not np.all(np.isfinite(times)) or not np.all(np.isfinite(states))
            or not np.all(np.isfinite(center)) or not np.all(np.isfinite([radius, speed_bound, start_time, tolerance]))
            or radius <= 0 or speed_bound <= 0 or tolerance <= 0):
        raise ValueError("invalid event grid, states or bounds")
    dim = len(center)
    distances = np.linalg.norm(states[:, :dim] - center, axis=1)
    evaluations = 0

    def scan(left, a, da, right, b, db):
        nonlocal evaluations
        width = right - left
        if da <= radius:
            return Contact(float(left), a.copy(), 0.)
        if min(da, db) - speed_bound * width / 2 > radius:
            return None
        if width <= tolerance:
            return Contact(float(right), b.copy(), float(width)) if db <= radius else None
        middle = (left + right) / 2
        evaluations += 1
        if evaluations > 4096:
            raise ContactResolutionError("first-entry event unresolved within 4096 evaluations")
        state = sample(middle)
        if not np.all(np.isfinite(state)):
            raise ContactResolutionError("nonfinite dense state during event localization")
        distance = float(np.linalg.norm(state[:dim] - center))
        return (scan(left, a, da, middle, state, distance)
                or scan(middle, state, distance, right, b, db))

    possible = np.flatnonzero((np.minimum(distances[:-1], distances[1:])
                              - speed_bound * np.diff(times) / 2 <= radius)
                             & (times[1:] >= start_time))
    for i in possible:
        left, a, da = times[i], states[i], distances[i]
        if left < start_time:
            left, a = start_time, sample(start_time)
            da = float(np.linalg.norm(a[:dim] - center))
        event = scan(left, a, da, times[i + 1], states[i + 1], distances[i + 1])
        if event is not None:
            return event
    return None


def plan_contact(plan, config):
    center = np.r_[config.target, config.target_altitude] if config.target_altitude is not None else config.target
    return first_entry(np.arange(len(plan.flight_states)) * plan.dt, plan.flight_states,
                       plan.sample_flight, center, config.target_radius, plan.speed_bound,
                       start_time=config.activation_time)


def recorded_sampler(records, flight, wind):
    """Observer-only dense sampling of actual held controls, including switches."""
    from .flight3d import propagate
    times = np.array([r[0] for r in records])

    def sample(time):
        index = int(np.clip(np.searchsorted(times, time, side="right") - 1, 0, len(records) - 1))
        t, state, control = records[index]
        return propagate(state, control, max(0., time - t), flight, wind=wind(t), substeps=4)

    return sample
