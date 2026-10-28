"""Own-state wind estimation and sampled held-control protection.

No fleet state or true simulator wind is available to these routines. The
protection is a bounded numerical search, not a certified invariant set or
an emergency collision-avoidance policy.
"""

import numpy as np

from .flight3d import envelope_ok, propagate, trim_control


class WindEstimator:
    """Estimate drift from displacement minus locally predicted air displacement.

    Position bias cancels if constant; bounded measurement noise does not.
    Horizontal low-pass estimation does not model aerodynamic gust forces.
    """

    def __init__(self):
        self.previous = None
        self.value = np.zeros(3)

    def update(self, measured, dt, flight, previous_control):
        if self.previous is not None and previous_control is not None:
            predicted = propagate(self.previous, previous_control, dt, flight, substeps=4)
            estimate = (measured[:3] - predicted[:3]) / dt
            estimate[2] = 0  # The experiment's wind model is horizontal.
            alpha = 1 - np.exp(-dt / 1.5)
            self.value += alpha * (estimate - self.value)
        self.previous = measured.copy()
        return self.value.copy()


def clip_control(state, control, config, dt):
    """Actuator and endpoint-bank clipping; no physical state is clipped."""
    result = np.array(control, dtype=float, copy=True)
    flight = config.flight
    result[0] = np.clip(result[0], 0, flight.max_thrust_acceleration)
    result[1] = np.clip(result[1], 0, flight.max_brake_acceleration)
    result[2] = np.clip(result[2], flight.min_load_factor,
                        max(flight.min_load_factor, flight.load_ceiling(state[3])))
    roll = np.deg2rad(config.limits.max_roll_rate_deg_s)
    bank = np.deg2rad(config.limits.max_bank_deg)
    low, high = max(-roll, (-bank - state[6]) / dt), min(roll, (bank - state[6]) / dt)
    # If already outside the bank envelope, bounded recovery cannot teleport it.
    result[3] = np.clip(result[3], low, high) if low <= high else np.clip(-state[6] / dt, -roll, roll)
    return result


def held_control_ok(state, control, config, dt):
    samples = propagate(state, control, np.linspace(0, dt, 21), config.flight, substeps=4)
    return envelope_ok(samples, control, config, tolerance=1e-8)


def protect_control(state, requested, config, dt):
    """Retain feasible requests; otherwise blend toward a verified local trim.

    Every retained candidate is independently sampled across the held interval.
    Bisection does not assume or claim convexity/global optimality: only tested
    feasible candidates are retained. If no fallback is found, report failure
    and return bounded actuators for continued diagnostic simulation.
    """
    requested = clip_control(state, requested, config, dt)
    if held_control_ok(state, requested, config, dt):
        return requested, False, True
    trim = clip_control(state, trim_control(state, config.flight), config, dt)
    if not held_control_ok(state, trim, config, dt):
        return requested, True, False
    best, low, high = trim, 0., 1.
    for _ in range(10):
        fraction = (low + high) / 2
        control = trim + fraction * (requested - trim)
        if held_control_ok(state, control, config, dt):
            best, low = control, fraction
        else:
            high = fraction
    return best, True, True
