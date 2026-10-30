"""Reduced-order, coordinated point-mass flight (SI, world Z up).

State: [x, y, z, true_airspeed, air_heading, flight_path_angle, bank].
Control: [propulsive_acceleration, extra_drag_acceleration, load_factor, roll_rate].
The polar is deliberately illustrative, not a calibrated aircraft model. No
angle of attack, sideslip, rotational inertia or ground contact is simulated.
"""

from dataclasses import dataclass
from math import cos, sin

import numpy as np

from .dynamics import G, wrap


@dataclass(frozen=True)
class FlightLimits:
    stall_speed: float = 6.0  # 1g, reference air density
    stall_margin: float = 1.15
    max_load_factor: float = 2.5
    min_load_factor: float = 0.2
    max_flight_path_deg: float = 45.0
    max_flight_path_rate_deg_s: float = 8.0
    max_thrust_acceleration: float = 5.0  # T_max / mass
    max_brake_acceleration: float = 5.0  # optional idealised airbrake, NOT reverse thrust
    parasitic_drag: float = 0.003  # D/m = c0 V² + ci n² / V²
    induced_drag: float = 20.0

    def __post_init__(self):
        values = list(self.__dict__.values())
        if not np.all(np.isfinite(values)) or min(values) < 0:
            raise ValueError("Flight parameters must be finite and nonnegative")
        if (self.stall_speed <= 0 or self.stall_margin < 1 or self.min_load_factor <= 0
                or self.max_load_factor <= self.min_load_factor
                or not 0 < self.max_flight_path_deg < 80
                or self.max_flight_path_rate_deg_s <= 0 or self.max_thrust_acceleration <= 0):
            raise ValueError("Invalid point-mass flight envelope")

    def load_ceiling(self, speed):
        return np.minimum(self.max_load_factor, (np.asarray(speed) / (self.stall_speed * self.stall_margin))**2)

    def drag(self, speed, load):
        return self.parasitic_drag * speed**2 + self.induced_drag * load**2 / speed**2


def air_velocity(state):
    s = np.asarray(state)
    horizontal = s[..., 3] * np.cos(s[..., 5])
    return np.stack((horizontal * np.cos(s[..., 4]), horizontal * np.sin(s[..., 4]),
                     s[..., 3] * np.sin(s[..., 5])), axis=-1)


def derivative(state, control, flight, wind=(0.0, 0.0, 0.0)):
    s, u = np.asarray(state), np.asarray(control)
    v, gamma, bank = s[..., 3], s[..., 5], s[..., 6]
    thrust, brake, load, roll = (u[..., k] for k in range(4))
    wind = np.asarray(wind)
    shape = (s.shape[:-1] if u.ndim == wind.ndim == 1 else
             np.broadcast_shapes(s.shape[:-1], u.shape[:-1], wind.shape[:-1]))
    result = np.empty((*shape, 7), dtype=float)
    # Avoid moveaxis/stack/broadcast allocations in every dense RK4 evaluation.
    # Keep the historical arithmetic order and all integration/checking points.
    cosine = np.cos(gamma)
    sine = np.sin(gamma)
    horizontal = v * cosine
    result[..., 0] = horizontal * np.cos(s[..., 4]) + wind[..., 0]
    result[..., 1] = horizontal * np.sin(s[..., 4]) + wind[..., 1]
    result[..., 2] = v * sine + wind[..., 2]
    result[..., 3] = thrust - brake - flight.drag(v, load) - G * sine
    result[..., 4] = G * load * np.sin(bank) / (v * cosine)
    result[..., 5] = G * (load * np.cos(bank) - cosine) / v
    result[..., 6] = roll
    return result


def _scalar_derivative(s, u, flight, wind):
    """Fast scalar path for the many short planning/controller integrations."""
    v, psi, gamma, bank = s[3:]
    thrust, brake, load, roll = u
    cg, sg = cos(gamma), sin(gamma)
    return np.array([v * cg * cos(psi) + wind[0], v * cg * sin(psi) + wind[1],
                     v * sg + wind[2], thrust - brake - flight.drag(v, load) - G * sg,
                     G * load * sin(bank) / (v * cg),
                     G * (load * cos(bank) - cg) / v, roll])


def propagate(state, control, duration, flight, wind=(0.0, 0.0, 0.0), substeps=2):
    """RK4 with held controls. Scalar or broadcasted states/durations.

    Two subdivisions per nominal 0.2s primitive; verification additionally
    checks a finer grid. This is numerical integration, not an exact flow.
    """
    state = np.asarray(state, dtype=float)
    h = np.asarray(duration, dtype=float) / substeps
    shape = np.broadcast_shapes(state.shape[:-1], np.shape(h), np.shape(control)[:-1])
    value = np.broadcast_to(state, (*shape, 7)).copy()
    scalar = not shape
    rhs = _scalar_derivative if scalar else derivative
    delta = h if scalar else np.broadcast_to(h, shape)[..., None]
    for _ in range(substeps):
        k1 = rhs(value, control, flight, wind)
        k2 = rhs(value + delta * k1 / 2, control, flight, wind)
        k3 = rhs(value + delta * k2 / 2, control, flight, wind)
        k4 = rhs(value + delta * k3, control, flight, wind)
        value += delta * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    value[..., 4] = wrap(value[..., 4])
    return value


def trim_control(state, flight):
    """Constant speed, slope and bank trim; caller must check the envelope."""
    speed, gamma, bank = state[3], state[5], state[6]
    load = cos(gamma) / cos(bank)
    axial = flight.drag(speed, load) + G * sin(gamma)
    return np.array([max(0.0, axial), max(0.0, -axial), load, 0.0])


def from_snapshot(snapshot):
    x, y, heading, rate = snapshot.state
    bank = np.arctan(snapshot.speed * rate / G)
    return np.array([x, y, snapshot.altitude, snapshot.speed, heading, 0.0, bank])


def project(state, control, flight):
    """Compatibility view [x,y,heading,yaw_rate,z,vz] for the existing UI."""
    s = np.asarray(state)
    rate = derivative(s, control, flight)[..., 4]
    return np.stack((s[..., 0], s[..., 1], s[..., 4], rate,
                     s[..., 2], s[..., 3] * np.sin(s[..., 5])), axis=-1)


def envelope_ok(states, controls, config, tolerance=1e-7):
    """Sampled flight-envelope check; not a certified interval ODE solver."""
    s, u = np.asarray(states), np.asarray(controls)
    f, lim = config.flight, config.limits
    if not np.all(np.isfinite(s)) or not np.all(np.isfinite(u)) or np.any(s[..., 3] <= 0):
        return False
    d = derivative(s, u, f)
    az = d[..., 3] * np.sin(s[..., 5]) + s[..., 3] * np.cos(s[..., 5]) * d[..., 5]
    vertical_velocity = s[..., 3] * np.sin(s[..., 5])
    eps = tolerance
    return bool(
        s[..., 3].min() >= lim.min_speed - eps and s[..., 3].max() <= lim.max_speed + eps
        and np.abs(s[..., 6]).max() <= np.deg2rad(lim.max_bank_deg) + eps
        and np.abs(s[..., 5]).max() <= np.deg2rad(f.max_flight_path_deg) + eps
        and s[..., 2].min() >= config.min_altitude - eps and s[..., 2].max() <= config.max_altitude + eps
        and vertical_velocity.max() <= config.max_climb_rate + eps
        and vertical_velocity.min() >= -config.descent_rate_limit - eps
        and np.abs(az).max() <= config.max_vertical_acceleration + eps
        and np.abs(d[..., 5]).max() <= np.deg2rad(f.max_flight_path_rate_deg_s) + eps
        and np.all(u[..., 0] >= -eps) and np.all(u[..., 0] <= f.max_thrust_acceleration + eps)
        and np.all(u[..., 1] >= -eps) and np.all(u[..., 1] <= f.max_brake_acceleration + eps)
        and np.all(u[..., 2] >= f.min_load_factor - eps)
        and np.all(u[..., 2] <= f.load_ceiling(s[..., 3]) + eps)
        and np.abs(u[..., 3]).max() <= np.deg2rad(lim.max_roll_rate_deg_s) + eps)
