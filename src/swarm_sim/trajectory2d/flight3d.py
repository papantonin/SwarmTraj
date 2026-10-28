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
    thrust, brake, load, roll = np.moveaxis(u, -1, 0)
    velocity = air_velocity(s) + np.asarray(wind)
    return np.concatenate((velocity, np.stack((
        thrust - brake - flight.drag(v, load) - G * np.sin(gamma),
        G * load * np.sin(bank) / (v * np.cos(gamma)),
        G * (load * np.cos(bank) - np.cos(gamma)) / v,
        np.broadcast_to(roll, np.shape(v)),
    ), axis=-1)), axis=-1)


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
    eps = tolerance
    return bool(
        s[..., 3].min() >= lim.min_speed - eps and s[..., 3].max() <= lim.max_speed + eps
        and np.abs(s[..., 6]).max() <= np.deg2rad(lim.max_bank_deg) + eps
        and np.abs(s[..., 5]).max() <= np.deg2rad(f.max_flight_path_deg) + eps
        and s[..., 2].min() >= config.min_altitude - eps and s[..., 2].max() <= config.max_altitude + eps
        and air_velocity(s)[..., 2].max() <= config.max_climb_rate + eps
        and air_velocity(s)[..., 2].min() >= -config.descent_rate_limit - eps
        and np.abs(az).max() <= config.max_vertical_acceleration + eps
        and np.abs(d[..., 5]).max() <= np.deg2rad(f.max_flight_path_rate_deg_s) + eps
        and np.all(u[..., 0] >= -eps) and np.all(u[..., 0] <= f.max_thrust_acceleration + eps)
        and np.all(u[..., 1] >= -eps) and np.all(u[..., 1] <= f.max_brake_acceleration + eps)
        and np.all(u[..., 2] >= f.min_load_factor - eps)
        and np.all(u[..., 2] <= f.load_ceiling(s[..., 3]) + eps)
        and np.abs(u[..., 3]).max() <= np.deg2rad(lim.max_roll_rate_deg_s) + eps)


def guidance(state, reference, feedforward, config, dt, correct_speed=False, wind_estimate=None,
             preserve_air_path=False, speed_feedforward=0.):
    """Local own-state-only controller, with load/stall/actuator saturation."""
    f, lim = config.flight, config.limits
    speed, gamma, bank = state[3], state[5], state[6]
    velocity = air_velocity(reference) + 0.8 * (reference[:3] - state[:3])
    air_path = np.arctan2(velocity[2], np.linalg.norm(velocity[:2])) if preserve_air_path else None
    if wind_estimate is not None:
        velocity = velocity - wind_estimate
    heading = np.arctan2(velocity[1], velocity[0])
    desired_yaw_rate = feedforward[0] + 1.5 * wrap(heading - state[4])
    desired_gamma = air_path if preserve_air_path else np.arctan2(velocity[2], np.linalg.norm(velocity[:2]))
    gamma_rate = feedforward[1] + 1.5 * (desired_gamma - gamma)
    slope_limit = np.deg2rad(f.max_flight_path_deg)
    up = min(slope_limit, np.arcsin(min(1.0, 0.98 * config.max_climb_rate / speed)))
    down = min(slope_limit, np.arcsin(min(1.0, 0.98 * config.descent_rate_limit / speed)))
    # Reserve margin for the held-force change within a primitive.
    max_pitch = min(np.deg2rad(f.max_flight_path_rate_deg_s),
                    0.95 * config.max_vertical_acceleration / max(speed, 1e-6))
    gamma_rate = np.clip(gamma_rate, max(-max_pitch, (-down - gamma) / dt),
                         min(max_pitch, (up - gamma) / dt))
    lift_vertical = G * cos(gamma) + speed * gamma_rate
    desired_bank = np.arctan2(speed * cos(gamma) * desired_yaw_rate, lift_vertical)
    # A bank that needs more lift than the stall/structure envelope is disallowed.
    ceiling = float(f.load_ceiling(speed))
    max_bank = min(np.deg2rad(lim.max_bank_deg),
                   np.arccos(np.clip(lift_vertical / (G * ceiling), 0, 1)))
    desired_bank = np.clip(desired_bank, -max_bank, max_bank)
    roll_limit = np.deg2rad(lim.max_roll_rate_deg_s)
    roll = np.clip((desired_bank - bank) / dt, -roll_limit, roll_limit)
    # Compute lift for the midpoint bank, avoiding fictitious independent yaw/z acceleration.
    mid_bank = bank + roll * dt / 2
    # A held lift force must be safe at BOTH ends of a rolling primitive.
    # Reduce roll if the vertical-acceleration interval would be empty.
    budget = 0.85 * config.max_vertical_acceleration / (G * cos(gamma))
    for _ in range(8):
        end_bank = bank + roll * dt
        min_cos = min(cos(bank), cos(end_bank))
        max_cos = 1.0 if bank * end_bank <= 0 else max(cos(bank), cos(end_bank))
        lower = max(f.min_load_factor, (cos(gamma) - budget) / min_cos)
        upper = min(ceiling, (cos(gamma) + budget) / max_cos)
        if lower <= upper:
            break
        roll *= 0.5
    mid_bank = bank + roll * dt / 2
    load = np.clip(lift_vertical / (G * cos(mid_bank)), lower, max(lower, upper))
    # Along-track errors also need speed authority; heading correction alone
    # cannot recover a phase error on a straight segment at constant speed.
    # Candidate construction tracks the configured airspeed. Execution can
    # additionally use speed authority to recover a clock/along-track error.
    desired_speed = (np.clip(np.linalg.norm(velocity), lim.min_speed, lim.max_speed)
                     if correct_speed else reference[3])
    acceleration = 1.5 * (desired_speed - speed)
    if speed_feedforward != 0:
        acceleration += speed_feedforward
    axial = acceleration + f.drag(speed, load) + G * sin(gamma)
    return np.array([np.clip(axial, 0, f.max_thrust_acceleration),
                     np.clip(-axial, 0, f.max_brake_acceleration), load, roll])
