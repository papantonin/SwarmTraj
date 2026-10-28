"""Continuous planar aircraft model: constant airspeed and bounded yaw acceleration.

Each primitive has affine yaw rate. Positions are integrated using Fresnel
integrals, not linearly interpolated between samples. This is a no-wind,
coordinated-turn surrogate, not a six-degree-of-freedom aerodynamic model.
"""

from dataclasses import dataclass, replace

import numpy as np
from scipy.special import fresnel

G = 9.81


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def propagate(state, speed, acceleration, duration):
    """Integrate [x, y, heading, yaw_rate]; inputs support NumPy broadcasting."""
    state = np.asarray(state, dtype=float)
    psi, omega = state[..., 2], state[..., 3]
    alpha, dt = np.broadcast_arrays(np.asarray(acceleration), np.asarray(duration))
    curved = np.abs(alpha) > 1e-12
    safe_alpha = np.where(curved, alpha, 1.0)
    factor = np.sqrt(np.abs(safe_alpha) / np.pi)
    z0 = omega / safe_alpha * factor
    z1 = (dt + omega / safe_alpha) * factor
    s0, c0 = fresnel(z0)
    s1, c1 = fresnel(z1)
    integral = (
        np.sqrt(np.pi / np.abs(safe_alpha))
        * ((c1 - c0) + 1j * np.sign(safe_alpha) * (s1 - s0))
        * np.exp(1j * (psi - omega**2 / (2 * safe_alpha)))
    )
    arc = dt * np.sinc(omega * dt / (2 * np.pi)) * np.exp(1j * (psi + omega * dt / 2))
    displacement = speed * np.where(curved, integral, arc)
    return np.stack([
        state[..., 0] + displacement.real,
        state[..., 1] + displacement.imag,
        wrap(psi + omega * dt + 0.5 * alpha * dt**2),
        omega + alpha * dt,
    ], axis=-1)


@dataclass(frozen=True)
class Limits:
    min_speed: float = 8.0
    max_speed: float = 16.0
    max_bank_deg: float = 50.0
    max_roll_rate_deg_s: float = 60.0

    def __post_init__(self):
        values = [self.min_speed, self.max_speed, self.max_bank_deg, self.max_roll_rate_deg_s]
        if not np.all(np.isfinite(values)) or not 0 < self.min_speed <= self.max_speed:
            raise ValueError("invalid speed limits")
        if not 0 < self.max_bank_deg < 90 or self.max_roll_rate_deg_s <= 0:
            raise ValueError("invalid bank/roll limits")

    def max_rate(self, speed):
        return G * np.tan(np.deg2rad(self.max_bank_deg)) / speed

    def max_acceleration(self, speed):
        # bank = atan(v * omega / g), hence |bank_dot| <= v/g * |alpha|.
        return G / speed * np.deg2rad(self.max_roll_rate_deg_s)

    def command(self, state, speed, desired_rate, dt):
        desired_rate = np.clip(desired_rate, -self.max_rate(speed), self.max_rate(speed))
        alpha = float(np.clip((desired_rate - state[3]) / dt,
                              -self.max_acceleration(speed), self.max_acceleration(speed)))
        # Avoid cancellation in Fresnel differences for near-zero accelerations.
        return 0.0 if abs(alpha) < 1e-5 else alpha


@dataclass(frozen=True)
class Snapshot:
    drone_id: int
    state: tuple[float, float, float, float]
    speed: float
    altitude: float = 0.0
    cruise_altitude: float | None = None

    @property
    def position3d(self):
        return np.array([*self.state[:2], self.altitude])

    def with_speed(self, speed, limits):
        """Configure airspeed before flight, preserving the signed loiter radius."""
        if not np.isfinite(speed) or not limits.min_speed <= speed <= limits.max_speed:
            raise ValueError(f"Required airspeed: {limits.min_speed:g} to {limits.max_speed:g} m/s.")
        rate = self.state[3] * speed / self.speed
        if abs(rate) > limits.max_rate(speed) + 1e-9:
            raise ValueError("Airspeed incompatible with the loiter radius: increase the radius or reduce the airspeed.")
        return replace(self, speed=float(speed), state=(*self.state[:3], float(rate)))


def ring_snapshot(count=4, radius=25.0, speed=12.0):
    """Common, fresh snapshot at t=0; positive turn direction, centre (0, 0)."""
    if count < 1 or radius <= 0 or speed <= 0:
        raise ValueError("count, radius and speed must be positive")
    return tuple(Snapshot(k, (radius * np.cos(a), radius * np.sin(a),
                              float(wrap(a + np.pi / 2)), speed / radius), speed)
                 for k, a in enumerate(np.arange(count) * 2 * np.pi / count))


def separate_loiter_snapshot(count=4, radius=25.0, speed=12.0, spacing=80.0):
    """One counterclockwise holding circle per aircraft on a centred square grid.

    ``spacing`` is the distance between neighbouring centres. Disjoint circles
    alone do not certify the mission: the planner must still check its margins
    and every departure/approach trajectory.
    """
    if not isinstance(count, (int, np.integer)) or count < 1:
        raise ValueError("count must be a positive integer")
    if not np.all(np.isfinite([radius, speed, spacing])) or radius <= 0 or speed <= 0:
        raise ValueError("radius, speed and spacing must be finite and positive")
    if spacing <= 2 * radius:
        raise ValueError("loiter spacing must exceed twice the radius")
    columns = int(np.ceil(np.sqrt(count)))
    centres = np.array([(k % columns, k // columns) for k in range(count)], dtype=float)
    centres = (centres - centres.mean(axis=0)) * spacing
    return tuple(Snapshot(s.drone_id,
                          (s.state[0] + centre[0], s.state[1] + centre[1], *s.state[2:]),
                          s.speed)
                 for s, centre in zip(ring_snapshot(count, radius, speed), centres))


def holding_circle(state, speed):
    """Return (centre, radius) for the initial constant-rate turn, or None."""
    x, y, heading, rate = state
    if abs(rate) < 1e-12:
        return None
    signed_radius = speed / rate
    return (np.array([x - signed_radius * np.sin(heading),
                      y + signed_radius * np.cos(heading)]), abs(signed_radius))
