"""Single-aircraft, level-flight PX4/Gazebo experiment; not an onboard bridge.

The existing generator supplies a reference curve. A derived waypoint mission
does not retain its timing, tracking-tube or separation claims. No controls
from the point-mass model are sent to PX4. Core planning remains unchanged.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class LocalOrigin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.latitude_deg, self.longitude_deg, self.altitude_m))
                or not -85 <= self.latitude_deg <= 85 or not -180 <= self.longitude_deg <= 180):
            raise ValueError('invalid local geodetic origin')

    def basis(self):
        lat, lon = math.radians(self.latitude_deg), math.radians(self.longitude_deg)
        return np.array([[-math.sin(lon), -math.sin(lat)*math.cos(lon), math.cos(lat)*math.cos(lon)],
                         [math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)*math.sin(lon)],
                         [0., math.cos(lat), math.sin(lat)]])

    def to_enu(self, latitude, longitude, altitude):
        return (geodetic_to_ecef(latitude, longitude, altitude)
                - geodetic_to_ecef(self.latitude_deg, self.longitude_deg, self.altitude_m)) @ self.basis()

    def from_enu(self, point):
        p = np.asarray(point, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)) or np.linalg.norm(p) > 5000:
            raise ValueError('the laboratory ENU reference must fit within 5 km')
        xyz = geodetic_to_ecef(self.latitude_deg, self.longitude_deg, self.altitude_m) + self.basis() @ p
        return ecef_to_geodetic(xyz)


def geodetic_to_ecef(latitude, longitude, altitude):
    lat, lon = math.radians(latitude), math.radians(longitude)
    a, e2 = 6378137., 6.6943799901413165e-3
    n = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    return np.array([(n+altitude)*math.cos(lat)*math.cos(lon),
                     (n+altitude)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+altitude)*math.sin(lat)])


def ecef_to_geodetic(point):
    x, y, z = map(float, point)
    a, e2 = 6378137., 6.6943799901413165e-3
    p = math.hypot(x, y)
    lat = math.atan2(z, p*(1-e2))
    for _ in range(12):
        n = a / math.sqrt(1-e2*math.sin(lat)**2)
        lat = math.atan2(z+e2*n*math.sin(lat), p)
    n = a / math.sqrt(1-e2*math.sin(lat)**2)
    return math.degrees(lat), math.degrees(math.atan2(y, x)), p/math.cos(lat)-n


def generate_reference(seed=7, obstacles=(), scenario=None):
    """Use the real catalog/physical validator, with one benign level transit."""
    from .trajectory2d import Config, Snapshot, plan_mission
    from .trajectory2d.aircraft_profiles import reference_aircraft_profile
    from .trajectory2d.planning import mission_document
    profile = reference_aircraft_profile()
    config = profile.apply(Config(dimensions=3, generator_mode='geometric', numerical_backend='python',
        target=(1200., 500.), horizon=100.,
        activation_time=0., min_extra_loiter=0., max_extra_loiter=0., candidates=4,
        attempts=30, detour=60., planning_strategy='adaptive', seed=seed,
        obstacles=tuple(obstacles)))
    snapshot = Snapshot(0, (300., 0., 0., 0.), 15., 80., 80.)
    if scenario is not None:
        from .px4_scenario import validate_scenario
        snapshots, config = scenario
        validate_scenario(snapshots, config)
        snapshot = snapshots[0]
    result = plan_mission((snapshot,), config)
    if not result.validated:
        raise ValueError(f'reference generation failed: {result.status}: {result.reason}')
    document = mission_document(result, config, (snapshot,), aircraft_profile=profile)
    plan = result.plans[0]
    times = np.arange(len(plan.positions)) * plan.dt
    return document, times, plan.positions


def waypoint_indices(points, spacing_m=40., allow_vertical=False):
    """Derived spatial samples only; the immutable original plan is retained."""
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
            or not np.all(np.isfinite(points)) or not math.isfinite(spacing_m) or spacing_m <= 0):
        raise ValueError('finite XYZ reference and positive spacing required')
    # A nominally level coupled rollout has small physical vertical transients
    # during bank changes. Keep those Z samples, never flatten the accepted plan.
    if not allow_vertical and np.ptp(points[:, 2]) > .1:
        raise ValueError('this first PX4 experiment supports nominally level references (<=0.1 m vertical excursion) only')
    indices, distance = [0], 0.
    for k, segment in enumerate(np.linalg.norm(np.diff(points, axis=0), axis=1), start=1):
        distance += segment
        if distance >= spacing_m:
            indices.append(k)
            distance = 0.
    if indices[-1] != len(points)-1:
        indices.append(len(points)-1)
    return indices


def mission_items(points, origin, airspeed=15., acceptance_radius=15., loiter=None, allow_vertical=False, recovery=None,
                  end_behavior='land'):
    """MAVLink mission records, not an executable network operation.

    Climb/join and the standard PX4 landing pattern are separate from the
    generated level reference. No takeoff/landing feasibility check is disabled.
    """
    points = np.asarray(points, dtype=float)
    if end_behavior not in ('land', 'remove-at-target'):
        raise ValueError('Unknown simulation end behavior')
    if end_behavior == 'remove-at-target' and recovery is not None:
        raise ValueError('Trajectory-only missions cannot contain a landing pattern')
    if not math.isfinite(airspeed) or not 10 <= airspeed <= 20:
        raise ValueError('Cessna bench airspeed must remain within 10..20 m/s')
    if not math.isfinite(acceptance_radius) or acceptance_radius <= 0:
        raise ValueError('positive acceptance radius required')
    indices = waypoint_indices(points, allow_vertical=allow_vertical)
    if loiter is not None:
        duration, dt = loiter['duration_s'], loiter['sample_dt_s']
        if not math.isfinite(duration) or duration < 5 or not math.isfinite(dt) or dt <= 0:
            raise ValueError('positive sampled initial loiter duration required')
        # Start on a forward post-loiter sample; the native circle supplies the
        # whole prefix, not a sparse set of chord waypoints cutting the circle.
        departure = min(int(math.ceil(duration/dt)), len(points)-1)
        tail = waypoint_indices(points[departure:], allow_vertical=allow_vertical)
        indices = [departure+k for k in tail[1:]]
    items = []
    def add(command, point=None, params=(0., 0., 0., None)):
        lat, lon, alt = origin.from_enu(point) if point is not None else (0., 0., origin.altitude_m)
        items.append({'seq': len(items), 'frame': 6 if point is not None else 2,
            'command': command, 'current': int(not items), 'autocontinue': 1,
            'param1': params[0], 'param2': params[1], 'param3': params[2], 'param4': params[3],
            'x': int(round(lat*1e7)), 'y': int(round(lon*1e7)),
            'z': float(alt-origin.altitude_m), 'mission_type': 0})
    add(22, points[0], (10., 0., 0., None))  # NAV_TAKEOFF; no forced arm or direct actuators.
    add(178, params=(0., float(airspeed), -1., 0.))  # DO_CHANGE_SPEED: airspeed.
    if loiter is not None:
        radius = loiter['radius_m'] * (-1 if loiter['counter_clockwise'] else 1)
        # Do not wait an extra revolution to align an exit heading. PX4 may
        # still round the following spatial leg; reference timing is not kept.
        add(19, np.asarray(loiter['center_enu_m']), (float(loiter['duration_s']), 0., radius, 1.))
    for k in indices:
        add(16, points[k], (0., float(acceptance_radius), 0., None))
    if end_behavior == 'remove-at-target':
        return items, indices
    # Required by this PX4 airframe: a real, reachable landing sequence, not a
    # dummy landing appended after an unreachable unlimited loiter. Flat world,
    # no wind/obstacles; 60 m height over 600 m is shallower than its 8 degree cap.
    add(189)  # DO_LAND_START.
    recovery = recovery or {'center_enu_m': [-600., 0., 60.], 'landing_enu_m': [0., 0., 0.], 'landing_yaw_ned_deg': 90.}
    add(31, np.asarray(recovery['center_enu_m']), (1., 100., 0., 1.))  # LOITER_TO_ALT, tangent exit.
    add(21, np.asarray(recovery['landing_enu_m']), (0., 0., 0., recovery['landing_yaw_ned_deg']))
    return items, indices


def polyline_distances(actual, reference):
    """Spatial XYZ distance, not synchronized tracking error or swept clearance."""
    p, r = np.asarray(actual, dtype=float), np.asarray(reference, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or not len(p) or r.ndim != 2 or r.shape[1] != 3 or len(r) < 2
            or not np.all(np.isfinite(p)) or not np.all(np.isfinite(r))):
        raise ValueError('finite nonempty XYZ samples and reference polyline required')
    a, delta = r[:-1], np.diff(r, axis=0)
    denom = np.maximum(np.sum(delta**2, axis=1), 1e-20)
    result = []
    for chunk in np.array_split(p, max(1, math.ceil(len(p)/256))):
        residual = chunk[:, None, :] - a[None]
        fraction = np.clip(np.sum(residual*delta[None], axis=2)/denom, 0., 1.)
        result.extend(np.linalg.norm(residual-fraction[..., None]*delta[None], axis=2).min(axis=1))
    return np.asarray(result)
