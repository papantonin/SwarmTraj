"""Shared editable scenario for the single-aircraft PX4/Gazebo prototype.

Mission inputs only: no arbitrary transport URLs, actuator commands or native
airframe overrides. Planning uses the historical illustrative physics profile,
not a calibrated Cessna model. The backend remains a spatial experiment.
"""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import numpy as np

from .trajectory2d import Config, Snapshot
from .trajectory2d.aircraft_profiles import reference_aircraft_profile
from .trajectory2d.obstacles import Obstacle
from .trajectory2d.dynamics import holding_circle


SCHEMA = 'swarm-px4-editor-v3'
PREVIOUS_SCHEMA = 'swarm-px4-editor-v2'
LEGACY_SCHEMA = 'swarm-px4-editor-v1'
MISSION_FIELDS = ('seed', 'target', 'target_altitude', 'target_radius', 'horizon',
                  'arrival_deadline', 'post_arrival_time',
                  'activation_time', 'min_extra_loiter', 'max_extra_loiter',
                  'approach_spread_deg', 'cruise_altitude_range', 'descent_angle_deg',
                  'obstacle_clearance', 'obstacles', 'generator_mode', 'numerical_backend')


def default_scenario(seed=7, obstacles=()):
    config = reference_aircraft_profile().apply(Config(dimensions=3, seed=seed,
        target=(1200., 500.), target_altitude=80., horizon=160., activation_time=40.,
        min_extra_loiter=0., max_extra_loiter=0., candidates=6, attempts=60,
        detour=60., planning_strategy='adaptive', obstacle_routing='spatial',
        min_altitude=20., max_altitude=100., obstacles=tuple(obstacles)))
    # Clockwise circle to the south of the starting point, clear of the
    # historical obstacle example. The editor point is the AIRCRAFT, not center.
    snapshot = Snapshot(0, (300., 0., 0., -.15), 15., 80., 80.)
    return (snapshot,), config


def validate_scenario(snapshots, config):
    if len(snapshots) != 1 or snapshots[0].drone_id != 0:
        raise ValueError('Gazebo editor currently supports exactly one drone (ID 0).')
    snapshot = snapshots[0]
    if config.dimensions != 3 or not config.coupled_flight or config.aircraft_profiles:
        raise ValueError('Gazebo editor requires the common 3D point-mass proxy; no alternate airframe bindings.')
    if config.generator_mode not in ('geometric', 'physical_mission'):
        raise ValueError('Gazebo editor generator_mode must be geometric or physical_mission.')
    expected = reference_aircraft_profile().apply(Config(dimensions=3))
    if config.limits != expected.limits or config.flight != expected.flight:
        raise ValueError('PX4 uses the pinned Cessna; changing the planning airframe does not change Gazebo.')
    values = (*snapshot.state, snapshot.speed, snapshot.altitude,
              *( () if snapshot.cruise_altitude is None else (snapshot.cruise_altitude,)))
    if not all(math.isfinite(v) for v in values) or not 10 <= snapshot.speed <= 16:
        raise ValueError('Gazebo prototype: airspeed must be between 10 and 16 m/s and all fields must be finite.')
    if not 20 <= snapshot.altitude <= 100:
        raise ValueError('Gazebo prototype: initial altitude must be between 20 and 100 m.')
    if config.descent_angle_deg != 0:
        raise ValueError('Gazebo prototype: only level target passage (0° descent angle) is supported.')
    circle = holding_circle(snapshot.state, snapshot.speed)
    if circle is None or not 80 <= circle[1] <= 500:
        raise ValueError('Gazebo prototype: a loiter-circle radius between 80 and 500 m is required.')
    if config.activation_time < 5:
        raise ValueError('Gazebo prototype: at least 5 s of initial loiter is required.')
    if config.target_altitude is not None and not 20 <= config.target_altitude <= 100:
        raise ValueError('Gazebo prototype: target altitude must be between 20 and 100 m.')
    for obstacle in config.obstacles:
        if max(abs(obstacle.center[k])+obstacle.half_extents[k] for k in (0, 1)) > 2800:
            raise ValueError('Obstacle outside the 6 km laboratory ground footprint.')
        if float(obstacle.distance([*snapshot.state[:2], 0.])) < config.obstacle_clearance:
            raise ValueError('The area below the aircraft is occupied: the prototype initializes PX4 on the ground before the airborne start.')
    if max(abs(v) for v in (*snapshot.state[:2], *config.target, *circle[0])) > 2300:
        raise ValueError('Keep the edited scene within ±2300 m of the laboratory origin.')


def scenario_record(snapshots, config):
    validate_scenario(snapshots, config)
    settings = {name: getattr(config, name) for name in MISSION_FIELDS}
    settings['obstacles'] = [asdict(o) for o in config.obstacles]
    return {'schema': SCHEMA, 'snapshots': [asdict(s) for s in snapshots], 'mission': settings}


def load_scenario(path):
    record = json.loads(Path(path).read_text())
    schema = record.get('schema') if isinstance(record, dict) else None
    legacy = schema == LEGACY_SCHEMA
    allowed = set(MISSION_FIELDS) - ({'generator_mode'} if legacy else set())
    if schema != SCHEMA:
        allowed.remove('numerical_backend')
    if (not isinstance(record, dict) or set(record) != {'schema', 'snapshots', 'mission'}
            or schema not in (SCHEMA, PREVIOUS_SCHEMA, LEGACY_SCHEMA) or not isinstance(record['snapshots'], list)
            or len(record['snapshots']) != 1 or not isinstance(record['mission'], dict)
            or not allowed-{'arrival_deadline', 'post_arrival_time'} <= set(record['mission'])
            or not set(record['mission']) <= allowed):
        raise ValueError('Invalid or unsupported single-aircraft editor scenario schema.')
    try:
        fields = dict(record['snapshots'][0])
        if set(fields) != {'drone_id', 'state', 'speed', 'altitude', 'cruise_altitude'}:
            raise ValueError('Unexpected snapshot fields')
        fields['state'] = tuple(fields['state'])
        if len(fields['state']) != 4:
            raise ValueError('Snapshot requires XY, heading and yaw rate')
        snapshots = (Snapshot(**fields),)
        mission = dict(record['mission'])
        if legacy:
            # Retain historical meaning even if the fresh-scene default changes.
            mission['generator_mode'] = 'geometric'
        if schema != SCHEMA:
            mission['numerical_backend'] = 'python'
        if (mission['generator_mode'] not in ('geometric', 'physical_mission')
                or mission['numerical_backend'] not in ('python', 'cpp')):
            raise ValueError('Saved scenarios require explicit generator and backend selections.')
        mission['target'] = tuple(mission['target'])
        mission['obstacles'] = tuple(Obstacle(**o) for o in mission['obstacles'])
        if mission['cruise_altitude_range'] is not None:
            mission['cruise_altitude_range'] = tuple(mission['cruise_altitude_range'])
        config = replace(default_scenario()[1], **mission)
        validate_scenario(snapshots, config)
        return snapshots, config
    except (TypeError, KeyError, AttributeError) as error:
        raise ValueError('Invalid single-aircraft editor scenario fields') from error


def loiter_record(document):
    """Spatial circle plus planned hold duration; NOT a timing certificate."""
    snapshot = Snapshot(**document['snapshots'][0])
    circle = holding_circle(snapshot.state, snapshot.speed)
    if circle is None:
        return None
    center, radius = circle
    return {'center_world_m': [*map(float, center), float(snapshot.altitude)],
            'radius_m': float(radius), 'counter_clockwise': snapshot.state[3] > 0,
            'duration_s': float(document['plans'][0]['release_time']),
            'airspeed_m_s': float(snapshot.speed)}


def derived_loiter(record, offset, dt):
    if record is None:
        return None
    return {**record, 'center_enu_m': (np.asarray(record['center_world_m'])-offset).tolist(),
            'sample_dt_s': float(dt)}


def select_recovery(points, obstacles, offset, clearance=30.):
    """Screen a bounded deterministic catalog of native landing patterns.

    This is a proposal-only geometric screen, not validation of PX4's actual
    swept body, timing or tracking. In particular, 30 m is an explicit display/
    experiment routing buffer, not a proven tracking-error bound.
    """
    points, offset = np.asarray(points, dtype=float), np.asarray(offset, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or offset.shape != (3,) or not np.all(np.isfinite(points))
            or not np.all(np.isfinite(offset)) or not math.isfinite(clearance) or clearance < 30):
        raise ValueError('Finite ENU reference/offset and recovery buffer of at least 30 m required.')
    obstacles = tuple(obstacles)
    def lower_bound(samples):
        if not obstacles:
            return math.inf, None
        spacing = float(np.linalg.norm(np.diff(samples, axis=0), axis=1).max(initial=0.))
        values = [float(np.min(o.distance(samples+offset)))-spacing/2 for o in obstacles]
        index = int(np.argmin(values))
        return values[index], index
    # Keep the historical four proposals first. Only expand the search, never
    # alter the accepted reference, native circle, altitude or routing buffers.
    headings = [0., math.pi/2, math.pi, -math.pi/2]
    headings += [k*math.pi/8 for k in range(16) if k % 4]
    proposals = [(distance, heading) for distance in (600., 800., 1000.) for heading in headings]
    rejected = []
    for distance, heading in proposals:
        forward = np.array([math.cos(heading), math.sin(heading), 0.])
        side = np.array([-forward[1], forward[0], 0.])
        center = -distance*forward+np.array([0., 0., 60.])
        theta = np.linspace(0, 2*math.pi, 361)
        circle = center + 100*np.column_stack((np.cos(theta), np.sin(theta), np.zeros(len(theta))))
        tangent_offset = 100*100/distance*forward
        spread = 100*math.sqrt(1-(100/distance)**2)*side
        tangents = [center+tangent_offset+spread, center+tangent_offset-spread]
        # The approach may join anywhere on the loiter circle. Cover the join
        # chord with an additional radius-100 routing buffer before selecting.
        join = np.linspace(np.asarray(points[-1]), center, 1001)
        info = {'heading_deg': math.degrees(heading) % 360, 'distance_m': distance}
        if max(float(np.max(np.abs(samples[:, :2]+offset[:2]))) for samples in (circle, join)) > 2850:
            rejected.append({**info, 'reason': 'ground_footprint'})
            continue
        sections = [('join', join, 100.), ('circle', circle, 0.),
                    *((f'landing_tangent_{k}', np.linspace(tangent, np.zeros(3), 401), 0.)
                      for k, tangent in enumerate(tangents))]
        margins = []
        for name, samples, buffer in sections:
            bound, obstacle = lower_bound(samples)
            margins.append((bound-buffer, name, obstacle))
        margin, section, obstacle = min(margins, key=lambda value: value[0])
        if margin >= clearance:
            return {'center_enu_m': center.tolist(), 'landing_enu_m': [0., 0., 0.],
                    'landing_yaw_ned_deg': (90-math.degrees(heading)) % 360,
                    'geometry_screen_buffer_m': clearance,
                    'screened_minimum_distance_m': margin if obstacles else None,
                    'selection': info, 'proposals_tested': len(rejected)+1,
                    'rejected_proposals': rejected,
                    'actual_px4_clearance_validated': False}
        rejected.append({**info, 'reason': section, 'obstacle_index': obstacle,
                         'screened_minimum_distance_m': margin})
    geometries = [r for r in rejected if 'screened_minimum_distance_m' in r]
    detail = ''
    if geometries:
        best = max(geometries, key=lambda r: r['screened_minimum_distance_m'])
        detail = (f" Best proposal: heading {best['heading_deg']:g} deg, distance {best['distance_m']:g} m; "
                  f"{best['reason']}, obstacle {best['obstacle_index']}, screened distance "
                  f"{best['screened_minimum_distance_m']:.1f} m < {clearance:g} m required.")
    else:
        detail = ' All proposals leave the laboratory ground footprint.'
    raise ValueError(f'No screened landing pattern among {len(proposals)} proposals.{detail} '
                     'This does not prove continuous-space infeasibility.')


def loiter_observations(rows, record, release_time):
    samples = [r for r in rows if r['mission_current'] == 2
               and (release_time is None or r['sim_time_s'] >= release_time)]
    result = {'sample_count': len(samples), 'verified_in_px4_experiment': False,
              'time_aligned_reference_following': False}
    if len(samples) < 2:
        return result
    xyz = np.array([[r[k] for k in ('east_m', 'north_m', 'up_m')] for r in samples])
    center = np.asarray(record['center_world_m'])
    delta = xyz[:, :2]-center[:2]
    radial = np.abs(np.linalg.norm(delta, axis=1)-record['radius_m'])
    angle = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
    travel = float((angle[-1]-angle[0])*(1 if record['counter_clockwise'] else -1))
    span = float(samples[-1]['sim_time_s']-samples[0]['sim_time_s'])
    result.update(observed_duration_s=span, radial_error_p95_m=float(np.percentile(radial, 95)),
                  radial_error_max_m=float(radial.max()), altitude_error_max_m=float(np.abs(xyz[:, 2]-center[2]).max()),
                  signed_angular_progress_rad=travel)
    result['verified_in_px4_experiment'] = bool(len(samples) >= 20 and span >= record['duration_s']-.5
        and np.percentile(radial, 95) <= 25. and np.max(np.abs(xyz[:, 2]-center[2])) <= 10.
        and travel >= .5*record['airspeed_m_s']*record['duration_s']/record['radius_m'])
    return result


def save_preview(path, document):
    """Same original trajectory and obstacle solids, no simulated second plan."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from types import SimpleNamespace
    from .trajectory2d.obstacle_view import draw_obstacles_3d
    from .px4_termination import flight_reference
    points = flight_reference(document)
    obstacles = tuple(Obstacle(**o) for o in document['config']['obstacles'])
    figure = Figure(figsize=(11, 7), layout='constrained')
    FigureCanvasAgg(figure)
    axis = figure.add_subplot(projection='3d')
    z = document['config']['target_altitude']
    target = np.r_[document['config']['target'], z if z is not None else points[0, 2]]
    # Set finite, deterministic display limits BEFORE adding 3D collections.
    # Some Matplotlib versions autoscale an empty collection's extreme bounds
    # before valid flight points exist, intermittently overflowing to infinity.
    bounds = np.vstack((points, target, *[np.array(o.center)+sign*np.array(o.half_extents)
                                         for o in obstacles for sign in (-1, 1)]))
    low, high = bounds.min(axis=0), bounds.max(axis=0)
    padding = np.maximum(10., .05*(high-low))
    for setter, a, b, pad in zip((axis.set_xlim, axis.set_ylim, axis.set_zlim), low, high, padding):
        setter(a-pad, b+pad, auto=False)
    draw_obstacles_3d(axis, SimpleNamespace(obstacles=obstacles))
    axis.plot(*points.T, color='deepskyblue', label='Calculated reference (including initial loiter)')
    axis.scatter(*points[0], color='red', s=50, label='Initial aircraft position')
    axis.scatter(*document['config']['target'], z if z is not None else points[0, 2], color='gold', s=80, label='Target')
    axis.set(xlabel='East (m)', ylabel='North (m)', zlabel='Up (m)', title='Edited scenario — theoretical reference, not PX4 tracking')
    axis.legend(fontsize=8)
    figure.savefig(path, dpi=130)
