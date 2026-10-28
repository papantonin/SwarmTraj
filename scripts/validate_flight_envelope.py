"""Isolated envelope classification and numerical regression experiments.

No planner, destination, guidance call or deployable flight-control export.
Rejected grid samples are classification evidence, not successful flights.
Requested dynamic fixtures that fail remain failed requests (exit code 2).
"""

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import platform

import numpy as np
import scipy

from swarm_sim.trajectory2d import Config
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.trajectory2d.execution_safety import envelope_margins
from swarm_sim.trajectory2d.flight3d import envelope_ok, propagate, trim_control
from swarm_sim.trajectory2d.aircraft_profiles import load_aircraft_profile
import validate_vertical_flight as vertical


EPS = vertical.TOLERANCES['envelope']
MARGIN_ERROR = 1e-10
UNITS = {
    'minimum_airspeed': 'm/s', 'maximum_airspeed': 'm/s', 'bank': 'rad',
    'flight_path_angle': 'rad', 'minimum_altitude': 'm', 'maximum_altitude': 'm',
    'climb_rate': 'm/s', 'descent_rate': 'm/s', 'vertical_acceleration': 'm/s^2',
    'flight_path_rate': 'rad/s', 'minimum_thrust': 'm/s^2', 'maximum_thrust': 'm/s^2',
    'minimum_brake': 'm/s^2', 'maximum_brake': 'm/s^2',
    'minimum_load': 'g', 'loaded_stall_or_structure': 'g', 'roll_rate': 'rad/s',
}
DEFAULT_SPEEDS = (8., 8.05, 9., 10., 11., 12., 13., 15.95, 16.)
SLOPES = (-45., -40., -30., -20., -10., 0., 10., 20., 30., 40., 45.)
BANKS = (-50., -25., 0., 25., 50.)


def oracle(state, control, config):
    """Separately written scalar inequalities; no production RHS/drag/velocity."""
    s, u = np.asarray(state, dtype=float), np.asarray(control, dtype=float)
    if s.shape != (7,) or u.shape != (4,):
        raise ValueError('scalar oracle requires state (7,) and controls (4,)')
    if not np.all(np.isfinite(s)) or not np.all(np.isfinite(u)):
        return {'accepted': False, 'violations': ['nonfinite_state_or_control'], 'margins': None}
    z, speed, gamma, bank = float(s[2]), float(s[3]), float(s[5]), float(s[6])
    if speed <= 0:
        return {'accepted': False, 'violations': ['nonpositive_airspeed'], 'margins': None}
    thrust, brake, load, roll = map(float, u)
    f, limits = config.flight, config.limits
    cg, sg = math.cos(gamma), math.sin(gamma)
    drag = f.parasitic_drag * speed**2 + f.induced_drag * load**2 / speed**2
    acceleration = thrust - brake - drag - G * sg
    lift_difference = load * math.cos(bank) - cg
    vertical_acceleration = acceleration * sg + G * cg * lift_difference
    slope_rate = G * lift_difference / speed
    stall_ceiling = (speed / (f.stall_speed * f.stall_margin))**2
    margins = {
        'minimum_airspeed': speed - limits.min_speed, 'maximum_airspeed': limits.max_speed - speed,
        'bank': math.radians(limits.max_bank_deg) - abs(bank),
        'flight_path_angle': math.radians(f.max_flight_path_deg) - abs(gamma),
        'minimum_altitude': z - config.min_altitude, 'maximum_altitude': config.max_altitude - z,
        'climb_rate': config.max_climb_rate - speed * sg,
        'descent_rate': config.descent_rate_limit + speed * sg,
        'vertical_acceleration': config.max_vertical_acceleration - abs(vertical_acceleration),
        'flight_path_rate': math.radians(f.max_flight_path_rate_deg_s) - abs(slope_rate),
        'minimum_thrust': thrust, 'maximum_thrust': f.max_thrust_acceleration - thrust,
        'minimum_brake': brake, 'maximum_brake': f.max_brake_acceleration - brake,
        'minimum_load': load - f.min_load_factor,
        'loaded_stall_or_structure': min(f.max_load_factor, stall_ceiling) - load,
        'roll_rate': math.radians(limits.max_roll_rate_deg_s) - abs(roll),
    }
    violations = [key for key, margin in margins.items() if margin < -EPS]
    return {'accepted': not violations, 'violations': violations, 'margins': margins}


def compare_sample(state, control, config):
    expected = oracle(state, control, config)
    observed = envelope_ok(state, control, config, tolerance=EPS)
    error = None
    if expected['margins'] is not None:
        measured = envelope_margins(state, control, config)
        if set(measured) != set(expected['margins']):
            raise RuntimeError('envelope margin schema changed')
        error = max(abs(float(measured[k]) - v) for k, v in expected['margins'].items())
    return {'expected_accepted': expected['accepted'], 'observed_accepted': observed,
            'violated_constraints': expected['violations'], 'margin_error_max': error,
            'contract_consistent': observed == expected['accepted'] and (error is None or error <= MARGIN_ERROR)}


def _state(speed=12., slope=0., bank=0.):
    return np.array([0., 0., 100., speed, .3, slope, bank])


def _trim(state, config):
    """Independent equilibrium test input, never clipped to the physical limits."""
    speed, gamma, bank = state[3], state[5], state[6]
    load = math.cos(gamma) / math.cos(bank)
    f = config.flight
    axial = f.parasitic_drag * speed**2 + f.induced_drag * load**2 / speed**2 + G * math.sin(gamma)
    return np.array([max(axial, 0.), max(-axial, 0.), load, 0.])


def boundary_samples(config):
    """Known inward/boundary/outward perturbations; coupled exclusions retained."""
    f, limits = config.flight, config.limits
    for outward in (-1e-4, 0., 1e-4):
        region = 'inside' if outward < 0 else 'outside' if outward > 0 else 'boundary'
        samples = []
        for key, component, value in (
            ('minimum_airspeed', 3, limits.min_speed - outward),
            ('maximum_airspeed', 3, limits.max_speed + outward),
            ('minimum_altitude', 2, config.min_altitude - outward),
            ('maximum_altitude', 2, config.max_altitude + outward),
        ):
            s = _state()
            s[component] = value
            samples.append((key, key, s, _trim(s, config)))
        for sign in (-1, 1):
            s = _state(bank=sign * (math.radians(limits.max_bank_deg) + outward))
            samples.append((f'bank_{sign}', 'bank', s, _trim(s, config)))
            s = _state()
            u = _trim(s, config)
            u[3] = sign * (math.radians(limits.max_roll_rate_deg_s) + outward)
            samples.append((f'roll_{sign}', 'roll_rate', s, u))
            u = _trim(s, config)
            u[2] = 1 + sign * (config.max_vertical_acceleration + outward) / G
            u[0] = f.parasitic_drag * s[3]**2 + f.induced_drag * u[2]**2 / s[3]**2
            samples.append((f'vertical_acceleration_{sign}', 'vertical_acceleration', s, u))
            s = _state(speed=10., slope=math.radians(-30.))
            u = np.array([0., 3.7 if sign > 0 else 5.,
                          math.cos(s[5]) + sign * s[3] * (math.radians(f.max_flight_path_rate_deg_s) + outward) / G, 0.])
            samples.append((f'slope_rate_{sign}', 'flight_path_rate', s, u))
        s = _state(slope=-(math.radians(f.max_flight_path_deg) + outward))
        samples.append(('negative_slope', 'flight_path_angle', s, np.array([0., 5., .8, 0.])))
        s = _state(slope=math.asin((config.max_climb_rate + outward) / 12.))
        samples.append(('climb_rate', 'climb_rate', s, _trim(s, config)))
        s = _state(speed=16., slope=-math.asin((config.descent_rate_limit + outward) / 16.))
        samples.append(('descent_rate', 'descent_rate', s, np.array([0., 5., .8, 0.])))
        for key, component, value in (
            ('minimum_thrust', 0, -outward), ('maximum_thrust', 0, f.max_thrust_acceleration + outward),
            ('minimum_brake', 1, -outward), ('maximum_brake', 1, f.max_brake_acceleration + outward),
            ('minimum_load', 2, f.min_load_factor - outward),
        ):
            s = _state()
            u = _trim(s, config)
            u[component] = value
            samples.append((key, key, s, u))
        stall = (8. / (f.stall_speed * f.stall_margin))**2
        s = _state(speed=8., bank=math.acos(1 / stall))
        u = _trim(s, config)
        u[2] = stall + outward
        samples.append(('loaded_stall', 'loaded_stall_or_structure', s, u))
        s = _state(speed=16.)
        u = _trim(s, config)
        u[2] = f.max_load_factor + outward
        samples.append(('structural_load', 'loaded_stall_or_structure', s, u))
        for name, target, s, u in samples:
            yield name, target, region, s, u


def profile_boundary_samples(config):
    """Parameter-relative probes; other coupled violations remain visible.

    These isolate inequality values, not necessarily flyable boundary states.
    Inaccessible vertical-rate surfaces can require a speed outside the envelope;
    the corresponding speed rejection is retained, not called a feasible trim.
    """
    f, lim = config.flight, config.limits
    speed = (lim.min_speed + lim.max_speed) / 2
    for outward in (-1e-4, 0., 1e-4):
        region = 'inside' if outward < 0 else 'outside' if outward > 0 else 'boundary'
        for name, component, value in (
            ('minimum_airspeed', 3, lim.min_speed - outward),
            ('maximum_airspeed', 3, lim.max_speed + outward),
            ('minimum_altitude', 2, config.min_altitude - outward),
            ('maximum_altitude', 2, config.max_altitude + outward),
        ):
            s = _state(speed)
            s[component] = value
            yield name, name, region, s, _trim(s, config)
        for sign in (-1, 1):
            for name, component, value in (
                ('bank', 6, sign * (math.radians(lim.max_bank_deg) + outward)),
                ('flight_path_angle', 5, sign * (math.radians(f.max_flight_path_deg) + outward)),
            ):
                s = _state(speed)
                s[component] = value
                yield f'{name}_{sign}', name, region, s, _trim(s, config)
            s = _state(speed)
            u = _trim(s, config)
            u[3] = sign * (math.radians(lim.max_roll_rate_deg_s) + outward)
            yield f'roll_{sign}', 'roll_rate', region, s, u
            u = _trim(s, config)
            u[2] = 1 + sign * (config.max_vertical_acceleration + outward) / G
            axial = f.parasitic_drag * speed**2 + f.induced_drag * u[2]**2 / speed**2
            u[:2] = max(axial, 0.), max(-axial, 0.)
            yield f'vertical_acceleration_{sign}', 'vertical_acceleration', region, s, u
            u = _trim(s, config)
            u[2] = 1 + sign * speed * (math.radians(f.max_flight_path_rate_deg_s) + outward) / G
            yield f'slope_rate_{sign}', 'flight_path_rate', region, s, u
        for name, sign, rate in (('climb_rate', 1, config.max_climb_rate),
                                 ('descent_rate', -1, config.descent_rate_limit)):
            v = max(speed, rate + 1.)
            s = _state(v, slope=sign * math.asin((rate + outward) / v))
            yield name, name, region, s, _trim(s, config)
        for name, component, value in (
            ('minimum_thrust', 0, -outward), ('maximum_thrust', 0, f.max_thrust_acceleration + outward),
            ('minimum_brake', 1, -outward), ('maximum_brake', 1, f.max_brake_acceleration + outward),
            ('minimum_load', 2, f.min_load_factor - outward),
            ('loaded_stall_or_structure', 2, min(f.max_load_factor, (speed / (f.stall_speed * f.stall_margin))**2) + outward),
        ):
            s = _state(speed)
            u = _trim(s, config)
            u[component] = value
            yield name, name, region, s, u


def check_boundaries(config, *, parameter_relative=False):
    records = []
    samples = profile_boundary_samples(config) if parameter_relative else boundary_samples(config)
    for name, target, region, state, control in samples:
        row = compare_sample(state, control, config)
        margin = oracle(state, control, config)['margins'][target]
        target_correct = (margin > EPS if region == 'inside' else margin < -EPS if region == 'outside'
                          else abs(margin) <= EPS)
        records.append({'case': name, 'target_constraint': target, 'target_region': region,
                        'target_margin': margin, 'margin_unit': UNITS[target],
                        'other_violations': [k for k in row['violated_constraints'] if k != target],
                        'target_classification_correct': target_correct, **row})
    return records


def check_guards(config):
    records = []
    for name, location, value in (('zero_speed', 3, 0.), ('negative_speed', 3, -1.),
                                  ('nan_state', 0, np.nan), ('infinite_state', 0, np.inf)):
        s = _state()
        s[location] = value
        records.append({'case': name, **compare_sample(s, np.array([1., 0., 1., 0.]), config)})
    for value, name in ((np.nan, 'nan_control'), (np.inf, 'infinite_control')):
        u = np.array([value, 0., 1., 0.])
        records.append({'case': name, **compare_sample(_state(), u, config)})
    return records


def check_equilibrium(speed, slope_deg, bank_deg, config):
    state = _state(speed, math.radians(slope_deg), math.radians(bank_deg))
    control = _trim(state, config)
    row = {'total_airspeed_m_s': speed, 'slope_deg': slope_deg, 'bank_deg': bank_deg,
           **compare_sample(state, control, config)}
    trim_error = float(np.max(np.abs(control - trim_control(state, config.flight))))
    row['trim_input_error_max'] = trim_error
    row['contract_consistent'] &= trim_error <= MARGIN_ERROR
    if not row['expected_accepted']:
        row['integrated'] = False
        return row
    # Two seconds of an existing steady equilibrium, not a maneuver generator.
    times = np.linspace(0., 2., 11)
    states = [state]
    for _ in times[1:]:
        states.append(propagate(states[-1], control, .2, config.flight))
    states = np.asarray(states)
    omega = G * math.tan(state[6]) / speed
    horizontal = speed * math.cos(state[5])
    expected = np.broadcast_to(state, states.shape).copy()
    if bank_deg == 0:
        expected[:, 0] += horizontal * times * math.cos(state[4])
        expected[:, 1] += horizontal * times * math.sin(state[4])
    else:
        expected[:, 0] += horizontal * (np.sin(state[4] + omega * times) - math.sin(state[4])) / omega
        expected[:, 1] += horizontal * (math.cos(state[4]) - np.cos(state[4] + omega * times)) / omega
    expected[:, 2] += speed * math.sin(state[5]) * times
    expected[:, 4] += omega * times
    angular_error = (states[:, 4:] - expected[:, 4:] + np.pi) % (2 * np.pi) - np.pi
    error = float(np.max(np.linalg.norm(states[:, :3] - expected[:, :3], axis=1)))
    speed_error = float(np.max(np.abs(states[:, 3] - expected[:, 3])))
    angle_error = float(np.max(np.abs(angular_error)))
    dense = propagate(states[:-1, None, :], control, np.linspace(0, .2, 5)[None, :], config.flight, substeps=8)
    dense_ok = envelope_ok(dense, control, config, tolerance=EPS)
    row.update(integrated=True, analytic_position_error_m=error, analytic_speed_error_m_s=speed_error,
               analytic_angle_error_rad=angle_error, sampled_envelope_ok=dense_ok)
    row['contract_consistent'] &= (error <= vertical.TOLERANCES['integration_position_m']
                                   and speed_error <= vertical.TOLERANCES['integration_speed_m_s']
                                   and angle_error <= vertical.TOLERANCES['integration_angle_rad'] and dense_ok)
    return row


def check_dynamic(speed, case, dt, config):
    report, trace = vertical.run_case(speed, case, dt, config)
    times, actual, reference = trace['time_s'], trace['actual'], trace['reference']
    controls = vertical.fixture_controls(times, speed, case, config.flight)
    dense = propagate(actual[:-1, None, :], controls[:-1, None, :],
                      np.linspace(0, dt, 5)[None, :], config.flight, substeps=8)
    first, minimum = {}, {}
    expected_held = True
    for k, block in enumerate(dense):
        for fraction, state in zip(np.linspace(0, dt, 5), block):
            item = oracle(state, controls[k], config)
            expected_held &= item['accepted']
            for name, margin in item['margins'].items():
                minimum[name] = min(minimum.get(name, math.inf), margin)
            for name in item['violations']:
                first.setdefault(name, float(times[k] + fraction))
    for time, state, control in zip(times, actual, controls):
        item = oracle(state, control, config)
        expected_held &= item['accepted']
        for name, margin in item['margins'].items():
            minimum[name] = min(minimum.get(name, math.inf), margin)
        for name in item['violations']:
            first[name] = min(first.get(name, math.inf), float(time))
    expected_reference = all(oracle(s, u, config)['accepted'] for s, u in zip(reference, controls))
    level = times >= vertical.START + vertical.PULSE - 1e-9
    expected_settled = (float(np.max(np.abs(actual[level, 3] * np.sin(actual[level, 5]))))
                        <= vertical.TOLERANCES['settled_vertical_speed_m_s']
                        and float(np.max(np.abs(actual[level, 5])))
                        <= math.radians(vertical.TOLERANCES['settled_angle_deg']))
    expected_accepted = (report['checks']['physics_consistent'] and expected_held
                         and expected_reference and expected_settled)
    classification = (expected_held == report['checks']['sampled_held_envelope_ok']
                      and expected_reference == report['checks']['sampled_reference_envelope_ok'])
    summary = (expected_settled == report['checks']['settled_level_section']
               and expected_accepted == report['trial_passed'])
    precision = report['checks']['physics_consistent']
    consistent = classification and summary and precision
    return {'total_airspeed_m_s': speed, 'case': case, 'dt_s': dt,
            'classification_consistent': bool(classification), 'summary_consistent': bool(summary),
            'numerical_precision_passed': bool(precision),
            'contract_consistent': bool(consistent), 'trial_accepted': report['trial_passed'],
            'first_observed_violations_s': first, 'minimum_oracle_margins': minimum, 'vertical_report': report}


def source_fingerprint():
    sources = vertical.source_fingerprint()
    root = Path(__file__).resolve().parents[1]
    sources[str(Path(__file__).resolve().relative_to(root))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return sources


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+')
    parser.add_argument('--aircraft-profile', type=Path, action='append', help='one fixed-wing JSON; parameter-relative boundary and trim grids')
    parser.add_argument('--dynamic', action='store_true', help='also run the unchanged mild vertical fixtures')
    parser.add_argument('--time-steps', type=float, nargs='+', default=[.2, .1, .05])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    profile = None
    config = Config(dimensions=3, max_descent_rate=10)
    if args.aircraft_profile:
        if len(args.aircraft_profile) != 1:
            parser.error('provide one aircraft-profile per bench run')
        try:
            profile = load_aircraft_profile(args.aircraft_profile[0])
            config = profile.apply(config)
        except (OSError, ValueError) as error:
            parser.error(str(error))
        if config.limits.min_speed <= 1e-4:
            parser.error('boundary fixtures require minimum airspeed above 0.0001 m/s')
    if args.speeds is None:
        args.speeds = (list(dict.fromkeys(np.linspace(config.limits.min_speed, config.limits.max_speed, 5).tolist()))
                       if profile else list(DEFAULT_SPEEDS))
    if (any(not math.isfinite(s) or not config.limits.min_speed <= s <= config.limits.max_speed for s in args.speeds)
            or len(set(args.speeds)) != len(args.speeds)):
        parser.error(f'speeds must be unique finite values within {config.limits.min_speed:g}..{config.limits.max_speed:g} m/s')
    if (any(not math.isfinite(dt) or not .025 <= dt <= .5 or any(
            abs(t / dt - round(t / dt)) > 1e-8 for t in (vertical.START, vertical.PULSE, vertical.END))
            for dt in args.time_steps) or len(set(args.time_steps)) != len(args.time_steps)):
        parser.error('time steps must be unique, 0.025..0.5 s and divide the fixture phases exactly')
    output = args.output or Path('results/benchmarks') / (
        'flight-envelope-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    if output.exists():
        parser.error('output must be a new file')
    slopes = np.linspace(-config.flight.max_flight_path_deg, config.flight.max_flight_path_deg, 11).tolist() if profile else SLOPES
    banks = np.linspace(-config.limits.max_bank_deg, config.limits.max_bank_deg, 5).tolist() if profile else BANKS
    sources = source_fingerprint()
    boundaries, guards, equilibria, dynamic = [], [], [], []
    document = {'schema': 'swarm-flight-envelope-validation-v1', 'completed': False,
                'all_checks_passed': False, 'all_contract_checks_passed': False,
                'all_requested_dynamic_trials_accepted': None,
                'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__,
                'source_sha256': sources, 'envelope_tolerance': EPS, 'margin_comparison_tolerance': MARGIN_ERROR,
                'numerical_tolerances': dict(vertical.TOLERANCES),
                'equilibrium_fixture': {'duration_s': 2., 'command_interval_s': .2,
                                        'initial_altitude_m': 100., 'initial_heading_rad': .3,
                                        'nominal_rk4_substeps': 2, 'dense_rk4_substeps': 8},
                'dynamic_fixture': {'peak_slope_deg': float(np.rad2deg(vertical.PEAK_RAD)),
                                    'initial_level_s': vertical.START, 'vertical_pulse_s': vertical.PULSE,
                                    'final_level_s': vertical.END - vertical.START - vertical.PULSE},
                'margin_units': UNITS, 'flight_limits': asdict(config.flight), 'kinematic_limits': asdict(config.limits),
                'altitude_range_m': [config.min_altitude, config.max_altitude],
                'maximum_climb_rate_m_s': config.max_climb_rate, 'maximum_descent_rate_m_s': config.descent_rate_limit,
                'maximum_vertical_acceleration_m_s2': config.max_vertical_acceleration,
                'requested_speeds_m_s': args.speeds, 'slope_grid_deg': slopes, 'bank_grid_deg': banks,
                'boundary_fixture': 'parameter_relative' if profile else 'historical_reference',
                'dynamic_requested': args.dynamic, 'requested_time_steps_s': args.time_steps,
                'boundaries': boundaries, 'guard_checks': guards, 'equilibria': equilibria, 'dynamic_trials': dynamic,
                'scope': 'Laboratory classification and numerical evidence only; no planner/guidance, '
                         'mission feasibility proof, certified ODE bounds or real-aircraft calibration.'}
    if profile is not None:
        document['aircraft_profile'] = profile.record()
    output.parent.mkdir(parents=True, exist_ok=True)
    def unchanged():
        if source_fingerprint() != sources:
            raise RuntimeError('source changed during envelope validation')
    with output.open('x', encoding='utf-8') as stream:
        try:
            boundaries.extend(check_boundaries(config, parameter_relative=profile is not None))
            guards.extend(check_guards(config))
            for speed in args.speeds:
                for slope in slopes:
                    for bank in banks:
                        equilibria.append(check_equilibrium(speed, slope, bank, config))
                if args.dynamic:
                    for case in vertical.CASES:
                        for dt in args.time_steps:
                            row = check_dynamic(speed, case, dt, config)
                            dynamic.append(row)
                            print(f"{case} V={speed:g} dt={dt:g}: "
                                  f"{'ACCEPTED' if row['trial_accepted'] else 'REJECTED'} "
                                  f"contract={row['contract_consistent']}", flush=True)
                unchanged()
            contract = (all(r['contract_consistent'] and r['target_classification_correct'] for r in boundaries)
                        and all(r['contract_consistent'] and not r['expected_accepted'] for r in guards)
                        and all(r['contract_consistent'] for r in equilibria)
                        and all(r['contract_consistent'] for r in dynamic))
            accepted = all(r['trial_accepted'] for r in dynamic) if args.dynamic else None
            violations = Counter(k for r in equilibria for k in r['violated_constraints'])
            document.update(completed=True, all_contract_checks_passed=contract,
                            all_requested_dynamic_trials_accepted=accepted,
                            all_checks_passed=bool(contract and accepted is not False),
                            counts={'boundary_samples': len(boundaries), 'guard_samples': len(guards),
                                    'trim_samples': len(equilibria),
                                    'accepted_trim_samples': sum(r['expected_accepted'] for r in equilibria),
                                    'rejected_trim_samples': sum(not r['expected_accepted'] for r in equilibria),
                                    'dynamic_trials': len(dynamic),
                                    'accepted_dynamic_trials': sum(r['trial_accepted'] for r in dynamic),
                                    'rejected_dynamic_trials': sum(not r['trial_accepted'] for r in dynamic),
                                    'dynamic_classification_disagreements': sum(not r['classification_consistent'] for r in dynamic),
                                    'dynamic_summary_disagreements': sum(not r['summary_consistent'] for r in dynamic),
                                    'dynamic_precision_failures': sum(not r['numerical_precision_passed'] for r in dynamic)},
                            trim_violation_counts=dict(violations))
        except Exception as error:
            document['error'] = {'type': type(error).__name__, 'message': str(error)}
            raise
        finally:
            json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
    print(json.dumps(document['counts'], sort_keys=True), flush=True)
    print(f"CONTRACTS {document['all_contract_checks_passed']}; ALL_CHECKS {document['all_checks_passed']}", flush=True)
    print(f'SAVED {output}', flush=True)
    return 0 if document['all_checks_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
