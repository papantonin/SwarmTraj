"""Isolated point-mass physics checks, with no destination or mission planner.

Manufactured, mild vertical-motion fixtures exercise climb/descent and return
to level. They are laboratory inputs, not a controller or a flight-plan source.
No controls or executable aircraft plans are exported.
"""

import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform

import numpy as np
import scipy
from scipy.integrate import solve_ivp

from swarm_sim.trajectory2d import Config
from swarm_sim.trajectory2d.dynamics import G
from swarm_sim.trajectory2d.execution_safety import envelope_margins
from swarm_sim.trajectory2d.flight3d import air_velocity, derivative, envelope_ok, propagate
from swarm_sim.trajectory2d.aircraft_profiles import load_aircraft_profile


CASES = ('level', 'climb_recovery', 'descent_recovery')
START, PULSE, END = 2., 10., 16.
PEAK_RAD = np.deg2rad(8.)
TOLERANCES = {'analytic_acceleration_m_s2': 1e-10, 'analytic_rate_rad_s': 1e-10,
              'analytic_velocity_m_s': 1e-10,
              'energy_rate_w_kg': 1e-10, 'integration_position_m': 2e-6,
              'integration_speed_m_s': 1e-6, 'integration_angle_rad': 1e-7,
              'settled_vertical_speed_m_s': .1, 'settled_angle_deg': .5,
              'envelope': 1e-7}


def slope(times, case):
    """A prescribed small-amplitude test signal, unrelated to a destination."""
    if case not in CASES:
        raise ValueError('unknown vertical test case')
    t = np.asarray(times, dtype=float)
    phase = np.clip((t - START) / PULSE, 0, 1)
    sign = {'level': 0, 'climb_recovery': 1, 'descent_recovery': -1}[case]
    gamma = sign * PEAK_RAD * np.sin(np.pi * phase)**2
    rate = sign * PEAK_RAD * np.pi / PULSE * np.sin(2 * np.pi * phase)
    active = (t > START) & (t < START + PULSE)
    return np.where(active, gamma, 0.), np.where(active, rate, 0.)


def fixture_controls(times, speed, case, flight):
    """Manufactured-solution test forces. Never clipped to conceal violations."""
    gamma, rate = slope(times, case)
    load = np.cos(gamma) + speed * rate / G
    axial = flight.parasitic_drag * speed**2 + flight.induced_drag * load**2 / speed**2 + G * np.sin(gamma)
    return np.stack((np.maximum(axial, 0), np.maximum(-axial, 0), load, np.zeros_like(gamma)), axis=-1)


def analytic_reference(times, speed, case):
    """Independent kinematic quadrature for the prescribed constant total speed."""
    def kinematics(t, _):
        gamma, _ = slope(t, case)
        return [speed * np.cos(gamma), speed * np.sin(gamma)]
    solution = solve_ivp(kinematics, (0, END), [0., 100.], t_eval=times,
                         method='DOP853', rtol=1e-12, atol=1e-12, max_step=.1)
    if not solution.success:
        raise RuntimeError('reference quadrature failed: ' + solution.message)
    gamma, _ = slope(times, case)
    zeros = np.zeros_like(times)
    return np.column_stack((solution.y[0], zeros, solution.y[1], np.full_like(times, speed),
                            zeros, gamma, zeros))


def independent_held_rhs(_, state, control, flight):
    """Separately written unbanked equations: [x,z,V,gamma], no engine RHS/drag."""
    _, _, speed, gamma = state
    thrust, brake, load, _ = control
    drag = flight.parasitic_drag * speed**2 + flight.induced_drag * load**2 / speed**2
    return [speed * np.cos(gamma), speed * np.sin(gamma),
            thrust - brake - drag - G * np.sin(gamma), G * (load - np.cos(gamma)) / speed]


def source_fingerprint():
    root = Path(__file__).resolve().parents[1]
    paths = [Path(__file__).resolve(), *(root / 'src/swarm_sim/trajectory2d' / name for name in
             ('flight3d.py', 'dynamics.py', 'execution_safety.py', 'planning.py', 'aircraft_profiles.py'))]
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def negative_envelope_checks(config, speed=None):
    """Deliberately invalid laboratory samples; never relabelled as safe flight."""
    if speed is None:
        speed = 12. if config.limits.min_speed <= 12 <= config.limits.max_speed else (config.limits.min_speed + config.limits.max_speed) / 2
    state = np.array([0., 0., 100., speed, 0., 0., 0.])
    control = fixture_controls(0., speed, 'level', config.flight)
    if not envelope_ok(state, control, config):
        raise ValueError('negative checks require an admissible level-flight baseline; select a suitable test speed')
    trials = []
    for name, component, value in (
        ('below_minimum_speed', 3, config.limits.min_speed - .5),
        ('above_maximum_speed', 3, config.limits.max_speed + .5),
        ('below_floor', 2, config.min_altitude - .25),
        ('above_ceiling', 2, config.max_altitude + .25),
        ('excess_bank', 6, np.deg2rad(config.limits.max_bank_deg + 1)),
        ('nonfinite_state', 0, np.nan),
    ):
        invalid = state.copy()
        invalid[component] = value
        trials.append((name, invalid, control))
    for name, component, value in (
        ('excess_thrust', 0, config.flight.max_thrust_acceleration + .25),
        ('excess_brake', 1, config.flight.max_brake_acceleration + .25),
        ('excess_structural_load', 2, config.flight.max_load_factor + .25),
        ('excess_roll_rate', 3, np.deg2rad(config.limits.max_roll_rate_deg_s + 1)),
        ('excess_vertical_acceleration', 2, 1 + 1.5 * config.max_vertical_acceleration / G),
        ('nonfinite_control', 0, np.nan),
    ):
        invalid = control.copy()
        invalid[component] = value
        trials.append((name, state, invalid))
    records = []
    for name, sample, forces in trials:
        accepted = envelope_ok(sample, forces, config)
        records.append({'case': name, 'expected_rejection': True,
                        'sampled_envelope_ok': accepted, 'check_passed': not accepted})
    return records


def run_case(speed, case, dt=.2, config=None):
    """Return diagnostics and read-only plotting traces, never an aircraft plan."""
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError('test speed must be finite and positive')
    if not np.isfinite(dt) or not .0125 <= dt <= .5 or any(
            abs(t / dt - round(t / dt)) > 1e-8 for t in (START, PULSE, END)):
        raise ValueError('dt must be 0.0125..0.5 s and exactly divide each fixture phase')
    config = config or Config(dimensions=3, max_descent_rate=10)
    if not config.coupled_flight:
        raise ValueError('the vertical bench requires coupled 3D physics')
    times = np.arange(round(END / dt) + 1) * dt
    reference = analytic_reference(times, speed, case)
    reference_controls = fixture_controls(times, speed, case, config.flight)
    controls = reference_controls[:-1]
    states = np.empty_like(reference)
    independent = np.empty((len(times), 4))
    states[0] = reference[0]
    independent[0] = reference[0, [0, 2, 3, 5]]
    for k, control in enumerate(controls):
        states[k + 1] = propagate(states[k], control, dt, config.flight)
        solution = solve_ivp(independent_held_rhs, (times[k], times[k + 1]), independent[k],
                             args=(control, config.flight), method='DOP853', rtol=1e-12, atol=1e-12)
        if not solution.success:
            raise RuntimeError('independent integration failed: ' + solution.message)
        independent[k + 1] = solution.y[:, -1]
    # Same held controls, including both sides of switches and refined interiors.
    dense = propagate(states[:-1, None, :], controls[:, None, :],
                      np.linspace(0, dt, 5)[None, :], config.flight, substeps=8)
    margins = envelope_margins(dense, controls[:, None, :], config)
    minimum_margins = {key: float(np.min(value)) for key, value in margins.items()}
    sampled_envelope_ok = (envelope_ok(dense, controls[:, None, :], config)
                           and envelope_ok(states, reference_controls, config))
    reference_envelope_ok = envelope_ok(reference, reference_controls, config)
    # Compare scalar, batched and independently integrated paths separately.
    fine = propagate(states[:-1], controls, dt, config.flight, substeps=8)
    integration_error = states[:, [0, 2, 3, 5]] - independent
    analytic_rhs = derivative(reference, reference_controls, config.flight)
    expected_velocity = np.column_stack((speed * np.cos(reference[:, 5]), np.zeros_like(times),
                                         speed * np.sin(reference[:, 5])))
    analytic_velocity_error = float(np.max(np.linalg.norm(analytic_rhs[:, :3] - expected_velocity, axis=1)))
    _, gamma_rate = slope(times, case)
    analytic_acceleration_error = float(np.max(np.abs(analytic_rhs[:, 3])))
    analytic_rate_error = float(np.max(np.abs(analytic_rhs[:, 5] - gamma_rate)))
    rhs = derivative(dense, controls[:, None, :], config.flight)
    speed_actual = dense[..., 3]
    load = controls[:, None, 2]
    drag = config.flight.parasitic_drag * speed_actual**2 + config.flight.induced_drag * load**2 / speed_actual**2
    work_rate = speed_actual * (controls[:, None, 0] - controls[:, None, 1] - drag)
    energy_rate_error = float(np.max(np.abs(speed_actual * rhs[..., 3] + G * rhs[..., 2] - work_rate)))
    position_error = float(np.max(np.linalg.norm(integration_error[:, :2], axis=1)))
    speed_error = float(np.max(np.abs(integration_error[:, 2])))
    angle_error = float(np.max(np.abs(integration_error[:, 3])))
    subdivision_position = float(np.max(np.linalg.norm(fine[:, :3] - states[1:, :3], axis=1)))
    subdivision_speed = float(np.max(np.abs(fine[:, 3] - states[1:, 3])))
    subdivision_angle = float(np.max(np.abs(fine[:, 5] - states[1:, 5])))
    # The complete final four-second level section must be settled, not just its last sample.
    recovery = times >= START + PULSE - 1e-9
    settled_vz = float(np.max(np.abs(air_velocity(states[recovery])[:, 2])))
    settled_angle = float(np.rad2deg(np.max(np.abs(states[recovery, 5]))))
    physics_ok = (analytic_velocity_error <= TOLERANCES['analytic_velocity_m_s']
                  and analytic_acceleration_error <= TOLERANCES['analytic_acceleration_m_s2']
                  and analytic_rate_error <= TOLERANCES['analytic_rate_rad_s']
                  and energy_rate_error <= TOLERANCES['energy_rate_w_kg']
                  and position_error <= TOLERANCES['integration_position_m']
                  and speed_error <= TOLERANCES['integration_speed_m_s']
                  and angle_error <= TOLERANCES['integration_angle_rad']
                  and subdivision_position <= TOLERANCES['integration_position_m']
                  and subdivision_speed <= TOLERANCES['integration_speed_m_s']
                  and subdivision_angle <= TOLERANCES['integration_angle_rad'])
    settled = (settled_vz <= TOLERANCES['settled_vertical_speed_m_s']
               and settled_angle <= TOLERANCES['settled_angle_deg'])
    checks = {'physics_consistent': bool(physics_ok), 'sampled_reference_envelope_ok': bool(reference_envelope_ok),
              'sampled_held_envelope_ok': bool(sampled_envelope_ok), 'settled_level_section': bool(settled)}
    report = {'case': case, 'total_airspeed_m_s': float(speed), 'dt_s': float(dt),
              'checks': checks, 'trial_passed': all(checks.values()),
              'failed_checks': [key for key, value in checks.items() if not value],
              'analytic_velocity_residual_m_s': analytic_velocity_error,
              'analytic_speed_acceleration_residual_m_s2': analytic_acceleration_error,
              'analytic_slope_rate_residual_rad_s': analytic_rate_error,
              'energy_rate_residual_w_kg': energy_rate_error,
              'independent_position_error_m': position_error, 'independent_speed_error_m_s': speed_error,
              'independent_slope_error_rad': angle_error,
              'subdivision_position_difference_m': subdivision_position,
              'subdivision_speed_difference_m_s': subdivision_speed,
              'subdivision_slope_difference_rad': subdivision_angle,
              'maximum_reference_position_error_m': float(np.max(np.linalg.norm(states[:, :3] - reference[:, :3], axis=1))),
              'maximum_reference_airspeed_error_m_s': float(np.max(np.abs(states[:, 3] - speed))),
              'level_section_maximum_vertical_speed_m_s': settled_vz,
              'level_section_maximum_slope_deg': settled_angle,
              'minimum_altitude_m': float(dense[..., 2].min()), 'maximum_altitude_m': float(dense[..., 2].max()),
              'minimum_envelope_margins': minimum_margins}
    for array in (times, states, reference):
        array.flags.writeable = False
    return report, {'time_s': times, 'actual': states, 'reference': reference}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=[10, 11, 12, 13])
    parser.add_argument('--cases', choices=CASES, nargs='+', default=list(CASES))
    parser.add_argument('--dt', type=float, default=.2)
    parser.add_argument('--aircraft-profile', type=Path, action='append', help='common fixed-wing JSON profile; speeds remain explicit test settings')
    parser.add_argument('--refine', action='store_true', help='also halve the held-command interval; not an RK4-order test')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if args.aircraft_profile is not None:
        if len(args.aircraft_profile) != 1:
            parser.error('only one common aircraft profile is supported per experiment')
        args.aircraft_profile = args.aircraft_profile[0]
    config = Config(dimensions=3, max_descent_rate=10)
    aircraft_profile = None
    if args.aircraft_profile is not None:
        try:
            aircraft_profile = load_aircraft_profile(args.aircraft_profile)
            config = aircraft_profile.apply(config)
        except (ValueError, OSError) as error:
            parser.error(str(error))
    if (any(not np.isfinite(v) or not config.limits.min_speed <= v <= config.limits.max_speed for v in args.speeds)
            or len(set(args.speeds)) != len(args.speeds) or len(set(args.cases)) != len(args.cases)):
        parser.error(f'speeds must be unique and within {config.limits.min_speed:g}..{config.limits.max_speed:g} m/s; cases must be unique')
    if not np.isfinite(args.dt) or not .025 <= args.dt <= .5 or any(
            abs(t / args.dt - round(t / args.dt)) > 1e-8 for t in (START, PULSE, END)):
        parser.error('dt must be 0.025..0.5 s and exactly divide each fixture phase')
    output = args.output or Path('results/benchmarks') / (
        'vertical-flight-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    if output.exists():
        parser.error('output must be a new file')
    sources = source_fingerprint()
    records = []
    # Claim neither real-flight safety nor mission feasibility from these fixtures.
    document = {'schema': 'swarm-vertical-physics-validation-v1', 'python': platform.python_version(),
                'numpy': np.__version__, 'scipy': scipy.__version__,
                'completed': False, 'all_trials_passed': False,
                'source_sha256': sources, 'tolerances': TOLERANCES,
                'flight_limits': asdict(config.flight), 'kinematic_limits': asdict(config.limits),
                'altitude_range_m': [config.min_altitude, config.max_altitude],
                'maximum_climb_rate_m_s': config.max_climb_rate, 'maximum_descent_rate_m_s': config.descent_rate_limit,
                'maximum_vertical_acceleration_m_s2': config.max_vertical_acceleration,
                'requested_speeds_m_s': args.speeds, 'requested_cases': args.cases, 'refinement_requested': args.refine,
                'fixture': {'initial_altitude_m': 100., 'peak_slope_deg': float(np.rad2deg(PEAK_RAD)),
                            'initial_level_s': START, 'vertical_pulse_s': PULSE,
                            'final_level_s': END - START - PULSE},
                'scope': 'Isolated unbanked manufactured-solution tests only; no mission, targeting, '
                         'collision certificate or real-aircraft calibration.', 'trials': records}
    if aircraft_profile is not None:
        document['aircraft_profile'] = aircraft_profile.record()
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve exclusively before any numerical work. A failed run cannot leave
    # a previous successful document or overwrite user evidence.
    with output.open('x', encoding='utf-8') as stream:
        try:
            document['negative_checks'] = (negative_envelope_checks(config, speed=args.speeds[0])
                                           if aircraft_profile is not None else negative_envelope_checks(config))
            for speed in args.speeds:
                for case in args.cases:
                    report, _ = run_case(speed, case, args.dt, config)
                    if args.refine:
                        refined, _ = run_case(speed, case, args.dt / 2, config)
                        report['halved_command_interval'] = refined
                        error = report['maximum_reference_position_error_m']
                        fine_error = refined['maximum_reference_position_error_m']
                        report['command_refinement_not_worse'] = fine_error <= max(error, TOLERANCES['integration_position_m'])
                        report['command_refinement_error_ratio'] = (fine_error / error if error > TOLERANCES['integration_position_m']
                                                                     else None)
                    if source_fingerprint() != sources:
                        raise RuntimeError('source changed during vertical validation')
                    records.append(report)
                    passed = report['trial_passed'] and report.get('halved_command_interval', report)['trial_passed']
                    passed &= report.get('command_refinement_not_worse', True)
                    print(f"{case} V={speed:g} m/s: {'PASS' if passed else 'FAIL'}", flush=True)
            passed = all(r['trial_passed'] and r.get('halved_command_interval', r)['trial_passed']
                         and r.get('command_refinement_not_worse', True) for r in records)
            passed &= all(r['check_passed'] for r in document['negative_checks'])
            document['all_trials_passed'] = passed
            document['completed'] = True
        except Exception as error:
            document.update(all_trials_passed=False, error={'type': type(error).__name__, 'message': str(error)})
            raise
        finally:
            json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
    print(f'SAVED {output}', flush=True)
    return 0 if document['all_trials_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
