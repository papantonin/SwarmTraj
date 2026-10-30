"""Four-aircraft parameter campaign with incremental, failure-preserving JSONL."""

import argparse
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import json
import hashlib
import os
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import load_obstacles
from swarm_sim.trajectory2d.perturbations import position_bias_vectors


PROFILES = {
    'nominal': {},
    'mild': {'wind': .05, 'clock_offset': .03},
    'gust_sensor': {'wind': .1, 'wind_model': 'gust', 'wind_heading_deg': 15.,
                    'clock_offset': .03, 'clock_drift': .0001,
                    'position_bias': .05, 'position_noise': .01},
    'outside_clock': {'clock_offset': .4},
}


@dataclass(frozen=True)
class Scenario:
    config: Config
    snapshots: tuple
    expected_status: str = 'validated'


def scenarios(workers=4, obstacles=()):
    base = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, max_descent_rate=10, descent_angle_deg=30,
                  cruise_altitude_range=(30, 120), planning_workers=workers,
                  planning_strategy='adaptive', initial_candidates=2, search_order='mrv',
                  selection_policy='max_margin', conflict_guidance='witness', obstacle_routing='spatial')
    snapshots = tuple(replace(s, altitude=30+25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    rich = replace(base, target=(500, 500), horizon=240, activation_time=30, approach_spread_deg=180,
                   obstacles=obstacles, reservation_policy='grid', interval_guidance='safe_windows',
                   speed_profile='smooth', speed_profile_range=(10, 14), cruise_levels=2)
    shifted = tuple(replace(s, state=(s.state[0]+160, s.state[1]-80, *s.state[2:]), altitude=40+20*k)
                    for k, s in enumerate(separate_loiter_snapshot(4, radius=35, spacing=100)))
    speeds = tuple(s.with_speed(v, base.limits) for s, v in zip(snapshots, (10., 11., 12., 13.)))
    slow = tuple(s.with_speed(10., base.limits) for s in snapshots)
    duplicates = (snapshots[0], replace(snapshots[0], drone_id=1), *snapshots[2:])
    return {
        'default': Scenario(base, snapshots),
        'spread360': Scenario(replace(base, horizon=240, approach_spread_deg=360), snapshots),
        'obstacles_strict': Scenario(rich, snapshots),
        'obstacles_mixed': Scenario(replace(rich, profile_policy='adaptive_mixed'), snapshots),
        'fixed_obstacles_strict': Scenario(replace(rich, max_extra_loiter=0), snapshots),
        'fixed_crossing': Scenario(replace(rich, target=(500, 300), horizon=210, activation_time=0,
            cruise_altitude_range=(40, 110), obstacles=(), cruise_levels=1, initial_candidates=1,
            max_extra_loiter=0), snapshots),
        'heterogeneous_speeds': Scenario(replace(base, target=(500, 500), horizon=240, activation_time=30,
            approach_spread_deg=180, obstacles=obstacles, reservation_policy='grid'), speeds),
        'heterogeneous_speeds_high_target': Scenario(replace(base, target=(500, 500), target_altitude=45,
            horizon=240, activation_time=30, approach_spread_deg=180, obstacles=obstacles,
            reservation_policy='grid'), speeds),
        'shifted_positions': Scenario(replace(base, target=(450, -150), target_altitude=45, horizon=210,
            activation_time=10, approach_spread_deg=180, descent_angle_deg=20,
            cruise_altitude_range=(40, 130)), shifted),
        'slow_narrow_delays': Scenario(replace(base, target=(350, 200), horizon=210, activation_time=5,
            approach_spread_deg=90, descent_angle_deg=20, cruise_altitude_range=(50, 100), max_extra_loiter=5), slow),
        'unreachable_horizon': Scenario(replace(base, target=(100000, 100000), horizon=30), snapshots,
                                         'destination_unreachable_within_horizon'),
        'invalid_initial_pair': Scenario(base, duplicates, 'initial_separation_violation'),
    }


def write_record(stream, record):
    stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + '\n')
    stream.flush()


def source_fingerprint():
    root = Path(__file__).resolve().parents[1]
    files = [Path(__file__).resolve(), *sorted((root / 'src/swarm_sim/trajectory2d').glob('*.py'))]
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    names = tuple(scenarios(1))
    parser.add_argument('--scenarios', nargs='+', choices=names, default=list(names))
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--profiles', nargs='+', choices=tuple(PROFILES), default=list(PROFILES))
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--candidate-guidance', choices=('none', 'rejections'), default='none')
    parser.add_argument('--disturbance-seed', type=int, default=23)
    parser.add_argument('--obstacles', type=Path, default=Path('examples/scenarios/static_obstacles.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if min(args.seeds) < 0 or args.workers < 0 or args.disturbance_seed < 0:
        parser.error('seeds and workers must be nonnegative')
    for name in ('scenarios', 'seeds', 'profiles'):
        values = getattr(args, name)
        if len(values) != len(set(values)):
            parser.error(f'duplicate --{name} entries are not allowed')
    output = args.output or Path('results/benchmarks') / (
        'scenario-campaign-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.jsonl')
    if output.exists():
        parser.error('output must be a new file')
    cases = scenarios(args.workers, load_obstacles(args.obstacles))
    success, mission_count, execution_count = True, 0, 0
    sources = source_fingerprint()
    def require_same_source():
        if source_fingerprint() != sources:
            raise RuntimeError('Campaign source changed during execution; do not mix spawn-worker code revisions')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        write_record(stream, {'type': 'header', 'schema': 'swarm-scenario-campaign-v1',
            'num_drones': 4, 'python': platform.python_version(),
            'numpy': np.__version__, 'logical_cpus': os.cpu_count(), 'profiles': PROFILES,
            'disturbance_seed': args.disturbance_seed, 'source_sha256': sources,
            'selected_scenarios': args.scenarios, 'selected_profiles': args.profiles, 'seeds': args.seeds,
            'scope': 'nominal numerical validation; not certification'})
        for name in args.scenarios:
            for seed in args.seeds:
                case = cases[name]
                config = replace(case.config, seed=seed, candidate_guidance=args.candidate_guidance)
                print(f'START {name} seed={seed}', flush=True)
                try:
                    require_same_source()
                    result = plan_mission(case.snapshots, config)
                    require_same_source()
                    digest = signature(result)
                    met = result.status == case.expected_status
                    mission = {'type': 'mission', 'scenario': name, 'seed': seed, 'config': asdict(config),
                        'snapshots': [asdict(s) for s in case.snapshots], 'status': result.status,
                        'reason': result.reason, 'expected_status': case.expected_status, 'expectation_met': met,
                        'signature_sha256': digest, 'diagnostics': result.diagnostics, 'timings': result.timings,
                        'attempted': sum(c['attempted'] for c in result.diagnostics.get('catalogs', []))}
                    write_record(stream, mission)
                    mission_count += 1
                    success &= met
                    print(json.dumps({k: mission[k] for k in ('scenario','seed','status','expectation_met','attempted')}), flush=True)
                    if not result.validated:
                        continue
                    for profile in args.profiles:
                        require_same_source()
                        settings = dict(PROFILES[profile])
                        offset = settings.pop('clock_offset', 0.)
                        drift = settings.pop('clock_drift', 0.)
                        bias = settings.pop('position_bias', 0.)
                        start = perf_counter()
                        report = simulate(result.plans, config,
                            clock_offsets=np.linspace(-offset, offset, 4), clock_drifts=np.linspace(-drift, drift, 4),
                            position_biases=position_bias_vectors([p.drone_id for p in result.plans], bias, 3),
                            disturbance_seed=args.disturbance_seed, **settings).report
                        require_same_source()
                        unchanged = signature(result) == digest
                        expected_rejection = profile == 'outside_clock'
                        met = unchanged and (not report['execution_validated'] if expected_rejection else report['execution_validated'])
                        write_record(stream, {'type': 'execution', 'scenario': name, 'seed': seed, 'profile': profile,
                            'expected_rejection': expected_rejection, 'expectation_met': met,
                            'same_nominal_plans': unchanged, 'signature_sha256': digest,
                            'execution_wall_s': perf_counter()-start, 'execution': report})
                        execution_count += 1
                        success &= met
                        print(json.dumps({'scenario': name, 'profile': profile, 'validated': report['execution_validated'],
                            'expectation_met': met, 'failures': report['execution_failures']}), flush=True)
                except Exception as error:
                    write_record(stream, {'type': 'error', 'scenario': name, 'seed': seed,
                        'error_type': type(error).__name__, 'message': str(error), 'accepted': False})
                    raise  # In particular, never accept partial worker output.
        write_record(stream, {'type': 'summary', 'missions': mission_count, 'executions': execution_count,
                              'all_expectations_met': bool(success)})
    print(f'SAVED {output}', flush=True)
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
