"""Four-drone execution campaigns; reuse identical plans across disturbances.

Failures are retained. Outside-bounds trials must be rejected, not relabeled
as safe. These finite experiments do not establish a robust invariant envelope.
"""

import argparse
from dataclasses import asdict, replace
from datetime import datetime
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import load_obstacles
from swarm_sim.trajectory2d.perturbations import position_bias_vectors


PROFILES = {
    'nominal': {},
    'regression': {'wind': .05, 'clock_offset': .05},
    'constant': {'wind': .2, 'wind_model': 'constant', 'wind_heading_deg': 35., 'clock_offset': .05},
    'gust_sensor': {'wind': .2, 'wind_model': 'gust', 'wind_heading_deg': 15.,
                    'clock_offset': .05, 'position_bias': .1, 'position_noise': .02},
    'clock_drift': {'wind': .1, 'wind_model': 'constant', 'wind_heading_deg': -20.,
                    'clock_offset': .03, 'clock_drift': .0002},
    'outside_bounds': {'wind': 1., 'wind_model': 'gust', 'clock_offset': .4,
                       'position_bias': 3., 'position_noise': .2, 'clock_drift': .002},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7, 19])
    parser.add_argument('--scenarios', nargs='+', default=['default_90s', 'spread180_150s', 'spread360_240s'],
                        choices=['default_90s', 'spread180_150s', 'spread360_240s', 'obstacles_180s'])
    parser.add_argument('--profiles', nargs='+', choices=list(PROFILES), default=list(PROFILES))
    parser.add_argument('--controllers', nargs='+', choices=['constrained', 'legacy'], default=['constrained'])
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--disturbance-seed', type=int, default=0)
    parser.add_argument('--baseline', type=Path, help='Earlier benchmark_adaptive JSON: compare exact plans and regression outcomes')
    parser.add_argument('--obstacles', type=Path, default=Path('examples/scenarios/static_obstacles.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if min(args.seeds) < 0 or args.disturbance_seed < 0 or args.workers < 0:
        parser.error('seeds and workers must be nonnegative')
    output = args.output or Path('results/benchmarks') / ('tracking-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    if output.exists():
        parser.error('output must be a new file')
    baseline = {}
    if args.baseline:
        previous = json.loads(args.baseline.read_text())
        baseline = {(r['scenario'], r['seed']): r for r in previous['runs'] if r['mode'] == 'adaptive_mrv'}
    base = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target_altitude=30, max_descent_rate=10, descent_angle_deg=30,
                  cruise_altitude_range=(30, 120), planning_workers=args.workers,
                  planning_strategy='adaptive', search_order='mrv')
    scenarios = {'default_90s': base,
                 'spread180_150s': replace(base, horizon=150, approach_spread_deg=180),
                 'spread360_240s': replace(base, horizon=240, approach_spread_deg=360)}
    if 'obstacles_180s' in args.scenarios:
        scenarios['obstacles_180s'] = replace(base, horizon=180, target=(500, 500),
            approach_spread_deg=180, obstacle_routing='spatial', obstacles=load_obstacles(args.obstacles),
            selection_policy='max_margin')
    snapshots = tuple(replace(s, altitude=30 + 25*k) for k, s in enumerate(separate_loiter_snapshot(4)))
    document = {'num_drones': 4, 'numpy_version': np.__version__, 'profiles': PROFILES,
                'disturbance_seed': args.disturbance_seed, 'baseline': str(args.baseline) if args.baseline else None,
                'missions': [], 'runs': []}
    success = True
    for scenario in args.scenarios:
        for seed in args.seeds:
            config = replace(scenarios[scenario], seed=seed)
            result = plan_mission(snapshots, config)
            digest = signature(result)
            old = baseline.get((scenario, seed))
            same = digest == old['signature_sha256'] if old else None
            document['missions'].append({'scenario': scenario, 'seed': seed, 'config': asdict(config),
                'snapshots': [asdict(s) for s in snapshots], 'status': result.status,
                'signature_sha256': digest, 'matches_baseline_plan': same})
            success &= result.validated and same is not False
            if not result.validated:
                print(f'PLANNING FAILURE {scenario} seed={seed}: {result.status}', flush=True)
                continue
            for controller in args.controllers:
                for name in args.profiles:
                    settings = dict(PROFILES[name])
                    offset = settings.pop('clock_offset', 0.)
                    drift = settings.pop('clock_drift', 0.)
                    bias = settings.pop('position_bias', 0.)
                    start = perf_counter()
                    evaluation = simulate(result.plans, config, controller=controller,
                        clock_offsets=np.linspace(-offset, offset, 4), clock_drifts=np.linspace(-drift, drift, 4),
                        position_biases=position_bias_vectors([p.drone_id for p in result.plans], bias, 3),
                        disturbance_seed=args.disturbance_seed, **settings)
                    report = evaluation.report
                    expected_rejection = name == 'outside_bounds'
                    met = not report['execution_validated'] if expected_rejection else report['execution_validated']
                    record = {'scenario': scenario, 'seed': seed, 'controller': controller, 'profile': name,
                        'execution_wall_s': perf_counter() - start, 'expected_rejection': expected_rejection,
                        'expectation_met': met, 'execution': report}
                    if name == 'regression' and old:
                        record['baseline_execution_validated'] = old['execution_validated']
                        record['baseline_execution_failures'] = old['execution_failures']
                    document['runs'].append(record)
                    success &= met
                    print(json.dumps({'scenario': scenario, 'seed': seed, 'controller': controller,
                        'profile': name, 'validated': report['execution_validated'], 'expectation_met': met,
                        'failures': report['execution_failures'], 'first_event': report['first_safety_event']}), flush=True)
    document['all_expectations_met'] = bool(success)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f'SAVED {output}', flush=True)
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
