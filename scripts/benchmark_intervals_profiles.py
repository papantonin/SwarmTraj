"""Paired four-aircraft comparisons: free-window generation and rich 3D profiles."""

import argparse
from dataclasses import asdict, replace
from datetime import datetime
import json
import os
from pathlib import Path
import platform

import numpy as np

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import load_obstacles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--modes', nargs='+', choices=('baseline', 'windows', 'profiles', 'combined', 'mixed', 'mixed_windows',
                                                    'passages', 'mixed_passages'),
                        default=['baseline', 'windows', 'profiles', 'combined'])
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--departure-delay', type=float, nargs=2, default=(0., 22.), metavar=('MIN_S', 'MAX_S'))
    parser.add_argument('--target', type=float, nargs=2, default=(500., 500.))
    parser.add_argument('--activation-time', type=float, default=30.)
    parser.add_argument('--horizon', type=float, default=240.)
    parser.add_argument('--cruise-range', type=float, nargs=2, default=(30., 120.))
    parser.add_argument('--cruise-levels', type=int, default=2)
    parser.add_argument('--initial-candidates', type=int, default=2)
    parser.add_argument('--without-obstacles', action='store_true', help='Explicit obstacle-free comparison')
    parser.add_argument('--simulate', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--obstacles', type=Path, default=Path('examples/scenarios/static_obstacles.json'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.repeats < 1 or any(seed < 0 for seed in args.seeds):
        parser.error('repeats must be positive and seeds nonnegative')
    output = args.output or Path('results/benchmarks') / (
        'intervals-profiles-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    if output.exists():
        parser.error('output must be a new file')
    base = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target=tuple(args.target), target_altitude=30, horizon=args.horizon,
                  activation_time=args.activation_time, descent_angle_deg=30, max_descent_rate=10,
                  cruise_altitude_range=tuple(args.cruise_range), planning_strategy='adaptive', search_order='mrv',
                  initial_candidates=args.initial_candidates, selection_policy='max_margin', conflict_guidance='witness',
                  approach_spread_deg=180, obstacle_routing='spatial', reservation_policy='grid',
                  planning_workers=args.workers, obstacles=() if args.without_obstacles else load_obstacles(args.obstacles))
    base = replace(base, min_extra_loiter=args.departure_delay[0], max_extra_loiter=args.departure_delay[1])
    snapshots = tuple(replace(s, altitude=30+25*k) for k,s in enumerate(separate_loiter_snapshot(4)))
    document = {'num_drones': 4, 'python': platform.python_version(), 'numpy': np.__version__,
                'logical_cpus': os.cpu_count(), 'snapshots': [asdict(s) for s in snapshots],
                'execution_enabled': args.simulate, 'runs': []}
    success = True
    for seed in args.seeds:
        for repeat in range(args.repeats):
            for mode in (args.modes if repeat % 2 == 0 else args.modes[::-1]):
                rich = mode in ('profiles', 'combined', 'mixed', 'mixed_windows', 'passages', 'mixed_passages')
                config = replace(base, seed=seed,
                    profile_policy='adaptive_mixed' if mode in ('mixed', 'mixed_windows', 'mixed_passages') else 'strict',
                    interval_guidance='safe_windows' if mode in ('windows', 'combined', 'mixed_windows', 'passages', 'mixed_passages') else 'none',
                    passage_guidance='safe_windows' if mode in ('passages', 'mixed_passages') else 'none',
                    cruise_levels=args.cruise_levels if rich else 1, speed_profile='smooth' if rich else 'constant',
                    speed_profile_range=(10, 14) if rich else None)
                print(f'START seed={seed} mode={mode} repeat={repeat+1}', flush=True)
                result = plan_mission(snapshots, config)
                catalogs = result.diagnostics.get('catalogs', [])
                guided = [p for c in catalogs for p in c['interval_proposals']]
                passages = [p for c in catalogs for p in c['passage_proposals']]
                record = {'mode': mode, 'seed': seed, 'repeat': repeat+1, 'config': asdict(config),
                          'status': result.status, 'counts': result.counts, 'search_nodes': result.search_nodes,
                          'attempted': sum(c['attempted'] for c in catalogs),
                          'interval_guided_attempted': len(guided),
                          'interval_guided_accepted': sum(p['accepted'] for p in guided),
                          'passage_guided_attempted': len(passages),
                          'passage_guided_accepted': sum(p['accepted'] for p in passages),
                          'passage_guided_selected': sum(any(p['candidate_id'] == plan.candidate_id and p['accepted']
                              for p in c['passage_proposals']) for c, plan in zip(catalogs, result.plans)),
                          'selected_profile_families': [p.profile_family for p in result.plans],
                          'profile_attempted_by_family': {family: sum(p['family'] == family for c in catalogs
                              for p in c['profile_proposals']) for family in sorted({p['family'] for c in catalogs
                              for p in c['profile_proposals']})},
                          'timings': result.timings, 'diagnostics': result.diagnostics,
                          'signature_sha256': signature(result)}
                if result.validated and args.simulate:
                    record['execution'] = simulate(result.plans, config).report
                    success &= record['execution']['execution_validated']
                success &= result.validated
                document['runs'].append(record)
                print(json.dumps({k:record[k] for k in ('seed','mode','status','attempted','interval_guided_accepted',
                                                      'passage_guided_attempted','passage_guided_accepted')}
                                 | {'planning_s':result.timings['total_wall_s'],
                                    'execution_validated':record.get('execution',{}).get('execution_validated')}), flush=True)
    document['all_requested_checks_passed'] = bool(success)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f'SAVED {output}', flush=True)
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
