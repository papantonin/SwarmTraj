"""Paired single-aircraft physical-construction trials; keep every failure.

This is a bounded integration bench, not the complete fleet campaign. The
historical geometry and direct physical family receive identical mission and
catalog limits. Specific propulsive work is not a battery/endurance estimate.
"""

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import platform
from time import perf_counter

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import mission_document, validate_individual


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New artifact directory')
    parser.add_argument('--speeds', type=float, nargs='+', default=[12., 13., 14.])
    parser.add_argument('--horizon', type=float, default=120.)
    parser.add_argument('--attempts', type=int, default=8)
    parser.add_argument('--execute', action=argparse.BooleanOptionalAction, default=False,
                        help='Run nominal and bounded gust/sensor execution for every accepted physical mission')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output must be a new directory')
    base = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30., descent_angle_deg=30.,
                  max_descent_rate=8., horizon=args.horizon, candidates=1, attempts=args.attempts,
                  cruise_altitude_range=(30., 120.), max_extra_loiter=0.)
    jobs = []
    for target_name, target in (('near', (140., 50.)), ('far', (500., 0.))):
        for altitude in (30., 80.):
            for speed in args.speeds:
                snapshot = replace(separate_loiter_snapshot(1)[0], altitude=altitude).with_speed(speed, base.limits)
                for mode in ('geometric', 'physical_mission'):
                    config = replace(base, target=target, generator_mode=mode)
                    identifier = f'{target_name}-z{altitude:g}-v{speed:g}-{mode}'
                    jobs.append((identifier, snapshot, config))
    args.output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[1]
    files = [Path(__file__).resolve(), *sorted((root / 'src/swarm_sim/trajectory2d').glob('*.py'))]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    manifest = {'created_utc': datetime.now(timezone.utc).isoformat(), 'python': platform.python_version(),
        'platform': platform.platform(), 'source_sha256': hashes,
        'scope': 'paired bounded single-aircraft construction; no continuous-space infeasibility or PX4 claim',
        'timing_scope': 'planning only; host load is not isolated',
        'jobs': [{'id': name, 'snapshot': asdict(s), 'config': asdict(c)} for name, s, c in jobs]}
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    records = []
    for identifier, snapshot, config in jobs:
        before = perf_counter()
        result = plan_mission((snapshot,), config)
        elapsed = perf_counter() - before
        document = mission_document(result, config, (snapshot,))
        with gzip.open(args.output / f'{identifier}.json.gz', 'wt', encoding='utf-8') as stream:
            json.dump(document, stream, allow_nan=False)
        row = {'id': identifier, 'mode': config.generator_mode, 'status': result.status,
            'planning_wall_s': elapsed, 'signature_sha256': signature(result),
            'mission_sha256': document['sha256'], 'counts': result.counts,
            'diagnostics': result.diagnostics, 'executions': []}
        if result.validated:
            if not all(validate_individual(p, snapshot, config) for p in result.plans):
                raise RuntimeError('independent revalidation failed')
            row['independently_revalidated'] = True
            row['phases'] = result.plans[0].generation_phases
            if args.execute and config.generator_mode == 'physical_mission':
                for name, settings in (('nominal', {}), ('gust_sensor', dict(wind=.1,
                    wind_model='gust', clock_offsets=[.03], clock_drifts=[.0001],
                    position_biases=[[.03, 0., 0.]], position_noise=.01, disturbance_seed=23))):
                    report = simulate(result.plans, config, **settings).report
                    row['executions'].append({'profile': name, 'report': report})
        records.append(row)
        with (args.output / 'journal.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
        print(json.dumps({k: row[k] for k in ('id', 'status', 'planning_wall_s', 'counts')}), flush=True)
        if any(hashlib.sha256((root / p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
            raise RuntimeError('sources changed during bench')
    summary = {'jobs': len(records), 'validated': {mode: sum(r['mode'] == mode and r['status'] == 'validated'
        for r in records) for mode in ('geometric', 'physical_mission')},
        'execution_failures': [{'id': r['id'], 'profile': e['profile'], 'failures': e['report']['execution_failures']}
            for r in records for e in r['executions'] if not e['report']['execution_validated']]}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    print(json.dumps(summary), flush=True)
    return 2 if summary['execution_failures'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
