"""Reproducible 1–5 aircraft planning campaign with failure-preserving artifacts.

Runs one isolated mission at a time. External timeouts are censored observations,
never planner infeasibility statuses. Resume uses the immutable manifest and
source hashes. No planner, physical limit, or acceptance criterion is changed.
"""

import argparse
from collections import Counter
import copy
from dataclasses import asdict, replace
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import signal
import subprocess
import sys
from time import perf_counter
import traceback

import numpy as np
import scipy

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, Snapshot, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.dynamics import Limits
from swarm_sim.trajectory2d.flight3d import FlightLimits
from swarm_sim.trajectory2d.obstacles import Obstacle, load_obstacles
from swarm_sim.trajectory2d.planning import mission_document


ROOT = Path(__file__).resolve().parents[1]
FAMILIES = ('baseline', 'spread180', 'spread360', 'obstacles', 'short_horizon',
            'deadline', 'fixed_departure', 'high_separation', 'dense',
            'heterogeneous', 'rich_strict', 'grid')
MODES = ('adaptive_first', 'adaptive_margin', 'fixed_first', 'unguided',
         'id_order', 'rejection_feedback')


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def sources():
    files = [Path(__file__).resolve(), ROOT / 'scripts/benchmark_planning.py',
             *sorted((ROOT / 'src/swarm_sim/trajectory2d').glob('*.py')),
             *sorted((ROOT / 'native').rglob('*.cpp')), *sorted((ROOT / 'native').rglob('*.hpp')),
             ROOT / 'native/CMakeLists.txt', ROOT / 'setup.py', ROOT / 'scripts/build_native.sh']
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def check_sources(manifest):
    if sources() != manifest['source_sha256']:
        raise RuntimeError('campaign source changed; create a separate campaign')
    expected = manifest.get('native_backend_identity')
    if expected is not None:
        from swarm_sim.trajectory2d.native_backend import identity
        if identity() != expected:
            raise RuntimeError('native backend binary/build identity changed')


def base_case(n, seed):
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp',
        target_altitude=30, max_descent_rate=10,
        descent_angle_deg=30, cruise_altitude_range=(30, 120), seed=seed,
        planning_strategy='adaptive', initial_candidates=2, search_order='mrv',
        selection_policy='max_margin', conflict_guidance='witness', obstacle_routing='spatial')
    snapshots = tuple(replace(s, altitude=30+20*k)
                      for k, s in enumerate(separate_loiter_snapshot(n)))
    return snapshots, config


def family_case(n, seed, family):
    snapshots, config = base_case(n, seed)
    if family == 'spread180':
        config = replace(config, approach_spread_deg=180, horizon=150)
    elif family == 'spread360':
        config = replace(config, approach_spread_deg=360, horizon=240)
    elif family == 'obstacles':
        config = replace(config, target=(500, 500), horizon=210, approach_spread_deg=180,
            obstacles=load_obstacles(ROOT / 'examples/scenarios/static_obstacles.json'))
    elif family == 'short_horizon':
        config = replace(config, horizon=45)
    elif family == 'deadline':
        config = replace(config, horizon=150, arrival_deadline=45)
    elif family == 'fixed_departure':
        config = replace(config, max_extra_loiter=0, horizon=150, approach_spread_deg=180)
    elif family == 'high_separation':
        config = replace(config, separation=30, horizon=150, approach_spread_deg=180)
    elif family == 'dense':
        snapshots = tuple(replace(s, altitude=60) for s in separate_loiter_snapshot(n, spacing=60))
        config = replace(config, horizon=150, descent_angle_deg=0)
    elif family == 'heterogeneous':
        snapshots = tuple(s.with_speed(10+k, config.limits) for k, s in enumerate(snapshots))
        config = replace(config, horizon=150, approach_spread_deg=180)
    elif family == 'rich_strict':
        config = replace(config, target=(500, 500), horizon=210, approach_spread_deg=180,
            cruise_levels=2, speed_profile='smooth', speed_profile_range=(10, 14))
    elif family == 'grid':
        config = replace(config, horizon=150, approach_spread_deg=180,
            reservation_policy='grid', interval_guidance='safe_windows')
    elif family != 'baseline':
        raise ValueError(f'unknown family: {family}')
    return snapshots, config


def random_case(n, scene_seed):
    """Stratified input draws; seeds describe scenes and planner RNG separately."""
    rng = np.random.default_rng(scene_seed)
    snapshots, config = base_case(n, int(rng.integers(0, 1000000)))
    radius, spacing = float(rng.uniform(30, 65)), float(rng.uniform(150, 250))
    shift = rng.uniform(-100, 100, 2)
    angle = float(rng.uniform(-np.pi, np.pi))
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    snapshots = tuple(replace(s, state=(*map(float, rotation @ np.array(s.state[:2]) + shift),
        float(s.state[2]+angle), s.state[3]), altitude=float(rng.uniform(40, 115)))
        .with_speed(float(rng.uniform(10, 14)), config.limits)
        for s in separate_loiter_snapshot(n, radius=radius, spacing=spacing))
    # Keep mission-level draws comparable across aircraft counts even though
    # each fleet consumes a different number of initial-state draws.
    rng = np.random.default_rng(np.random.SeedSequence([scene_seed, 1]))
    target = tuple(map(float, rng.uniform((300, -250), (650, 550))))
    config = replace(config, target=target, target_altitude=float(rng.uniform(25, 60)),
        horizon=float(rng.choice((150, 180, 210))), descent_angle_deg=float(rng.choice((0, 15, 30))),
        approach_spread_deg=float(rng.choice((0, 90, 180, 360))),
        max_extra_loiter=float(rng.choice((0, 10, 22))), separation=float(rng.choice((8, 15, 30))))
    return snapshots, config


def make_jobs(counts, seeds, random_scenes, suites):
    jobs = []
    def add(suite, family, n, seed, snapshots, config, expected=None, **extra):
        jobs.append({'id': f'{suite}-{family}-n{n}-s{seed}', 'suite': suite, 'family': family,
            'num_drones': n, 'seed': seed, 'config': asdict(config),
            'snapshots': [asdict(s) for s in snapshots], 'expected_status': expected, **extra})
    if 'core' in suites:
        for family in FAMILIES:
            for seed in seeds:
                for n in counts:
                    add('core', family, n, seed, *family_case(n, seed, family))
    if 'random' in suites:
        for scene in range(random_scenes):
            for n in counts:
                add('random', 'stratified', n, 1000+scene, *random_case(n, 1000+scene))
    if 'ablation' in suites:
        for seed in seeds[:2]:
            for n in counts:
                snapshots, config = family_case(n, seed, 'spread180')
                for mode in MODES:
                    changes = {'adaptive_first': {'selection_policy': 'first'},
                        'adaptive_margin': {}, 'fixed_first': {'planning_strategy': 'fixed', 'selection_policy': 'first'},
                        'unguided': {'conflict_guidance': 'none'}, 'id_order': {'search_order': 'id'},
                        'rejection_feedback': {'candidate_guidance': 'rejections'}}[mode]
                    add('ablation', mode, n, seed, snapshots, replace(config, **changes),
                        comparison_group=f'spread180-n{n}-s{seed}')
    if 'replay' in suites:
        for n in counts:
            snapshots, config = base_case(n, seeds[0])
            for mode, workers in (('serial', 1), ('parallel2', 2)):
                add('replay', mode, n, seeds[0], snapshots, replace(config, planning_workers=workers),
                    comparison_group=f'baseline-n{n}-s{seeds[0]}')
    if 'controls' in suites:
        for n in counts:
            snapshots, config = base_case(n, seeds[0])
            add('controls', 'unreachable', n, seeds[0], snapshots,
                replace(config, target=(100000, 100000)), 'destination_unreachable_within_horizon')
            bad = (replace(snapshots[0], speed=100), *snapshots[1:])
            add('controls', 'invalid_speed', n, seeds[0], bad, config, 'invalid_initial_state')
            obstacle = Obstacle.cylinder(*config.target, 25, 0, 80)
            add('controls', 'blocked_target', n, seeds[0], snapshots,
                replace(config, obstacles=(obstacle,)), 'target_region_obstructed')
            if n > 1:
                duplicate = (snapshots[0], replace(snapshots[0], drone_id=1), *snapshots[2:])
                add('controls', 'initial_collision', n, seeds[0], duplicate, config, 'initial_separation_violation')
    return jobs


def decode(job):
    fields = dict(job['config'])
    fields.setdefault('dimensions', 2)
    fields.setdefault('generator_mode', 'geometric')
    fields.setdefault('numerical_backend', 'python')
    if (fields['generator_mode'] not in ('geometric', 'physical_mission', 'physical_recovery_seed')
            or fields['numerical_backend'] not in ('python', 'cpp')):
        raise ValueError('Saved campaign requires explicit planner selections.')
    fields['limits'] = Limits(**fields['limits'])
    fields['flight'] = FlightLimits(**fields['flight'])
    fields['obstacles'] = tuple(Obstacle(**o) for o in fields['obstacles'])
    fields['aircraft_profiles'] = ()
    for key in ('target', 'cruise_altitude_range', 'speed_profile_range'):
        if fields[key] is not None:
            fields[key] = tuple(fields[key])
    snapshots = tuple(Snapshot(**{**s, 'state': tuple(s['state'])}) for s in job['snapshots'])
    return snapshots, Config(**fields)


def replay_jobs(reference, suites, generator_mode, numerical_backend=None):
    """Copy saved inputs, applying only explicit generator/backend overrides."""
    jobs = copy.deepcopy([j for j in reference['jobs'] if j['suite'] in suites])
    if not jobs or len({j['id'] for j in jobs}) != len(jobs):
        raise ValueError('reference must contain a nonempty unique job selection')
    for job in jobs:
        # Missing fields retain historical meaning independently of fresh defaults.
        job['config'].setdefault('generator_mode', 'geometric')
        job['config'].setdefault('numerical_backend', 'python')
        if generator_mode is not None:
            job['config']['generator_mode'] = generator_mode
        if numerical_backend is not None:
            job['config']['numerical_backend'] = numerical_backend
    return jobs


def select_jobs(jobs, job_ids=None, planning_workers=None):
    """Explicit subset and process-count overrides; never alter mission limits.

    Dedicated serial/spawn replay pairs retain their saved worker assignments.
    Returned records are copies, so the reference manifest remains immutable.
    """
    if planning_workers is not None and planning_workers < 0:
        raise ValueError('planning_workers must be nonnegative')
    if job_ids is not None:
        if len(set(job_ids)) != len(job_ids) or not job_ids:
            raise ValueError('job IDs must be nonempty and unique')
        unknown = set(job_ids) - {j['id'] for j in jobs}
        if unknown:
            raise ValueError(f'unknown or excluded job IDs: {sorted(unknown)}')
    selected = copy.deepcopy([j for j in jobs if job_ids is None or j['id'] in job_ids])
    if planning_workers is not None:
        for job in selected:
            if job['suite'] != 'replay':
                job['config']['planning_workers'] = planning_workers
    return selected


def unsupported_generator_configuration(job, error):
    fields = job['config']
    return (fields.get('generator_mode', 'geometric') in ('physical_mission', 'physical_recovery_seed')
            and str(error) in ('physical generator_mode requires coupled 3D flight',
                              'physical generator_mode does not support geometric passage-window fitting'))


def record_configuration_rejection(directory, job, error):
    """Record an actual input-validation rejection, without constructing a plan."""
    document = {'schema': 'swarm-campaign-configuration-rejection-v1',
        'config': job['config'], 'snapshots': job['snapshots'],
        'status': 'unsupported_generator_configuration', 'reason': str(error),
        'counts': [], 'search_nodes': 0, 'minimum_distance_lower_bound': None,
        'required_distance': None, 'plans': [],
        'planning_diagnostics': {'stage': 'configuration', 'planning_executed': False}}
    payload = {key: document[key] for key in ('status', 'counts', 'search_nodes',
        'minimum_distance_lower_bound', 'required_distance', 'plans')}
    digest = hashlib.sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest()
    document['sha256'] = digest
    with gzip.open(directory / 'missions' / f'{job["id"]}.json.gz', 'xt', encoding='utf-8') as stream:
        json.dump(document, stream, allow_nan=False)
    record = {'type': 'result', 'id': job['id'], 'status': document['status'],
        'reason': document['reason'], 'counts': [], 'search_nodes': 0,
        'diagnostics': document['planning_diagnostics'], 'mission_sha256': digest,
        'signature_sha256': hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest(),
        'attempted': 0, 'accepted_candidates': 0, 'executions': []}
    write_json(directory / 'completed' / f'{job["id"]}.json', record)


def worker(directory, job_id):
    manifest = json.loads((directory / 'manifest.json').read_text())
    job = next(j for j in manifest['jobs'] if j['id'] == job_id)
    check_sources(manifest)
    try:
        snapshots, config = decode(job)
    except ValueError as error:
        if not unsupported_generator_configuration(job, error):
            raise
        record_configuration_rejection(directory, job, error)
        return
    start = perf_counter()
    result = plan_mission(snapshots, config)
    planning_s = perf_counter() - start
    usage = resource.getrusage(resource.RUSAGE_SELF)
    children = resource.getrusage(resource.RUSAGE_CHILDREN)
    document = mission_document(result, config, snapshots)
    with gzip.open(directory / 'missions' / f'{job_id}.json.gz', 'xt', encoding='utf-8') as stream:
        json.dump(document, stream, allow_nan=False)
    catalogs = result.diagnostics.get('catalogs', [])
    pairs = result.diagnostics.get('selection', {}).get('selected_pairs', [])
    record = {'type': 'result', 'id': job_id, 'status': result.status, 'reason': result.reason,
        'planning_wall_s': planning_s, 'timings': result.timings, 'counts': result.counts,
        'search_nodes': result.search_nodes, 'diagnostics': result.diagnostics,
        'signature_sha256': signature(result), 'mission_sha256': document['sha256'],
        'attempted': sum(c['attempted'] for c in catalogs),
        'accepted_candidates': sum(c['accepted'] for c in catalogs),
        'minimum_distance_lower_bound_m': result.minimum_distance_lower_bound,
        'required_distance_m': result.required_distance,
        'minimum_pair_slack_m': min((p['extra_margin_m'] for p in pairs), default=None),
        'arrival_times_s': [p.arrival_time for p in result.plans],
        'release_times_s': [p.release_time for p in result.plans],
        'path_lengths_m': [float(np.linalg.norm(np.diff(p.positions, axis=0), axis=1).sum()) for p in result.plans],
        'self_peak_rss_kib': usage.ru_maxrss, 'largest_child_peak_rss_kib': children.ru_maxrss,
        'cpu_s_including_imports_and_children': usage.ru_utime+usage.ru_stime+children.ru_utime+children.ru_stime,
        'executions': []}
    # Save planning completion before optional execution, so a later timeout
    # cannot hide a completed plan. The parent retains the checkpoint verbatim.
    write_json(directory / 'checkpoints' / f'{job_id}.json', record)
    if (manifest['execute'] and result.validated and job['suite'] == 'core'
            and job['seed'] == manifest['seeds'][0]
            and job['family'] in manifest.get('execution_families', ('baseline', 'obstacles', 'heterogeneous'))):
        from swarm_sim.trajectory2d.execution import simulate
        from swarm_sim.trajectory2d.perturbations import position_bias_vectors
        for profile in ('nominal', 'gust_sensor', 'outside_clock'):
            before = perf_counter()
            settings = {}
            if profile == 'gust_sensor':
                settings = {'wind': .1, 'wind_model': 'gust', 'wind_heading_deg': 15.,
                    'clock_offsets': np.linspace(-.03, .03, len(snapshots)),
                    'clock_drifts': np.linspace(-.0001, .0001, len(snapshots)),
                    'position_biases': position_bias_vectors([p.drone_id for p in result.plans], .05, 3),
                    'position_noise': .01, 'disturbance_seed': 23}
            elif profile == 'outside_clock':
                settings = {'clock_offsets': np.full(len(snapshots), .4)}
            report = simulate(result.plans, config,
                execution_backend=manifest.get('execution_backend', 'python'), **settings).report
            unchanged = signature(result) == record['signature_sha256']
            record['executions'].append({'profile': profile, 'execution_wall_s': perf_counter()-before,
                'same_plans': unchanged, 'expected_acceptance': profile != 'outside_clock',
                'expectation_met': unchanged and report['execution_validated'] == (profile != 'outside_clock'),
                'report': report})
            # Atomic replace of our own checkpoint; no user input is overwritten.
            temp = directory / 'checkpoints' / f'{job_id}.tmp'
            temp.write_text(json.dumps(record, allow_nan=False))
            temp.replace(directory / 'checkpoints' / f'{job_id}.json')
    check_sources(manifest)
    write_json(directory / 'completed' / f'{job_id}.json', record)


def run(directory, manifest):
    journal = directory / 'journal.jsonl'
    previous = [json.loads(line) for line in journal.read_text().splitlines()] if journal.exists() else []
    done = {r['id'] for r in previous}
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for index, job in enumerate(manifest['jobs'], 1):
        if job['id'] in done:
            continue
        check_sources(manifest)
        job_id = job['id']
        # A resumed, interrupted job may have partial own files. Preserve them
        # in a numbered attempt directory before trying it again.
        stale = [(directory / p / f'{job_id}{suffix}') for p, suffix in
                 (('missions', '.json.gz'), ('checkpoints', '.json'), ('completed', '.json'), ('logs', '.log'))]
        existing = [p for p in stale if p.exists()]
        if existing:
            attempt = directory / 'interrupted' / f'{job_id}-{datetime.now().strftime("%Y%m%d%H%M%S%f")}'
            attempt.mkdir(parents=True)
            for path in existing:
                path.rename(attempt / path.name)
        print(f'START {index}/{len(manifest["jobs"])} {job_id}', flush=True)
        started = datetime.now(timezone.utc).isoformat()
        start = perf_counter()
        with (directory / 'logs' / f'{job_id}.log').open('x') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker',
                job_id, '--output', str(directory)], stdout=log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True)
            timed_out = False
            try:
                process.wait(timeout=manifest['timeout_s'])
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        elapsed = perf_counter()-start
        completed = directory / 'completed' / f'{job_id}.json'
        checkpoint = directory / 'checkpoints' / f'{job_id}.json'
        if completed.exists() and process.returncode == 0 and not timed_out:
            record = json.loads(completed.read_text())
            record['completion'] = 'complete'
        elif checkpoint.exists():
            record = json.loads(checkpoint.read_text())
            record['completion'] = 'execution_timeout' if timed_out else 'worker_error_after_planning'
        else:
            record = {'id': job_id, 'type': 'result', 'completion': 'planning_timeout' if timed_out else 'worker_error',
                      'status': None}
        record.update({k: job[k] for k in ('suite', 'family', 'num_drones', 'seed', 'expected_status')})
        record.update(started_utc=started, process_wall_s=elapsed, returncode=process.returncode,
                      timeout_s=manifest['timeout_s'])
        if job['expected_status'] is not None:
            record['expectation_met'] = record['completion'] == 'complete' and record['status'] == job['expected_status']
        with journal.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, allow_nan=False)+'\n')
            stream.flush()
            os.fsync(stream.fileno())
        print(json.dumps({'id': job_id, 'status': record['status'], 'completion': record['completion'],
            'seconds': round(elapsed, 3), 'attempted': record.get('attempted')}), flush=True)
        check_sources(manifest)
        if 'worker_error' in record['completion']:
            raise RuntimeError(f'worker failed: inspect {directory / "logs" / (job_id+".log")}')
    rows = [json.loads(line) for line in journal.read_text().splitlines()]
    if len(rows) != len(manifest['jobs']) or len({r['id'] for r in rows}) != len(rows):
        raise RuntimeError('journal coverage/uniqueness check failed')
    print('COMPLETE '+json.dumps(dict(Counter(r['status'] or r['completion'] for r in rows))), flush=True)
    return 2 if any(r.get('expectation_met') is False or any(not e['expectation_met']
        for e in r.get('executions', [])) for r in rows) else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--counts', type=int, nargs='+', default=[1, 2, 3, 4, 5])
    parser.add_argument('--seeds', type=int, nargs='+', default=[7, 19, 42])
    parser.add_argument('--random-scenes', type=int, default=10)
    parser.add_argument('--suites', nargs='+', choices=('controls', 'core', 'random', 'ablation', 'replay'),
                        default=['controls', 'core', 'random', 'ablation', 'replay'])
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--execute', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--execution-backend', choices=('cpp',), default='cpp',
                        help='Explicit local execution backend for accepted saved plans; observer stays Python')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--reference', type=Path, help='Replay saved manifest inputs instead of regenerating scenes')
    parser.add_argument('--generator-mode', choices=('physical_mission',),
                        help='Explicit generator override; all other saved mission inputs stay unchanged')
    parser.add_argument('--numerical-backend', choices=('cpp',),
                        help='Explicit physical-planner backend override; cpp requires coupled 3D physical generation')
    parser.add_argument('--planning-workers', type=int,
                        help='Explicit catalog process override, except saved serial/spawn replay pairs')
    parser.add_argument('--execution-families', nargs='+', choices=FAMILIES,
                        help='Identical execution coverage can be requested for both generators')
    parser.add_argument('--jobs', nargs='+', help='Explicit saved job IDs; subsets never qualify a default promotion')
    parser.add_argument('--worker', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    directory = args.output.resolve()
    if args.worker:
        try:
            worker(directory, args.worker)
        except Exception:
            traceback.print_exc()
            return 1
        return 0
    if (not args.counts or any(n not in range(1, 6) for n in args.counts)
            or not args.seeds or min(args.seeds) < 0 or args.random_scenes < 0
            or not np.isfinite(args.timeout) or args.timeout <= 0
            or args.planning_workers is not None and args.planning_workers < 0):
        parser.error('counts must be 1–5; seeds/scenes nonnegative; timeout positive and finite')
    for name in ('counts', 'seeds', 'suites', 'execution_families'):
        values = getattr(args, name)
        if values is not None and len(set(values)) != len(values):
            parser.error(f'duplicate {name} are not allowed')
    if args.resume:
        manifest = json.loads((directory / 'manifest.json').read_text())
        check_sources(manifest)
    else:
        if directory.exists():
            parser.error('output directory must be new (or use --resume)')
        directory.mkdir(parents=True)
        for folder in ('missions', 'checkpoints', 'completed', 'logs'):
            (directory / folder).mkdir()
        # Controls first, followed by balanced families and seed/count sweeps.
        jobs = make_jobs(args.counts, args.seeds, args.random_scenes, ['controls'] if args.suites == ['controls'] else [])
        if args.suites != ['controls']:
            jobs = make_jobs(args.counts, args.seeds, args.random_scenes, args.suites)
            jobs.sort(key=lambda j: j['suite'] != 'controls')
        reference = None
        if args.reference:
            reference = json.loads(args.reference.read_text())
            jobs = replay_jobs(reference, args.suites, args.generator_mode, args.numerical_backend)
            args.counts, args.seeds = reference['counts'], reference['seeds']
            args.timeout = reference['timeout_s']
        elif args.generator_mode is not None:
            for job in jobs:
                job['config']['generator_mode'] = args.generator_mode
        if args.numerical_backend is not None:
            for job in jobs:
                job['config']['numerical_backend'] = args.numerical_backend
        try:
            jobs = select_jobs(jobs, args.jobs, args.planning_workers)
        except ValueError as error:
            parser.error(str(error))
        manifest = {'schema': 'swarm-scaling-campaign-v1', 'created_utc': datetime.now(timezone.utc).isoformat(),
            'source_sha256': sources(), 'python': platform.python_version(), 'numpy': np.__version__,
            'scipy': scipy.__version__, 'platform': platform.platform(), 'logical_cpus': os.cpu_count(),
            'cpu_info': subprocess.check_output(['lscpu'], text=True),
            'mem_info': Path('/proc/meminfo').read_text(), 'counts': args.counts, 'seeds': args.seeds,
            'timeout_s': args.timeout, 'execute': args.execute, 'execution_backend': args.execution_backend, 'jobs': jobs,
            'scope': 'coupled point-mass planner; no Gazebo/PX4; serialized jobs; no hard real-time guarantee',
            'timing_scope': 'planning_wall_s excludes imports/export/execution; process_wall_s includes all',
            'memory_scope': 'Linux ru_maxrss: parent peak and largest child peak, not simultaneous fleet RSS',
            'execution_families': args.execution_families or (['baseline', 'obstacles', 'heterogeneous', 'rich_strict', 'grid']
                                   if args.generator_mode in ('physical_mission', 'physical_recovery_seed')
                                   else ['baseline', 'obstacles', 'heterogeneous']),
            'blas_threads': 1}
        if reference is not None:
            manifest['reference'] = {'path': str(args.reference.resolve()),
                'sha256': hashlib.sha256(args.reference.read_bytes()).hexdigest(),
                'generator_mode_override': args.generator_mode,
                'numerical_backend_override': args.numerical_backend,
                'planning_workers_override': args.planning_workers,
                'job_ids': args.jobs,
                'scope': 'same saved snapshots, mission constraints and timeout; explicit generator/backend/process overrides only'}
        if args.execution_backend == 'cpp' or any(job['config'].get('numerical_backend', 'python') == 'cpp' for job in jobs):
            from swarm_sim.trajectory2d.native_backend import identity
            manifest['native_backend_identity'] = identity()
        write_json(directory / 'manifest.json', manifest)
    return run(directory, manifest)


if __name__ == '__main__':
    raise SystemExit(main())
