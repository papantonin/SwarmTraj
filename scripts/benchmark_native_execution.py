"""Replay immutable saved plans through Python/C++ local execution and Python observers.

Fresh, serialized processes measure full simulate(), including unchanged global
evaluation. This is offline point-mass execution, not PX4/Gazebo flight latency.
"""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
from time import perf_counter, process_time

import numpy as np

from benchmark_scaling_campaign import decode
from diagnose_campaign_execution import load_plan
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.native_backend import identity, require_execution_native
from swarm_sim.trajectory2d.perturbations import position_bias_vectors

ROOT = Path(__file__).resolve().parents[1]
PROFILES = ('nominal', 'gust_sensor', 'outside_clock')
ARRAY_TOLERANCE = 1e-8


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sources():
    paths = [*sorted((ROOT / 'src/swarm_sim/trajectory2d').glob('*.py')),
        *sorted((ROOT / 'native').rglob('*.cpp')), *sorted((ROOT / 'native').rglob('*.hpp')),
        ROOT / 'native/CMakeLists.txt', ROOT / 'setup.py', Path(__file__).resolve(),
        ROOT / 'scripts/benchmark_scaling_campaign.py', ROOT / 'scripts/diagnose_campaign_execution.py']
    return {str(p.relative_to(ROOT)): digest(p) for p in paths}


def load_document(path):
    path = Path(path)
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        document = json.load(stream)
    canonical = {k: v for k, v in document.items() if k != 'sha256'}
    expected = hashlib.sha256(json.dumps(canonical, sort_keys=True, allow_nan=False).encode()).hexdigest()
    if document.get('sha256') != expected:
        raise ValueError(f'mission hash mismatch: {path}')
    # decode() is the common-airframe campaign decoder. Do not silently discard
    # per-ID profiles; those are covered by dedicated own-physics regressions.
    if document.get('aircraft_profiles') or document['config'].get('aircraft_profiles'):
        raise ValueError('this saved-campaign benchmark supports common-airframe missions only')
    _, config = decode(document)
    plans = tuple(load_plan(p) for p in document['plans'])
    if not plans or [p.to_dict() for p in plans] != document['plans']:
        raise ValueError('saved plans failed exact reconstruction')
    return document, config, plans


def settings(profile, plans):
    if profile == 'nominal':
        return {}
    if profile == 'outside_clock':
        return {'clock_offsets': np.full(len(plans), .4)}
    if profile != 'gust_sensor':
        raise ValueError('unknown execution profile')
    return dict(wind=.1, wind_model='gust', wind_heading_deg=15.,
        clock_offsets=np.linspace(-.03, .03, len(plans)),
        clock_drifts=np.linspace(-.0001, .0001, len(plans)),
        position_biases=position_bias_vectors([p.drone_id for p in plans], .05, 3),
        position_noise=.01, disturbance_seed=23)


def check_frozen(manifest):
    if sources() != manifest['sources'] or identity() != manifest['native']:
        raise RuntimeError('execution sources or native binary changed')
    for item in manifest['missions']:
        if digest(item['path']) != item['file_sha256']:
            raise RuntimeError('saved mission changed')


def worker(output, index):
    manifest = json.loads((output / 'manifest.json').read_text())
    check_frozen(manifest)
    trial = manifest['trials'][index]
    document, config, plans = load_document(trial['mission'])
    options = settings(trial['profile'], plans)
    start, cpu = perf_counter(), process_time()
    evaluation = simulate(plans, config, execution_backend=trial['backend'], **options)
    wall_s, cpu_s = perf_counter() - start, process_time() - cpu
    if [p.to_dict() for p in plans] != document['plans']:
        raise RuntimeError('execution mutated the immutable plans')
    arrays = {k: getattr(evaluation, k) for k in
        ('times', 'flight_actual', 'actual', 'nominal', 'distances', 'tracking_errors')}
    array_bytes = sum(a.nbytes for a in arrays.values())
    np.savez_compressed(output / f'{index}.npz', **arrays)
    row = {**trial, 'seconds': wall_s, 'cpu_s': cpu_s,
        'peak_rss_bytes': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        'evaluation_array_bytes': array_bytes, 'mission_sha256': document['sha256'],
        'report': evaluation.report}
    check_frozen(manifest)
    (output / f'{index}.json').write_text(json.dumps(row, indent=2, allow_nan=False))


def compare_reports(a, b, path='report'):
    """All discrete decisions/events exact; numeric observer results tolerate rounding."""
    errors = []
    excluded = {'execution_backend', 'execution_backend_identity'}
    if isinstance(a, dict) and isinstance(b, dict):
        keys_a, keys_b = a.keys() - excluded, b.keys() - excluded
        if keys_a != keys_b:
            return [path + ': keys differ']
        for key in sorted(keys_a):
            errors.extend(compare_reports(a[key], b[key], path + '.' + key))
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [path + ': length differs']
        for i, (x, y) in enumerate(zip(a, b)):
            errors.extend(compare_reports(x, y, path + f'[{i}]'))
    elif type(a) is not type(b):
        errors.append(path + ': types differ')
    elif isinstance(a, float):
        if not np.isclose(a, b, atol=ARRAY_TOLERANCE, rtol=1e-10):
            errors.append(path + f': {a} != {b}')
    elif a != b:
        errors.append(path + f': {a} != {b}')
    return errors


def compare_array(a, b, name, single_aircraft):
    """Only the absent-pair distance sentinel may be infinite; states must be finite."""
    if a.shape != b.shape:
        return None, name + ': shape differs'
    if name == 'distances' and single_aircraft:
        if not (np.all(np.isposinf(a)) and np.all(np.isposinf(b))):
            return None, 'single-aircraft absent-pair distance sentinel differs'
        return 0., None
    if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
        return None, name + ': nonfinite numerical data'
    difference = float(np.max(np.abs(a - b), initial=0.))
    return difference, name + ': exceeds absolute 1e-8 tolerance' if difference > ARRAY_TOLERANCE else None


def summarize(output, manifest, rows, *, filename='summary.json'):
    errors, pairs = [], []
    for i, trial in enumerate(manifest['trials']):
        if trial['backend'] != 'cpp':
            continue
        j = next(j for j, t in enumerate(manifest['trials']) if
            (t['mission'], t['profile'], t['repeat'], t['backend']) ==
            (trial['mission'], trial['profile'], trial['repeat'], 'python'))
        a, b = rows[j], rows[i]
        if a.get('completion') != 'complete' or b.get('completion') != 'complete':
            errors.append(f'{i}: incomplete pair')
            continue
        pair_errors = compare_reports(a['report'], b['report'])
        delta = 0.
        with np.load(output / f'{j}.npz') as x, np.load(output / f'{i}.npz') as y:
            if set(x.files) != set(y.files):
                pair_errors.append('array names differ')
            else:
                for name in x.files:
                    difference, error = compare_array(x[name], y[name], name,
                        len(a['report']['controller_diagnostics']) == 1)
                    if difference is not None:
                        delta = max(delta, difference)
                    if error:
                        pair_errors.append(error)
        historical = trial.get('historical')
        if historical is not None:
            for backend_row in (a, b):
                for key in ('execution_validated', 'execution_failures'):
                    if backend_row['report'][key] != historical[key]:
                        pair_errors.append('historical ' + key + ' differs')
        errors.extend(f'{i}: {error}' for error in pair_errors)
        pairs.append({k: trial[k] for k in ('id', 'profile', 'repeat')} | dict(
            python_s=a['seconds'], cpp_s=b['seconds'], speedup=a['seconds'] / b['seconds'],
            python_cpu_s=a['cpu_s'], cpp_cpu_s=b['cpu_s'],
            python_peak_rss_bytes=a['peak_rss_bytes'], cpp_peak_rss_bytes=b['peak_rss_bytes'],
            evaluation_array_bytes=b['evaluation_array_bytes'], max_array_delta=delta,
            accepted=b['report']['execution_validated'], errors=pair_errors))
    result = {'scope': manifest['scope'], 'absolute_array_tolerance': ARRAY_TOLERANCE,
        'pairs': pairs, 'errors': errors, 'passed': not errors}
    (output / filename).write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def reanalyze(output):
    """Recheck saved measurements after an analysis-only change; preserve old results."""
    manifest = json.loads((output / 'manifest.json').read_text())
    current = sources()
    script = str(Path(__file__).resolve().relative_to(ROOT))
    changed = sorted(k for k in current.keys() | manifest['sources'].keys()
                     if current.get(k) != manifest['sources'].get(k))
    if set(changed) - {script} or identity() != manifest['native']:
        raise RuntimeError('reanalysis allows only this analysis script to change; runtime must match')
    for item in manifest['missions']:
        if digest(item['path']) != item['file_sha256']:
            raise RuntimeError('saved mission changed')
    rows = [json.loads(line) for line in (output / 'journal.jsonl').read_text().splitlines()]
    if len(rows) != len(manifest['trials']):
        raise RuntimeError('incomplete execution journal')
    artifacts = {}
    for i, (row, trial) in enumerate(zip(rows, manifest['trials'])):
        if any(row.get(k) != v for k, v in trial.items()):
            raise RuntimeError('journal trial differs from manifest')
        if row['completion'] == 'complete':
            if row != json.loads((output / f'{i}.json').read_text()) | {'completion': 'complete'}:
                raise RuntimeError('worker record differs from journal')
            artifacts[f'{i}.npz'] = digest(output / f'{i}.npz')
    filename = 'reanalysis.json'
    if (output / filename).exists():
        raise ValueError('reanalysis output already exists; preserve the prior analysis')
    result = summarize(output, manifest, rows, filename=filename)
    result['analysis_provenance'] = dict(changed_sources=changed,
        measurement_script_sha256=manifest['sources'][script], analysis_script_sha256=current[script],
        artifact_sha256=artifacts, original_manifest_sha256=digest(output / 'manifest.json'),
        original_journal_sha256=digest(output / 'journal.jsonl'),
        scope='Same saved worker measurements; only comparison/analysis changed; original summary preserved')
    (output / filename).write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument('--missions', type=Path, nargs='+')
    inputs.add_argument('--campaign', type=Path, help='Saved campaign with completed execution expectations')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', nargs='+', choices=PROFILES, default=list(PROFILES))
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--worker', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--reanalyze', action='store_true',
        help='Recheck existing measurements after an analysis-only edit; preserve original summary')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if args.worker is not None:
        return worker(output, args.worker)
    if args.reanalyze:
        result = reanalyze(output)
        print(json.dumps({'passed': result['passed'], 'pairs': len(result['pairs']), 'errors': result['errors']}))
        return 0 if result['passed'] else 1
    if args.repeats < 1 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('repeats and timeout must be positive')
    if len(set(args.profiles)) != len(args.profiles):
        parser.error('profiles must be unique')
    if not (args.missions or args.campaign) or output.exists():
        parser.error('provide saved missions/campaign and a new output directory')
    require_execution_native()
    missions = []
    if args.campaign:
        for path in sorted((args.campaign / 'completed').glob('*.json')):
            row = json.loads(path.read_text())
            if row.get('executions'):
                missions.append(dict(id=path.stem,
                    path=str((args.campaign / 'missions' / (path.stem + '.json.gz')).resolve()),
                    historical={e['profile']: {k: e['report'][k] for k in
                        ('execution_validated', 'execution_failures')} for e in row['executions']}))
    else:
        missions = [dict(id=f'{i}-{p.parent.name}', path=str(p.resolve()), historical={})
                    for i, p in enumerate(args.missions)]
    if not missions or len({m['path'] for m in missions}) != len(missions):
        parser.error('missions must be nonempty and unique')
    for m in missions:
        document, _, _ = load_document(m['path'])
        m.update(file_sha256=digest(m['path']), mission_sha256=document['sha256'])
    trials = []
    for m in missions:
        for profile in args.profiles:
            for repeat in range(args.repeats):
                # Alternate order to limit systematic warmup/desktop-load bias.
                for backend in (('python', 'cpp') if repeat % 2 == 0 else ('cpp', 'python')):
                    trials.append(dict(id=m['id'], mission=m['path'], profile=profile,
                        repeat=repeat, backend=backend, historical=m['historical'].get(profile)))
    manifest = dict(schema='swarm-native-execution-v1', sources=sources(), native=identity(),
        missions=missions, trials=trials, python=platform.python_version(), numpy=np.__version__,
        cpu_info=subprocess.check_output(['lscpu'], text=True), timeout_s=args.timeout,
        scope='Full offline simulate() with own-state local control and unchanged Python observer; '
              'same saved plans; serialized fresh subprocesses; BLAS threads=1; uncontrolled desktop')
    output.mkdir(parents=True)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    rows = []
    for i, trial in enumerate(trials):
        print(f"START {i} {trial['id']} {trial['profile']} {trial['backend']}", flush=True)
        with (output / f'{i}.log').open('x') as log:
            try:
                proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--output',
                    str(output), '--worker', str(i)], env=env, stdout=log, stderr=subprocess.STDOUT,
                    timeout=args.timeout)
                if proc.returncode:
                    row = dict(trial, completion='worker_error', returncode=proc.returncode)
                else:
                    row = json.loads((output / f'{i}.json').read_text()) | {'completion': 'complete'}
            except subprocess.TimeoutExpired:
                row = dict(trial, completion='timeout')
        rows.append(row)
        with (output / 'journal.jsonl').open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
        print(json.dumps({k: row[k] for k in ('id', 'profile', 'backend', 'completion', 'seconds') if k in row}), flush=True)
    check_frozen(manifest)
    result = summarize(output, manifest, rows)
    print(json.dumps({'passed': result['passed'], 'pairs': len(result['pairs']), 'errors': result['errors']}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
