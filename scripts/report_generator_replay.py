"""Compare a generator replay against saved inputs and revalidate selected plans.

The journal audit checks hashes; the separate saved-plan checks rerun numerical
validators, full-horizon pair bounds and grid allocation without a new search.
Neither check certifies RK4 error or establishes PX4/Gazebo execution safety.
"""

import argparse
from collections import Counter
from dataclasses import replace
from datetime import datetime, timezone
import gzip
import hashlib
from html import escape
from itertools import combinations
import json
import os
from pathlib import Path
import time

import numpy as np

from benchmark_scaling_campaign import check_sources, decode
from diagnose_campaign_execution import load_plan
from report_scaling_campaign import audit, report, summarize
from swarm_sim.trajectory2d.kinodynamic_recovery import pullout_from_state, validate_pullout
from swarm_sim.trajectory2d.obstacles import Obstacle
from swarm_sim.trajectory2d.planning import (
    continuous_distance_lower_bound, required_distance, validate_individual,
)
from swarm_sim.trajectory2d.reservations import first_conflict, occupancy


def read_rows(directory):
    return [json.loads(line) for line in (directory / 'journal.jsonl').read_text().splitlines()]


def progress_html(manifest, rows):
    exploratory = [r for r in rows if r['suite'] in ('core', 'random')]
    counts = Counter(r['status'] or r['completion'] for r in exploratory)
    controls = [r for r in rows if r['suite'] == 'controls']
    statuses = ''.join(f'<li>{escape(status)}: {count}</li>' for status, count in counts.items())
    stamp = datetime.now(timezone.utc).isoformat()
    return f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta http-equiv="refresh" content="15"><title>Physical generator campaign — running</title>
<style>body{{font:17px system-ui;max-width:900px;margin:50px auto;padding:0 20px;line-height:1.6}}</style>
<h1>Physical generator: campaign running</h1><p>Updated {stamp}. Refreshes every 15 seconds.</p>
<p><b>{len(exploratory)}/230 exploratory missions recorded.</b>
{len(rows)}/{len(manifest['jobs'])} total jobs recorded, including controls and replays.</p>
<p>Validated so far: {counts['validated']}. Negative controls matched:
{sum(r.get('expectation_met') is True for r in controls)}/{len(controls)} recorded.</p>
<ul>{statuses}</ul><p>Default remains geometric. This is provisional progress, not a qualification report.
Unsupported configuration requests, if any, remain explicit rejections in the
230-mission denominator. A parsed configuration is not a validated trajectory.</p>
<p>Original snapshots, seeds, physical limits, margins, candidate/attempt budgets and
{manifest['timeout_s']:g} s per-job cap are preserved. Explicit generator, numerical
backend and process-count overrides are recorded in the manifest.</p>
<p><a href="../manifest.json">Immutable inputs</a> · <a href="../journal.jsonl">Raw journal</a></p>
<p>The final comparative HTML, PDF and CSV reports replace this page after completion.
They include saved-plan numerical revalidation and default-promotion blockers.</p></html>'''


def wait_for_campaign(directory, manifest, campaign_pid):
    """Supervise the existing process without restarting it or changing its sources."""
    out = directory / 'analysis'
    out.mkdir(exist_ok=True)
    while True:
        journal = directory / 'journal.jsonl'
        raw = journal.read_text() if journal.exists() else ''
        # The writer may be in the middle of its last append. Parse only flushed
        # complete lines here; final analysis uses strict read_rows instead.
        lines = raw.splitlines(keepends=True)
        rows = [json.loads(line) for line in lines if line.endswith('\n')]
        temp = out / 'progress.tmp'
        temp.write_text(progress_html(manifest, rows))
        temp.replace(out / 'index.html')
        if len(rows) == len(manifest['jobs']):
            return
        try:
            os.kill(campaign_pid, 0)
        except ProcessLookupError:
            print('Campaign process stopped; generating failure-preserving partial report.', flush=True)
            return
        time.sleep(5)


def input_differences(job, original, planning_workers=None, generator_mode=None, numerical_backend=None):
    """Allow recorded generator/backend/process overrides, never safety inputs."""
    errors = []
    allowed = set()
    if generator_mode is not None:
        allowed.add('generator_mode')
        if job['config'].get('generator_mode', 'geometric') != generator_mode:
            errors.append('generator_mode differs from recorded override')
    if planning_workers is not None and job['suite'] != 'replay':
        allowed.add('planning_workers')
        if job['config'].get('planning_workers') != planning_workers:
            errors.append('planning_workers differs from recorded override')
    if numerical_backend is not None:
        allowed.add('numerical_backend')
        if job['config'].get('numerical_backend', 'python') != numerical_backend:
            errors.append('numerical_backend differs from recorded override')
    if {k: v for k, v in job.items() if k != 'config'} != {
            k: v for k, v in original.items() if k != 'config'}:
        errors.append('job fields or snapshots differ')
    fields, previous = dict(job['config']), dict(original['config'])
    # Historical records predate the optional backend; absence means Python.
    fields.setdefault('numerical_backend', 'python')
    previous.setdefault('numerical_backend', 'python')
    fields.setdefault('generator_mode', 'geometric')
    previous.setdefault('generator_mode', 'geometric')
    if {k: v for k, v in fields.items() if k not in allowed} != {
            k: v for k, v in previous.items() if k not in allowed}:
        errors.append('mission fields other than explicit generator/backend/process overrides differ')
    return errors


def revalidate(directory, jobs, rows):
    checked = pairs = reservations = pullouts = native_pullouts = corruptions = late_obstacles = 0
    errors = []
    for row in rows:
        if row['status'] != 'validated':
            continue
        job = jobs[row['id']]
        snapshots, config = decode(job)
        by_id = {s.drone_id: s for s in snapshots}
        with gzip.open(directory / 'missions' / (row['id'] + '.json.gz'), 'rt') as stream:
            document = json.load(stream)
        plans = tuple(load_plan(p) for p in document['plans'])
        for plan, stored in zip(plans, document['plans']):
            context = f'{row["id"]}: drone {plan.drone_id}'
            if plan.to_dict() != stored or not validate_individual(plan, by_id[plan.drone_id], config, backend='python'):
                errors.append(context + ': individual revalidation failed')
                continue
            checked += 1
            phases = plan.generation_phases
            if (config.generator_mode != 'geometric' and
                    (not phases or phases[0][1] != 0 or phases[-1][2] != len(plan.flight_controls)
                     or any(a[2] != b[1] for a, b in zip(phases, phases[1:])))):
                errors.append(context + ': incomplete phase metadata')
            for name, start, end in phases:
                if name != 'pullout':
                    continue
                primitive = pullout_from_state(plan.flight_states[start], config.for_drone(plan.drone_id))
                states, controls = plan.flight_states[start:end + 1], plan.flight_controls[start:end]
                valid = primitive is not None and validate_pullout(primitive, config.for_drone(plan.drone_id))
                exact = valid and np.array_equal(primitive.states, states) and np.array_equal(primitive.controls, controls)
                equivalent_native = (valid and config.numerical_backend == 'cpp'
                    and primitive.states.shape == states.shape and primitive.controls.shape == controls.shape
                    and np.allclose(primitive.states, states, rtol=0, atol=1e-9)
                    and np.allclose(primitive.controls, controls, rtol=0, atol=1e-10))
                if exact:
                    pullouts += 1
                elif equivalent_native:
                    native_pullouts += 1
                else:
                    errors.append(context + ': pullout connection differs')
            controls = plan.flight_controls.copy()
            controls[-1, 2] = 100.
            if validate_individual(replace(plan, flight_controls=controls), by_id[plan.drone_id], config, backend='python'):
                errors.append(context + ': corrupted final command accepted')
            else:
                corruptions += 1
            x, y, z = plan.flight_states[-1, :3]
            obstacle = Obstacle.box(x - 2, y - 2, max(0., z - 2), x + 2, y + 2, z + 2)
            if validate_individual(plan, by_id[plan.drone_id], replace(config, obstacles=(*config.obstacles, obstacle)), backend='python'):
                errors.append(context + ': post-arrival endpoint obstacle accepted')
            else:
                late_obstacles += 1
        for a, b in combinations(plans, 2):
            if continuous_distance_lower_bound(a, b) < required_distance(a, b, config):
                errors.append(row['id'] + ': full-horizon pair check failed')
            pairs += 1
        if config.reservation_policy == 'grid':
            allocated = [occupancy(p, config) for p in plans]
            for a, b in combinations(allocated, 2):
                if first_conflict(a, b) is not None:
                    errors.append(row['id'] + ': full-horizon reservation conflict')
                reservations += 1
    return {'scope': 'saved-plan numerical revalidation; no catalog generation or joint search',
        'individual_plans': checked, 'full_horizon_pairs': pairs,
        'reservation_pairs': reservations, 'exact_pullout_connections': pullouts,
        'native_pullout_connections_within_tolerance': native_pullouts,
        'native_pullout_comparison_atol': {'states': 1e-9, 'controls': 1e-10, 'rtol': 0},
        'corrupt_final_commands_rejected': corruptions,
        'post_arrival_obstacles_rejected': late_obstacles, 'errors': errors}


def compare(old_rows, rows):
    reference = {r['id']: r for r in old_rows}
    comparisons = []
    for row in rows:
        if row['suite'] not in ('core', 'random'):
            continue
        original = reference[row['id']]
        old_ok, new_ok = original['status'] == 'validated', row['status'] == 'validated'
        outcome = ('retained' if old_ok else 'gained') if new_ok else ('lost' if old_ok else 'neither')
        comparisons.append({'id': row['id'], 'num_drones': row['num_drones'],
            'family': row['family'], 'old_status': original['status'] or original['completion'],
            'new_status': row['status'] or row['completion'], 'outcome': outcome,
            'old_completion': original['completion'], 'new_completion': row['completion'],
            'old_planning_s': original.get('planning_wall_s'), 'new_planning_s': row.get('planning_wall_s')})
    return comparisons


def execution_coverage(manifest, rows):
    """Check each required job/profile, so duplicates cannot hide missing trials."""
    families = manifest.get('execution_families', ('baseline', 'obstacles', 'heterogeneous'))
    expected = {(r['id'], profile) for r in rows
                if r['status'] == 'validated' and r['suite'] == 'core'
                and r['seed'] == manifest['seeds'][0] and r['family'] in families
                for profile in ('nominal', 'gust_sensor', 'outside_clock')}
    recorded = Counter((r['id'], e['profile']) for r in rows for e in r.get('executions', []))
    def describe(keys):
        return [{'id': job, 'profile': profile} for job, profile in sorted(keys)]
    missing = describe(expected - recorded.keys())
    unexpected = describe(recorded.keys() - expected)
    duplicates = describe(key for key, count in recorded.items() if count != 1)
    return {'expected_trials': len(expected), 'recorded_trials': sum(recorded.values()),
            'missing_trials': missing, 'unexpected_trials': unexpected, 'duplicate_trials': duplicates,
            'complete': bool(manifest['execute'] and not (missing or unexpected or duplicates))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--wait-for-pid', type=int, help='Show live progress, then report when this campaign finishes')
    args = parser.parse_args()
    directory = args.directory.resolve()
    manifest = json.loads((directory / 'manifest.json').read_text())
    if args.wait_for_pid is not None:
        if args.wait_for_pid <= 0:
            parser.error('campaign PID must be positive')
        wait_for_campaign(directory, manifest, args.wait_for_pid)
    reference_path = Path(manifest['reference']['path'])
    original = json.loads(reference_path.read_text())
    rows, old_rows = read_rows(directory), read_rows(reference_path.parent)
    jobs = {j['id']: j for j in manifest['jobs']}
    old_jobs = {j['id']: j for j in original['jobs']}
    integrity = audit(directory, manifest, rows)
    errors = list(integrity['errors'])
    check_sources(manifest)
    if hashlib.sha256(reference_path.read_bytes()).hexdigest() != manifest['reference']['sha256']:
        raise RuntimeError('reference manifest changed')
    for job in jobs.values():
        errors.extend(f'{job["id"]}: {error}' for error in input_differences(
            job, old_jobs[job['id']], manifest['reference'].get('planning_workers_override'),
            manifest['reference'].get('generator_mode_override'),
            manifest['reference'].get('numerical_backend_override')))
    if manifest['timeout_s'] != original['timeout_s']:
        errors.append('timeout differs from reference')
    checks = revalidate(directory, jobs, rows)
    errors.extend(checks['errors'])
    comparison = compare(old_rows, rows)
    summary = summarize(manifest, rows)
    out = report(directory, manifest, rows, summary, integrity)
    decision_reasons = []
    if not integrity['complete'] or len(comparison) != 230:
        decision_reasons.append('Incomplete 230-mission coverage or artifact audit')
    if errors:
        decision_reasons.append('Input or numerical revalidation errors')
    unsupported = sum(r['status'] == 'unsupported_generator_configuration' for r in rows)
    if unsupported:
        decision_reasons.append(f'{unsupported} strict mission requests are unsupported')
    counts = Counter(c['outcome'] for c in comparison)
    if counts['lost']:
        decision_reasons.append(f'{counts["lost"]} previously validated missions are not validated')
    unfinished = sum(r['completion'] != 'complete' for r in rows)
    if unfinished:
        decision_reasons.append(f'{unfinished} jobs have timeout or worker-error outcomes')
    if summary['controls']['expectations_met'] != 19:
        decision_reasons.append('Not all 19 negative controls matched')
    if len(summary['replay']) != 10 or any(r['matches'] is not True for r in summary['replay']):
        decision_reasons.append('Not all 10 exact serial/spawn comparisons passed')
    if summary['executions']['expectations_met'] != summary['executions']['total']:
        decision_reasons.append('Execution expectations failed')
    coverage = execution_coverage(manifest, rows)
    if not coverage['complete']:
        decision_reasons.append('Requested execution trials are disabled, missing, duplicated or unexpected')
    result = {'reference': manifest['reference'],
        'native_backend_identity': manifest.get('native_backend_identity'),
        'exploratory_missions': len(comparison),
        'old_validated': sum(c['old_status'] == 'validated' for c in comparison),
        'new_validated': sum(c['new_status'] == 'validated' for c in comparison),
        'paired_outcomes': dict(counts), 'unsupported': unsupported,
        'by_count': {str(n): dict(Counter(c['outcome'] for c in comparison if c['num_drones'] == n))
                     for n in manifest['counts']}, 'comparison': comparison,
        'numerical_revalidation': checks, 'input_and_audit_errors': errors,
        'execution_coverage': coverage,
        'promotion': {'eligible_from_this_campaign': not decision_reasons,
                      'blocking_reasons': decision_reasons,
                      'default_changed': False},
        'limitations': ['RK4 integration error is not certified', 'No PX4/Gazebo trials',
            'Historical and replay timings were measured at different times on an uncontrolled desktop',
            'Finite catalog/search failure is not a continuous-space infeasibility proof',
            'Standalone test-suite evidence is separate from this campaign report'],
        'reporter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (out / 'generator-comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    table = []
    for n, group in summary['by_count'].items():
        old_count = sum(c['old_status'] == 'validated' for c in comparison if str(c['num_drones']) == n)
        paired = result['by_count'][n]
        denominator = sum(str(c['num_drones']) == n for c in comparison)
        table.append(f'<tr><td>{n}</td><td>{old_count}/{denominator}</td><td>{group["validated"]}/{denominator}</td>'
            f'<td>{paired.get("gained", 0)}</td><td>{paired.get("lost", 0)}</td>'
            f'<td>{group["planning_timeouts"]}</td></tr>')
    details = ''.join('<tr>' + ''.join(f'<td>{escape(str(c[k]))}</td>' for k in
        ('id', 'old_status', 'new_status', 'new_completion', 'outcome')) + '</tr>' for c in comparison)
    reasons_html = ''.join(f'<li>{escape(reason)}</li>' for reason in decision_reasons)
    html = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Generator replay — saved-input comparison</title><style>
body{{font:16px system-ui;max-width:1150px;margin:40px auto;padding:0 20px;color:#182535}}
table{{border-collapse:collapse;width:100%;margin:20px 0}}th,td{{padding:8px;border-bottom:1px solid #ccd5df;text-align:left}}
.decision{{background:#fff1d6;padding:16px;border-left:5px solid #a36700}}pre{{white-space:pre-wrap}}
</style><h1>Generator replay: {len(comparison)} exploratory missions</h1>
<p>Same saved missions, seeds, budgets, limits and {manifest['timeout_s']:g} s external timeout.
Generator override: {escape(str(manifest['reference'].get('generator_mode_override')))};
numerical backend override: {escape(str(manifest['reference'].get('numerical_backend_override')))};
catalog processes override: {escape(str(manifest['reference'].get('planning_workers_override')))}
(saved serial/spawn replay assignments retained).
{len(rows)}/{len(jobs)} total jobs recorded.</p>
<p class="decision"><b>Default retained: geometric.</b></p><ul>{reasons_html}</ul>
<table><tr><th>Aircraft</th><th>Historical validated</th><th>Replay validated</th>
<th>Gained</th><th>Lost</th><th>Planning timeouts</th></tr>{''.join(table)}</table>
<p>Old: {result['old_validated']}/{len(comparison)}. New: {result['new_validated']}/{len(comparison)}.
Retained: {counts['retained']}; gained: {counts['gained']}; lost: {counts['lost']}.
Unsupported strict requests remain in the denominator.</p>
<h2>Verification</h2><p>Negative controls: {summary['controls']['expectations_met']}/19.
Exact serial/spawn comparisons: {sum(r['matches'] is True for r in summary['replay'])}/10.
Execution expectations: {summary['executions']['expectations_met']}/{summary['executions']['total']}.</p>
<p>Artifact consistency errors: {len(integrity['errors'])}. Separate numerical revalidation:
{checks['individual_plans']} individual plans, {checks['full_horizon_pairs']} full-horizon pairs,
{checks['exact_pullout_connections']} exact recovery connections and
{checks['native_pullout_connections_within_tolerance']} native connections within
1e-9 state / 1e-10 control absolute tolerance. Numerical errors: {len(checks['errors'])}.</p>
<p>Corrupt final commands rejected: {checks['corrupt_final_commands_rejected']}.
Post-arrival endpoint obstacles rejected: {checks['post_arrival_obstacles_rejected']}.</p>
<p><a href="report.pdf">Measured campaign PDF</a> · <a href="missions.csv">Mission CSV</a> ·
<a href="executions.csv">Execution CSV</a> · <a href="generator-comparison.json">Full comparison and checks</a> ·
<a href="../manifest.json">Immutable manifest</a> · <a href="../journal.jsonl">Raw journal</a></p>
<h2>Limits</h2><ul>{''.join(f'<li>{escape(v)}</li>' for v in result['limitations'])}</ul>
<h2>Every exploratory mission</h2><table><tr><th>Mission</th><th>Historical</th><th>Replay</th>
<th>New worker completion</th><th>Paired outcome</th></tr>{details}</table></html>'''
    (out / 'index.html').write_text(html)
    (out / 'completion.json').write_text(json.dumps({'finished_utc': datetime.now(timezone.utc).isoformat(),
        'recorded': len(rows), 'planned': len(jobs), 'complete': integrity['complete'],
        'numerical_errors': len(checks['errors']), 'promotion': result['promotion']}, indent=2))
    print(json.dumps({k: result[k] for k in ('old_validated', 'new_validated', 'paired_outcomes',
                                          'unsupported', 'promotion', 'input_and_audit_errors')}, indent=2))
    return 0 if integrity['complete'] and not errors else 2


if __name__ == '__main__':
    raise SystemExit(main())
