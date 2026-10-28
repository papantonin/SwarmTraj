"""Audit and summarize a scaling campaign; never re-label failures as success.

The artifact audit checks consistency and canonical hashes, not independent
trajectory physics. Plots and timing statistics exclude explicit negative
controls and keep externally censored observations visible.
"""

import argparse
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np


def quantiles(values):
    if not values:
        return {'n': 0, 'median': None, 'p90': None, 'p95': None, 'max': None}
    return {'n': len(values), 'median': float(np.median(values)), 'p90': float(np.percentile(values, 90)),
            'p95': float(np.percentile(values, 95)), 'max': float(max(values))}


def pair_slack(row):
    if row.get('minimum_pair_slack_m') is not None:
        return row['minimum_pair_slack_m']
    # This matrix uses a common physical envelope and no aircraft-profile map:
    # every pair has the same required bound, including different setpoints.
    lower, required = row.get('minimum_distance_lower_bound_m'), row.get('required_distance_m')
    return lower-required if lower is not None and required is not None else None


def audit(directory, manifest, rows):
    expected = {j['id']: j for j in manifest['jobs']}
    seen = [r['id'] for r in rows]
    errors = []
    if len(set(seen)) != len(seen):
        errors.append('duplicate journal IDs')
    if set(seen) - set(expected):
        errors.append('unexpected journal IDs')
    checked = 0
    for row in rows:
        if row.get('status') is None:
            continue
        path = directory / 'missions' / (row['id']+'.json.gz')
        if not path.exists():
            errors.append(f'missing mission: {row["id"]}')
            continue
        with gzip.open(path, 'rt') as stream:
            doc = json.load(stream)
        digest = doc.pop('sha256')
        canonical = json.dumps(doc, sort_keys=True, allow_nan=False).encode()
        if hashlib.sha256(canonical).hexdigest() != digest or digest != row.get('mission_sha256'):
            errors.append(f'canonical hash mismatch: {row["id"]}')
        payload = {key: doc[key] for key in ('status', 'counts', 'search_nodes',
                   'minimum_distance_lower_bound', 'required_distance', 'plans')}
        sig = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if sig != row.get('signature_sha256') or row['status'] != doc['status']:
            errors.append(f'result mismatch: {row["id"]}')
        job = expected.get(row['id'])
        if job and (doc['config'] != json.loads(json.dumps(job['config']))
                    or doc['snapshots'] != json.loads(json.dumps(job['snapshots']))):
            errors.append(f'input mismatch: {row["id"]}')
        if doc['status'] == 'validated' and len(doc['plans']) != row['num_drones']:
            errors.append(f'incomplete plan selection: {row["id"]}')
        checked += 1
    return {'planned': len(expected), 'recorded': len(rows), 'missing_ids': sorted(set(expected)-set(seen)),
            'artifact_hashes_checked': checked, 'errors': errors,
            'complete': set(expected) == set(seen) and not errors,
            'scope': 'journal/artifact consistency; not independent physical trajectory revalidation'}


def summarize(manifest, rows):
    exploratory = [r for r in rows if r['suite'] in ('core', 'random')]
    groups = {}
    for n in manifest['counts']:
        subset = [r for r in exploratory if r['num_drones'] == n]
        timed = [r['planning_wall_s'] for r in subset if 'planning_wall_s' in r]
        valid = [r for r in subset if r['status'] == 'validated']
        groups[str(n)] = {'missions': len(subset), 'validated': len(valid),
            'planning_timeouts': sum(r['completion'] == 'planning_timeout' for r in subset),
            'execution_timeouts': sum(r['completion'] == 'execution_timeout' for r in subset),
            'statuses': dict(Counter(r['status'] or r['completion'] for r in subset)),
            'completed_planning_seconds': quantiles(timed),
            'validated_planning_seconds': quantiles([r['planning_wall_s'] for r in valid]),
            'attempts_completed_planning': quantiles([r['attempted'] for r in subset if 'attempted' in r])}
    executions = [{'id': r['id'], **e} for r in rows for e in r.get('executions', [])]
    controls = [r for r in rows if r['suite'] == 'controls']
    rejection_counts = Counter()
    for r in exploratory:
        for catalog in r.get('diagnostics', {}).get('catalogs', []):
            rejection_counts.update(catalog['rejections'])
    replay = []
    lookup = {(r['suite'], r['family'], r['num_drones'], r['seed']): r for r in rows}
    for n in manifest['counts']:
        seed = manifest['seeds'][0]
        reference = lookup.get(('core', 'baseline', n, seed))
        for family in ('serial', 'parallel2'):
            other = lookup.get(('replay', family, n, seed))
            if not reference or not other:
                continue
            comparable = reference.get('signature_sha256') is not None and other.get('signature_sha256') is not None
            replay.append({'num_drones': n, 'variant': family,
                'comparable': comparable,
                'matches': reference['signature_sha256'] == other['signature_sha256'] if comparable else None,
                'reference_planning_s': reference.get('planning_wall_s'),
                'variant_planning_s': other.get('planning_wall_s')})
    paired = []
    for n in manifest['counts']:
        for seed in manifest['seeds'][:2]:
            first = lookup.get(('ablation', 'adaptive_first', n, seed))
            margin = lookup.get(('ablation', 'adaptive_margin', n, seed))
            if first and margin:
                paired.append({'num_drones': n, 'seed': seed, 'first_status': first['status'],
                    'margin_status': margin['status'], 'first_s': first.get('planning_wall_s'),
                    'margin_s': margin.get('planning_wall_s'), 'first_slack_m': pair_slack(first),
                    'margin_slack_m': pair_slack(margin)})
    return {'scope': 'exploratory = core + random; controls, ablations and replays reported separately',
        'num_jobs': len(rows), 'statuses_all': dict(Counter(r['status'] or r['completion'] for r in rows)),
        'completion': dict(Counter(r['completion'] for r in rows)), 'by_count': groups,
        'exploratory_jobs': len(exploratory), 'exploratory_validated': sum(r['status'] == 'validated' for r in exploratory),
        'controls': {'total': len(controls), 'expectations_met': sum(r.get('expectation_met') is True for r in controls)},
        'executions': {'total': len(executions), 'expectations_met': sum(e['expectation_met'] for e in executions),
            'by_profile': {p: {'total': sum(e['profile'] == p for e in executions),
                              'accepted': sum(e['report']['execution_validated'] for e in executions if e['profile'] == p),
                              'expectations_met': sum(e['expectation_met'] for e in executions if e['profile'] == p)}
                           for p in sorted({e['profile'] for e in executions})},
            'failures': [{'id': e['id'], 'profile': e['profile'], 'failures': e['report']['execution_failures']}
                         for e in executions if not e['expectation_met']]},
        'followups': {suite: {family: {'trials': len(group),
                     'validated': sum(r['status'] == 'validated' for r in group),
                     'statuses': dict(Counter(r['status'] or r['completion'] for r in group)),
                     'completed_planning_seconds': quantiles([r['planning_wall_s'] for r in group if 'planning_wall_s' in r])}
                     for family in sorted({r['family'] for r in rows if r['suite'] == suite})
                     if (group := [r for r in rows if r['suite'] == suite and r['family'] == family])}
                     for suite in ('budgets', 'search_caps', 'horizon', 'profiles')
                     if any(r['suite'] == suite for r in rows)},
        'rejection_counts': dict(rejection_counts.most_common()), 'replay': replay, 'first_vs_margin': paired,
        'total_serial_process_wall_s': sum(r['process_wall_s'] for r in rows)}


def write_csv(directory, rows):
    flat = []
    for r in rows:
        diag = r.get('diagnostics', {})
        search = diag.get('search', {})
        timings = r.get('timings', {})
        flat.append({k: r.get(k) for k in ('id', 'suite', 'family', 'num_drones', 'seed', 'status',
            'completion', 'planning_wall_s', 'process_wall_s', 'timeout_s', 'attempted', 'accepted_candidates',
            'search_nodes', 'minimum_distance_lower_bound_m', 'required_distance_m', 'minimum_pair_slack_m',
            'self_peak_rss_kib', 'largest_child_peak_rss_kib', 'signature_sha256')} | {
            'minimum_pair_slack_m': pair_slack(r),
            'catalog_wall_s': timings.get('catalog_wall_s'), 'search_wall_s': timings.get('search_wall_s'),
            'optimization_wall_s': timings.get('optimization_wall_s'),
            'pair_checks': search.get('pair_checks'), 'pair_cache_hits': search.get('pair_cache_hits'),
            'reservation_conflicts': search.get('reservation_conflicts'), 'expansion_rounds': len(diag.get('rounds', [])),
            'rejections_json': json.dumps(dict(sum_counters([c['rejections'] for c in diag.get('catalogs', [])]))),
            'counts_json': json.dumps(r.get('counts')), 'arrival_times_json': json.dumps(r.get('arrival_times_s')),
            'expected_status': r.get('expected_status'), 'expectation_met': r.get('expectation_met')})
    if flat:
        with (directory / 'missions.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
            writer.writeheader()
            writer.writerows(flat)
    executions = []
    for row in rows:
        for e in row.get('executions', []):
            report = e['report']
            executions.append({'id': row['id'], 'num_drones': row['num_drones'], 'family': row['family'],
                'profile': e['profile'], 'execution_wall_s': e['execution_wall_s'],
                'accepted': report['execution_validated'], 'expectation_met': e['expectation_met'],
                'failures_json': json.dumps(report['execution_failures']),
                'tracking_upper_m': max(report['tracking_error_upper_bounds_m']),
                'minimum_pair_distance_lower_m': report['minimum_distance_lower_bound_m']})
    if executions:
        with (directory / 'executions.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(executions[0]))
            writer.writeheader()
            writer.writerows(executions)


def sum_counters(items):
    result = Counter()
    for item in items:
        result.update(item)
    return result


def fmt(value):
    return '—' if value is None else f'{value:.2f}'


def report(directory, manifest, rows, summary, integrity):
    out = directory / 'analysis'
    out.mkdir(exist_ok=True)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    (out / 'audit.json').write_text(json.dumps(integrity, indent=2, allow_nan=False))
    write_csv(out, rows)
    lines = ['# Centralized trajectory generation: 1–5 aircraft campaign', '',
        f"Recorded **{len(rows)} / {len(manifest['jobs'])}** requested jobs. Artifact hashes checked: "
        f"{integrity['artifact_hashes_checked']}. Integrity errors: {len(integrity['errors'])}.", '',
        '## Measurement protocol', '',
        '- Coupled coordinated point-mass dynamics; no Gazebo/PX4 flight in this campaign.',
        '- Jobs run sequentially on the recorded host. Other desktop workloads were not isolated.',
        '- Python '+manifest['python']+', NumPy '+manifest['numpy']+', SciPy '+manifest['scipy']+'.',
        '- '+next((line.strip() for line in manifest['cpu_info'].splitlines() if line.startswith('Model name:')), 'CPU recorded in manifest')+'.',
        '- One catalog worker except explicit parallel replay. BLAS/OpenMP threads fixed to one.',
        '- Planning wall time excludes Python imports, JSON export and execution. Process time includes them.',
        f"- External per-job cap: {manifest['timeout_s']:g} s, including optional execution. A saved planning checkpoint survives execution timeout.",
        '- Completed-time percentiles exclude planning timeouts; they are conditional observations, not uncensored latency estimates.',
        '- Baseline: dt=0.2 s, 28 candidates maximum/aircraft, 90 attempts/aircraft, initial batch 2, 30000 search nodes.',
        '- Safety/physical limits are unchanged; every mission input is saved, including explicit stress-test constraints.',
        '- RSS fields are per-process high-water marks; they are not total simultaneous memory use.', '',
        '## Exploratory outcomes (core + random only)', '',
        '| Aircraft | Missions | Validated | Planning timeouts | Median completed (s) | p95 completed (s) | Max completed (s) |',
        '|---:|---:|---:|---:|---:|---:|---:|']
    for n, g in summary['by_count'].items():
        q = g['completed_planning_seconds']
        lines.append(f"| {n} | {g['missions']} | {g['validated']} | {g['planning_timeouts']} | {fmt(q['median'])} | {fmt(q['p95'])} | {fmt(q['max'])} |")
    lines += ['', 'The family mix is deliberately heterogeneous and includes difficult/deadline cases. '
        'These proportions describe this test matrix, not an operational mission success probability.', '',
        '## Outcomes by scenario family', '', '| Family | Aircraft | Validated / trials | Other statuses |', '|---|---:|---:|---|']
    families = sorted({r['family'] for r in rows if r['suite'] in ('core', 'random')})
    for family in families:
        for n in manifest['counts']:
            group = [r for r in rows if r['suite'] in ('core', 'random') and r['family'] == family and r['num_drones'] == n]
            if group:
                others = Counter(r['status'] or r['completion'] for r in group if r['status'] != 'validated')
                lines.append(f"| {family} | {n} | {sum(r['status'] == 'validated' for r in group)} / {len(group)} | {dict(others)} |")
    lines += ['', '## Negative controls, execution and reproducibility', '',
        f"- Expected negative planning statuses: {summary['controls']['expectations_met']} / {summary['controls']['total']} matched.",
        f"- Execution profile expectations: {summary['executions']['expectations_met']} / {summary['executions']['total']} matched.",
        '- The outside_clock profile deliberately exceeds the clock assumption and must be rejected.',
        '- No failed run is removed from the journal. Missing or timed-out executions are not passing checks.', '',
        '```json', json.dumps(summary['executions'], indent=2), '```', '',
        '| Aircraft | Replay | Exact result match | Original (s) | Replay (s) |', '|---:|---|---|---:|---:|']
    for r in summary['replay']:
        lines.append(f"| {r['num_drones']} | {r['variant']} | {r['matches']} | {fmt(r['reference_planning_s'])} | {fmt(r['variant_planning_s'])} |")
    lines += ['', 'Exact signatures include status, selected arrays, candidate counts, search nodes and separation values; '
        'they exclude wall-clock times and worker-count options. A match compares recorded outputs; it is not a new safety proof.', '',
        '## Paired algorithm comparisons', '',
        'Ablations use identical spread180 inputs, physical limits and cumulative attempt/candidate budgets. '
        'They change search/generation policy only. First-feasible and margin selection have different objectives.', '',
        '| Mode | Trials | Validated | Timeout | Median completed planning (s) |', '|---|---:|---:|---:|---:|']
    for mode in sorted({r['family'] for r in rows if r['suite'] == 'ablation'}):
        group = [r for r in rows if r['suite'] == 'ablation' and r['family'] == mode]
        q = quantiles([r['planning_wall_s'] for r in group if 'planning_wall_s' in r])
        lines.append(f"| {mode} | {len(group)} | {sum(r['status']=='validated' for r in group)} | "
            f"{sum(r['completion']=='planning_timeout' for r in group)} | {fmt(q['median'])} |")
    if summary['followups']:
        lines += ['', '## Targeted follow-up experiments', '',
            'Candidate/attempt budgets vary together in the budget suite. Search caps change only the search-node allowance. '
            'Horizon changes are separate explicit mission requirements. Mixed profiles explicitly allow simpler families; '
            'their success is not strict rich-profile fulfillment.', '',
            '| Suite | Setting | Validated / trials | Completed median (s) | Statuses |', '|---|---|---:|---:|---|']
        for suite, settings in summary['followups'].items():
            for setting, group in settings.items():
                lines.append(f"| {suite} | {setting} | {group['validated']} / {group['trials']} | "
                    f"{fmt(group['completed_planning_seconds']['median'])} | {group['statuses']} |")
    lines += ['', '## Recorded first-failure rejection stages', '',
        'Counts cover completed exploratory catalogues only. Each rejected proposal contributes its first failing stage; '
        'timeouts can hide unfinished catalogue statistics. They do not measure all possible defects in each proposal.', '',
        '| Stage | Count |', '|---|---:|']
    lines += [f'| {k} | {v} |' for k, v in summary['rejection_counts'].items()]
    lines += ['', '## Interpretation and limitations', '',
        '- no_candidate: at least one aircraft has no accepted candidate in its bounded catalogue.',
        '- catalog_exhausted: no compatible joint selection was found in explored catalogues.',
        '- search_budget_exhausted: the bounded search stopped before obtaining a selection.',
        '- planning_timeout: externally censored run; planner outcome and final internal work remain unknown.',
        '- Initial-state, obstacle-target and straight-line reachability rejections are separate preflight outcomes.',
        '- A finite catalogue failure never proves continuous-space infeasibility; no margins were relaxed.',
        '- Full-horizon numerical validation is conditional on the model, tracking tubes, clock bounds and integration accuracy.',
        '- RK4 has no certified integration-error enclosure. No real-aircraft or PX4 separation certification follows.',
        '- Core seeds vary candidate RNG on fixed family inputs. Random scene IDs independently draw broader mission inputs.',
        '- Fleet size also changes the centered grid, initial phases and altitude distribution. These are scenario-scaling experiments, not a pure fixed-geometry complexity benchmark.',
        '- Three core seeds and two ablation seeds are small samples. Replay timing is a single extra observation, not a stable speedup estimate.',
        '- Only ten random scene IDs per fleet size by default; this is not exhaustive coverage or a reliability certification.',
        '- Isolated physics, calibrated aircraft, heterogeneous airframe files, 6-DoF and Gazebo performance are outside this campaign.',
        '- Journal/hash audit verifies artifact consistency, not independent trajectory revalidation.', '',
        'Raw data: ../manifest.json, ../journal.jsonl, ../missions/*.json.gz, ../logs/*.log. '
        'Convenience exports: missions.csv, executions.csv, summary.json, audit.json and report.pdf.', '']
    if summary.get('diagnostic_replays'):
        lines += ['## Execution diagnostic replay', '',
            'This observer-only trace reuses the exact stored plans and returns every original acceptance predicate unchanged. '
            'The original failed execution remains a failure in all campaign totals.', '']
        for item in summary['diagnostic_replays']:
            lines += [f"### {item['job_id']} / {item['profile']}", '',
                f"Original/replayed failure gates: {item['execution_failures']}. "
                f"First observer safety event: {item['first_safety_event']}.", '',
                '```json', json.dumps(item['failed_predicates'], indent=2), '```', '']
    if summary.get('proposal_screen_diagnostic'):
        lines += ['## Systematic proposal-screen rejection at higher airspeed', '',
            'The heterogeneous-speed case passes the coupled-model descent preflight. However, its legacy geometric proposal '
            'uses a different vertical-speed construction and 90% of the vertical-acceleration limit before retiming. '
            'At 13 and 14 m/s its recovery drop exceeds the 30 m target altitude. The screen rejects all 90 attempts for '
            'each affected aircraft before coupled rollout. More seeds or attempts cannot change this particular screen.', '',
            'The coupled recovery geometry below is a necessary/template comparison, not a constructive full-trajectory feasibility proof.', '',
            '| Airspeed (m/s) | Coupled recovery drop (m) | Proposal recovery drop (m) | Proposal screen rejected |',
            '|---:|---:|---:|---|']
        for row in summary['proposal_screen_diagnostic']['rows']:
            lines.append(f"| {row['total_airspeed_m_s']:g} | {row['coupled_recovery_drop_m']:.2f} | "
                f"{row['proposal_recovery_drop_m']:.2f} | {row['proposal_screen_error'] is not None} |")
    (out / 'report.md').write_text('\n'.join(lines))
    make_plots(out, manifest, rows, summary, integrity)
    return out


def make_plots(out, manifest, rows, summary, integrity):
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    exploratory = [r for r in rows if r['suite'] in ('core', 'random')]
    with PdfPages(out / 'report.pdf') as pdf:
        def save(fig, name):
            fig.savefig(out / (name+'.png'), dpi=160, bbox_inches='tight')
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
        fig = plt.figure(figsize=(11.7, 8.3))
        fig.text(.07, .91, 'Centralized trajectory generation', fontsize=23, weight='bold')
        fig.text(.07, .85, '1–5 aircraft · measured campaign', fontsize=17)
        text = [f"{len(rows)} / {len(manifest['jobs'])} jobs recorded; {summary['exploratory_jobs']} exploratory missions.",
            f"{summary['exploratory_validated']} exploratory plans validated; negative controls reported separately.",
            f"Negative controls: {summary['controls']['expectations_met']} / {summary['controls']['total']} expected outcomes.",
            f"Execution expectations: {summary['executions']['expectations_met']} / {summary['executions']['total']} matched.",
            f"Artifacts checked: {integrity['artifact_hashes_checked']}; consistency errors: {len(integrity['errors'])}.", '',
            'Coupled point-mass model; no PX4 / Gazebo flight.',
            'Serialized jobs, one catalog worker except explicit parallel replay.',
            'Baseline: 28 candidates, 90 attempts per aircraft, initial adaptive batch 2.',
            f"External cap: {manifest['timeout_s']:g} seconds per process, including optional execution.",
            'Planning time excludes imports, export and execution.', '',
            'Timeouts are censored outcomes, never infeasibility proofs.',
            'Completed-time quantiles omit planning timeouts; see the timeout counts.',
            'Sample proportions describe this deliberately varied test matrix.',
            'Desktop workload and thermal/frequency effects are not controlled.', '',
            'Numerical validation remains conditional on the model and assumptions.',
            'Read report.md and the raw manifest/journal for complete settings and limitations.']
        fig.text(.07, .76, '\n'.join(text), va='top', linespacing=1.6, fontsize=11)
        save(fig, 'overview')

        fig, ax = plt.subplots(figsize=(11.7, 8.3))
        ax.axis('off')
        ax.set_title('Measured exploratory results by aircraft count', fontsize=17, pad=24)
        table_rows = []
        for n, group in summary['by_count'].items():
            q = group['completed_planning_seconds']
            table_rows.append([n, str(group['missions']), str(group['validated']), str(group['planning_timeouts']),
                               fmt(q['median']), fmt(q['p95']), fmt(q['max'])])
        table = ax.table(cellText=table_rows, colLabels=['Aircraft', 'Trials', 'Validated', 'Timeouts',
                         'Median (s)', 'p95 (s)', 'Max (s)'], loc='upper center', cellLoc='center', bbox=[0, .53, 1, .36])
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        text = ['Times above cover completed planning only (including rejected outcomes).',
                'Core + random matrix; negative controls/ablations/replays are excluded.', '',
                'Baseline-only completed planning observations (three candidate seeds):']
        for n in manifest['counts']:
            group = [r for r in rows if r['suite'] == 'core' and r['family'] == 'baseline' and r['num_drones'] == n]
            q = quantiles([r['planning_wall_s'] for r in group if 'planning_wall_s' in r])
            text.append(f"  {n} aircraft: {sum(r['status']=='validated' for r in group)}/{len(group)} validated; median {fmt(q['median'])} s; max {fmt(q['max'])} s.")
        text += ['', 'Fleet size also changes grid/phase/altitude inputs; this is scenario scaling.',
                 'Three seeds are insufficient to estimate operational reliability or a stable tail.']
        ax.text(0, .44, '\n'.join(text), va='top', transform=ax.transAxes, linespacing=1.6, fontsize=11)
        save(fig, 'scaling_table')

        counts = manifest['counts']
        families = list(dict.fromkeys(r['family'] for r in exploratory))
        matrix = np.full((len(families), len(counts)), np.nan)
        fig, ax = plt.subplots(figsize=(11.7, 8.3))
        for y, family in enumerate(families):
            for x, n in enumerate(counts):
                group = [r for r in exploratory if r['family'] == family and r['num_drones'] == n]
                if group:
                    passed = sum(r['status'] == 'validated' for r in group)
                    matrix[y, x] = passed / len(group)
                    ax.text(x, y, f'{passed}/{len(group)}', ha='center', va='center', color='black')
        im = ax.imshow(matrix, vmin=0, vmax=1, cmap='RdYlGn', aspect='auto')
        ax.set(xticks=range(len(counts)), xticklabels=counts, yticks=range(len(families)), yticklabels=families,
               xlabel='Aircraft count', title='Exploratory planning outcomes: validated / requested\nTimeouts remain in denominators')
        fig.colorbar(im, ax=ax, label='Fraction validated on this matrix')
        fig.tight_layout()
        save(fig, 'success_matrix')

        fig, axes = plt.subplots(1, 2, figsize=(11.7, 7.5))
        for ax, validated_only, title in ((axes[0], False, 'All completed planning outcomes'),
                                          (axes[1], True, 'Validated plans only')):
            data, positions = [], []
            for n in counts:
                values = [r['planning_wall_s'] for r in exploratory if r['num_drones'] == n
                          and 'planning_wall_s' in r and (not validated_only or r['status'] == 'validated')]
                if values:
                    data.append(values)
                    positions.append(n)
            if data:
                ax.boxplot(data, positions=positions, showfliers=True)
            ax.set(title=title, xlabel='Aircraft count', ylabel='Planning wall seconds', yscale='log', xticks=counts)
            ax.grid(axis='y', alpha=.25)
        fig.suptitle('Latency distributions conditional on completed planning\nExternal planning timeouts omitted; reported separately')
        fig.tight_layout()
        save(fig, 'planning_latency')

        fig, axes = plt.subplots(2, 1, figsize=(11.7, 8.3))
        categories = sorted({r['status'] or r['completion'] for r in exploratory})
        bottom = np.zeros(len(counts))
        for status in categories:
            values = [sum((r['status'] or r['completion']) == status and r['num_drones'] == n for r in exploratory) for n in counts]
            axes[0].bar(counts, values, bottom=bottom, label=status)
            bottom += values
        axes[0].set(title='All exploratory outcomes, including censored runs', ylabel='Missions', xticks=counts)
        axes[0].legend(fontsize=8, loc='upper left', bbox_to_anchor=(1.01, 1))
        names = list(summary['rejection_counts'])[:12][::-1]
        axes[1].barh(names, [summary['rejection_counts'][k] for k in names], color='#5279a8')
        axes[1].set(title='First-failure stages in completed exploratory catalogues', xlabel='Rejected proposals')
        fig.tight_layout()
        save(fig, 'failure_modes')

        fig, axes = plt.subplots(1, 2, figsize=(11.7, 7.5))
        for r in exploratory:
            if 'planning_wall_s' not in r:
                continue
            color = '#338858' if r['status'] == 'validated' else '#bf5a4c'
            axes[0].scatter(r['attempted'], r['planning_wall_s'], color=color, alpha=.5, s=18)
            if r['status'] == 'validated':
                times = r['timings']
                axes[1].scatter(r['num_drones'], times.get('catalog_wall_s', 0)/r['planning_wall_s'], alpha=.4, s=18)
        axes[0].set(xlabel='Attempted candidates (fleet sum)', ylabel='Planning wall seconds', yscale='log',
                    title='Cost of candidate generation\nGreen: validated; red: other completed status')
        axes[1].set(xlabel='Aircraft count', ylabel='Catalog wall time / total planning time',
                    xticks=counts, ylim=(0, 1.03), title='Generation + individual validation share\nValidated exploratory missions')
        fig.tight_layout()
        save(fig, 'cost_drivers')

        fig, axes = plt.subplots(2, 2, figsize=(11.7, 8.3))
        for r in exploratory:
            if 'planning_wall_s' not in r:
                continue
            n = r['num_drones']
            color = '#338858' if r['status'] == 'validated' else '#bf5a4c'
            axes[0, 0].scatter(n, r['self_peak_rss_kib']/1024, s=18, alpha=.4, color=color)
            axes[0, 1].scatter(n, r['search_nodes']+1, s=18, alpha=.4, color=color)
            slack = pair_slack(r)
            if slack is not None:
                axes[1, 0].scatter(n, slack, s=18, alpha=.4)
            if r.get('arrival_times_s'):
                job = next(j for j in manifest['jobs'] if j['id'] == r['id'])
                axes[1, 1].scatter(n, max(r['arrival_times_s'])/job['config']['horizon'], s=18, alpha=.4)
        axes[0, 0].set(ylabel='Worker peak RSS (MiB)', title='Memory: process high-water mark\nNot aggregate concurrent memory')
        axes[0, 1].set(ylabel='Search nodes + 1', yscale='log', title='Bounded joint-search work')
        axes[1, 0].set(ylabel='Minimum selected pair slack (m)', title='Remaining robust separation margin\nNo pair margin for one aircraft')
        axes[1, 1].set(ylabel='Latest nominal arrival / horizon', ylim=(0, 1), title='Arrival timing of selected plans\nFull recovery must also fit')
        for ax in axes.flat:
            ax.set(xlabel='Aircraft count', xticks=counts)
            ax.grid(alpha=.2)
        fig.tight_layout()
        save(fig, 'resources_and_selected_plans')

        modes = sorted({r['family'] for r in rows if r['suite'] == 'ablation'})
        if modes:
            fig, axes = plt.subplots(1, 2, figsize=(11.7, 8.3))
            data, labels = [], []
            for mode in modes:
                values = [r['planning_wall_s'] for r in rows if r['suite'] == 'ablation' and r['family'] == mode and 'planning_wall_s' in r]
                if values:
                    data.append(values)
                    labels.append(mode)
            axes[0].boxplot(data, orientation='horizontal', tick_labels=labels)
            axes[0].set(xscale='log', xlabel='Planning seconds (completed outcomes)', title='Paired spread180 ablations')
            for n in counts:
                pairs = [r for r in summary['first_vs_margin'] if r['num_drones'] == n
                         and r['first_slack_m'] is not None and r['margin_slack_m'] is not None]
                if pairs:
                    axes[1].scatter([p['first_slack_m'] for p in pairs], [p['margin_slack_m'] for p in pairs], label=f'{n} aircraft')
            bounds = axes[1].get_xlim()
            axes[1].plot(bounds, bounds, '--', color='gray')
            axes[1].set(xlabel='First-feasible minimum pair slack (m)', ylabel='Margin-policy minimum pair slack (m)',
                        title='Margin selection: matched successful pairs')
            if axes[1].get_legend_handles_labels()[0]:
                axes[1].legend()
            fig.tight_layout()
            save(fig, 'ablation')

        if summary['followups']:
            fig, axes = plt.subplots(1, 2, figsize=(11.7, 8.3))
            for family in ('c2_a10', 'c8_a30', 'c28_a90', 'c56_a180'):
                group = [r for r in rows if r['suite'] == 'budgets' and r['family'] == family]
                if group:
                    medians = [np.median([r['planning_wall_s'] for r in group if r['num_drones'] == n and 'planning_wall_s' in r])
                               if any(r['num_drones'] == n and 'planning_wall_s' in r for r in group) else np.nan for n in counts]
                    axes[0].plot(counts, medians, 'o-', label=family)
            axes[0].set(xlabel='Aircraft count', ylabel='Median completed planning seconds', yscale='log',
                        title='Candidate / attempt budget sensitivity\nTwo fixed baseline seeds per size')
            axes[0].legend()
            for n in (1, 3, 5):
                horizons, fractions = [], []
                for horizon in (60, 90, 120, 180):
                    group = [r for r in rows if r['suite'] == 'horizon' and r['family'] == f'h{horizon}' and r['num_drones'] == n]
                    if group:
                        horizons.append(horizon)
                        fractions.append(sum(r['status'] == 'validated' for r in group)/len(group))
                if horizons:
                    axes[1].plot(horizons, fractions, 'o-', label=f'{n} aircraft')
            axes[1].set(xlabel='Explicit horizon (s)', ylabel='Validated fraction on two seeds', ylim=(-.05, 1.05),
                        title='Horizon sensitivity\nChanged mission input, unchanged physical limits')
            axes[1].legend()
            fig.tight_layout()
            save(fig, 'followup_sensitivity')

        if summary.get('diagnostic_replays'):
            fig = plt.figure(figsize=(11.7, 8.3))
            fig.text(.07, .91, 'Execution gate diagnostic replay', fontsize=21, weight='bold')
            text = ['Exact stored plans; original checks and thresholds remain unchanged.',
                    'The campaign rejection is retained, including if diagnostics suggest an observer issue.', '']
            for item in summary['diagnostic_replays']:
                text += [item['job_id']+' / '+item['profile'],
                         'Rejected gates: '+', '.join(item['execution_failures']),
                         'First safety event: '+str(item['first_safety_event'])]
                for failed in item['failed_predicates']:
                    text.append(f"Execution predicate line {failed['line']}; aircraft index {failed['caller_drone_index']}.")
                    for name, detail in failed['constraints'].items():
                        text.append(f"  {name}: minimum margin {detail['minimum_margin']:.8g}; first time {detail.get('observed_time_s')} s.")
                        actual = detail.get('actual_boundary_checks', [])
                        if actual:
                            text.append(f"  Actual boundary bindings accepted: {sum(a['envelope_accepted'] for a in actual)}/{len(actual)}.")
                            text.append(f"  Actual boundary controls matching the tested control: {sum(a['matches_tested_control'] for a in actual)}/{len(actual)}.")
                text += ['']
            text += ['Inspect the detailed JSON trace and source before changing any check.',
                     'No production code or acceptance criterion was changed in this campaign.']
            fig.text(.07, .81, '\n'.join(text), va='top', linespacing=1.6, fontsize=11)
            save(fig, 'execution_diagnostic')

        if summary.get('proposal_screen_diagnostic'):
            fig, ax = plt.subplots(figsize=(11.7, 8.3))
            data = summary['proposal_screen_diagnostic']['rows']
            speed = [r['total_airspeed_m_s'] for r in data]
            ax.plot(speed, [r['coupled_recovery_drop_m'] for r in data], 'o-', label='Coupled preflight recovery geometry')
            ax.plot(speed, [r['proposal_recovery_drop_m'] for r in data], 'o-', label='Legacy proposal recovery geometry')
            ax.axhline(data[0]['target_altitude_m'], linestyle='--', color='black', label='Available target altitude: 30 m')
            ax.set(xlabel='Total airspeed setpoint (m/s)', ylabel='Recovery altitude loss (m)',
                   xticks=speed, title='A systematic geometric proposal limitation\n13–14 m/s proposals rejected before coupled rollout')
            ax.legend(loc='upper left')
            ax.grid(alpha=.25)
            fig.text(.13, .02, 'Read-only template comparison. Coupled preflight acceptance is not proof of a feasible full mission.', fontsize=10)
            fig.tight_layout(rect=(0, .04, 1, 1))
            save(fig, 'proposal_screen')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--follow-up', type=Path, nargs='*', default=[])
    parser.add_argument('--diagnostic', type=Path, nargs='*', default=[])
    parser.add_argument('--proposal-diagnostic', type=Path)
    args = parser.parse_args(argv)
    manifest = json.loads((args.directory / 'manifest.json').read_text())
    rows = [json.loads(line) for line in (args.directory / 'journal.jsonl').read_text().splitlines()]
    integrity = audit(args.directory, manifest, rows)
    for directory in args.follow_up:
        extra_manifest = json.loads((directory / 'manifest.json').read_text())
        extra_rows = [json.loads(line) for line in (directory / 'journal.jsonl').read_text().splitlines()]
        extra_audit = audit(directory, extra_manifest, extra_rows)
        if extra_manifest['source_sha256'] != manifest['source_sha256']:
            extra_audit['errors'].append('follow-up planning sources differ')
            extra_audit['complete'] = False
        for key in ('planned', 'recorded', 'artifact_hashes_checked'):
            integrity[key] += extra_audit[key]
        integrity['errors'] += [str(directory)+': '+e for e in extra_audit['errors']]
        integrity['missing_ids'] += extra_audit['missing_ids']
        integrity['complete'] &= extra_audit['complete']
        manifest['jobs'] += extra_manifest['jobs']
        rows += extra_rows
    if len({r['id'] for r in rows}) != len(rows):
        integrity['errors'].append('duplicate IDs across combined campaigns')
        integrity['complete'] = False
    summary = summarize(manifest, rows)
    summary['provenance'] = {'main_campaign': str(args.directory),
                             'followup_campaigns': [str(p) for p in args.follow_up],
                             'reporter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    summary['diagnostic_replays'] = []
    if args.proposal_diagnostic:
        summary['proposal_screen_diagnostic'] = json.loads(args.proposal_diagnostic.read_text())
    for path in args.diagnostic:
        item = json.loads(path.read_text())
        summary['diagnostic_replays'].append({k: item[k] for k in ('job_id', 'profile', 'failed_predicates')} | {
            'execution_failures': item['report']['execution_failures'],
            'first_safety_event': item['report']['first_safety_event'], 'artifact': str(path)})
    out = report(args.directory, manifest, rows, summary, integrity)
    print(json.dumps({'analysis': str(out), 'recorded': len(rows), 'planned': len(manifest['jobs']),
                      'audit_errors': integrity['errors'], 'complete': integrity['complete']}, indent=2))
    return 0 if integrity['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
