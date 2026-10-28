"""Audit saved simulation campaigns without planning or executing any flight.

This checks journal consistency, not trajectories or the authenticity of records.
Only complete, internally consistent journals can satisfy the audit gate.
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re


SCHEMA = 'swarm-scenario-campaign-v1'
PROFILES = ('nominal', 'mild', 'gust_sensor', 'outside_clock')
SHA256 = re.compile(r'[0-9a-f]{64}\Z')
ACCEPTANCE_KEYS = (
    'all_arrived', 'tube_conditions_verified', 'separation_verified_in_execution_model',
    'arrival_deadline_verified_in_execution', 'post_arrival_time_verified_in_execution',
    'approach_sectors_verified_in_execution', 'vertical_limits_verified',
    'descent_angle_verified_in_execution', 'obstacle_clearance_verified_in_execution_model',
    'controller_protection_verified',
)
METRICS = {
    'minimum_sampled_distance_m': min,
    'minimum_distance_lower_bound_m': min,
    'minimum_sampled_obstacle_distance_m': min,
    'minimum_obstacle_distance_lower_bound_m': min,
    'tracking_error_upper_bounds_m': max,
}


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('non-finite JSON number')
    return number


def _constant(value):
    raise ValueError(f'invalid JSON constant: {value}')


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _selection(value, name, *, seeds=False):
    _require(isinstance(value, list) and bool(value), f'{name} must be a nonempty list')
    valid = all(type(v) is int and v >= 0 for v in value) if seeds else all(
        isinstance(v, str) and bool(v) for v in value)
    _require(valid and len(set(value)) == len(value), f'{name} has invalid or duplicate entries')
    return value


def _execution(report):
    """Cross-check the recorded acceptance gate; do not rerun numerical checks."""
    _require(isinstance(report, dict), 'execution must be an object')
    _require(report.get('flight_model') in ('point_mass', 'legacy'), 'unknown flight model')
    _require(report.get('reservation_policy', 'none') in ('none', 'grid'), 'unknown reservation policy')
    keys = list(ACCEPTANCE_KEYS)
    if report['flight_model'] == 'point_mass':
        keys.extend(('physical_limits_verified', 'contact_events_verified'))
    if report.get('reservation_policy') == 'grid':
        keys.append('reservation_adherence_verified')
    _require(all(type(report.get(k)) is bool for k in keys), 'missing/non-boolean acceptance criterion')
    failures = [k for k in keys if not report[k]]
    _require(report.get('execution_failures') == failures, 'execution_failures contradict acceptance criteria')
    _require(type(report.get('execution_validated')) is bool
             and report['execution_validated'] == (not failures), 'execution_validated contradicts acceptance criteria')
    return report['execution_validated']


def audit(path):
    """Return a JSON-safe audit, preserving failures and incomplete evidence."""
    path = Path(path)
    issues, counts, metrics = [], Counter(), {key: [] for key in METRICS}
    unmet = []
    header, summary = None, None
    missions, executions = {}, set()
    expected_missions, profiles = None, None
    all_met, error_seen = True, False
    line_number, summary_line = 0, None
    try:
        with path.open(encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line, object_pairs_hook=_object, parse_float=_float, parse_constant=_constant)
                    _require(isinstance(row, dict), 'record must be an object')
                    kind = row.get('type')
                    _require(summary is None, 'record after terminal summary')
                    if line_number == 1:
                        _require(kind == 'header', 'first record must be a header')
                    if kind == 'header':
                        _require(line_number == 1 and header is None, 'unexpected/duplicate header')
                        _require(row.get('schema', SCHEMA) == SCHEMA, 'unsupported campaign schema')
                        selections = {}
                        for name in ('selected_scenarios', 'seeds', 'selected_profiles'):
                            if name not in row and 'schema' not in row:
                                selections[name] = None
                            else:
                                selections[name] = _selection(row.get(name), name, seeds=name == 'seeds')
                        _require(type(row.get('num_drones')) is int and row['num_drones'] > 0,
                                 'num_drones must be a positive integer')
                        sources = row.get('source_sha256')
                        if sources is None and 'schema' not in row:
                            issues.append('historical header has no source fingerprints')
                        else:
                            _require(isinstance(sources, dict) and bool(sources) and all(
                                isinstance(k, str) and isinstance(v, str) and SHA256.fullmatch(v)
                                for k, v in sources.items()), 'missing/invalid source fingerprints')
                        profiles = selections['selected_profiles']
                        _require(profiles is None or all(p in PROFILES for p in profiles),
                                 'unsupported execution profile in campaign schema')
                        header = row
                        names, seeds = selections['selected_scenarios'], selections['seeds']
                        if names is not None and seeds is not None:
                            expected_missions = {(name, seed) for name in names for seed in seeds}
                        if None in selections.values():
                            issues.append('historical header does not declare the full requested schedule; coverage is unknown')
                        continue
                    _require(header is not None, 'no valid header')
                    if kind == 'mission':
                        key = (row.get('scenario'), row.get('seed'))
                        _require(isinstance(key[0], str) and bool(key[0]) and type(key[1]) is int and key[1] >= 0
                                 and (expected_missions is None or key in expected_missions), 'unrequested/invalid mission')
                        _require(key not in missions, 'duplicate mission')
                        _require(all(isinstance(row.get(k), str) and row[k] for k in ('status', 'expected_status')),
                                 'missing mission status/expectation')
                        digest = row.get('signature_sha256')
                        _require(isinstance(digest, str) and SHA256.fullmatch(digest), 'invalid mission signature')
                        met = row['status'] == row['expected_status']
                        _require(type(row.get('expectation_met')) is bool and row['expectation_met'] == met,
                                 'mission expectation_met contradicts statuses')
                        missions[key] = row
                        counts['missions'] += 1
                        counts['planned' if row['status'] == 'validated' else 'planning_rejected'] += 1
                        if row['status'] != 'validated':
                            counts['expected_planning_rejections' if met else 'unexpected_planning_rejections'] += 1
                        counts['mission_expectations_met' if met else 'mission_expectations_unmet'] += 1
                        if not met:
                            unmet.append({'record': 'mission', 'scenario': key[0], 'seed': key[1],
                                          'status': row['status'], 'expected_status': row['expected_status']})
                        all_met &= met
                    elif kind == 'execution':
                        key = (row.get('scenario'), row.get('seed'))
                        _require(type(key[1]) is int and key in missions, 'execution without preceding mission')
                        mission = missions[key]
                        _require(mission['status'] == 'validated', 'execution of rejected mission')
                        profile = row.get('profile')
                        _require(profile in PROFILES and (profiles is None or profile in profiles),
                                 'unrequested execution profile')
                        trial = (*key, profile)
                        _require(trial not in executions, 'duplicate execution')
                        _require(type(row.get('same_nominal_plans')) is bool, 'missing plan immutability result')
                        _require(row.get('signature_sha256') == mission['signature_sha256'],
                                 'execution signature differs from mission')
                        rejected = profile == 'outside_clock'
                        _require(type(row.get('expected_rejection')) is bool and row['expected_rejection'] == rejected,
                                 'expected_rejection contradicts profile')
                        report = row.get('execution')
                        accepted = _execution(report)
                        met = row['same_nominal_plans'] and (not accepted if rejected else accepted)
                        _require(type(row.get('expectation_met')) is bool and row['expectation_met'] == met,
                                 'execution expectation_met contradicts results')
                        # Check metric types even on rejected trials; never turn absent
                        # obstacle observations into zero-distance measurements.
                        values = {}
                        for field in METRICS:
                            value = report.get(field)
                            if value is None:
                                values[field] = []
                                continue
                            items = value if field == 'tracking_error_upper_bounds_m' else [value]
                            _require(isinstance(items, list) and bool(items) and all(
                                type(v) in (float, int) and math.isfinite(v) for v in items),
                                f'invalid metric: {field}')
                            # Obstacle surface distances are signed: negative
                            # values in rejected executions are valid evidence.
                            if field in ('minimum_sampled_distance_m', 'tracking_error_upper_bounds_m'):
                                _require(min(items) >= 0, f'negative distance/error metric: {field}')
                            if field == 'tracking_error_upper_bounds_m':
                                _require(len(items) == header['num_drones'] and min(items) >= 0,
                                         'invalid per-aircraft tracking metric')
                            values[field] = items
                        executions.add(trial)
                        counts['executions'] += 1
                        category = ('expected_execution_rejections' if rejected and not accepted else
                                    'unexpected_execution_acceptances' if rejected else
                                    'positive_execution_passes' if met else 'positive_execution_failures')
                        counts[category] += 1
                        counts['execution_expectations_met' if met else 'execution_expectations_unmet'] += 1
                        if not met:
                            unmet.append({'record': 'execution', 'scenario': key[0], 'seed': key[1],
                                          'profile': profile, 'execution_validated': accepted,
                                          'same_nominal_plans': row['same_nominal_plans'],
                                          'execution_failures': report['execution_failures']})
                        if not row['same_nominal_plans']:
                            counts['plan_mutations'] += 1
                        if met and not rejected:
                            for field, items in values.items():
                                if items:
                                    metrics[field].append(METRICS[field](items))
                        all_met &= met
                    elif kind == 'error':
                        _require(row.get('accepted') is False, 'error record must not be accepted')
                        error_seen = True
                        counts['errors'] += 1
                        all_met = False
                    elif kind == 'summary':
                        for field in ('missions', 'executions'):
                            _require(type(row.get(field)) is int and row[field] == counts[field],
                                     f'summary {field} disagrees with records')
                        _require(type(row.get('all_expectations_met')) is bool
                                 and row['all_expectations_met'] == all_met,
                                 'summary expectation result disagrees with records')
                        summary = row
                        summary_line = line_number
                    else:
                        raise ValueError(f'unknown record type: {kind!r}')
                except (ValueError, TypeError, OverflowError, RecursionError) as error:
                    issues.append(f'line {line_number}: {error}')
    except (OSError, UnicodeError) as error:
        issues.append(f'cannot read journal: {error}')
    missing_missions = sorted(expected_missions - missions.keys()) if expected_missions is not None else []
    missing_executions = sorted((*key, profile) for key, mission in missions.items()
                               if mission['status'] == 'validated' for profile in (profiles or [])
                               if (*key, profile) not in executions)
    if header is None:
        issues.append('missing valid header')
    if summary is None:
        issues.append('missing valid terminal summary (possibly interrupted)')
    if missing_missions:
        issues.append(f'{len(missing_missions)} requested mission(s) missing')
    if missing_executions:
        issues.append(f'{len(missing_executions)} requested execution(s) missing')
    if error_seen:
        issues.append('campaign contains an error record')
    coverage_declared = expected_missions is not None and profiles is not None
    complete = (coverage_declared and header is not None and summary is not None and summary_line == line_number
                and not missing_missions and not missing_executions and not error_seen)
    return {'path': str(path), 'complete': complete, 'audit_valid': not issues,
            'coverage_declared': coverage_declared,
            'all_expectations_met': bool(all_met and complete and not issues),
            'counts': dict(counts), 'issues': issues, 'unmet_expectations': unmet,
            'missing_missions': [list(key) for key in missing_missions],
            'missing_executions': [list(key) for key in missing_executions],
            'accepted_positive_metrics': {key: {'value': METRICS[key](values) if values else None,
                                               'observations': len(values)} for key, values in metrics.items()}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journals', type=Path, nargs='+')
    parser.add_argument('--json', action='store_true', help='print machine-readable results')
    args = parser.parse_args(argv)
    if len({path.resolve() for path in args.journals}) != len(args.journals):
        parser.error('duplicate input path; refusing to count the same journal twice')
    audits = [audit(path) for path in args.journals]
    success = all(item['all_expectations_met'] for item in audits)
    totals = Counter()
    for item in audits:
        totals.update(item['counts'])
    result = {'schema': 'swarm-campaign-audit-v1', 'all_expectations_met': success,
              'counts': dict(totals), 'journals': audits,
              'scope': 'Recorded evidence consistency only; no trajectory revalidation or certification. '
                       'Repeated experiments in different files count as separate trials.'}
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    else:
        for item in audits:
            state = 'PASS' if item['all_expectations_met'] else 'FAIL'
            print(f"{state} {item['path']} (complete={item['complete']}, consistent={item['audit_valid']})")
            print('  ' + json.dumps(item['counts'], sort_keys=True))
            for issue in item['issues']:
                print(f'  {issue}')
            for trial in item['unmet_expectations']:
                print('  UNMET ' + json.dumps(trial, sort_keys=True))
        print('TOTAL ' + json.dumps(result['counts'], sort_keys=True))
        print(result['scope'])
    return 0 if success else 2


if __name__ == '__main__':
    raise SystemExit(main())
