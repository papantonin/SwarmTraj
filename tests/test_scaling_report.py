"""Reports must distinguish planning success, execution failure and censoring."""

import importlib
from pathlib import Path

import pytest


@pytest.fixture
def reporter(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module('report_scaling_campaign')


def test_timeouts_are_not_zero_second_completed_runs(reporter):
    manifest = {'counts': [1], 'seeds': [7]}
    rows = [
        {'id': 'a', 'num_drones': 1, 'suite': 'core', 'family': 'baseline', 'seed': 7,
         'status': 'validated', 'completion': 'complete', 'planning_wall_s': 2., 'process_wall_s': 3., 'attempted': 2},
        {'id': 'b', 'num_drones': 1, 'suite': 'random', 'family': 'stratified', 'seed': 1000,
         'status': None, 'completion': 'planning_timeout', 'process_wall_s': 120.},
        {'id': 'c', 'num_drones': 1, 'suite': 'controls', 'family': 'unreachable', 'seed': 7,
         'status': 'destination_unreachable_within_horizon', 'completion': 'complete',
         'planning_wall_s': .001, 'process_wall_s': 1., 'expectation_met': True},
    ]
    summary = reporter.summarize(manifest, rows)
    group = summary['by_count']['1']
    assert group['missions'] == 2 and group['validated'] == 1 and group['planning_timeouts'] == 1
    assert group['completed_planning_seconds']['median'] == 2.
    assert group['completed_planning_seconds']['n'] == 1
    assert summary['controls'] == {'total': 1, 'expectations_met': 1}


def test_missing_duplicate_and_unexpected_records_are_audit_failures(reporter, tmp_path):
    manifest = {'jobs': [{'id': 'a'}, {'id': 'b'}]}
    result = reporter.audit(tmp_path, manifest, [{'id': 'a', 'status': None}])
    assert not result['complete'] and result['missing_ids'] == ['b']
    result = reporter.audit(tmp_path, manifest, [{'id': 'a', 'status': None}, {'id': 'a', 'status': None}])
    assert 'duplicate journal IDs' in result['errors']
    result = reporter.audit(tmp_path, manifest, [{'id': 'c', 'status': None}])
    assert 'unexpected journal IDs' in result['errors']


def test_missing_mission_cannot_pass_artifact_audit(reporter, tmp_path):
    result = reporter.audit(tmp_path, {'jobs': [{'id': 'a'}]}, [{'id': 'a', 'status': 'validated'}])
    assert not result['complete']
    assert result['errors'] == ['missing mission: a']


def test_empty_time_groups_stay_missing(reporter):
    assert reporter.quantiles([]) == {'n': 0, 'median': None, 'p90': None, 'p95': None, 'max': None}
