"""The campaign retains failures, rejects invalid cases and never hides exceptions."""

import importlib
import json
from pathlib import Path
import sys

import pytest

from swarm_sim.trajectory2d import Result


@pytest.fixture
def campaign(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module('validate_scenarios')


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_all_campaign_cases_use_four_aircraft_and_explicit_negative_expectations(campaign):
    cases = campaign.scenarios(1)
    assert all(len(case.snapshots) == 4 for case in cases.values())
    assert cases['fixed_obstacles_strict'].expected_status == 'validated'
    assert cases['unreachable_horizon'].expected_status == 'destination_unreachable_within_horizon'
    assert cases['invalid_initial_pair'].expected_status == 'initial_separation_violation'
    for snapshot in cases['heterogeneous_speeds'].snapshots:
        assert snapshot.speed / snapshot.state[3] == pytest.approx(25)


def test_negative_cases_are_reported_without_execution(campaign, monkeypatch, tmp_path):
    output = tmp_path / 'negative.jsonl'
    monkeypatch.setattr(campaign, 'simulate', lambda *a, **kw: pytest.fail('rejected mission must not execute'))
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', '--workers', '1', '--scenarios',
        'unreachable_horizon', 'invalid_initial_pair', '--output', str(output)])
    assert campaign.main() == 0
    data = records(output)
    missions = [r for r in data if r['type'] == 'mission']
    assert len(missions) == 2 and all(r['expectation_met'] for r in missions)
    assert data[-1]['all_expectations_met'] and data[-1]['executions'] == 0
    assert data[0]['schema'] == 'swarm-scenario-campaign-v1'
    auditor = importlib.import_module('audit_campaigns')
    assert auditor.audit(output)['all_expectations_met']


def test_unexpected_planning_failure_is_preserved_and_returns_failure(campaign, monkeypatch, tmp_path):
    output = tmp_path / 'failed.jsonl'
    monkeypatch.setattr(campaign, 'plan_mission', lambda *a: Result('no_candidate', 'Synthetic rejection'))
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', '--scenarios', 'default', '--output', str(output)])
    assert campaign.main() == 2
    data = records(output)
    assert data[1]['status'] == 'no_candidate' and not data[1]['expectation_met']
    assert data[-1]['missions'] == 1 and not data[-1]['all_expectations_met']


def test_worker_error_is_flushed_then_propagated_without_partial_acceptance(campaign, monkeypatch, tmp_path):
    output = tmp_path / 'error.jsonl'
    def fail(*a):
        raise RuntimeError('worker failure')
    monkeypatch.setattr(campaign, 'plan_mission', fail)
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', '--scenarios', 'default', '--output', str(output)])
    with pytest.raises(RuntimeError, match='worker failure'):
        campaign.main()
    data = records(output)
    assert len(data) == 2 and data[-1]['type'] == 'error'
    assert data[-1]['accepted'] is False and data[-1]['error_type'] == 'RuntimeError'


def test_existing_campaign_output_is_never_overwritten(campaign, monkeypatch, tmp_path):
    output = tmp_path / 'existing.jsonl'
    output.write_text('user data\n')
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', '--output', str(output)])
    with pytest.raises(SystemExit) as error:
        campaign.main()
    assert error.value.code == 2 and output.read_text() == 'user data\n'


def test_source_change_aborts_and_is_recorded_before_planning(campaign, monkeypatch, tmp_path):
    output = tmp_path / 'changed.jsonl'
    versions = iter(({'source': 'before'}, {'source': 'after'}))
    monkeypatch.setattr(campaign, 'source_fingerprint', lambda: next(versions))
    monkeypatch.setattr(campaign, 'plan_mission', lambda *a: pytest.fail('changed source must not be used'))
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', '--scenarios', 'default', '--output', str(output)])
    with pytest.raises(RuntimeError, match='source changed'):
        campaign.main()
    data = records(output)
    assert data[-1]['type'] == 'error' and data[-1]['accepted'] is False


@pytest.mark.parametrize('option,values', (('--seeds', ['7', '7']),
                                         ('--scenarios', ['default', 'default']),
                                         ('--profiles', ['nominal', 'nominal'])))
def test_duplicate_campaign_selections_rejected_before_output(campaign, monkeypatch, tmp_path, option, values):
    output = tmp_path / 'duplicate.jsonl'
    monkeypatch.setattr(sys, 'argv', ['validate_scenarios.py', option, *values, '--output', str(output)])
    with pytest.raises(SystemExit) as error:
        campaign.main()
    assert error.value.code == 2 and not output.exists()
