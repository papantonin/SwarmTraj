"""Offline journals are evidence only when complete and internally consistent."""

import importlib
import json
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.fixture
def auditor(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module('audit_campaigns')


@pytest.fixture
def journal(auditor):
    report = {key: True for key in auditor.ACCEPTANCE_KEYS}
    report.update(flight_model='point_mass', physical_limits_verified=True,
                  contact_events_verified=True, execution_failures=[], execution_validated=True,
                  minimum_sampled_distance_m=25., minimum_distance_lower_bound_m=24.,
                  minimum_sampled_obstacle_distance_m=None, minimum_obstacle_distance_lower_bound_m=None,
                  tracking_error_upper_bounds_m=[.1, .2, .3, .4])
    return [
        {'type': 'header', 'schema': auditor.SCHEMA, 'num_drones': 4,
         'selected_scenarios': ['example'], 'selected_profiles': ['nominal'], 'seeds': [7],
         'source_sha256': {'source.py': 'a'*64}},
        {'type': 'mission', 'scenario': 'example', 'seed': 7, 'status': 'validated',
         'expected_status': 'validated', 'expectation_met': True, 'signature_sha256': 'b'*64},
        {'type': 'execution', 'scenario': 'example', 'seed': 7, 'profile': 'nominal',
         'same_nominal_plans': True, 'signature_sha256': 'b'*64, 'expected_rejection': False,
         'expectation_met': True, 'execution': report},
        {'type': 'summary', 'missions': 1, 'executions': 1, 'all_expectations_met': True},
    ]


def save(tmp_path, journal):
    path = tmp_path / 'campaign.jsonl'
    path.write_text(''.join(json.dumps(row, allow_nan=False) + '\n' for row in journal))
    return path


def test_complete_positive_journal_and_readonly_audit(auditor, journal, tmp_path):
    path = save(tmp_path, journal)
    before = path.read_bytes()
    result = auditor.audit(path)
    assert result['all_expectations_met'] and result['complete'] and result['audit_valid']
    assert result['counts']['positive_execution_passes'] == 1
    assert result['accepted_positive_metrics']['tracking_error_upper_bounds_m'] == {'value': .4, 'observations': 1}
    assert result['accepted_positive_metrics']['minimum_sampled_obstacle_distance_m'] == {'value': None, 'observations': 0}
    assert path.read_bytes() == before


def test_historical_header_without_schema_is_supported(auditor, journal, tmp_path):
    del journal[0]['schema']
    assert auditor.audit(save(tmp_path, journal))['all_expectations_met']


def test_legacy_unscoped_header_counts_evidence_without_inventing_coverage(auditor, journal, tmp_path):
    for field in ('schema', 'selected_scenarios', 'selected_profiles', 'seeds', 'source_sha256'):
        del journal[0][field]
    result = auditor.audit(save(tmp_path, journal))
    assert result['counts']['missions'] == result['counts']['positive_execution_passes'] == 1
    assert not result['coverage_declared'] and not result['complete']
    assert not result['audit_valid'] and not result['all_expectations_met']
    assert len(result['issues']) == 2


@pytest.mark.parametrize('cut', (0, 1, 2, 3))
def test_interruption_never_becomes_success(auditor, journal, tmp_path, cut):
    result = auditor.audit(save(tmp_path, journal[:cut]))
    assert not result['all_expectations_met'] and not result['complete']
    assert any('summary' in issue for issue in result['issues'])


def test_forged_success_summary_cannot_hide_missing_profile(auditor, journal, tmp_path):
    journal[0]['selected_profiles'].append('mild')
    result = auditor.audit(save(tmp_path, journal))
    assert not result['all_expectations_met'] and not result['complete']
    assert result['missing_executions'] == [['example', 7, 'mild']]


def test_missing_requested_mission(auditor, journal, tmp_path):
    journal[0]['seeds'].append(19)
    result = auditor.audit(save(tmp_path, journal))
    assert not result['complete'] and result['missing_missions'] == [['example', 19]]


@pytest.mark.parametrize('record', (0, 1, 2, 3))
def test_duplicate_records_are_not_counted_as_success(auditor, journal, tmp_path, record):
    journal.insert(record + 1, journal[record])
    result = auditor.audit(save(tmp_path, journal))
    assert not result['audit_valid'] and not result['all_expectations_met']


def test_expected_planning_rejection_has_no_execution(auditor, journal, tmp_path):
    journal[1].update(status='initial_separation_violation', expected_status='initial_separation_violation')
    del journal[2]
    journal[-1]['executions'] = 0
    result = auditor.audit(save(tmp_path, journal))
    assert result['all_expectations_met']
    assert result['counts']['expected_planning_rejections'] == 1
    assert not result['accepted_positive_metrics']['minimum_sampled_distance_m']['observations']


def test_unexpected_planning_failure_retains_failed_expectation(auditor, journal, tmp_path):
    journal[1].update(status='no_candidate', expectation_met=False)
    del journal[2]
    journal[-1].update(executions=0, all_expectations_met=False)
    result = auditor.audit(save(tmp_path, journal))
    assert result['complete'] and result['audit_valid'] and not result['all_expectations_met']
    assert result['counts']['unexpected_planning_rejections'] == 1
    assert result['unmet_expectations'] == [{'record': 'mission', 'scenario': 'example', 'seed': 7,
                                            'status': 'no_candidate', 'expected_status': 'validated'}]


def test_outside_clock_rejection_counts_without_positive_metrics(auditor, journal, tmp_path):
    journal[0]['selected_profiles'] = ['outside_clock']
    journal[2].update(profile='outside_clock', expected_rejection=True)
    journal[2]['execution'].update(tube_conditions_verified=False, execution_validated=False,
                                    execution_failures=['tube_conditions_verified'])
    result = auditor.audit(save(tmp_path, journal))
    assert result['all_expectations_met']
    assert result['counts']['expected_execution_rejections'] == 1
    assert result['accepted_positive_metrics']['minimum_sampled_distance_m']['value'] is None


def test_unexpected_outside_clock_acceptance_fails(auditor, journal, tmp_path):
    journal[0]['selected_profiles'] = ['outside_clock']
    journal[2].update(profile='outside_clock', expected_rejection=True, expectation_met=False)
    journal[-1]['all_expectations_met'] = False
    result = auditor.audit(save(tmp_path, journal))
    assert result['audit_valid'] and not result['all_expectations_met']
    assert result['counts']['unexpected_execution_acceptances'] == 1


def test_positive_execution_rejection_retained(auditor, journal, tmp_path):
    journal[2].update(expectation_met=False)
    journal[2]['execution'].update(physical_limits_verified=False, execution_validated=False,
                                    execution_failures=['physical_limits_verified'])
    journal[-1]['all_expectations_met'] = False
    result = auditor.audit(save(tmp_path, journal))
    assert result['audit_valid'] and not result['all_expectations_met']
    assert result['counts']['positive_execution_failures'] == 1
    assert result['unmet_expectations'][0]['execution_failures'] == ['physical_limits_verified']


def test_signed_obstacle_penetration_is_valid_failed_evidence(auditor, journal, tmp_path):
    journal[2]['expectation_met'] = False
    journal[2]['execution'].update(obstacle_clearance_verified_in_execution_model=False,
        execution_validated=False, execution_failures=['obstacle_clearance_verified_in_execution_model'],
        minimum_sampled_obstacle_distance_m=-3., minimum_obstacle_distance_lower_bound_m=-3.2)
    journal[-1]['all_expectations_met'] = False
    result = auditor.audit(save(tmp_path, journal))
    assert result['audit_valid'] and not result['all_expectations_met']
    assert result['counts']['positive_execution_failures'] == 1
    assert result['accepted_positive_metrics']['minimum_sampled_obstacle_distance_m']['observations'] == 0


def test_summary_must_be_terminal(auditor, journal, tmp_path):
    journal.append({'type': 'error', 'accepted': False})
    result = auditor.audit(save(tmp_path, journal))
    assert not result['complete'] and not result['audit_valid']


@pytest.mark.parametrize('mutation', (
    lambda j: j[0].update(schema='unknown'),
    lambda j: j[0].update(num_drones=True),
    lambda j: j[0].update(seeds=[7, 7]),
    lambda j: j[0].update(selected_profiles=['unknown']),
    lambda j: j[0].update(source_sha256={}),
    lambda j: j[1].update(expectation_met=1),
    lambda j: j[2].update(signature_sha256='c'*64),
    lambda j: j[2].update(expected_rejection=True),
    lambda j: j[2].update(profile='unknown'),
    lambda j: j[2]['execution'].update(execution_validated=False),
    lambda j: j[2]['execution'].update(execution_failures=['physical_limits_verified']),
    lambda j: j[2]['execution'].update(physical_limits_verified=False),
    lambda j: j[2]['execution'].pop('contact_events_verified'),
    lambda j: j[2]['execution'].update(reservation_policy='grid'),
    lambda j: j[2]['execution'].update(reservation_policy='unknown'),
    lambda j: j[2]['execution'].update(minimum_sampled_distance_m=True),
    lambda j: j[2]['execution'].update(minimum_sampled_distance_m=-1),
    lambda j: j[2]['execution'].update(tracking_error_upper_bounds_m=[.2]),
    lambda j: j[-1].update(missions=True),
    lambda j: j[-1].update(all_expectations_met=False),
))
def test_inconsistent_records_fail_closed(auditor, journal, tmp_path, mutation):
    mutation(journal)
    result = auditor.audit(save(tmp_path, journal))
    assert not result['audit_valid'] and not result['all_expectations_met']


def test_mutated_plan_is_not_a_positive_pass(auditor, journal, tmp_path):
    journal[2].update(same_nominal_plans=False, expectation_met=False)
    journal[-1]['all_expectations_met'] = False
    result = auditor.audit(save(tmp_path, journal))
    assert result['audit_valid'] and not result['all_expectations_met']
    assert result['counts']['plan_mutations'] == 1
    assert result['counts']['positive_execution_failures'] == 1


@pytest.mark.parametrize('bad_line', ('{', 'NaN', '{"type":"header","type":"summary"}',
                                    '{"value":Infinity}', '{"value":1e10000}', '[]', ''))
def test_malformed_json_is_reported_not_raised(auditor, tmp_path, bad_line):
    path = tmp_path / 'bad.jsonl'
    path.write_text(bad_line + '\n')
    result = auditor.audit(path)
    assert not result['audit_valid'] and any('line 1:' in issue for issue in result['issues'])


def test_error_record_cannot_be_success(auditor, journal, tmp_path):
    journal[-1:] = [{'type': 'error', 'accepted': False, 'error_type': 'RuntimeError', 'message': 'worker failure'}]
    result = auditor.audit(save(tmp_path, journal))
    assert not result['complete'] and result['counts']['errors'] == 1


def test_unreadable_file_and_unknown_record(auditor, journal, tmp_path):
    assert not auditor.audit(tmp_path / 'absent.jsonl')['audit_valid']
    journal.insert(2, {'type': 'unknown'})
    assert not auditor.audit(save(tmp_path, journal))['audit_valid']


def test_cli_exit_codes_json_and_no_duplicate_inputs(auditor, journal, tmp_path, capsys):
    path = save(tmp_path, journal)
    assert auditor.main([str(path), '--json']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['counts']['executions'] == 1
    assert report['journals'][0]['accepted_positive_metrics']['minimum_sampled_distance_m']['value'] == 25
    with pytest.raises(SystemExit) as error:
        auditor.main([str(path), str(path)])
    assert error.value.code == 2
    path.write_text(path.read_text().rsplit('\n', 2)[0] + '\n')
    assert auditor.main([str(path)]) == 2


def test_auditor_runs_without_simulator_imports(journal, tmp_path):
    path = save(tmp_path, journal)
    script = Path(__file__).resolve().parents[1] / 'scripts/audit_campaigns.py'
    result = subprocess.run([sys.executable, '-I', '-S', str(script), str(path), '--json'],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['all_expectations_met']


@pytest.mark.parametrize('model', ('point_mass', 'legacy'))
@pytest.mark.parametrize('reservations', ('grid', 'none'))
def test_audit_acceptance_contract_matches_recorded_engine_gate(auditor, journal, model, reservations):
    from swarm_sim.trajectory2d.execution_safety import execution_failures
    report = journal[2]['execution']
    report.update(flight_model=model, reservation_policy=reservations, reservation_adherence_verified=True)
    assert not execution_failures(report) and auditor._execution(report)
    for key in (*auditor.ACCEPTANCE_KEYS, 'physical_limits_verified', 'contact_events_verified',
                'reservation_adherence_verified'):
        trial = dict(report, **{key: False})
        failures = execution_failures(trial)
        trial.update(execution_failures=failures, execution_validated=not failures)
        assert auditor._execution(trial) == (not failures)
