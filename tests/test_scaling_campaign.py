"""Campaign integrity: input coverage, failure visibility, and source guards."""

import importlib
import json
from pathlib import Path
import subprocess

import pytest


@pytest.fixture
def campaign(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'scripts'))
    return importlib.import_module('benchmark_scaling_campaign')


def test_matrix_is_unique_and_covers_all_counts_and_seed_pairs(campaign):
    jobs = campaign.make_jobs([1, 2, 3, 4, 5], [7, 19, 42], 10,
                             ['core', 'random', 'ablation', 'replay', 'controls'])
    assert len(jobs) == 319
    assert len({j['id'] for j in jobs}) == len(jobs)
    for n in range(1, 6):
        subset = [j for j in jobs if j['num_drones'] == n]
        assert {j['suite'] for j in subset} == {'core', 'random', 'ablation', 'replay', 'controls'}
        for job in subset:
            snapshots, config = campaign.decode(job)
            assert len(snapshots) == n
            assert {s.drone_id for s in snapshots} == set(range(n))
            assert config.coupled_flight and config.dt == .2
            assert config.position_error == 2 and config.clock_error == .15


def test_random_scenes_repeat_exactly_and_retain_full_inputs(campaign):
    jobs = campaign.make_jobs([1, 5], [7], 3, ['random'])
    assert jobs == campaign.make_jobs([1, 5], [7], 3, ['random'])
    assert len({j['config']['target'] for j in jobs}) == 3
    for job in jobs:
        snapshots, config = campaign.decode(job)
        assert all(10 <= s.speed <= 14 and 40 <= s.altitude <= 115 for s in snapshots)
        assert config.cruise_altitude_range == (30, 120)


def test_ablation_changes_only_declared_search_options(campaign):
    jobs = campaign.make_jobs([4], [7], 0, ['ablation'])
    base = next(j for j in jobs if j['family'] == 'adaptive_margin')
    allowed = {'planning_strategy', 'selection_policy', 'conflict_guidance', 'search_order', 'candidate_guidance'}
    for job in jobs:
        assert job['snapshots'] == base['snapshots']
        changed = {k for k in job['config'] if job['config'][k] != base['config'][k]}
        assert changed <= allowed


def test_controls_are_real_rejections_for_each_fleet_size(campaign):
    for job in campaign.make_jobs([1, 2, 3, 4, 5], [7], 0, ['controls']):
        snapshots, config = campaign.decode(job)
        result = campaign.plan_mission(snapshots, config)
        assert result.status == job['expected_status'], job['id']
        assert not result.plans


@pytest.mark.parametrize('checkpoint', [False, True])
def test_timeout_is_censored_and_preserves_completed_planning(campaign, monkeypatch, tmp_path, checkpoint):
    for folder in ('logs', 'checkpoints', 'completed', 'missions'):
        (tmp_path / folder).mkdir()
    job = campaign.make_jobs([1], [7], 0, ['core'])[0]
    manifest = {'jobs': [job], 'source_sha256': campaign.sources(), 'timeout_s': .01}
    class Process:
        pid = 987654321
        returncode = -15
        calls = 0
        def __init__(self, *args, **kwargs):
            if checkpoint:
                campaign.write_json(tmp_path / 'checkpoints' / (job['id']+'.json'),
                                    {'id': job['id'], 'status': 'validated', 'planning_wall_s': .005})
        def wait(self, timeout=None):
            self.calls += 1
            if self.calls == 1:
                raise subprocess.TimeoutExpired('synthetic worker', timeout)
    monkeypatch.setattr(campaign.subprocess, 'Popen', Process)
    monkeypatch.setattr(campaign.os, 'killpg', lambda *a: None)
    campaign.run(tmp_path, manifest)
    record = json.loads((tmp_path / 'journal.jsonl').read_text())
    assert record['completion'] == ('execution_timeout' if checkpoint else 'planning_timeout')
    assert record['status'] == ('validated' if checkpoint else None)
    assert record['timeout_s'] == .01


def test_resume_skips_recorded_jobs_and_source_changes_abort(campaign, monkeypatch, tmp_path):
    job = campaign.make_jobs([1], [7], 0, ['controls'])[0]
    manifest = {'jobs': [job], 'source_sha256': campaign.sources(), 'timeout_s': 1}
    row = {'id': job['id'], 'status': 'destination_unreachable_within_horizon', 'completion': 'complete'}
    (tmp_path / 'journal.jsonl').write_text(json.dumps(row)+'\n')
    monkeypatch.setattr(campaign.subprocess, 'Popen', lambda *a, **kw: pytest.fail('must not rerun'))
    assert campaign.run(tmp_path, manifest) == 0
    (tmp_path / 'journal.jsonl').write_text('')
    monkeypatch.setattr(campaign, 'sources', lambda: {'changed': 'yes'})
    with pytest.raises(RuntimeError, match='source changed'):
        campaign.run(tmp_path, manifest)


def test_reference_replay_changes_only_generator_and_supports_all_230_requests(campaign):
    reference = {'jobs': campaign.make_jobs([1, 2, 3, 4, 5], [7, 19, 42], 10,
                                            ['controls', 'core', 'random', 'replay'])}
    before = json.dumps(reference, sort_keys=True)
    replay = campaign.replay_jobs(reference, ['core', 'random'], 'physical_mission')
    assert len(replay) == 230
    assert sum(j['family'] == 'rich_strict' for j in replay) == 15
    originals = {j['id']: j for j in reference['jobs']}
    for job in replay:
        original = originals[job['id']]
        assert {k: v for k, v in job.items() if k != 'config'} == {
            k: v for k, v in original.items() if k != 'config'}
        assert {k: v for k, v in job['config'].items() if k != 'generator_mode'} == {
            k: v for k, v in original['config'].items() if k != 'generator_mode'}
        snapshots, config = campaign.decode(job)
        assert config.generator_mode == 'physical_mission' and len(snapshots) == job['num_drones']
    assert json.dumps(reference, sort_keys=True) == before


def test_worker_records_unsupported_without_planning_and_artifact_audits(campaign, monkeypatch, tmp_path):
    reporter = importlib.import_module('report_scaling_campaign')
    jobs = campaign.make_jobs([1], [7], 0, ['core'])
    job = next(j for j in jobs if j['family'] == 'rich_strict')
    job['config']['generator_mode'] = 'physical_mission'
    job['config']['passage_guidance'] = 'safe_windows'
    job['config']['reservation_policy'] = 'grid'
    manifest = {'jobs': [job], 'source_sha256': campaign.sources()}
    campaign.write_json(tmp_path / 'manifest.json', manifest)
    for folder in ('missions', 'completed'):
        (tmp_path / folder).mkdir()
    monkeypatch.setattr(campaign, 'plan_mission', lambda *a: pytest.fail('must reject before planning'))
    campaign.worker(tmp_path, job['id'])
    row = json.loads((tmp_path / 'completed' / (job['id']+'.json')).read_text())
    row['num_drones'] = 1
    assert row['status'] == 'unsupported_generator_configuration'
    assert row['attempted'] == 0 and not row['executions']
    assert not row['diagnostics']['planning_executed']
    assert reporter.audit(tmp_path, manifest, [row])['errors'] == []


def test_worker_does_not_mask_unexpected_configuration_errors(campaign, monkeypatch, tmp_path):
    job = next(j for j in campaign.make_jobs([1], [7], 0, ['core']) if j['family'] == 'rich_strict')
    job['config']['generator_mode'] = 'physical_mission'
    campaign.write_json(tmp_path / 'manifest.json', {'jobs': [job], 'source_sha256': campaign.sources()})
    def fail(_):
        raise ValueError('unexpected input corruption')
    monkeypatch.setattr(campaign, 'decode', fail)
    with pytest.raises(ValueError, match='unexpected input corruption'):
        campaign.worker(tmp_path, job['id'])


def test_generator_comparison_retains_regressions_timeouts_and_unsupported(campaign):
    reporter = importlib.import_module('report_generator_replay')
    old = [{'id': str(i), 'status': status, 'completion': 'complete'} for i, status in enumerate(
        ('validated', 'no_candidate', 'validated', 'validated', 'validated'))]
    new = [{'id': str(i), 'status': status, 'completion': completion, 'suite': 'core',
            'family': 'baseline', 'num_drones': 1} for i, (status, completion) in enumerate((
                ('validated', 'complete'), ('validated', 'complete'), ('no_candidate', 'complete'),
                (None, 'planning_timeout'), ('unsupported_generator_configuration', 'complete')))]
    comparison = reporter.compare(old, new)
    assert [c['outcome'] for c in comparison] == ['retained', 'gained', 'lost', 'lost', 'lost']
    assert comparison[3]['new_status'] == 'planning_timeout'
    assert comparison[4]['new_status'] == 'unsupported_generator_configuration'


def test_replay_report_detects_safety_and_snapshot_changes(campaign):
    reporter = importlib.import_module('report_generator_replay')
    original = campaign.make_jobs([1], [7], 0, ['core'])[0]
    job, = campaign.replay_jobs({'jobs': [original]}, ['core'], 'physical_mission')
    assert reporter.input_differences(job, original, generator_mode='physical_mission') == []
    job['config']['position_error'] = 0
    job['snapshots'][0]['speed'] += 1
    assert len(reporter.input_differences(job, original, generator_mode='physical_mission')) == 2


def test_explicit_process_override_preserves_replay_pairs_and_safety(campaign):
    reporter = importlib.import_module('report_generator_replay')
    original = campaign.make_jobs([2], [7], 0, ['core', 'replay'])
    before = json.dumps(original, sort_keys=True)
    chosen = [original[0]['id'], *(j['id'] for j in original if j['suite'] == 'replay')]
    jobs = campaign.select_jobs(original, chosen, 2)
    assert len(jobs) == 3
    assert [j['config']['planning_workers'] for j in jobs] == [2, 1, 2]
    assert json.dumps(original, sort_keys=True) == before
    assert reporter.input_differences(jobs[0], original[0])
    assert not reporter.input_differences(jobs[0], original[0], 2)
    assert reporter.input_differences(jobs[0], original[0], 1)
    assert not reporter.input_differences(jobs[0], original[0], 2, 'physical_mission')
    assert reporter.input_differences(jobs[0], original[0], 2, 'geometric')
    jobs[0]['config']['position_error'] = 0
    assert reporter.input_differences(jobs[0], original[0], 2)
    for identifiers in ([], ['unknown'], [chosen[0], chosen[0]]):
        with pytest.raises(ValueError):
            campaign.select_jobs(original, identifiers, 2)
    with pytest.raises(ValueError):
        campaign.select_jobs(original, planning_workers=-1)


def test_live_progress_does_not_present_partial_campaign_as_validated(campaign):
    reporter = importlib.import_module('report_generator_replay')
    manifest = {'jobs': list(range(259)), 'timeout_s': 120}
    rows = [{'suite': 'core', 'status': 'validated', 'completion': 'complete'},
            {'suite': 'controls', 'status': 'invalid_initial_state', 'completion': 'complete',
             'expectation_met': True}]
    html = reporter.progress_html(manifest, rows)
    assert '1/230' in html and '2/259' in html
    assert 'provisional progress' in html and 'Recorded generator/backend selections are preserved' in html
    assert 'Unsupported configuration requests' in html


def test_explicit_backend_replay_preserves_legacy_inputs_and_guards_binary(campaign, monkeypatch):
    reporter = importlib.import_module('report_generator_replay')
    native = importlib.import_module('swarm_sim.trajectory2d.native_backend')
    original = campaign.make_jobs([1], [7], 0, ['core'])[0]
    original['config'].pop('numerical_backend')
    before = json.dumps(original, sort_keys=True)
    python_job, = campaign.replay_jobs({'jobs': [original]}, ['core'], 'physical_mission')
    assert reporter.input_differences(python_job, original, generator_mode='physical_mission') == []
    job, = campaign.replay_jobs({'jobs': [original]}, ['core'], 'physical_mission', 'cpp')
    assert reporter.input_differences(job, original)
    assert not reporter.input_differences(job, original, generator_mode='physical_mission', numerical_backend='cpp')
    assert reporter.input_differences(job, original, generator_mode='physical_mission', numerical_backend='python')
    job['config']['position_error'] = 0
    assert reporter.input_differences(job, original, generator_mode='physical_mission', numerical_backend='cpp')
    assert json.dumps(original, sort_keys=True) == before
    monkeypatch.setattr(native, 'identity', lambda: {'binary_sha256': 'current'})
    manifest = {'source_sha256': campaign.sources(), 'native_backend_identity': native.identity()}
    campaign.check_sources(manifest)
    manifest['native_backend_identity'] = {'binary_sha256': 'changed'}
    with pytest.raises(RuntimeError, match='identity changed'):
        campaign.check_sources(manifest)


def test_replay_audit_rejects_unrecorded_generator_override(campaign):
    reporter = importlib.import_module('report_generator_replay')
    original = campaign.make_jobs([1], [7], 0, ['core'])[0]
    original['config'].pop('generator_mode')
    preserved, = campaign.replay_jobs({'jobs': [original]}, ['core'], None)
    assert not reporter.input_differences(preserved, original)
    changed, = campaign.replay_jobs({'jobs': [original]}, ['core'], 'physical_mission')
    assert reporter.input_differences(changed, original)
    assert not reporter.input_differences(changed, original, generator_mode='physical_mission')


def test_saved_campaign_omissions_keep_historical_modes_and_explicit_choices(campaign):
    job = campaign.make_jobs([1], [7], 0, ['core'])[0]
    job['config'].pop('generator_mode')
    job['config'].pop('numerical_backend')
    original = json.dumps(job, sort_keys=True)
    _, config = campaign.decode(job)
    assert (config.generator_mode, config.numerical_backend) == ('geometric', 'python')
    preserved, = campaign.replay_jobs({'jobs': [job]}, ['core'], None)
    assert preserved['config']['generator_mode'] == 'geometric'
    assert preserved['config']['numerical_backend'] == 'python'
    assert json.dumps(job, sort_keys=True) == original
    native, = campaign.replay_jobs({'jobs': [job]}, ['core'], 'physical_mission', 'cpp')
    assert campaign.decode(native)[1].numerical_backend == 'cpp'
    for field in ('generator_mode', 'numerical_backend'):
        invalid = json.loads(json.dumps(preserved))
        invalid['config'][field] = None
        with pytest.raises(ValueError, match='explicit'):
            campaign.decode(invalid)


def test_campaign_cli_records_explicit_native_override(campaign, monkeypatch, tmp_path):
    native = importlib.import_module('swarm_sim.trajectory2d.native_backend')
    monkeypatch.setattr(native, 'identity', lambda: {'binary_sha256': 'test-only'})
    saved = campaign.make_jobs([1], [7], 0, ['core'])[0]
    reference = tmp_path/'reference.json'
    reference.write_text(json.dumps({'jobs': [saved], 'counts': [1], 'seeds': [7], 'timeout_s': 120}))
    captured = []
    monkeypatch.setattr(campaign, 'run', lambda directory, manifest: captured.append(manifest) or 0)
    assert campaign.main(['--output', str(tmp_path/'new'), '--reference', str(reference),
        '--suites', 'core', '--generator-mode', 'physical_mission', '--numerical-backend', 'cpp',
        '--no-execute']) == 0
    manifest, = captured
    assert manifest['native_backend_identity'] == {'binary_sha256': 'test-only'}
    assert manifest['reference']['numerical_backend_override'] == 'cpp'
    assert manifest['jobs'][0]['config']['numerical_backend'] == 'cpp'
    assert manifest['timeout_s'] == 120


def test_duplicate_execution_cannot_hide_missing_profile_or_job(campaign):
    reporter = importlib.import_module('report_generator_replay')
    manifest = {'seeds': [7], 'execute': True, 'execution_families': ['baseline', 'rich_strict']}
    rows = [{'id': family, 'suite': 'core', 'family': family, 'seed': 7, 'status': 'validated',
             'completion': 'complete', 'executions': [{'profile': profile} for profile in
                 ('nominal', 'gust_sensor', 'outside_clock')]}
            for family in ('baseline', 'rich_strict')]
    assert reporter.execution_coverage(manifest, rows)['complete']
    # The aggregate remains six trials, but one duplicated nominal run must
    # never stand in for the missing rich-profile gust execution.
    rows[1]['executions'][1] = {'profile': 'nominal'}
    coverage = reporter.execution_coverage(manifest, rows)
    assert coverage['recorded_trials'] == coverage['expected_trials'] == 6
    assert not coverage['complete']
    assert coverage['missing_trials'] == [{'id': 'rich_strict', 'profile': 'gust_sensor'}]
    assert coverage['duplicate_trials'] == [{'id': 'rich_strict', 'profile': 'nominal'}]


def test_execution_coverage_rejects_disabled_or_unexpected_trials(campaign):
    reporter = importlib.import_module('report_generator_replay')
    manifest = {'seeds': [7], 'execute': False}
    assert not reporter.execution_coverage(manifest, [])['complete']
    manifest['execute'] = True
    row = {'id': 'rejected', 'suite': 'core', 'family': 'baseline', 'seed': 7,
           'status': 'no_candidate', 'executions': [{'profile': 'nominal'}]}
    coverage = reporter.execution_coverage(manifest, [row])
    assert not coverage['complete']
    assert coverage['unexpected_trials'] == [{'id': 'rejected', 'profile': 'nominal'}]
