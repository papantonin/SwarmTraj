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
