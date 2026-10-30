"""A paired benchmark must reject corrupt inputs and changed discrete decisions."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from benchmark_native_execution import compare_array, compare_reports, load_document
import numpy as np


def test_report_comparison_retains_negative_acceptance_and_event_identity():
    a = dict(execution_backend='python', execution_validated=False,
        execution_failures=['tube_conditions_verified'],
        safety_events=[dict(kind='clock_bound_exceeded', drone_id=3, first_observed_time_s=.4)],
        final_error=1.)
    b = {**a, 'execution_backend': 'cpp', 'execution_backend_identity': {'abi': 1}, 'final_error': 1. + 1e-12}
    assert compare_reports(a, b) == []
    assert compare_reports(a, {**b, 'execution_validated': True})
    assert compare_reports(a, {**b, 'safety_events': []})
    assert compare_reports(a, {**b, 'final_error': 1.1})


def test_saved_mission_hash_is_checked_before_reconstruction(tmp_path):
    path = tmp_path / 'mission.json'
    path.write_text(json.dumps({'sha256': 'corrupted'}))
    with pytest.raises(ValueError, match='hash mismatch'):
        load_document(path)
    doc = {'config': {'aircraft_profiles': [[0, 'must not be discarded']]}}
    doc['sha256'] = hashlib.sha256(json.dumps(doc, sort_keys=True, allow_nan=False).encode()).hexdigest()
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='common-airframe'):
        load_document(path)


def test_absent_pair_infinity_is_distinct_from_invalid_physics():
    absent = np.full(3, np.inf)
    assert compare_array(absent, absent, 'distances', True) == (0., None)
    for a, b, name, single in ((absent, np.zeros(3), 'distances', True),
            (absent, absent, 'distances', False), (absent, absent, 'flight_actual', True),
            (np.full(3, np.nan), np.full(3, np.nan), 'distances', True)):
        assert compare_array(a, b, name, single)[1] is not None


def test_reanalysis_rejects_changed_runtime_sources(tmp_path, monkeypatch):
    import benchmark_native_execution as benchmark
    manifest = dict(sources={'runtime.py': 'before'}, native={'abi': 1})
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(benchmark, 'sources', lambda: {'runtime.py': 'after'})
    monkeypatch.setattr(benchmark, 'identity', lambda: {'abi': 1})
    with pytest.raises(RuntimeError, match='runtime must match'):
        benchmark.reanalyze(tmp_path)


@pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                    reason='optional C++ extension not built')
def test_cli_execution_backend_rejects_legacy_before_planning(monkeypatch, tmp_path):
    import runpy
    import swarm_sim.trajectory2d as trajectory
    monkeypatch.setattr(trajectory, 'plan_mission', lambda *a, **kw: pytest.fail('must reject before planning'))
    script = SCRIPTS.parent / 'examples/trajectory_2d.py'
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, 'argv', [str(script), '--no-gui', '--execution-backend', 'cpp',
        '--tracking-controller', 'legacy', '--output', str(tmp_path)])
    with pytest.raises(SystemExit) as error:
        namespace['main']()
    assert error.value.code == 2
