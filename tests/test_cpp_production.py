"""Native-only production, explicit archive migration and independent auditing."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import runpy
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.production import require_production
from swarm_sim.scenario_migration import migrate_scenario

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('config', [Config(dimensions=2),
    Config(dimensions=3, numerical_backend='python'),
    Config(dimensions=3, generator_mode='geometric'),
    Config(dimensions=3, flight_model='legacy')])
def test_retired_inputs_remain_readable_but_fail_before_native_work(config, monkeypatch):
    import swarm_sim.trajectory2d.native_planning as native
    monkeypatch.setattr(native, 'require_planner', lambda: pytest.fail('retired inputs reached native work'))
    before = repr(config)
    with pytest.raises(ValueError, match='retired.*not been converted'):
        plan_mission(separate_loiter_snapshot(1), config)
    assert repr(config) == before


def test_required_native_engine_and_public_default():
    from swarm_sim.trajectory2d.native_planning import require_planner
    from swarm_sim.trajectory2d.native_backend import require_execution_native
    assert require_planner().build_info()['proposal_api'] == 1
    require_execution_native()
    snapshot = replace(separate_loiter_snapshot(1)[0], altitude=80.)
    result = plan_mission((snapshot,))
    assert result.validated
    assert result.plans[0].flight_states is not None


@pytest.mark.parametrize('options', [
    ['--numerical-backend', 'python'], ['--execution-backend', 'python'],
    ['--dimensions', '2'], ['--flight-model', 'legacy'],
    ['--generator-mode', 'geometric'], ['--tracking-controller', 'legacy']])
def test_retired_cli_requests_fail_before_planning_or_output(options, monkeypatch, tmp_path):
    import swarm_sim.trajectory2d as package
    monkeypatch.setattr(package, 'plan_mission', lambda *a, **k: pytest.fail('retired request reached planner'))
    namespace = runpy.run_path(str(ROOT/'examples/trajectory_2d.py'))
    output = tmp_path/'run'
    monkeypatch.setattr(sys, 'argv', ['trajectory_3d.py', '--no-gui', '--output', str(output), *options])
    with pytest.raises(SystemExit) as error:
        namespace['main']()
    assert error.value.code == 2 and not output.exists()


@pytest.mark.parametrize('fleet', [False, True])
@pytest.mark.parametrize('version', [1, 2, 3])
def test_explicit_migration_preserves_inputs_and_records_changed_semantics(tmp_path, fleet, version):
    if fleet:
        from swarm_sim.px4_fleet import default_fleet as default, fleet_record as record, load_fleet as load
    else:
        from swarm_sim.px4_scenario import default_scenario as default, scenario_record as record, load_scenario as load
    snapshots, config = default()
    archived = record(snapshots, replace(config, generator_mode='geometric', numerical_backend='python'))
    archived['schema'] = archived['schema'][:-1] + str(version)
    if version < 3:
        del archived['mission']['numerical_backend']
    if version == 1:
        del archived['mission']['generator_mode']
    source = tmp_path/'archived.json'
    original = json.dumps(archived).encode()
    source.write_bytes(original)
    before = load(source)
    with pytest.raises(ValueError, match='retired'):
        require_production(before[1])
    output = tmp_path/'native.json'
    evidence = migrate_scenario(source, output)
    new_snapshots, new_config = load(output)
    assert source.read_bytes() == original
    assert new_snapshots == before[0]
    assert new_config == replace(before[1], generator_mode='physical_mission', numerical_backend='cpp')
    require_production(new_config)
    assert evidence['source_sha256'] == hashlib.sha256(original).hexdigest()
    assert evidence['generator_semantics_changed']
    assert not evidence['planning_performed'] and not evidence['trajectory_validation_transferred']
    assert json.loads((tmp_path/'native.json.migration.json').read_text()) == evidence
    with pytest.raises(ValueError, match='no file is overwritten'):
        migrate_scenario(source, output)
    assert source.read_bytes() == original


def test_physical_python_archive_requires_only_explicit_backend_migration(tmp_path):
    from swarm_sim.px4_scenario import default_scenario, scenario_record, load_scenario
    snapshots, config = default_scenario()
    source = tmp_path/'python.json'
    source.write_text(json.dumps(scenario_record(snapshots, replace(config, numerical_backend='python'))))
    evidence = migrate_scenario(source, tmp_path/'cpp.json')
    assert evidence['changes'] == {'numerical_backend': {'before': 'python', 'after': 'cpp'}}
    assert not evidence['generator_semantics_changed']
    assert load_scenario(source)[1].numerical_backend == 'python'


def test_editor_has_no_backend_switch_and_blocks_archived_launch():
    from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor
    from swarm_sim.px4_scenario import default_scenario
    snapshots, config = default_scenario()
    for current, launchable in [(config, True), (replace(config, numerical_backend='python'), False)]:
        editor = ScenarioEditor(snapshots, current.target, current)
        try:
            assert not hasattr(editor, 'backend') and not hasattr(editor, 'generator')
            assert editor.start_button.active is launchable
            if not launchable:
                editor.start()
                assert editor.config.numerical_backend == 'python'
                assert not editor.accepted
                assert 'retired' in editor.status.get_text()
        finally:
            plt.close(editor.figure)
