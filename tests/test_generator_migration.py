"""Explicit generator selection and strict, backwards-compatible scene records."""
from dataclasses import replace
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pytest

from swarm_sim import px4_scenario as single
from swarm_sim import px4_fleet as fleet
from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor


@pytest.mark.parametrize('module, factory, record, load', [
    (single, single.default_scenario, single.scenario_record, single.load_scenario),
    (fleet, fleet.default_fleet, fleet.fleet_record, fleet.load_fleet)])
@pytest.mark.parametrize('mode, backend', [('geometric','python'),('physical_mission','python'),('physical_mission','cpp')])
def test_v3_backend_and_strict_v1_v2_meanings(tmp_path, monkeypatch, module, factory, record, load, mode, backend):
    snapshots, base = factory()
    config = replace(base, generator_mode=mode, numerical_backend=backend)
    data = record(snapshots, config)
    assert data['schema'] == module.SCHEMA and data['schema'].endswith('-v3')
    assert data['mission']['generator_mode'] == mode
    assert data['mission']['numerical_backend'] == backend
    path = tmp_path/'scene.json'
    def save(value):path.write_text(json.dumps(value))
    save(data)
    assert load(path) == (snapshots, config)
    # Future defaults must never change historical archived meanings.
    name = 'default_scenario' if module is single else 'default_fleet'
    monkeypatch.setattr(module, name, lambda *a, **kw: (snapshots,
        replace(base,generator_mode='physical_mission',numerical_backend='cpp')))
    for schema in (module.LEGACY_SCHEMA,module.PREVIOUS_SCHEMA):
        old=json.loads(json.dumps(data));old['schema']=schema
        del old['mission']['numerical_backend']
        if schema==module.LEGACY_SCHEMA:del old['mission']['generator_mode']
        save(old)
        expected=replace(base,generator_mode='geometric' if schema==module.LEGACY_SCHEMA else mode,
                         numerical_backend='python')
        assert load(path)==(snapshots,expected)
        old['mission']['numerical_backend']='cpp';save(old)
        with pytest.raises(ValueError):load(path)
        if schema==module.LEGACY_SCHEMA:
            del old['mission']['numerical_backend'];old['mission']['generator_mode']='physical_mission';save(old)
            with pytest.raises(ValueError):load(path)
    for key in ('generator_mode','numerical_backend'):
        missing=json.loads(json.dumps(data));del missing['mission'][key];save(missing)
        with pytest.raises(ValueError):load(path)
    invalid=json.loads(json.dumps(data));invalid['mission'].update(generator_mode='geometric',numerical_backend='cpp');save(invalid)
    with pytest.raises(ValueError):load(path)


def test_editor_switches_and_resets_generator_without_changing_constraints():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30.)
    editor = ScenarioEditor(separate_loiter_snapshot(), config.target, config)
    try:
        assert editor.generator.active
        editor.generator.set_active(0)
        assert editor.config == replace(config, generator_mode='physical_mission')
        assert editor.snapshots == list(editor.original[0])
        editor.reset()
        assert editor.config == config and not editor.generator.get_status()[0]
    finally:
        plt.close(editor.figure)


def test_editor_rejects_unsupported_physical_passage_request_without_fallback():
    config = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', speed_profile='smooth', speed_profile_range=(10.,14.),
                    reservation_policy='grid', passage_guidance='safe_windows')
    editor = ScenarioEditor(separate_loiter_snapshot(), config.target, config)
    try:
        editor.generator.set_active(0)
        assert editor.config == config and not editor.generator.get_status()[0]
        assert 'does not support' in editor.status.get_text()
    finally:
        plt.close(editor.figure)


def test_planar_editor_keeps_historical_generator_disabled():
    editor = ScenarioEditor(separate_loiter_snapshot(), (140.,50.), Config())
    try:
        assert not editor.generator.active
        editor.generator.set_active(0)
        assert editor.config.generator_mode == 'geometric'
        assert not editor.generator.get_status()[0]
    finally:
        plt.close(editor.figure)


@pytest.mark.parametrize('count', [1, 4])
@pytest.mark.parametrize('backend', ['python','cpp'])
def test_cli_applies_explicit_overrides_before_editor_without_launching(monkeypatch, tmp_path, count, backend):
    from pathlib import Path
    from swarm_sim import px4_gazebo as adapter
    import swarm_sim.trajectory2d.scenario_editor as ui
    root = Path(__file__).resolve().parents[1]
    path = root/'examples/scenarios'/('px4_single_loiter.json' if count == 1 else 'px4_four_loiter.json')
    original_bytes = path.read_bytes()
    expected = single.load_scenario(path) if count == 1 else fleet.load_fleet(path)
    seen = []
    def cancel(snapshots, target, config, **kwargs):
        seen.append((snapshots, config))
        return None
    monkeypatch.setattr(ui, 'edit_scenario', cancel)
    monkeypatch.setattr(adapter, 'managed_run', lambda *a, **kw: pytest.fail('cancelled editor must not launch'))
    output = tmp_path/'cancelled'
    assert adapter.main(['--num-drones', str(count), '--scenario', str(path),
                         '--edit-scenario', '--run', '--generator-mode', 'physical_mission',
                         '--numerical-backend', backend, '--workers', '2', '--output', str(output)]) == 0
    assert seen == [(expected[0], replace(expected[1], generator_mode='physical_mission', numerical_backend=backend, planning_workers=2))]
    assert not output.exists() and path.read_bytes() == original_bytes


def test_historical_single_bench_rejects_generator_override(tmp_path):
    from swarm_sim import px4_gazebo as adapter
    output = tmp_path/'historical'
    assert adapter.main(['--generator-mode', 'physical_mission', '--output', str(output)]) == 2
    assert not output.exists()


@pytest.mark.parametrize('original_cpp',[False,True])
def test_editor_backend_selection_reset_and_incompatible_generator(monkeypatch,original_cpp):
    from swarm_sim.trajectory2d import native_backend
    monkeypatch.setattr(native_backend,'require_native',lambda:None)
    base=Config(dimensions=3, generator_mode='geometric', numerical_backend='python',target_altitude=30.)
    config=replace(base,generator_mode='physical_mission',numerical_backend='cpp') if original_cpp else base
    editor=ScenarioEditor(separate_loiter_snapshot(),config.target,config)
    try:
        if not original_cpp:
            assert not editor.backend.active
            editor.backend.set_active(0)
            assert editor.config==config and not editor.backend.get_status()[0]
            editor.generator.set_active(0)
            assert editor.backend.active
            editor.backend.set_active(0)
        assert editor.config.numerical_backend=='cpp'
        editor.generator.set_active(0)
        assert editor.config.generator_mode=='physical_mission' and editor.config.numerical_backend=='cpp'
        assert editor.generator.get_status()[0] and 'Switch off C++' in editor.status.get_text()
        editor.backend.set_active(0)
        editor.generator.set_active(0)
        assert editor.config==base and not editor.backend.active
        editor.reset()
        assert editor.config==config
        assert editor.backend.get_status()[0]==original_cpp
    finally:plt.close(editor.figure)


def test_editor_missing_native_module_rejects_selection_and_launch(monkeypatch):
    from swarm_sim.trajectory2d import native_backend
    def unavailable():raise RuntimeError('C++ backend unavailable; build_native.sh')
    monkeypatch.setattr(native_backend,'require_native',unavailable)
    config=Config(dimensions=3, numerical_backend='python',generator_mode='physical_mission',target_altitude=30.)
    editor=ScenarioEditor(separate_loiter_snapshot(),config.target,config)
    try:
        editor.backend.set_active(0)
        assert editor.config==config and not editor.backend.get_status()[0]
        assert 'unavailable' in editor.status.get_text()
        editor.config=replace(config,numerical_backend='cpp')
        editor._sync_backend()
        editor.start()
        assert not editor.accepted and plt.fignum_exists(editor.figure.number)
        assert 'unavailable' in editor.status.get_text()
    finally:plt.close(editor.figure)


@pytest.mark.parametrize('count',[1,4])
def test_cli_backend_requires_physical_generator_and_missing_binary_fails_closed(monkeypatch,tmp_path,count):
    from swarm_sim import px4_gazebo as adapter
    from swarm_sim.trajectory2d import native_backend
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    path=root/'examples/scenarios'/('px4_single_loiter.json' if count==1 else 'px4_four_loiter.json')
    monkeypatch.setattr(adapter,'managed_run',lambda *a,**kw:pytest.fail('must not launch'))
    output=tmp_path/'invalid'
    assert adapter.main(['--num-drones',str(count),'--scenario',str(path),
        '--numerical-backend','cpp','--output',str(output)])==2
    def unavailable():raise RuntimeError('C++ backend unavailable')
    monkeypatch.setattr(native_backend,'require_native',unavailable)
    assert adapter.main(['--num-drones',str(count),'--scenario',str(path),'--generator-mode','physical_mission',
        '--numerical-backend','cpp','--output',str(output)])==2
    assert not output.exists()
    if count==1:
        assert adapter.main(['--numerical-backend','cpp','--output',str(output)])==2
