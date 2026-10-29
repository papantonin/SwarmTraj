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
@pytest.mark.parametrize('mode', ['geometric', 'physical_mission'])
def test_v2_records_generator_and_v1_never_inherits_a_new_default(tmp_path, monkeypatch, module, factory, record, load, mode):
    snapshots, config = factory()
    config = replace(config, generator_mode=mode)
    data = record(snapshots, config)
    assert data['schema'] == module.SCHEMA and data['mission']['generator_mode'] == mode
    path = tmp_path/'scene.json'
    path.write_text(json.dumps(data))
    assert load(path) == (snapshots, config)
    # Simulate a future fresh-scene default. Explicit v1 interpretation must
    # still recover the historical geometric meaning, not the new default.
    name = 'default_scenario' if module is single else 'default_fleet'
    monkeypatch.setattr(module, name, lambda *a, **kw: (snapshots, replace(config, generator_mode='physical_mission')))
    data['schema'] = module.LEGACY_SCHEMA
    del data['mission']['generator_mode']
    path.write_text(json.dumps(data))
    assert load(path)[1].generator_mode == 'geometric'
    data['mission']['generator_mode'] = 'physical_mission'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load(path)  # v1 must not silently accept a field its schema never defined.
    data['schema'] = module.SCHEMA
    del data['mission']['generator_mode']
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load(path)  # v2 requires an explicit choice.


def test_editor_switches_and_resets_generator_without_changing_constraints():
    config = Config(dimensions=3, target_altitude=30.)
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
    config = Config(dimensions=3, speed_profile='smooth', speed_profile_range=(10.,14.),
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
def test_cli_applies_explicit_overrides_before_editor_without_launching(monkeypatch, tmp_path, count):
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
                         '--workers', '2', '--output', str(output)]) == 0
    assert seen == [(expected[0], replace(expected[1], generator_mode='physical_mission', planning_workers=2))]
    assert not output.exists() and path.read_bytes() == original_bytes


def test_historical_single_bench_rejects_generator_override(tmp_path):
    from swarm_sim import px4_gazebo as adapter
    output = tmp_path/'historical'
    assert adapter.main(['--generator-mode', 'physical_mission', '--output', str(output)]) == 2
    assert not output.exists()
