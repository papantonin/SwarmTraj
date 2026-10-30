"""New native defaults, explicit reference choices and immutable archived modes."""
from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import runpy
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d import native_backend
from swarm_sim.trajectory2d.planning import validate_individual

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('settings,expected', [
    ({}, ('physical_mission', 'cpp')),
    ({'dimensions': 3}, ('physical_mission', 'cpp')),
    ({'dimensions': 3, 'flight_model': 'legacy'}, ('geometric', 'python')),
    ({'dimensions': 3, 'generator_mode': 'geometric'}, ('geometric', 'python')),
    ({'dimensions': 3, 'numerical_backend': 'python'}, ('physical_mission', 'python')),
    ({'dimensions': 3, 'generator_mode': 'physical_recovery_seed'}, ('physical_recovery_seed', 'cpp')),
])
def test_omitted_modes_resolve_once_to_concrete_values(settings, expected):
    config = Config(**settings)
    assert (config.generator_mode, config.numerical_backend) == expected
    assert asdict(config)['generator_mode'] == expected[0]
    assert asdict(config)['numerical_backend'] == expected[1]
    assert replace(config, horizon=120).numerical_backend == expected[1]
    assert config.planning_workers == 1


@pytest.mark.parametrize('settings', [
    {'dimensions': 2, 'numerical_backend': 'cpp'},
    {'dimensions': 3, 'flight_model': 'legacy', 'numerical_backend': 'cpp'},
    {'dimensions': 3, 'generator_mode': 'geometric', 'numerical_backend': 'cpp'},
    {'dimensions': 2, 'generator_mode': 'physical_mission'},
    {'dimensions': 3, 'passage_guidance': 'safe_windows'},
])
def test_defaults_do_not_repair_explicitly_unsupported_requests(settings):
    with pytest.raises(ValueError):
        Config(**settings)


def test_missing_default_native_backend_fails_before_catalog_or_python_fallback(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    def unavailable():
        raise RuntimeError('C++ backend unavailable; run ./scripts/build_native.sh')
    monkeypatch.setattr(native_backend, 'require_native', unavailable)
    monkeypatch.setattr(planning, '_plan_mission', lambda *a: pytest.fail('unexpected Python fallback'))
    with pytest.raises(RuntimeError, match='build_native.sh'):
        plan_mission(separate_loiter_snapshot(1), Config(dimensions=3))


@pytest.mark.parametrize('options,mode,backend,workers', [
    ([], 'physical_mission', 'cpp', 1),
    (['--workers', '0'], 'physical_mission', 'cpp', 0),
    (['--workers', '2'], 'physical_mission', 'cpp', 2),
])
def test_cli_uses_effective_modes_and_preserves_explicit_worker_choices(monkeypatch, tmp_path, options, mode, backend, workers):
    import swarm_sim.trajectory2d as package
    import swarm_sim.trajectory2d.native_planning as native
    class Reached(Exception):
        pass
    def inspect(snapshots, config):
        assert (config.generator_mode, config.numerical_backend, config.planning_workers) == (mode, backend, workers)
        raise Reached
    monkeypatch.setattr(package, 'plan_mission', inspect)
    monkeypatch.setattr(native, 'require_planner', lambda: None)
    namespace = runpy.run_path(str(ROOT/'examples/trajectory_2d.py'))
    monkeypatch.setattr(sys, 'argv', ['trajectory_3d.py', '--no-gui', '--output', str(tmp_path/'run'), *options])
    with pytest.raises(Reached):
        namespace['main']()


def test_cli_missing_default_extension_reports_build_command_without_output(monkeypatch, tmp_path, capsys):
    import swarm_sim.trajectory2d.native_planning as native
    def unavailable():
        raise RuntimeError('C++ backend unavailable; run ./scripts/build_native.sh')
    monkeypatch.setattr(native, 'require_planner', unavailable)
    namespace = runpy.run_path(str(ROOT/'examples/trajectory_2d.py'))
    monkeypatch.setattr(sys, 'argv', ['trajectory_3d.py', '--no-gui', '--output', str(tmp_path/'run')])
    with pytest.raises(SystemExit) as error:
        namespace['main']()
    assert error.value.code == 2 and 'build_native.sh' in capsys.readouterr().err
    assert not (tmp_path/'run').exists()


@pytest.mark.parametrize('count', [1, 4])
def test_fresh_editor_and_cpp_examples_select_native_while_archives_keep_python(tmp_path, count):
    from swarm_sim import px4_scenario as single, px4_fleet as fleet
    module = single if count == 1 else fleet
    factory = module.default_scenario if count == 1 else module.default_fleet
    record = module.scenario_record if count == 1 else module.fleet_record
    load = module.load_scenario if count == 1 else module.load_fleet
    snapshots, config = factory()
    assert (config.generator_mode, config.numerical_backend) == ('physical_mission', 'cpp')
    from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor
    editor = ScenarioEditor(snapshots, config.target, config)
    try:
        assert editor.start_button.active
        assert not hasattr(editor, "generator") and not hasattr(editor, "backend")
        editor.reset()
        assert editor.config == config
    finally:
        plt.close(editor.figure)
    name = 'single' if count == 1 else 'four'
    old = load(ROOT/f'examples/scenarios/px4_{name}_loiter.json')[1]
    new = load(ROOT/f'examples/scenarios/px4_{name}_loiter_cpp.json')[1]
    assert (old.generator_mode, old.numerical_backend) == ('geometric', 'python')
    assert (new.generator_mode, new.numerical_backend) == ('physical_mission', 'cpp')
    assert replace(new, generator_mode='geometric', numerical_backend='python') == old
    for field in ('generator_mode', 'numerical_backend'):
        data = record(snapshots, config)
        data['mission'][field] = None
        path = tmp_path/'invalid.json'
        path.write_text(json.dumps(data))
        with pytest.raises(ValueError, match='explicit'):
            load(path)


@pytest.mark.parametrize('count', [1, 4])
def test_fresh_px4_launcher_passes_native_defaults_to_editor_without_starting_docker(monkeypatch, tmp_path, count):
    from swarm_sim import px4_gazebo
    import swarm_sim.trajectory2d.scenario_editor as ui
    seen = []
    def cancel(snapshots, target, config, **kwargs):
        seen.append(config)
        return None
    monkeypatch.setattr(ui, 'edit_scenario', cancel)
    monkeypatch.setattr(px4_gazebo, 'managed_run', lambda *a: pytest.fail('cancelled editor launched Docker'))
    output = tmp_path/'cancelled'
    assert px4_gazebo.main(['--num-drones', str(count), '--edit-scenario', '--run', '--output', str(output)]) == 0
    assert len(seen) == 1
    assert (seen[0].generator_mode, seen[0].numerical_backend) == ('physical_mission', 'cpp')
    assert not output.exists()


@pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None, reason='native extension not built')
def test_default_native_plan_matches_explicit_selection_and_python_validator():
    config = Config(dimensions=3, target=(500., 0.), target_altitude=30.,
                    descent_angle_deg=30., max_descent_rate=8., horizon=120.,
                    cruise_altitude_range=(70., 100.), max_extra_loiter=0., candidates=1, attempts=3)
    snapshot = replace(separate_loiter_snapshot(1)[0], altitude=80.).with_speed(13., config.limits)
    result = plan_mission((snapshot,), config)
    explicit = plan_mission((snapshot,), replace(config, generator_mode='physical_mission', numerical_backend='cpp'))
    assert result.validated and result.plans[0].to_dict() == explicit.plans[0].to_dict()
    assert validate_individual(result.plans[0], snapshot, config, backend='python')
