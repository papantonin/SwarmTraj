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










def test_historical_single_bench_rejects_generator_override(tmp_path):
    from swarm_sim import px4_gazebo as adapter
    output = tmp_path/'historical'
    assert adapter.main(['--generator-mode', 'physical_mission', '--output', str(output)]) == 2
    assert not output.exists()






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
    path=root/'examples/scenarios'/('px4_single_loiter_cpp.json' if count==1 else 'px4_four_loiter_cpp.json')
    assert adapter.main(['--num-drones',str(count),'--scenario',str(path),'--generator-mode','physical_mission',
        '--numerical-backend','cpp','--output',str(output)])==2
    assert not output.exists()
    if count==1:
        assert adapter.main(['--numerical-backend','cpp','--output',str(output)])==2
