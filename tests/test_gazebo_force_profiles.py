"""Force benches bind own physical profiles without inventing airframe geometry."""
from dataclasses import FrozenInstanceError,replace
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import pytest
from swarm_sim.gazebo_force_model import ForceModel,ForceAssumptions
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.px4_prototype import build_model
from test_px4_prototype import template


def profile(name='custom_fixed_wing'):
    return load_aircraft_profile(f'examples/aircraft/{name}.json')


def test_binding_is_immutable_strict_and_separate_from_physical_identity(tmp_path):
    p=profile();old=p.record();assumptions=ForceAssumptions(5.6,-.03,'user_supplied')
    model=ForceModel(p,assumptions)
    assert model.plugin_values()['max_brake']==0 and model.profile.airframe.mass_kg==2.5
    assert model.test_speeds()==(10.,12.,15.,16.)
    assert model.cruise_test_speed()==12
    assert p.record()==old
    assert model.record()['binding_sha256']!=ForceModel(p).record()['binding_sha256']
    assert model.record()['profile']==ForceModel(p).record()['profile']
    with pytest.raises(FrozenInstanceError):model.assumptions=ForceAssumptions()
    path=tmp_path/'assumptions.json';path.write_text(json.dumps(assumptions.to_dict()))
    assert ForceAssumptions.load(path)==assumptions
    with pytest.raises(ValueError,match='v3 physical'):
        ForceModel(profile('fixed_wing_reference'))
    without_speeds=replace(p,speeds=None);fallback=ForceModel(without_speeds)
    assert len(fallback.test_speeds())==4
    assert fallback.cruise_test_speed()==pytest.approx(sum(fallback.test_speeds()[::3])/2)


@pytest.mark.parametrize('values',[(True,0.),(float('nan'),0.),(0.,0.),(4.75,float('inf')),(4.75,2.)])
def test_bad_lift_assumptions_reject(values):
    with pytest.raises(ValueError):ForceAssumptions(*values)


def test_unknown_or_duplicate_fields_reject(tmp_path):
    path=tmp_path/'bad.json'
    document=ForceAssumptions().to_dict();document['mass_kg']=5
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError,match='exact versioned'):ForceAssumptions.load(path)
    path.write_text('{"schema":"x","schema":"y"}')
    with pytest.raises(ValueError,match='duplicate'):ForceAssumptions.load(path)
    with pytest.raises(ValueError,match='outside bench'):
        ForceModel(profile()).plugin_values(brake_fraction=1.1)


@pytest.mark.parametrize('name',['compact_5kg','custom_fixed_wing'])
def test_cpp_qualification_uses_profile_mass_polar_brake_and_explicit_lift_assumptions(tmp_path,name):
    assumptions=ForceAssumptions(5.6,-.03,'user_supplied')
    config=tmp_path/'lift.json';config.write_text(json.dumps(assumptions.to_dict()))
    output=tmp_path/'trial'
    run=subprocess.run(['.venv/bin/python','scripts/qualify_compact_forces.py','--output',str(output),
        '--aircraft-profile',f'examples/aircraft/{name}.json','--force-assumptions',str(config)],capture_output=True,text=True)
    assert run.returncode==0,run.stderr
    report=json.loads((output/'report.json').read_text())
    assert report['passed'] and len(report['trials'])==24
    assert report['profile']==profile(name).record()
    assert report['force_assumptions']==assumptions.record()
    assert report['binding_sha256']==ForceModel(profile(name),assumptions).record()['binding_sha256']


def test_unsupported_lift_requests_are_retained_as_failed_trials(tmp_path):
    config=tmp_path/'lift.json';config.write_text(json.dumps(ForceAssumptions(.01,0.).to_dict()))
    output=tmp_path/'trial'
    run=subprocess.run(['.venv/bin/python','scripts/qualify_compact_forces.py','--output',str(output),
                       '--force-assumptions',str(config)],capture_output=True,text=True)
    assert run.returncode==1
    report=json.loads((output/'report.json').read_text())
    assert not report['passed'] and len(report['trials'])==24
    assert all(not row['passed'] and 'forward-flow' in row['error'] for row in report['trials'])


def test_prototype_keeps_geometry_gate_but_uses_separate_lift_binding():
    lidar=Path('examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text()
    assumptions=ForceAssumptions(5.6,-.03)
    sdf,record=build_model(profile('compact_5kg'),template(),lidar,matched_polar=True,force_assumptions=assumptions)
    plugin=ET.fromstring(sdf).find("model/plugin[@name='swarm::CompactPolar']")
    assert float(plugin.findtext('lift_slope'))==5.6
    assert record['force_model']['force_assumptions']==assumptions.record()
    with pytest.raises(ValueError,match='restricted'):
        build_model(profile(),template(),lidar,matched_polar=True)
    with pytest.raises(ValueError,match='require the explicit'):
        build_model(profile('compact_5kg'),template(),lidar,force_assumptions=assumptions)
