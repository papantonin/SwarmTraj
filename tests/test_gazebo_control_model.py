"""Own-airframe identity, FLU moment signs, and trim authority are explicit."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest
from swarm_sim.gazebo_force_model import ForceModel,ForceAssumptions
from swarm_sim.gazebo_control_model import ControlModel
from swarm_sim.trajectory2d import load_aircraft_profile
from swarm_sim.px4_prototype import build_model
from test_px4_prototype import template

PROFILE=Path('examples/aircraft/compact_5kg.json')
CONTROL=Path('examples/gazebo/compact_physics/control_model.json')


def models():
    force=ForceModel(load_aircraft_profile(PROFILE));return force,ControlModel.load(CONTROL,force)


def module():
    spec=importlib.util.spec_from_file_location('control_qualification','scripts/qualify_control_model.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


@pytest.fixture(scope='module')
def probe(tmp_path_factory):
    m=module();binary=m.compile_probe(tmp_path_factory.mktemp('controls'))
    return m,binary,*models()


def test_cpp_matches_independent_rate_damping_and_control_signs(probe):
    m,binary,force,control=probe;v=force.cruise_test_speed();alpha=control.alpha_trim_rad
    velocity=v*np.array([np.cos(alpha),0,-np.sin(alpha)])
    zero=m.probe(binary,force,control,velocity)
    np.testing.assert_allclose(zero[3:6],0,atol=1e-12)
    for axis in range(3):
        rates=np.zeros(3);rates[axis]=.2
        damped=m.probe(binary,force,control,velocity,rates)
        assert damped[3+axis]<0
        assert np.dot(damped[3:6],rates)<0
    roll=m.probe(binary,force,control,velocity,positions=(-.25,.25,0,0,0))
    pitch=m.probe(binary,force,control,velocity,positions=(0,0,.25,0,0))
    yaw=m.probe(binary,force,control,velocity,positions=(0,0,0,.25,0))
    assert roll[3]>0 and pitch[4]<0 and yaw[5]<0
    clipped=m.probe(binary,force,control,velocity,positions=(-10,10,10,10,10))
    np.testing.assert_array_equal(clipped[6:],[1,1,1,1])


def test_trim_positive_and_missing_brake_negative(probe):
    m,binary,force,control=probe
    for speed,angle in [(force.cruise_test_speed(),0),(force.profile.airspeed_for('dive'),-30)]:
        row=m.trim(force,control,speed,angle)
        assert abs(row['normal_residual_n'])<1e-8 and abs(row['elevator_fraction'])<=1
    a=replace(force.profile.airframe,max_airbrake_drag_n=0)
    no_brake=ForceModel(replace(force.profile,airframe=a,flight=replace(force.profile.flight,**a.derived())))
    with pytest.raises(ValueError,match='authority'):m.trim(no_brake,control,force.profile.airspeed_for('dive'),-30)


def test_control_identity_schema_inertia_and_geometry_are_strict(tmp_path):
    force,control=models()
    with pytest.raises(ValueError,match='binding'):
        control.require_matches(ForceModel(force.profile,ForceAssumptions(5.,-.06)))
    with pytest.raises(ValueError,match='inertias'):replace(control,base_inertia_kg_m2=(1,1,3))
    with pytest.raises(ValueError,match='finite numbers'):replace(control,pitch_control=True)
    path=tmp_path/'bad.json';document=control.to_dict();document['ignored_parameter']=1;path.write_text(json.dumps(document))
    with pytest.raises(ValueError,match='exact'):ControlModel.load(path,force)
    lidar=Path('examples/gazebo/models/rc_cessna_lidar/model.sdf').read_text()
    sdf,record=build_model(force.profile,template(),lidar,matched_polar=True,control_model=control)
    import xml.etree.ElementTree as ET
    model=ET.fromstring(sdf).find('model')
    assert sum(float(e.text) for e in model.findall('link/inertial/mass'))==pytest.approx(5,abs=1e-12)
    assert model.find("joint[@name='airbrake_joint']") is not None
    assert record['control_model']==control.record() and not record['px4_flight_qualified']
    assert record['actuator_source'].startswith('own measured')


def test_startup_failure_after_readiness_cannot_report_success(tmp_path,monkeypatch):
    """A later parameter/readback error must invalidate an earlier ready state."""
    import types
    mavsdk=pytest.importorskip('mavsdk')
    import mavsdk.plugins.telemetry as telemetry_module
    import mavsdk.plugins.param as param_module
    spec=importlib.util.spec_from_file_location('compact_startup','scripts/qualify_compact_px4_startup.py')
    startup=importlib.util.module_from_spec(spec);spec.loader.exec_module(startup)
    force,control=models()
    (tmp_path/'assumptions.json').write_text(json.dumps(dict(force_model=force.record(),control_model=control.record())))
    flag=tmp_path/'dockerenv';flag.touch()
    def own_path(value):
        return flag if value=='/.dockerenv' else tmp_path if value=='/runresults' else Path(value)
    monkeypatch.setattr(startup,'Path',own_path)
    monkeypatch.setenv('SWARM_COMPACT_PX4_STARTUP','1');monkeypatch.setenv('PX4_SIM_MODEL','gz_compact_5kg_prototype')
    destroyed=[]
    class SDK:
        def __init__(self,*a):pass
        def add_any_connection(self,url):assert url=='udpin://127.0.0.1:14540'
        def first_autopilot(self,*a):return object()
        def destroy(self):destroyed.append(True)
    class Telemetry:
        def __init__(self,*a):pass
        def subscribe_health(self,callback):callback(types.SimpleNamespace(is_armable=True,is_global_position_ok=True,is_home_position_ok=True),None)
        def subscribe_home(self,callback):callback(object(),None)
        def subscribe_armed(self,callback):callback(False,None)
    class Param:
        def __init__(self,*a):pass
        def get_param_int(self,key):return 4999
        def get_param_float(self,key):
            if key.startswith('SIM_'):raise RuntimeError('injected servo readback failure')
            return 25.
    monkeypatch.setattr(mavsdk,'Mavsdk',SDK);monkeypatch.setattr(telemetry_module,'Telemetry',Telemetry);monkeypatch.setattr(param_module,'Param',Param)
    monkeypatch.setattr(startup.subprocess,'run',lambda *a,**k:types.SimpleNamespace(stdout='PX4_SITL '+startup.PX4_HASH,stderr='',returncode=0))
    report=startup.worker(actuator_test=True)
    assert not report['passed'] and 'injected servo' in report['error']
    assert destroyed==[True]
    assert json.loads((tmp_path/'startup-report.json').read_text())['passed'] is False


def test_physics_observer_rejects_missing_partial_and_stale_data(tmp_path):
    # Preserve package context for relative imports without starting any runtime.
    from swarm_sim.px4_compact_trial import own_physics_velocity
    path=tmp_path/'states.csv';path.write_text('time,vx,vy,vz,fx,fy,fz\n1,3,4,0,0,0,0\n1.01,broken')
    assert own_physics_velocity(path,1.)==[3.,4.,0.]
    with pytest.raises(RuntimeError,match='stale'):own_physics_velocity(path,2.)
    path.write_text('time,vx,vy,vz,fx,fy,fz\n1,nan,4,0,0,0,0\n')
    with pytest.raises(RuntimeError,match='invalid'):own_physics_velocity(path,1.)
