"""Warm initialization remains a finite impulse, separate from flight acceptance."""
from datetime import timedelta
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import pytest

from swarm_sim.px4_compact_trial import initial_state_check, warm_initialize,cruise_turns_passed


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()


def state():
    config=dict(state=[0.,0.,80.,100/3.6,0.,0.,0.],body_pitch_rad=-.12)
    ack=dict(config_sha256=digest(config),mission_sha256='native',impulse_steps=1,persistent_velocity_control=False,
        snapshot=dict(sim_time_s=30.,position_world_m=[0.,0.,80.],velocity_world_m_s=[100/3.6,0.,0.],
                      rpy_rad=[0.,-.12,0.],angular_velocity_rad_s=[0.,0.,0.]))
    return config,ack


@pytest.mark.parametrize('field,value',[
    ('position_world_m',[0.,0.,81.]),('velocity_world_m_s',[26.,0.,0.]),
    ('rpy_rad',[0.,0.,0.]),('angular_velocity_rad_s',[0.,0.,.11])])
def test_physical_gate_rejects_each_independent_initial_error(field,value):
    config,ack=state()
    assert initial_state_check(config,ack,'native')['passed']
    ack['snapshot'][field]=value
    assert not initial_state_check(config,ack,'native')['passed']


@pytest.mark.parametrize('field,value',[
    ('config_sha256','other'),('mission_sha256','other'),('impulse_steps',True),
    ('impulse_steps',2),('persistent_velocity_control',True)])
def test_ack_identity_and_finite_initialization_are_required(field,value):
    config,ack=state();ack[field]=value
    with pytest.raises(ValueError,match='acknowledgement'):initial_state_check(config,ack,'native')


def test_initial_state_nan_is_never_success():
    config,ack=state();ack['snapshot']['velocity_world_m_s'][0]=float('nan')
    with pytest.raises(ValueError,match='physical initial state'):initial_state_check(config,ack,'native')


def test_cruise_turn_phase_needs_native_progress_speed_hold_and_actual_turn():
    progress=types.SimpleNamespace(current=6,total=9)
    assert cruise_turns_passed(progress,5.,45.,5.)
    assert not cruise_turns_passed(None,100.,180.,15.)
    assert not cruise_turns_passed(types.SimpleNamespace(current=5,total=9),100.,180.,15.)
    assert not cruise_turns_passed(progress,4.9,180.,15.)
    assert not cruise_turns_passed(progress,100.,44.9,15.)
    assert not cruise_turns_passed(progress,100.,180.,4.9)


def test_compact_camera_requires_its_observed_target_not_cessna(monkeypatch):
    from swarm_sim import px4_gazebo as adapter
    status=json.dumps(dict(trackMode='FOLLOW',followTarget=dict(name='compact_5kg_prototype_0'),
        followPgain=1.,followOffset=dict(x=-6.,y=-3.,z=3.)))
    assert not adapter.camera_status_matches(status)
    assert adapter.camera_status_matches(status,'compact_5kg_prototype_0')
    calls=[]
    def transport(args):
        calls.append(args)
        return status if '-e' in args else 'data: true'
    monkeypatch.setattr(adapter,'camera_gz',transport)
    result=adapter.initialize_camera_follow(target='compact_5kg_prototype_0')
    assert result['target']=='compact_5kg_prototype_0'
    assert any('data: "compact_5kg_prototype_0"' in args for args in calls)
    with pytest.raises(ValueError,match='model name'):adapter.initialize_camera_follow(target='unsafe"name')


@pytest.mark.parametrize('options',[
    dict(airborne_start=True),dict(flight_phase='cruise-turns'),dict(gui=True)])
def test_invalid_phase_options_fail_before_creating_runtime(tmp_path,options):
    spec=importlib.util.spec_from_file_location('compact_startup','scripts/qualify_compact_px4_startup.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    output=tmp_path/'run'
    with pytest.raises(ValueError):module.qualify(output,tmp_path/'absent.so',**options)
    assert not output.exists()


def test_failed_physical_gate_prevents_controller_reset_and_mission_start(tmp_path,monkeypatch):
    config,ack=state();ack['snapshot']['position_world_m'][2]=81.
    (tmp_path/'airborne-initial-state.json').write_text(json.dumps(ack))
    calls=[]
    mission=types.SimpleNamespace(set_current_mission_item=lambda index:calls.append(('entry',index)))
    import swarm_sim.px4_compact_trial as module
    monkeypatch.setattr(module.subprocess,'run',lambda *a,**k:pytest.fail('reset must not precede valid physical state'))
    with pytest.raises(RuntimeError,match='physical warm-start'):
        warm_initialize(mission,None,tmp_path,config,'native',{})
    assert calls==[('entry',1)]
    assert not json.loads((tmp_path/'airborne-check.json').read_text())['passed']


def test_failed_estimator_gate_prevents_native_controller_reset(tmp_path,monkeypatch):
    import swarm_sim.px4_compact_trial as module
    config,ack=state();(tmp_path/'airborne-initial-state.json').write_text(json.dumps(ack))
    mission=types.SimpleNamespace(set_current_mission_item=lambda index:None)
    observations=types.SimpleNamespace(latest=dict(in_air=True))
    def bounded_wait(predicate,timeout,message,interval):
        if not predicate():raise TimeoutError(message)
    monkeypatch.setattr(module,'wait_for',bounded_wait)
    monkeypatch.setattr(module,'airborne_estimate_check',lambda *args:dict(passed=False,reason='nominal_state_mismatch'))
    monkeypatch.setattr(module.subprocess,'run',lambda *a,**k:pytest.fail('reset must not precede estimator acquisition'))
    report={}
    with pytest.raises(TimeoutError,match='estimated airborne'):
        warm_initialize(mission,observations,tmp_path,config,'native',report)
    assert not report['airborne_estimator_acquisition']['passed']
    assert not json.loads((tmp_path/'airborne-estimator-check.json').read_text())['passed']


def test_passing_gates_reset_only_native_longitudinal_controller_once(tmp_path,monkeypatch):
    import swarm_sim.px4_compact_trial as module
    config,ack=state();(tmp_path/'airborne-initial-state.json').write_text(json.dumps(ack))
    calls=[];mission=types.SimpleNamespace(set_current_mission_item=lambda index:calls.append(index))
    observations=types.SimpleNamespace(latest=dict(in_air=True))
    monkeypatch.setattr(module,'airborne_estimate_check',lambda *args:dict(passed=True,reason='ready'))
    monkeypatch.setattr(module.subprocess,'run',lambda args,**kwargs:calls.append(args))
    report={};warm_initialize(mission,observations,tmp_path,config,'native',report)
    assert calls==[1,['/opt/px4-gazebo/bin/px4-fw_lat_lon_control','stop'],['/opt/px4-gazebo/bin/px4-fw_lat_lon_control','start']]
    assert report['airborne_controller_reset_count']==1
    assert report['airborne_estimator_acquisition']['passed']


@pytest.fixture
def bootstrap(tmp_path,monkeypatch):
    """Fake transport only: inspect the real plugin's finite command lifecycle."""
    class Vector:
        def __init__(self,*values):self.values=values
        def __sub__(self,other):return Vector(*(a-b for a,b in zip(self.values,other.values)))
        def __mul__(self,value):return Vector(*(a*value for a in self.values))
    class Model:
        def __init__(self,entity):self.poses=[]
        def name(self,ecm):return 'compact_5kg_prototype_0'
        def canonical_link(self,ecm):return 'base'
        def link_by_name(self,ecm,name):return name
        def set_world_pose_cmd(self,ecm,pose):self.poses.append(pose)
    class Link:
        def __init__(self,entity):self.forces=[]
        def enable_velocity_checks(self,ecm):pass
        def valid(self,ecm):return True
        def world_linear_velocity(self,ecm):return Vector(0.,0.,0.)
        def add_world_force(self,ecm,force):self.forces.append(force.values)
    monkeypatch.setitem(sys.modules,'gz',types.ModuleType('gz'))
    monkeypatch.setitem(sys.modules,'gz.math7',types.SimpleNamespace(Pose3d=lambda *args:args,Vector3d=Vector))
    monkeypatch.setitem(sys.modules,'gz.sim8',types.SimpleNamespace(Model=Model,Link=Link))
    spec=importlib.util.spec_from_file_location('compact_bootstrap','examples/gazebo/compact_physics/compact_airborne_bootstrap.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    dockerenv=tmp_path/'dockerenv';dockerenv.touch()
    monkeypatch.setattr(module,'Path',lambda path:dockerenv if path=='/.dockerenv' else tmp_path if path=='/runresults' else Path(path))
    monkeypatch.setenv('SWARM_COMPACT_PX4_STARTUP','1');monkeypatch.setenv('SWARM_COMPACT_AIRBORNE','1')
    model=tmp_path/'models/compact_5kg_prototype';model.mkdir(parents=True)
    root=ET.Element('sdf');entity=ET.SubElement(root,'model')
    for name,mass in [('body',4.8),('surface',.2)]:
        node=ET.SubElement(entity,'link',name=name);ET.SubElement(ET.SubElement(node,'inertial'),'mass').text=str(mass)
    ET.ElementTree(root).write(model/'model.sdf')
    config=dict(schema='swarm-compact-warm-start-v1',model_sdf_sha256=hashlib.sha256((model/'model.sdf').read_bytes()).hexdigest(),
        force_binding_sha256='force',control_sha256='control',state=[0.,0.,80.,100/3.6,0.,0.,0.],body_pitch_rad=-.12)
    (tmp_path/'airborne-config.json').write_text(json.dumps(config))
    (tmp_path/'assumptions.json').write_text(json.dumps(dict(force_model=dict(binding_sha256='force'),control_model=dict(sha256='control'),
        airborne_config_sha256=digest(config),airframe_defaults=dict(FW_AIRSPD_TRIM=100/3.6))))
    mission=[dict(native='own')];(tmp_path/'native-flight-mission.json').write_text(json.dumps(mission))
    release=dict(config_sha256=digest(config),mission_sha256=digest(mission),release=True)
    info=types.SimpleNamespace(paused=False,dt=timedelta(seconds=.001),sim_time=timedelta(seconds=30))
    return module,tmp_path,config,release,info


def test_no_release_no_force_then_exactly_one_mass_proportional_impulse(bootstrap):
    module,output,config,release,info=bootstrap
    plugin=module.get_system();plugin.configure(1,None,None,None)
    plugin.pre_update(info,None)
    assert plugin.model.poses==[] and all(not link.forces for link,_ in plugin.links)
    (output/'airborne-release.json').write_text(json.dumps(release))
    plugin.pre_update(info,None)  # pose clears ground contact first
    assert len(plugin.model.poses)==1 and all(not link.forces for link,_ in plugin.links)
    plugin.pre_update(info,None)
    for link,mass in plugin.links:
        assert link.forces==[pytest.approx((config['state'][3]*mass/.001,0.,0.))]
    # Repeated execution cannot become a velocity replay or second release.
    for _ in range(100):plugin.pre_update(info,None)
    assert len(plugin.model.poses)==1 and all(len(link.forces)==1 for link,_ in plugin.links)


def test_wrong_native_mission_cannot_release_own_model(bootstrap):
    module,output,_,release,info=bootstrap
    plugin=module.get_system();plugin.configure(1,None,None,None)
    release['mission_sha256']='other';(output/'airborne-release.json').write_text(json.dumps(release))
    with pytest.raises(ValueError,match='release'):plugin.pre_update(info,None)
    assert not plugin.model.poses and all(not link.forces for link,_ in plugin.links)


def test_changed_model_cannot_enable_bootstrap(bootstrap):
    module,output,_,_,_=bootstrap
    model=output/'models/compact_5kg_prototype/model.sdf';model.write_text(model.read_text()+'\n')
    with pytest.raises(ValueError,match='identity'):module.get_system().configure(1,None,None,None)
