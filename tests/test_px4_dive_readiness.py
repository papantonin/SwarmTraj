from dataclasses import replace
from pathlib import Path
import math
import pytest
from swarm_sim.gazebo_force_model import ForceModel
from swarm_sim.gazebo_control_model import ControlModel
from swarm_sim.px4_dive_readiness import BRAKE_DEFAULTS,dive_readiness
from swarm_sim.trajectory2d import load_aircraft_profile


def fixture():
    force=ForceModel(load_aircraft_profile(Path('examples/aircraft/compact_5kg.json')))
    control=ControlModel.load(Path('examples/gazebo/compact_physics/control_model.json'),force)
    return force,control,dict(BRAKE_DEFAULTS,FW_AIRSPD_MAX=55.55556,FW_T_SINK_MAX=3.5,FW_P_LIM_MIN=-15.)


def test_cruise_settings_cannot_be_reported_as_dive_ready():
    force,control,params=fixture();r=dive_readiness(force,control,params)
    assert set(r['blockers'])=={'native_sink_setting','native_pitch_limit'}
    assert r['requested_sink_m_s']==pytest.approx(175/3.6*.5)
    assert r['requested_pitch_deg']==pytest.approx(-28.847664,abs=.001)
    assert r['trim']['brake_n']==pytest.approx(14.839194,abs=1e-5)
    assert not r['settings_compatible'] and not r['closed_loop_qualified']


def test_compatible_settings_never_prove_closed_loop_qualification():
    force,control,params=fixture();params.update(FW_T_SINK_MAX=25.,FW_P_LIM_MIN=-30.)
    r=dive_readiness(force,control,params)
    assert r['settings_compatible'] and not r['closed_loop_qualified']
    assert not r['documented_range_compatible'] and not r['autonomous_waypoint_braking_supported']
    assert not r['flight_trial_ready']
    params['CA_SV_CS4_TYPE']=9
    assert 'native_airbrake_allocation' in dive_readiness(force,control,params)['blockers']
    params['FW_AIRSPD_MAX']=40.
    assert 'native_airspeed_limit' in dive_readiness(force,control,params)['blockers']


@pytest.mark.parametrize('value',[math.nan,math.inf,True])
def test_invalid_native_readbacks_fail(value):
    force,control,params=fixture();params['FW_T_SINK_MAX']=value
    with pytest.raises(ValueError,match='readbacks'):dive_readiness(force,control,params)


def test_no_missing_readbacks_or_physical_binding_substitution():
    force,control,params=fixture();del params['FW_P_LIM_MIN']
    with pytest.raises(ValueError,match='readbacks'):dive_readiness(force,control,params)
    with pytest.raises(ValueError,match='binding'):
        dive_readiness(force,replace(control,force_binding_sha256='other'),params)


def test_native_brake_observer_and_hold_reject_stale_or_interrupted_response(tmp_path):
    from swarm_sim.px4_compact_trial import own_physics_brake,brake_hold
    path=tmp_path/'physics.csv'
    values=[1.,27.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,.4,0.]
    path.write_text(','.join(map(str,values))+'\n')
    assert own_physics_brake(path,1.)==.4
    with pytest.raises(RuntimeError,match='stale'):own_physics_brake(path,2.)
    assert brake_hold(None,None,1.,.4)==1.
    assert brake_hold(1.,1.1,1.2,.4)==1.
    assert brake_hold(1.,1.1,2.,.4)==2.
    assert brake_hold(1.,1.1,1.2,.3) is None
    with pytest.raises(ValueError):brake_hold(1.,1.1,1.2,math.nan)
