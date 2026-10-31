"""User-imposed compact profile: native flight, own physics and unchanged rejects."""
from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, load_aircraft_profile, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.aircraft_profiles import aircraft_profile_from_dict
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.flight3d import envelope_ok, trim_control
from swarm_sim.trajectory2d.native_backend import evaluate
from swarm_sim.trajectory2d.planning import validate_individual

PROFILE=Path(__file__).resolve().parents[1]/'examples/aircraft/compact_5kg.json'


def mission(profile, target_altitude=200.):
    config=profile.apply(Config(target=(3000.,0.),target_altitude=target_altitude,
        descent_angle_deg=30.,max_altitude=700.,cruise_altitude_range=(500.,550.),
        horizon=240.,candidates=4,attempts=24))
    snapshot=replace(separate_loiter_snapshot(1,radius=400.,spacing=1000.)[0],altitude=500.)
    return snapshot.with_speed(profile.airspeed_for('dive'),profile.limits),config


def test_user_imposed_parameters_and_profile_identity():
    profile=load_aircraft_profile(PROFILE)
    assert profile.airframe.mass_kg==5.
    assert profile.bounding_radius_m==.3
    assert profile.airspeed_for('cruise')*3.6==pytest.approx(100.,abs=1e-12)
    assert profile.airspeed_for('dive')*3.6==pytest.approx(175.,abs=1e-12)
    assert aircraft_profile_from_dict(profile.to_dict())==profile
    assert profile.calibration_status=='illustrative'
    _,config=mission(profile)
    assert config.descent_angle_deg==30 and config.aircraft_radius==.3


@pytest.mark.parametrize('regime,angle',[('cruise',0.),('dive',-30.)])
def test_steady_requested_flight_has_physical_and_native_control_authority(regime,angle):
    profile=load_aircraft_profile(PROFILE)
    _,config=mission(profile)
    state=np.array([0.,0.,500.,profile.airspeed_for(regime),0.,math.radians(angle),0.])
    control=trim_control(state,profile.flight)
    assert envelope_ok(state,control,config)
    assert evaluate('held_control',state,control,config,hold_dt=.2)
    derivative=np.frombuffer(evaluate('rhs',state,control,config),dtype=float)
    np.testing.assert_allclose(derivative[3:],0.,atol=1e-12,rtol=0)
    if regime=='dive':
        assert derivative[2]==pytest.approx(-175/3.6/2)
        assert 0<control[1]<profile.flight.max_brake_acceleration


def test_complete_native_dive_is_independently_validated_and_executed():
    profile=load_aircraft_profile(PROFILE)
    snapshot,config=mission(profile)
    result=plan_mission((snapshot,),config)
    assert result.validated,(result.status,result.reason)
    assert validate_individual(result.plans[0],snapshot,config,backend='python')
    report=simulate(result.plans,config).report
    assert report['execution_validated'],report['execution_failures']


def test_four_aircraft_spread180_alignment_feedback_is_valid_and_spawn_deterministic():
    profile=load_aircraft_profile(PROFILE)
    _,config=mission(profile)
    config=replace(config,approach_spread_deg=180.,candidate_guidance='rejections',
        planning_strategy='adaptive',search_order='mrv',selection_policy='max_margin',
        conflict_guidance='witness',obstacle_routing='spatial')
    snapshots=tuple(replace(s,altitude=500.+25*k).with_speed(profile.airspeed_for('dive'),profile.limits)
                    for k,s in enumerate(separate_loiter_snapshot(4,radius=400.,spacing=1000.)))
    result=plan_mission(snapshots,config)
    assert result.validated,(result.status,result.reason)
    angles=sorted(math.degrees(p.approach_angle) for p in result.plans)
    # Measure circular angular gaps, including the uncovered half-circle.
    gaps=np.diff([*angles,angles[0]+360.])
    np.testing.assert_allclose(sorted(gaps),[60.,60.,60.,180.],atol=1e-10,rtol=0)
    hints=[h for cat in result.diagnostics['catalogs'] for h in cat['candidate_proposals']
           if h['mode']=='longer_approach']
    assert hints and any(h['accepted'] for h in hints)
    assert all(5<=h['approach_lead_extra_s']<=30 for h in hints)
    assert all(cat['attempted']<=config.attempts for cat in result.diagnostics['catalogs'])
    for plan,snapshot in zip(result.plans,snapshots):
        assert validate_individual(plan,snapshot,config,backend='python')
    parallel=plan_mission(snapshots,replace(config,planning_workers=2))
    assert parallel.validated and parallel.counts==result.counts
    assert [p.to_dict() for p in parallel.plans]==[p.to_dict() for p in result.plans]
    report=simulate(result.plans,config).report
    assert report['execution_validated'],report['execution_failures']


@pytest.mark.parametrize('failure',['no_airbrake','low_target'])
def test_incompatible_dive_requests_remain_rejected(failure):
    profile=load_aircraft_profile(PROFILE)
    if failure=='no_airbrake':
        airframe=replace(profile.airframe,max_airbrake_drag_n=0.)
        profile=replace(profile,airframe=airframe,flight=replace(profile.flight,**airframe.derived()))
    snapshot,config=mission(profile,target_altitude=2. if failure=='low_target' else 200.)
    result=plan_mission((snapshot,),config)
    assert result.status=='descent_not_feasible' and not result.plans
    assert ('airbrake' if failure=='no_airbrake' else 'post-passage') in result.reason
