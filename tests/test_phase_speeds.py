"""Declared own operating speeds must be physically attained and audited."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from swarm_sim.trajectory2d import Config, load_aircraft_profile, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import validate_individual


def scene(count=1):
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    config=profile.apply(Config(target=(3000.,0.),target_altitude=200.,descent_angle_deg=30.,
        max_altitude=700.,cruise_altitude_range=(500.,550.),horizon=420.,candidates=4,attempts=24,
        approach_spread_deg=180. if count>1 else 0.,candidate_guidance='rejections',
        planning_strategy='adaptive',search_order='mrv',selection_policy='efficient',
        conflict_guidance='witness',obstacle_routing='spatial',approach_routing='tangent',
        speed_schedule='operating',speed_profile_acceleration=1.5))
    snapshots=tuple(replace(s,altitude=500.+25*k).with_speed(profile.airspeed_for('loiter'),profile.limits)
                    for k,s in enumerate(separate_loiter_snapshot(count,radius=400.,spacing=1000.)))
    return snapshots,config,profile


@pytest.fixture(scope='module')
def fleet():
    snapshots,config,profile=scene(4)
    result=plan_mission(snapshots,config)
    assert result.validated,result
    return snapshots,config,profile,result


def test_operating_fleet_integrates_attains_executes_and_is_spawn_deterministic(fleet):
    snapshots,config,profile,result=fleet
    for snapshot,plan in zip(snapshots,result.plans):
        assert validate_individual(plan,snapshot,config,backend='python')
        assert validate_individual(plan,snapshot,config,backend='cpp')
        assert plan.flight_states[0,3]==profile.airspeed_for('loiter')
        assert plan.flight_states[-1,3]==pytest.approx(profile.airspeed_for('cruise'),abs=.2)
        assert np.max(np.abs(np.diff(plan.speed_reference)))<=1.5*config.dt+1e-8
        names=[p[0] for p in plan.generation_phases]
        assert all(name in names for name in ('cruise_preparation','approach_acceleration','deceleration','cruise_recovery'))
        cruise=next((a,b) for name,a,b in plan.generation_phases if name=='transit')
        assert np.median(plan.flight_states[cruise[0]:cruise[1],3])==pytest.approx(100/3.6,abs=.2)
        assert plan.sample_flight(plan.arrival_time)[3]==pytest.approx(175/3.6,abs=.2)
    parallel=plan_mission(snapshots,replace(config,planning_workers=2))
    assert parallel.validated
    assert [p.to_dict() for p in result.plans]==[p.to_dict() for p in parallel.plans]
    report=simulate(result.plans,config).report
    assert report['operating_speeds_verified'],report['operating_speed_checks']
    assert report['execution_validated'],report['execution_failures']


@pytest.mark.parametrize('backend',['python','cpp'])
@pytest.mark.parametrize('corruption',['missing','jump','unattained','wrong_terminal'])
def test_independent_validators_reject_corrupt_speed_contract(fleet,backend,corruption):
    snapshots,config,_,result=fleet
    plan=result.plans[0];reference=plan.speed_reference.copy()
    if corruption=='missing':reference=None
    elif corruption=='jump':reference[20]=config.limits.max_speed
    elif corruption=='unattained':
        # Physically unchanged history: invent a different declared cruise speed.
        speeds=list(config.operating_airspeeds);speeds[2]+=1
        config=replace(config,operating_airspeeds=speeds)
        reference[-1]=speeds[2]
    else:
        config=replace(config,terminal_regime='descent')
    diagnostics={}
    assert not validate_individual(replace(plan,speed_reference=reference),snapshots[0],config,diagnostics,backend=backend)
    assert diagnostics['rejection'].startswith('validation_operating_'),diagnostics


def test_operating_configuration_bindings_and_initial_speed():
    snapshots,config,profile=scene()
    assert config.operating_airspeeds==profile.operating_speed_values()
    assert replace(config,operating_airspeeds=list(config.operating_airspeeds))==config
    with pytest.raises(ValueError,match='strict single-level'):
        replace(config,speed_profile='smooth',speed_profile_range=(25.5,49.))
    with pytest.raises(ValueError,match='strict single-level'):
        replace(config,cruise_levels=2)
    with pytest.raises(ValueError,match='operating_airspeeds'):
        replace(config,operating_airspeeds=(True,27.,28.,33.,48.))
    with pytest.raises(ValueError,match='loaded aircraft'):
        plan_mission(snapshots,Config(speed_schedule='operating'))
    wrong=tuple(s.with_speed(profile.airspeed_for('cruise'),profile.limits) for s in snapshots)
    assert plan_mission(wrong,config).status=='invalid_initial_state'
    own=replace(config,operating_airspeeds=None,aircraft_profiles=((0,profile),)).for_drone(0)
    assert not own.aircraft_profiles and own.operating_airspeeds==profile.operating_speed_values()
    profile.require_matches(own)
    with pytest.raises(ValueError,match='operating speeds differ'):
        profile.require_matches(replace(own,terminal_regime='descent',operating_airspeeds=(25.5,27.,28.,33.,48.)))


def test_terminal_speed_not_loiter_speed_drives_feasibility():
    snapshots,config,_=scene()
    assert plan_mission(snapshots,replace(config,max_descent_rate=20.)).status=='descent_not_feasible'
    assert plan_mission(snapshots,replace(config,target_altitude=2.)).status=='descent_not_feasible'
    assert plan_mission(snapshots,replace(config,flight=replace(config.flight,max_brake_acceleration=0.))).status=='descent_not_feasible'


def test_observer_rejects_wrong_actual_speed_even_with_nominal_position(fleet):
    from swarm_sim.trajectory2d.execution_safety import operating_speed_diagnostics,execution_failures
    _,config,_,result=fleet
    states=np.stack([p.flight_states for p in result.plans],axis=1).copy()
    times=np.arange(len(states))*config.dt
    contacts=[SimpleNamespace(time=p.arrival_time,state=p.sample_flight(p.arrival_time)) for p in result.plans]
    configs=[config.for_drone(p.drone_id) for p in result.plans]
    assert operating_speed_diagnostics(states,times,result.plans,configs,contacts)['operating_speeds_verified']
    states[...,3]=config.operating_airspeeds[4]
    report=operating_speed_diagnostics(states,times,result.plans,configs,contacts)
    assert not report['operating_speeds_verified']
    base=dict.fromkeys(('all_arrived','tube_conditions_verified','separation_verified_in_execution_model',
        'arrival_deadline_verified_in_execution','post_arrival_time_verified_in_execution',
        'approach_sectors_verified_in_execution','vertical_limits_verified','descent_angle_verified_in_execution',
        'obstacle_clearance_verified_in_execution_model','controller_protection_verified','physical_limits_verified'),True)
    assert execution_failures(dict(base,flight_model='point_mass',**report))==['operating_speeds_verified']
