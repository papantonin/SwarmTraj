"""Curvature-bounded references remain physically integrated and independently checked."""
from dataclasses import replace
import numpy as np
import pytest
from swarm_sim.trajectory2d import Config,load_aircraft_profile,plan_mission,separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import validate_individual


def scene():
    profile=load_aircraft_profile('examples/aircraft/compact_5kg.json')
    config=profile.apply(Config(target=(3000.,0.),target_altitude=200.,descent_angle_deg=30.,
        max_altitude=700.,cruise_altitude_range=(500.,550.),horizon=240.,candidates=4,attempts=24,
        approach_spread_deg=180.,candidate_guidance='rejections',planning_strategy='adaptive',
        search_order='mrv',selection_policy='max_margin',conflict_guidance='witness',obstacle_routing='spatial'))
    snapshots=tuple(replace(s,altitude=500.+25*k).with_speed(profile.airspeed_for('dive'),profile.limits)
                    for k,s in enumerate(separate_loiter_snapshot(4,radius=400.,spacing=1000.)))
    return snapshots,config


def transit_metrics(plan):
    _,a,b=next(row for row in plan.generation_phases if row[0]=='transit')
    turns=np.abs(np.diff(np.unwrap(plan.flight_states[a:b+1,4]))).sum()
    return (b-a)*plan.dt,turns


def test_tangent_reduces_outer_turns_without_changing_constraints_and_executes():
    snapshots,config=scene()
    before=plan_mission(snapshots,config)
    config=replace(config,approach_routing='tangent')
    after=plan_mission(snapshots,config)
    assert before.validated and after.validated
    for k in (0,2):
        t0,a0=transit_metrics(before.plans[k]);t1,a1=transit_metrics(after.plans[k])
        assert t1<t0 and a1<a0
    for snapshot,plan in zip(snapshots,after.plans):
        assert validate_individual(plan,snapshot,config,backend='python')
        assert plan.arrival_time<=config.latest_nominal_arrival
    report=simulate(after.plans,config).report
    assert report['execution_validated'],report['execution_failures']
    parallel=plan_mission(snapshots,replace(config,planning_workers=2))
    assert parallel.validated
    assert [p.to_dict() for p in parallel.plans]==[p.to_dict() for p in after.plans]


def test_bad_mode_and_lossy_shared_px4_export_reject():
    with pytest.raises(ValueError,match='approach_routing'):Config(approach_routing='bad')
    from swarm_sim.px4_scenario import default_scenario,scenario_record
    snapshots,config=default_scenario()
    with pytest.raises(ValueError,match='do not yet persist'):
        scenario_record(snapshots,replace(config,approach_routing='tangent'))
