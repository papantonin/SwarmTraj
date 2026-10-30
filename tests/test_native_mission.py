"""Complete C++ mission API, native assignment and retained independent oracles."""
from dataclasses import replace
import importlib.util
import numpy as np
import pytest
from swarm_sim.trajectory2d import plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d import native_planning as native
from swarm_sim.trajectory2d.planning import approach_angles,validate_individual
from test_native_backend import case

pytestmark=pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                            reason='optional C++ extension not built')


@pytest.mark.parametrize('count',[1,2,3,4,5,8])
@pytest.mark.parametrize('spread',[0.,180.,360.])
@pytest.mark.parametrize('assignment',['id','geometric'])
def test_native_sector_assignment_matches_scipy_order_and_ties(count,spread,assignment):
    _,c=case(numerical_backend='cpp',approach_spread_deg=spread,approach_assignment=assignment)
    snapshots=tuple(replace(s,altitude=80.) for s in reversed(separate_loiter_snapshot(count)))
    a,b=approach_angles(snapshots,c),native.approach_angles(snapshots,c)
    assert a.keys()==b.keys()
    for key in a:
        if a[key] is None:assert b[key] is None
        else:assert b[key]==pytest.approx(a[key],abs=1e-14,rel=0)


def test_complete_native_mission_has_no_python_planning_callback(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.native_backend as kernels
    s,c=case(numerical_backend='cpp')
    monkeypatch.setattr(planning,'_plan_mission',lambda *a,**k:pytest.fail('native mission called Python planner'))
    monkeypatch.setattr(planning,'validate_individual',lambda *a,**k:pytest.fail('native mission called Python validator'))
    monkeypatch.setattr(kernels,'construct',lambda *a,**k:pytest.fail('native mission crossed construction binding per candidate'))
    result=plan_mission((s,),c)
    assert result.validated and not result.plans[0].flight_states.flags.writeable
    assert validate_individual(result.plans[0],s,c,backend='python')


@pytest.mark.parametrize('changes',[{}, {'horizon':45.},{'arrival_deadline':20.},
    {'post_arrival_time':110.},{'target':(1e5,0.)},{'flight_model':'point_mass','max_descent_rate':1.}])
def test_native_complete_status_counts_and_negative_requests(changes):
    s,c=case(**changes)
    a=plan_mission((s,),c);b=plan_mission((s,),replace(c,numerical_backend='cpp'))
    assert (a.status,a.reason,a.counts,a.search_nodes)==(b.status,b.reason,b.counts,b.search_nodes)
    for p,q in zip(a.plans,b.plans):
        assert (p.candidate_id,p.generation_phases)==(q.candidate_id,q.generation_phases)
        np.testing.assert_allclose(p.flight_states,q.flight_states,atol=1e-9,rtol=0)


def test_native_invalid_input_and_missing_capability_fail_before_computation(monkeypatch):
    s,c=case(numerical_backend='cpp')
    with pytest.raises(ValueError,match='at least one'):plan_mission((),c)
    with pytest.raises(ValueError,match='unique'):plan_mission((s,s),c)
    class Old:
        def build_info(self):return {'abi':1,'catalog_api':1}
    monkeypatch.setattr(native.native,'require_native',lambda:Old())
    with pytest.raises(RuntimeError,match='rebuild'):plan_mission((s,),c)


def test_native_mission_preserves_heterogeneous_physics_and_sphere_bindings():
    from swarm_sim.trajectory2d import Config
    from swarm_sim.trajectory2d.aircraft_profiles import reference_aircraft_profile
    from swarm_sim.trajectory2d.planning import _plan_mission
    c=Config(dimensions=3,generator_mode='physical_mission',numerical_backend='cpp',target_altitude=30.,
        max_descent_rate=10.,descent_angle_deg=30.,cruise_altitude_range=(30.,120.),planning_strategy='adaptive',
        initial_candidates=2,candidates=8,attempts=30)
    snapshots=tuple(replace(s,altitude=30+20*k) for k,s in enumerate(separate_loiter_snapshot(2)))
    a=replace(reference_aircraft_profile(),bounding_radius_m=1.2)
    b=replace(a,name='second own native airframe',bounding_radius_m=1.8,limits=replace(a.limits,max_speed=14.))
    c=replace(c,aircraft_profiles=((0,a),(1,b)))
    full=plan_mission(snapshots,c);hybrid=_plan_mission(snapshots,c)
    assert full.validated and hybrid.validated
    assert (full.counts,full.search_nodes)==(hybrid.counts,hybrid.search_nodes)
    for p,q,s in zip(full.plans,hybrid.plans,snapshots):
        assert p.aircraft_profile==c.profile_for(p.drone_id)
        assert p.airspeed_limit==c.for_drone(p.drone_id).limits.max_speed
        np.testing.assert_array_equal(p.flight_states,q.flight_states)
        assert validate_individual(p,s,c,backend='python')


def test_native_invalid_speed_altitude_and_grid_tolerance_preserve_reference_behavior():
    s,c=case()
    for snapshot in (replace(s,speed=np.nan),replace(s,altitude=np.nan),replace(s,speed=100.)):
        a=plan_mission((snapshot,),c);b=plan_mission((snapshot,),replace(c,numerical_backend='cpp'))
        assert (a.status,a.reason)==(b.status,b.reason)
    c=replace(c,horizon=120.+1e-6,post_arrival_time=0.,clock_error=0.)
    a=plan_mission((s,),c);b=plan_mission((s,),replace(c,numerical_backend='cpp'))
    assert a.validated and b.validated
    assert a.counts==b.counts and a.search_nodes==b.search_nodes
