"""Native proposal/catalog parity, including resumes and explicit mixed families."""
import importlib.util
import numpy as np
import pytest
from swarm_sim.trajectory2d.catalog import Catalog, extend_catalog
from swarm_sim.trajectory2d.planning import validate_individual
from test_native_backend import case

pytestmark=pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                            reason='optional C++ extension not built')


def compare(a,b):
    assert a.attempted==b.attempted and a.rng_state==b.rng_state
    assert a.rejections==b.rejections and a.rejection_details==b.rejection_details
    assert len(a.plans)==len(b.plans)
    for x,y in zip(a.plans,b.plans):
        assert (x.candidate_id,x.release_time,x.arrival_time,x.generation_phases,x.profile_family)==(
            y.candidate_id,y.release_time,y.arrival_time,y.generation_phases,y.profile_family)
        np.testing.assert_allclose(x.flight_states,y.flight_states,atol=1e-9,rtol=0)
        np.testing.assert_allclose(x.flight_controls,y.flight_controls,atol=1e-10,rtol=0)
        assert not y.flight_states.flags.writeable
    for name in ('guided_proposals','interval_contexts','interval_proposals','profile_events','profile_proposals','candidate_proposals'):
        assert getattr(a,name)==getattr(b,name), name


@pytest.mark.parametrize('kind',['baseline','rich','mixed','obstacle','feedback'])
def test_catalog_resume_keeps_random_stream_decisions_and_order(kind):
    changes=dict(numerical_backend='cpp',candidates=4,attempts=24)
    if kind in ('rich','mixed','feedback'):
        changes.update(cruise_levels=3,speed_profile='smooth',speed_profile_range=(10.,14.),
            horizon=210.,target=(500.,500.),cruise_altitude_range=(30.,120.),max_descent_rate=10.)
    if kind=='mixed':changes.update(profile_policy='adaptive_mixed',planning_strategy='adaptive')
    if kind in ('obstacle','feedback'):
        from swarm_sim.trajectory2d.obstacles import Obstacle
        changes.update(obstacles=(Obstacle.cylinder(700.,0.,30.,0.,180.),),obstacle_routing='spatial')
    if kind=='feedback':changes['candidate_guidance']='rejections'
    s,c=case(**changes);a,b=Catalog(),Catalog()
    if kind=='feedback':
        a.rejections=b.rejections={'validation_cruise_level_not_reached':2,'obstacle_clearance':2}
    for goal,reason in ((1,'initial'),(2,'conflict'),(4,'margin')):
        a=extend_catalog((s,c,None,a,goal,None,None,reason),backend='python')
        b=extend_catalog((s,c,None,b,goal,None,None,reason),backend='cpp')
        compare(a,b)
        for p in b.plans:
            assert validate_individual(p,s,c,backend='python')


def test_guided_departure_windows_and_overridden_draws_match():
    from swarm_sim.trajectory2d.conflict_guidance import ConflictWitness,RepairHint
    from swarm_sim.trajectory2d.interval_guidance import DepartureWindows
    s,c=case(numerical_backend='cpp',candidates=4,attempts=24,conflict_guidance='witness',
        reservation_policy='grid',interval_guidance='safe_windows',max_extra_loiter=6.)
    hint=RepairHint(ConflictWitness((0,1),(3,4),4.,((0.,0.,80.),(10.,5.,80.)),10.,50.),0)
    window=DepartureWindows(3,(1,),((c.activation_time+.2,c.activation_time+.9),),2)
    a,b=Catalog(),Catalog()
    for goal in (1,2,4):
        a=extend_catalog((s,c,None,a,goal,hint,window,'conflict'),backend='python')
        b=extend_catalog((s,c,None,b,goal,hint,window,'conflict'),backend='cpp')
        compare(a,b)


def test_native_catalog_does_not_call_python_proposal_or_validator(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    s,c=case(numerical_backend='cpp',attempts=3)
    monkeypatch.setattr(planning,'_candidate',lambda *a,**k:pytest.fail('native catalog called Python proposal'))
    monkeypatch.setattr(planning,'validate_individual',lambda *a,**k:pytest.fail('native catalog called Python validator'))
    catalog=extend_catalog((s,c,None,Catalog(),1))
    assert catalog.plans
