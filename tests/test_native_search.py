"""Native bounded selection preserves ordering, budgets, margins and hard checks."""
from dataclasses import replace
import importlib.util
import itertools
import numpy as np
import pytest
from swarm_sim.trajectory2d.catalog import Catalog
from swarm_sim.trajectory2d.native_planning import NativeJointSearch
from swarm_sim.trajectory2d.search import JointSearch
from test_execution_boundaries import straight_flight

pytestmark=pytest.mark.skipif(importlib.util.find_spec('swarm_sim._trajectory_native') is None,
                            reason='optional C++ extension not built')


def shifted(p,drone,candidate,y):
    flight=p.flight_states.copy();flight[:,1]+=y
    projected=p.states.copy();projected[:,1]+=y
    return replace(p,drone_id=drone,candidate_id=candidate,states=projected,flight_states=flight)


def ids(plans):return None if plans is None else [(p.drone_id,p.candidate_id) for p in plans]


@pytest.mark.parametrize('order',['id','mrv'])
@pytest.mark.parametrize('budget',[1,2,100])
def test_first_feasible_and_margin_refinement_match_exact_search_trace(order,budget):
    c,p=straight_flight();c=replace(c,generator_mode='physical_mission',numerical_backend='cpp',
        search_order=order,max_search_nodes=budget,margin_search_nodes=4)
    domains=[[shifted(p,0,0,0.),shifted(p,0,1,30.)],
             [shifted(p,1,0,5.),shifted(p,1,1,60.)],[shifted(p,2,0,10.),shifted(p,2,1,90.)]]
    a,b=JointSearch(c),NativeJointSearch(c)
    pa,pb=a.solve(domains),b.solve(domains)
    assert ids(pa)==ids(pb) and a.nodes==b.nodes and a.exhausted==b.exhausted
    assert a.diagnostics()==b.diagnostics()
    if pa is not None:
        sa,qa=a.refine(domains,pa);sb,qb=b.refine(domains,pb)
        assert ids(sa)==ids(sb) and qa==qb and a.nodes==b.nodes
        assert a.diagnostics()==b.diagnostics()
        feasible=[plans for plans in itertools.product(*domains) if all(
            JointSearch(c).compatible(x,y) for x,y in itertools.combinations(plans,2))]
        assert feasible
        assert all(b.compatible(x,y) for x,y in itertools.combinations(sb,2))


@pytest.mark.parametrize('grid',[False,True])
def test_pair_boundaries_reservations_witnesses_and_independent_recheck(grid):
    c,p=straight_flight();c=replace(c,generator_mode='physical_mission',numerical_backend='cpp',
        conflict_guidance='witness',reservation_policy='grid' if grid else 'none')
    a,b=JointSearch(c),NativeJointSearch(c)
    first=shifted(p,0,0,0.)
    for k,y in enumerate((5.,20.,20.+1e-7,20.+2e-7,25.,35.,60.)):
        second=shifted(p,1,k,y)
        assert a.compatible(first,second)==b.compatible(first,second)
        assert a.margin(first,second)==b.margin(first,second)
    assert a.diagnostics()==b.diagnostics()
    assert a.repair_hints([0,1])==b.repair_hints([0,1])
    assert b.verify_joint((first,shifted(p,1,99,60.)))['checks']
    with pytest.raises((RuntimeError,ValueError),match='joint verification failed'):
        b.verify_joint((first,shifted(p,1,100,5.)))
    if grid:
        with pytest.raises((RuntimeError,ValueError),match='reservation verification failed'):
            b.verify_joint((first,shifted(p,1,101,25.)))


def test_expansion_and_safe_departure_windows_match():
    from swarm_sim.trajectory2d import Snapshot
    c,p=straight_flight();c=replace(c,generator_mode='physical_mission',numerical_backend='cpp',
        reservation_policy='grid',interval_guidance='safe_windows',min_extra_loiter=0.,max_extra_loiter=2.)
    domains=[[shifted(p,0,0,0.)],[shifted(p,1,0,25.)]]
    catalogs=[Catalog(plans=d,attempted=2) for d in domains]
    snapshots=[Snapshot(i,(0.,0.,0.,0.),12.,altitude=80.) for i in range(2)]
    a,b=JointSearch(c),NativeJointSearch(c)
    assert ids(a.solve(domains))==ids(b.solve(domains))
    assert a.expansion_indices(catalogs,snapshots)==b.expansion_indices(catalogs,snapshots)
    assert a.interval_hints(catalogs,[0,1])==b.interval_hints(catalogs,[0,1])


def test_reservation_budget_failure_propagates_and_changed_cached_geometry_rejects():
    from swarm_sim.trajectory2d.reservations import ReservationBudgetError
    c,p=straight_flight();c=replace(c,generator_mode='physical_mission',numerical_backend='cpp',
        reservation_policy='grid',reservation_max_entries=1)
    first,second=shifted(p,0,0,0.),shifted(p,1,0,60.)
    for search in (JointSearch(c),NativeJointSearch(c)):
        with pytest.raises(ReservationBudgetError):search.compatible(first,second)
    search=NativeJointSearch(replace(c,reservation_policy='none'))
    search.compatible(first,second)
    with pytest.raises(ValueError,match='candidate changed'):
        search.compatible(first,shifted(p,1,0,5.))
    with pytest.raises(ValueError,match='candidate changed'):
        search.compatible(first,replace(second,airspeed_limit=15.))
    with pytest.raises(ValueError,match='candidate changed'):
        search.compatible(first,replace(second,profile_family='unrequested'))


def test_complete_margin_search_matches_exhaustive_catalog_oracle():
    c,p=straight_flight();c=replace(c,generator_mode='physical_mission',numerical_backend='cpp',
        max_search_nodes=1000,margin_search_nodes=1000)
    domains=[[shifted(p,i,j,y) for j,y in enumerate(ys)] for i,ys in enumerate(((0.,30.),(5.,60.),(10.,90.)))]
    search=NativeJointSearch(c);incumbent=search.solve(domains);best,quality=search.refine(domains,incumbent)
    oracle=JointSearch(c)
    feasible=[plans for plans in itertools.product(*domains) if all(oracle.compatible(a,b) for a,b in itertools.combinations(plans,2))]
    assert quality['catalog_optimal']
    assert search.score(best)==max(oracle.score(plans) for plans in feasible)
