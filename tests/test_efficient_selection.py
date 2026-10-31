"""Finite-catalog ranking oracle; physical validation remains a separate gate."""
from dataclasses import replace
from itertools import product
import numpy as np
import pytest
from swarm_sim.trajectory2d import Config,plan_mission
from swarm_sim.trajectory2d.native_planning import NativeJointSearch
from swarm_sim.trajectory2d.execution import simulate
from test_execution_boundaries import straight_flight
from search_oracle import margins,shifted
from test_tangent_approach import scene


def objective(plans,config):
    slack=margins(plans,config)
    arrivals=[p.arrival_time for p in plans];distance=0.
    for plan in plans:
        xyz=np.column_stack((plan.states[:,:2],plan.vertical_states[:,0]))
        end=plan.arrival_time/plan.dt;k=int(np.floor(end))
        distance+=np.linalg.norm(np.diff(xyz[:k+1],axis=0),axis=1).sum()
        if k+1<len(xyz):distance+=(end-k)*np.linalg.norm(xyz[k+1]-xyz[k])
    return (min(slack[0],config.efficiency_slack_cap_m) if slack else config.efficiency_slack_cap_m,
            -max(arrivals),-sum(arrivals),-distance)


@pytest.mark.parametrize('seed',range(8))
def test_efficient_selection_matches_independent_exhaustive_oracle(seed):
    config,base=straight_flight()
    config=replace(config,selection_policy='efficient',max_search_nodes=10000,margin_search_nodes=10000)
    rng=np.random.default_rng(seed)
    domains=[[replace(shifted(base,i,j,y),arrival_time=float(rng.choice([2.,2.5,3.,4.])))
              for j,y in enumerate([60.*i,*rng.uniform(60.*i-70,60.*i+70,3)])] for i in range(3)]
    search=NativeJointSearch(config);initial=search.solve(domains)
    assert initial is not None
    best,quality=search.refine(domains,initial)
    admissible=[plans for plans in product(*domains) if all(m>=1e-7 for m in margins(plans,config))]
    assert quality['catalog_optimal']
    np.testing.assert_allclose(search.efficiency_score(best),max(objective(p,config) for p in admissible),atol=1e-12,rtol=0)
    assert all(m>=1e-7 for m in margins(best,config))


def test_single_aircraft_efficiency_and_shared_budget_preserve_incumbent():
    config,base=straight_flight();config=replace(config,selection_policy='efficient')
    domains=[[replace(base,candidate_id=0,arrival_time=3.),replace(base,candidate_id=1,arrival_time=2.)]]
    search=NativeJointSearch(config);initial=search.solve(domains);best,quality=search.refine(domains,initial)
    assert quality['catalog_optimal'] and best[0].candidate_id==1
    bounded=NativeJointSearch(replace(config,max_search_nodes=1))
    initial=bounded.solve(domains);best,quality=bounded.refine(domains,initial)
    assert best[0].candidate_id==initial[0].candidate_id and bounded.nodes==1
    assert not quality['catalog_optimal'] and bounded.optimization_budget_exhausted


def test_tangent_fleet_efficiency_reduces_arrival_makespan_and_executes():
    snapshots,config=scene();config=replace(config,approach_routing='tangent')
    before=plan_mission(snapshots,config)
    efficient=replace(config,selection_policy='efficient')
    after=plan_mission(snapshots,efficient)
    assert before.validated and after.validated
    assert max(p.arrival_time for p in after.plans)<max(p.arrival_time for p in before.plans)
    selection=after.diagnostics['selection']
    assert selection['selected_efficiency_score'][0]>=config.efficiency_slack_cap_m
    report=simulate(after.plans,efficient).report
    assert report['execution_validated'],report['execution_failures']
    parallel=plan_mission(snapshots,replace(efficient,planning_workers=2))
    assert parallel.validated and [p.to_dict() for p in parallel.plans]==[p.to_dict() for p in after.plans]


@pytest.mark.parametrize('value',[-1,float('nan'),float('inf'),True,'20',None])
def test_invalid_efficiency_preference_rejected(value):
    with pytest.raises(ValueError,match='efficiency_slack_cap'):Config(efficiency_slack_cap_m=value)
