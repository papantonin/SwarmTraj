"""Native margin search compared with exhaustive independent geometry checks."""
from dataclasses import replace
import numpy as np
import pytest
from swarm_sim.trajectory2d.native_planning import NativeJointSearch
from test_execution_boundaries import straight_flight
from search_oracle import feasible, margins, shifted


@pytest.mark.parametrize('seed', range(8))
def test_four_aircraft_catalog_optimum_matches_independent_enumeration(seed):
    config, plan = straight_flight()
    config = replace(config, max_search_nodes=10000, margin_search_nodes=10000)
    rng = np.random.default_rng(seed)
    domains = [[shifted(plan, i, j, y) for j, y in enumerate(
        [50. * i, *rng.uniform(50. * i - 70., 50. * i + 70., 3)])] for i in range(4)]
    search = NativeJointSearch(config)
    initial = search.solve(domains)
    assert initial is not None
    selected, info = search.refine(domains, initial)
    oracle = feasible(domains, config)
    assert oracle and info['catalog_optimal']
    np.testing.assert_allclose(search.score(selected), max(margins(p, config) for p in oracle), atol=1e-12, rtol=0)
    assert margins(selected, config) >= margins(initial, config)
    assert all(m >= 1e-7 for m in margins(selected, config))


def test_secondary_margins_improve_with_a_fixed_bottleneck():
    config, plan = straight_flight()
    config = replace(config, max_search_nodes=1000, margin_search_nodes=1000)
    domains = [[shifted(plan, i, j, y) for j, y in enumerate(ys)]
               for i, ys in enumerate(((0.,), (50.,), (100., 150.), (200., 250.)))]
    search = NativeJointSearch(config)
    initial = search.solve(domains)
    selected, info = search.refine(domains, initial)
    assert info['catalog_optimal']
    assert margins(selected, config) == max(margins(p, config) for p in feasible(domains, config))
    assert margins(selected, config)[0] == margins(initial, config)[0]
    assert margins(selected, config) > margins(initial, config)


def test_exhausted_global_budget_retains_the_verified_incumbent():
    config, plan = straight_flight()
    config = replace(config, max_search_nodes=4)
    domains = [[shifted(plan, i, j, 50. * i + 10. * j) for j in range(2)] for i in range(4)]
    search = NativeJointSearch(config)
    initial = search.solve(domains)
    assert initial is not None and search.nodes == 4
    selected, info = search.refine(domains, initial)
    assert [(p.drone_id, p.candidate_id) for p in selected] == [(p.drone_id, p.candidate_id) for p in initial]
    assert search.nodes == 4 and search.optimization_nodes == 0
    assert not info['catalog_optimal'] and search.optimization_budget_exhausted
    assert not search.exhausted
    assert all(m >= 1e-7 for m in margins(selected, config))


def test_refinement_budget_is_cumulative_and_preserves_safety():
    config, plan = straight_flight()
    config = replace(config, max_search_nodes=1000, margin_search_nodes=3)
    domains = [[shifted(plan, i, j, 50. * i + 15. * j) for j in range(3)] for i in range(4)]
    search = NativeJointSearch(config)
    initial = search.solve(domains)
    first, _ = search.refine(domains, initial)
    used = search.optimization_nodes
    second, _ = search.refine(domains, first)
    assert used <= search.optimization_nodes <= config.margin_search_nodes
    assert margins(second, config) >= margins(first, config) >= margins(initial, config)


def test_single_aircraft_has_no_pairwise_objective():
    config, plan = straight_flight()
    search = NativeJointSearch(config)
    assert search.score((plan,)) == ()
