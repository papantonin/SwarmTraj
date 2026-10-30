"""Tiny exhaustive geometry oracle; no alternative production joint solver."""
from dataclasses import replace
from itertools import combinations, product
from swarm_sim.trajectory2d.planning import continuous_distance_lower_bound, required_distance


def shifted(plan, drone, candidate, y):
    flight, projected = plan.flight_states.copy(), plan.states.copy()
    flight[:, 1] += y
    projected[:, 1] += y
    return replace(plan, drone_id=drone, candidate_id=candidate, flight_states=flight, states=projected)


def margins(plans, config):
    return tuple(sorted(continuous_distance_lower_bound(a, b) - required_distance(a, b, config)
                        for a, b in combinations(plans, 2)))


def feasible(domains, config):
    return [plans for plans in product(*domains) if all(m >= 1e-7 for m in margins(plans, config))]
