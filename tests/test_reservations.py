"""Conservative cell/time allocation is additional to physical separation."""

from dataclasses import replace
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.reservations import Occupancy, occupancy, first_conflict, safe_intervals, ReservationBudgetError
from swarm_sim.trajectory2d.search import JointSearch
from test_execution_boundaries import straight_flight
from search_oracle import shifted, margins


def test_grid_cannot_make_physically_unsafe_native_plans_compatible():
    config, plan = straight_flight()
    first, second = shifted(plan, 0, 0, 0.), shifted(plan, 1, 0, 0.)
    assert margins((first, second), config)[0] < 0
    for policy in ('none', 'grid'):
        assert not JointSearch(replace(config, reservation_policy=policy)).compatible(first, second)


def test_native_grid_rejects_geometrically_separated_plans_sharing_a_resource():
    config, plan = straight_flight()
    config = replace(config, reservation_policy='grid', reservation_cell_size=100.)
    first, second = shifted(plan, 0, 0, 0.), shifted(plan, 1, 0, 30.)
    assert margins((first, second), config)[0] > 0
    assert first_conflict(occupancy(first, config), occupancy(second, config)) is not None
    assert JointSearch(replace(config, reservation_policy='none')).compatible(first, second)
    assert not JointSearch(config).compatible(first, second)


def test_final_native_reservation_check_ignores_an_incorrect_cached_decision(monkeypatch):
    config, plan = straight_flight()
    config = replace(config, reservation_policy='grid', reservation_cell_size=100.)
    plans = (shifted(plan, 0, 0, 0.), shifted(plan, 1, 0, 30.))
    search = JointSearch(config)
    monkeypatch.setattr(search, 'compatible', lambda *a: True)
    with pytest.raises((RuntimeError, ValueError), match='reservation verification'):
        search.verify_joint(plans)


def test_native_mission_reservation_budget_failure_is_not_infeasibility():
    from test_native_backend import case
    _, config = case(reservation_policy='grid', reservation_max_entries=1, candidates=1, horizon=150.)
    snapshots = tuple(replace(s, altitude=80.).with_speed(13., config.limits)
                      for s in separate_loiter_snapshot(2))
    result = plan_mission(snapshots, config)
    assert result.status == 'reservation_budget_exhausted'
    assert not result.validated and not result.plans


@pytest.mark.parametrize('values', [{'reservation_policy': 'bad'}, {'reservation_cell_size': 0},
    {'reservation_cell_size': float('nan')}, {'reservation_max_entries': 0}, {'reservation_max_entries': True}])
def test_invalid_reservation_config(values):
    with pytest.raises(ValueError):
        Config(**values)


def test_closed_intervals_reject_touching_and_allow_geometric_reuse_later():
    first = Occupancy(0, 0, {(0, 0): ((1, 2),)}, 1.)
    touching = Occupancy(1, 0, {(0, 0): ((2, 3),)}, 1.)
    later = replace(touching, cells={(0, 0): ((2.1, 3),)})
    assert first_conflict(first, touching)['start_s'] == 2
    assert first_conflict(first, later) is None
    assert first_conflict(first, replace(touching, cells={(1, 0): ((1, 3),)})) is None
    safe = safe_intervals([(1, 2), (1.5, 3), (5, 6)], 8)
    assert safe[0][1] < 1 and safe[1][0] > 3 and safe[1][1] < 5 and safe[-1][0] > 6


def test_reservations_cover_between_sample_crossings_and_negative_cells():
    config = Config(dt=1, reservation_policy='grid', reservation_cell_size=5)
    a = Trajectory(0, 0, 12, 1, np.array([[-6., -1, 0, 0], [6, -1, 0, 0]]), np.zeros(1), 1, 0)
    b = replace(a, drone_id=1, states=np.array([[0., -7, 0, 0], [0, 5, 0, 0]]))
    reserved_a, reserved_b = occupancy(a, config), occupancy(b, config)
    assert first_conflict(reserved_a, reserved_b) is not None
    assert any(min(cell) < 0 for cell in reserved_a.cells)
    assert reserved_a.inflation_m == pytest.approx(4 + 2 + 12 * (.15 + .5))
    with pytest.raises(ReservationBudgetError):
        occupancy(a, replace(config, reservation_max_entries=1))
