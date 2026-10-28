"""Conservative cell/time allocation is additional to physical separation."""

from dataclasses import replace
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.reservations import Occupancy, occupancy, first_conflict, safe_intervals, ReservationBudgetError
from swarm_sim.trajectory2d.search import JointSearch


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


def test_grid_does_not_override_physical_separation_check(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    a = Trajectory(0, 0, 12, .2, np.zeros((2, 4)), np.zeros(1), 1, 0)
    monkeypatch.setattr(planning, 'continuous_distance_lower_bound', lambda *args: 0.)
    search = JointSearch(Config(reservation_policy='grid'))
    assert not search.compatible(a, replace(a, drone_id=1))
    assert not search.occupancies  # No grid argument can make unsafe geometry pass.


def test_reservation_budget_exhaustion_is_not_an_infeasibility_proof():
    config = Config(reservation_policy='grid', reservation_max_entries=1, candidates=1)
    result = plan_mission(separate_loiter_snapshot(), config)
    assert result.status == 'reservation_budget_exhausted'
    assert not result.validated and not result.plans


def test_cell_time_constraint_can_reject_an_otherwise_separated_pair():
    a = Trajectory(0, 0, 12, .2, np.tile([1., 1, 0, 0], (2, 1)), np.zeros(1), 1, 0)
    b = replace(a, drone_id=1, states=np.tile([30., 1, 0, 0], (2, 1)))
    config = Config(reservation_policy='grid', reservation_cell_size=100)
    assert JointSearch(replace(config, reservation_policy='none')).compatible(a, b)
    search = JointSearch(config)
    assert not search.compatible(a, b)
    assert search.diagnostics()['reservation_conflicts'] == 1


def test_independent_final_reservation_check_rejects_wrong_search(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    points = [(1, 1), (30, 1), (1, 30), (30, 30)]
    def candidate(snapshot, config, candidate_id, rng, *args, **kwargs):
        return Trajectory(snapshot.drone_id, candidate_id, 12, .2,
                          np.tile([*points[snapshot.drone_id], 0, 0], (2, 1)), np.zeros(1), 1, 0)
    monkeypatch.setattr(planning, '_candidate', candidate)
    monkeypatch.setattr(planning, 'validate_individual', lambda *args, **kwargs: True)
    monkeypatch.setattr(JointSearch, 'compatible', lambda *args: True)
    with pytest.raises(RuntimeError, match='independent reservation'):
        plan_mission(separate_loiter_snapshot(), Config(candidates=1, reservation_policy='grid', reservation_cell_size=100))
