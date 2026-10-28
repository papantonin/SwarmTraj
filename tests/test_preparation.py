"""Preparation uses the designation clock, never a reset physical initial state."""

from dataclasses import replace
import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, separate_loiter_snapshot
from swarm_sim.trajectory2d.preparation import Preparation, readiness, prepare_mission
from swarm_sim.trajectory2d.planning import Result


@pytest.mark.parametrize("values", [{"guard_s": -1}, {"upload_delay_s": float('nan')},
    {"simulated_planning_s": -1}, {"missing_ack_ids": (0, 0)}, {"missing_ack_ids": (-1,)}])
def test_invalid_preparation(values):
    with pytest.raises(ValueError):
        Preparation(**values)


def test_uploads_are_sequential_guarded_and_motion_is_predicted_from_old_snapshot():
    snapshots = separate_loiter_snapshot()
    config = Config(activation_time=10)
    gate = readiness(snapshots[::-1], config, Preparation(upload_delay_s=1), planning_s=2)
    assert gate['all_ready']
    assert [a['uploaded_at_s'] for a in gate['aircraft']] == [3, 4, 5, 6]
    assert gate['latest_ready_time_s'] == pytest.approx(9.35)
    assert not np.allclose(gate['aircraft'][0]['predicted_activation_state'][:2], snapshots[0].state[:2])
    assert not readiness(snapshots, config, Preparation(missing_ack_ids=(2,)), 2)['all_ready']
    assert not readiness(snapshots, config, Preparation(), 10)['all_ready']
    with pytest.raises(ValueError, match='unknown'):
        readiness(snapshots, config, Preparation(missing_ack_ids=(9,)), 2)


def test_not_ready_removes_executable_mission_and_preserves_original_planning_failure(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    monkeypatch.setattr(planning, 'plan_mission', lambda *args: Result('validated', 'synthetic gate only'))
    result = prepare_mission(separate_loiter_snapshot(), Config(activation_time=10),
                             Preparation(simulated_planning_s=20))
    assert result.status == 'preparation_not_ready'
    assert not result.plans and not result.validated
    monkeypatch.setattr(planning, 'plan_mission', lambda *args: Result('no_candidate', 'synthetic'))
    result = prepare_mission(separate_loiter_snapshot(), Config(), Preparation(simulated_planning_s=20))
    assert result.status == 'no_candidate'
    assert not result.diagnostics['preparation']['holding_motion_validated']


def test_simulated_timeline_is_reproducible_but_wall_times_stay_outside_manifest(monkeypatch):
    import swarm_sim.trajectory2d.planning as planning
    monkeypatch.setattr(planning, 'plan_mission', lambda *args: Result('validated', 'synthetic gate only'))
    args = (separate_loiter_snapshot(), Config(activation_time=10), Preparation(simulated_planning_s=2))
    first, second = prepare_mission(*args), prepare_mission(*args)
    assert first.validated and first.diagnostics == second.diagnostics
    assert first.timings['preparation_timeline']['activation_time_s'] == 10
