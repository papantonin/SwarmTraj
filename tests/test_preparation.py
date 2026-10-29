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


@pytest.mark.parametrize('backend,duration,ready',[('python',8.,False),('cpp',.2,True)])
def test_measured_backend_latency_gates_fixed_activation_without_refresh(monkeypatch,backend,duration,ready):
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.preparation as preparation
    snapshots=separate_loiter_snapshot(3)
    config=Config(dimensions=3,generator_mode='physical_mission',numerical_backend=backend,activation_time=3.)
    seen=[]
    def plan(actual_snapshots,actual_config):
        seen.append((actual_snapshots,actual_config))
        return Result('validated','synthetic planning result for timeline only')
    monkeypatch.setattr(planning,'plan_mission',plan)
    clock=iter((100.,100.+duration));monkeypatch.setattr(preparation,'perf_counter',lambda:next(clock))
    result=prepare_mission(snapshots,config)
    gate=result.timings['preparation_timeline']
    assert seen==[(snapshots,config)] and config.activation_time==3.
    assert gate['activation_time_s']==3. and gate['latest_ready_time_s']==pytest.approx(2.35)
    assert gate['aircraft'][-1]['acknowledged_at_s']==pytest.approx(duration+.85)
    assert gate['all_ready']==ready and result.validated==ready
    assert result.timings['planning_measured_wall_s']==pytest.approx(duration)
    assert result.status==('validated' if ready else 'preparation_not_ready')


@pytest.mark.parametrize('duration,ready',[(1.5-1e-8,True),(1.5,True),(1.5+1e-8,False)])
def test_three_aircraft_readiness_boundary_retains_original_guard(duration,ready):
    config=Config(activation_time=3.)
    gate=readiness(separate_loiter_snapshot(3),config,Preparation(),duration)
    assert gate['all_ready']==ready


@pytest.mark.parametrize('backend',['python','cpp'])
def test_explicit_simulated_latency_and_missing_ack_override_fast_computation(monkeypatch,backend):
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.preparation as preparation
    monkeypatch.setattr(planning,'plan_mission',lambda *a:Result('validated','timeline only'))
    config=Config(dimensions=3,generator_mode='physical_mission',numerical_backend=backend,activation_time=3.)
    for settings,ready in ((Preparation(simulated_planning_s=.2),True),
                           (Preparation(simulated_planning_s=8.),False),
                           (Preparation(simulated_planning_s=.2,missing_ack_ids=(1,)),False)):
        clock=iter((0.,9.));monkeypatch.setattr(preparation,'perf_counter',lambda:next(clock))
        result=prepare_mission(separate_loiter_snapshot(3),config,settings)
        assert result.validated==ready
        assert result.timings['preparation_timeline']['activation_time_s']==3.
        assert result.timings['planning_measured_wall_s']==9.
