"""Own-reference C++ sampling retains an independent Python audit oracle."""
from dataclasses import replace
import numpy as np
import pytest
from swarm_sim.trajectory2d import plan_mission
from swarm_sim.trajectory2d import native_backend as native
from swarm_sim.trajectory2d.execution import Executor
from swarm_sim.trajectory2d.flight3d import trim_control
from test_native_backend import case


@pytest.fixture(scope='module', params=['baseline', 'rich'])
def reference(request):
    changes = {}
    if request.param == 'rich':
        changes = dict(cruise_levels=3, speed_profile='smooth', speed_profile_range=(10.,14.),
            horizon=210., target=(500.,500.), cruise_altitude_range=(30.,120.), max_descent_rate=10., attempts=24)
    snapshot, config = case(**changes)
    result = plan_mission((snapshot,), config)
    assert result.validated
    return config, result.plans[0]


def audited_feedforward(plan, config, time, duration):
    result = np.zeros(4)
    cursor, end = time, time+duration
    while cursor < end-1e-10:
        if cursor < -1e-10:
            boundary, control = min(0.,end), trim_control(plan.flight_states[0], config.flight)
        elif cursor >= plan.horizon-1e-10:
            boundary, control = end, trim_control(plan.flight_states[-1], config.flight)
        else:
            index = int(np.clip(np.floor(cursor/plan.dt+1e-9),0,len(plan.flight_controls)-1))
            boundary, control = min((index+1)*plan.dt,end), plan.flight_controls[index]
        result += max(0.,boundary-cursor)/duration*control
        cursor = boundary
    return result


def test_native_window_matches_independent_dynamics_and_both_boundary_trims(reference):
    config, plan = reference
    handle = native.create_reference(plan, config)
    times = [-.03,-1e-11,0.,.015,.2-1e-12,.2,.2+1e-12,
             plan.horizon-.015,plan.horizon,plan.horizon+.03]
    times.extend(np.random.default_rng(619).uniform(-.1,plan.horizon+.1,30))
    for time in times:
        for duration in (.0001,.19998,.20002,.45):
            state, control = native.reference_window(handle,time,duration)
            np.testing.assert_allclose(state,plan.sample_flight(time),atol=1e-9,rtol=0)
            np.testing.assert_allclose(control,audited_feedforward(plan,config,time,duration),atol=1e-12,rtol=0)
            assert not state.flags.writeable and not control.flags.writeable


def test_native_reference_owns_its_history_and_rejects_invalid_intervals(reference):
    config, plan = reference
    states, controls = plan.flight_states.copy(), plan.flight_controls.copy()
    module = native.require_execution_native()
    handle = module.reference_new(states,controls,native.parameters(config))
    expected = native.reference_window(handle,.015,.2)
    states[:]=0.; controls[:]=0.
    actual = native.reference_window(handle,.015,.2)
    for a,b in zip(expected,actual):
        np.testing.assert_array_equal(a,b)
    for time,duration in [(np.nan,.2),(0.,0.),(0.,-.1),(0.,np.inf)]:
        with pytest.raises(ValueError):
            native.reference_window(handle,time,duration)
    with pytest.raises(ValueError):
        module.reference_new(states[:-1],controls,native.parameters(config))


def test_production_controller_never_calls_python_reference_dynamics(reference, monkeypatch):
    config, plan = reference
    import swarm_sim.trajectory2d.flight3d as physics
    monkeypatch.setattr(physics,'propagate',lambda *a,**k:pytest.fail('Python dynamics reached the production controller'))
    executor = Executor(plan,config.limits)
    command = executor.flight_command(.015,.2,config)
    assert np.all(np.isfinite(command)) and not executor.filter_unresolved
    with pytest.raises(ValueError,match='changed during execution'):
        executor.flight_command(.215,.2,replace(config))
