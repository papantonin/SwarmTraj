from dataclasses import replace

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.planning import validate_individual
from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor


def snapshots():
    return tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()))


@pytest.mark.parametrize("kwargs", [{"descent_angle_deg": -1}, {"descent_angle_deg": 90},
                                   {"descent_angle_deg": np.nan}, {"descent_angle_deg": np.inf},
                                   {"descent_angle_deg": 10, "target_altitude": None},
                                   {"descent_angle_deg": 10, "dimensions": 2, "target_altitude": None}])
def test_invalid_descent_configuration(kwargs):
    params = dict(dimensions=3, target_altitude=30)
    params.update(kwargs)
    with pytest.raises(ValueError):
        Config(**params)


def test_common_descent_angle_passage_recovery_and_separation():
    # Preserve this historical 2.5D catalogue regression, including its ID slots.
    config = Config(dimensions=3, flight_model="legacy", approach_assignment="id", target=(500, 300), target_altitude=30, horizon=150,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180,
                    cruise_altitude_range=(40, 110), descent_angle_deg=10)
    initial = snapshots()
    result = plan_mission(initial, config)
    assert result.validated, (result.status, result.counts)
    for s, p in zip(initial, result.plans):
        assert p.release_time == 0
        assert validate_individual(p, s, config)
        passage = p.sample(p.arrival_time)
        assert passage[4] == pytest.approx(30, abs=1e-8)
        assert np.rad2deg(np.arctan2(-passage[5], p.speed)) == pytest.approx(10, abs=1e-8)
        np.testing.assert_allclose(p.vertical_states[-1], [p.cruise_altitude, 0], atol=1e-8)
        assert config.min_altitude < p.vertical_states[:, 0].min() < config.target_altitude
        # Recovery starts continuously, never by resetting the vertical speed.
        just_after = p.sample(p.arrival_time + 0.1)
        assert abs(just_after[5] - passage[5]) <= config.max_vertical_acceleration * 0.1 + 1e-8
        assert not validate_individual(p, s, replace(config, descent_angle_deg=5))
    evaluation = simulate(result.plans, config, clock_offsets=np.linspace(-0.05, 0.05, 4), wind=0.05)
    assert evaluation.report["all_arrived"]
    assert evaluation.report["vertical_limits_verified"]
    assert evaluation.report["separation_verified_in_execution_model"]
    assert evaluation.report["tube_conditions_verified"]
    assert evaluation.report["descent_angle_verified_in_execution"]
    np.testing.assert_allclose(evaluation.report["actual_descent_angles_deg"], 10, atol=1)


@pytest.mark.parametrize("changes", [{"descent_angle_deg": 30}, {"target_altitude": 0},
                                     {"cruise_altitude_range": (30, 31)}])
def test_unachievable_descent_or_recovery_is_rejected_before_generation(changes):
    config = Config(dimensions=3, flight_model="legacy", target_altitude=30, descent_angle_deg=10, cruise_altitude_range=(40, 110))
    result = plan_mission(snapshots(), replace(config, **changes))
    assert result.status == "descent_not_feasible"
    assert not result.plans
    assert result.counts == ()


def test_editor_descent_limit_reset_and_required_xyz_mode():
    import matplotlib.pyplot as plt
    editor = ScenarioEditor(snapshots(), (500, 300), Config(dimensions=3, target_altitude=30,
                            cruise_altitude_range=(40, 110)))
    try:
        editor.descent_input.set_val("30")
        editor.start()
        assert not editor.accepted
        assert "descent angle limited" in editor.status.get_text()
        editor.descent_input.set_val("10")
        assert editor.apply_fields()
        assert editor.config.descent_angle_deg == 10
        editor.common_altitude.set_active(0)
        assert editor.common_altitude.get_status()[0]
        assert editor.config.target_altitude == 30
        editor.reset()
        assert editor.config.descent_angle_deg == 0
        assert editor.descent_input.text == "0"
        editor.descent_input.set_val("10")
        editor.start()
        assert editor.accepted
    finally:
        plt.close(editor.figure)
