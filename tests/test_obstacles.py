"""Static solids: geometry, between-sample bounds, routing and four-aircraft flight."""

from dataclasses import asdict, replace
import json

import numpy as np
import pytest

from swarm_sim.trajectory2d import Config, Obstacle, Trajectory, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import (execution_clearance, load_obstacles,
    nominal_clearance_required, obstacle_preflight, plan_clearance)
from swarm_sim.trajectory2d.planning import validate_individual


@pytest.mark.parametrize("kind,point,expected", [
    ("box", (0, 0, 0), -1), ("box", (4, 0, 0), 3),
    ("box", (2, 3, 0), np.sqrt(2)), ("box", (0, 0, 5), 2),
    ("cylinder", (0, 0, 0), -1), ("cylinder", (0, 0, 5), 2),
    ("cylinder", (2, 0, 4), np.sqrt(2)), ("cylinder", (0, 1, 0), 0),
])
def test_signed_distances_include_sides_roofs_and_interiors(kind, point, expected):
    obstacle = Obstacle(kind, (0, 0, 0), (1, 2, 3) if kind == "box" else (1, 1, 3))
    assert obstacle.distance(point) == pytest.approx(expected)


def test_2d_projection_and_3d_overflight_are_distinct():
    for obstacle in (Obstacle.cylinder(0, 0, 2, 0, 10), Obstacle.box(-2, -2, 0, 2, 2, 10)):
        assert obstacle.distance((0, 0)) == -2
        assert obstacle.distance((0, 0, 30)) == 20
        values = obstacle.distance(np.array([[[0, 0, 30], [0, 0, 5]]]))
        np.testing.assert_allclose(values, [[20, -2]])


def test_signed_distance_is_1_lipschitz_on_random_points():
    rng = np.random.default_rng(42)
    a, b = rng.normal(size=(2, 1000, 3)) * 20
    for obstacle in (Obstacle.cylinder(2, 3, 5, -10, 30), Obstacle.box(-2, -5, -1, 4, 7, 10)):
        assert np.all(np.abs(obstacle.distance(a) - obstacle.distance(b)) <= np.linalg.norm(a - b, axis=1) + 1e-10)


@pytest.mark.parametrize("parameters", [
    ("sphere", (0, 0, 0), (1, 1, 1)), ("box", (0, 0), (1, 1, 1)),
    ("box", (np.nan, 0, 0), (1, 1, 1)), ("box", (0, 0, 0), (0, 1, 1)),
    ("cylinder", (0, 0, 0), (1, 2, 1)), ("box", (0, 0, 0), (1, 1, np.inf)),
])
def test_invalid_obstacles_are_rejected(parameters):
    with pytest.raises(ValueError):
        Obstacle(*parameters)


@pytest.mark.parametrize("values", [{"obstacle_clearance": -1}, {"obstacle_clearance": np.nan},
                                     {"obstacles": []}, {"obstacles": ({},)}])
def test_invalid_obstacle_configuration(values):
    with pytest.raises(ValueError):
        Config(**values)


def test_json_round_trip_and_invalid_records(tmp_path):
    obstacles = (Obstacle.cylinder(20, 50, 10, 0, 100), Obstacle.box(1, 2, 3, 5, 6, 7))
    path = tmp_path / "obstacles.json"
    path.write_text(json.dumps([asdict(o) for o in obstacles]))
    assert load_obstacles(path) == obstacles
    for record in ({}, [None], [{"kind": "box"}], [{"kind": "sphere", "center": [0, 0, 0], "half_extents": [1, 1, 1]}]):
        path.write_text(json.dumps(record))
        with pytest.raises(ValueError):
            load_obstacles(path)


def test_between_sample_obstacle_collision_is_not_missed():
    # Synthetic constant-speed primitive crosses the cylinder at t=0.5.
    states = np.array([[-10, 0, 0, 0], [10, 0, 0, 0]])
    plan = Trajectory(0, 0, 20, 1, states, np.zeros(1), 1, 0)
    config = Config(obstacles=(Obstacle.cylinder(0, 0, 1, -1, 1),),
                    obstacle_clearance=0, position_error=0, clock_error=0)
    assert config.obstacles[0].distance(plan.positions).min() == 9
    assert plan_clearance(plan, config) == -1
    assert plan_clearance(plan, config) < nominal_clearance_required(plan.speed_bound, config)


def test_execution_checker_does_not_validate_with_a_violated_speed_bound():
    states = np.array([[0, 0, 0, 0], [1, 0, 0, 0]])
    plan = Trajectory(0, 0, 12, 0.2, states, np.zeros(1), 0.2, 0)
    config = Config(obstacles=(Obstacle.cylinder(100, 100, 1, 0, 100),))
    report = execution_clearance(plan.positions[:, None], [plan], config, 0.02, 0, speed_bound_valid=False)
    assert not report["obstacle_clearance_verified_in_execution_model"]
    assert report["minimum_obstacle_distance_lower_bound_m"] is None


def test_initial_obstacle_and_blocked_destination_have_specific_diagnostics():
    snapshots = tuple(replace(s, altitude=80.) for s in separate_loiter_snapshot(4))
    x, y = snapshots[0].state[:2]
    inside = Config(dimensions=3, target_altitude=80., obstacles=(Obstacle.cylinder(x, y, 10, 0, 100),))
    assert plan_mission(snapshots, inside).status == "initial_obstacle_violation"
    margin = Config(dimensions=3, target_altitude=80., obstacles=(Obstacle.cylinder(x + 16, y, 10, 0, 100),))
    assert obstacle_preflight(snapshots, margin)[0] == "initial_obstacle_margin_not_certified"
    blocked = Config(dimensions=3, target_altitude=80., obstacles=(Obstacle.cylinder(140, 50, 20, 0, 100),))
    result = plan_mission(snapshots, blocked)
    assert result.status == "target_region_obstructed" and not result.plans




@pytest.fixture(scope="module")
def obstacle_mission():
    config = Config(dimensions=3, generator_mode='physical_mission', numerical_backend='cpp', target=(500, 300), target_altitude=30, horizon=180,
                    cruise_altitude_range=(40, 110), descent_angle_deg=30, max_descent_rate=10,
                    activation_time=0, max_extra_loiter=0, approach_spread_deg=180, candidates=4,
                    planning_strategy="adaptive", search_order="mrv",
                    obstacles=(Obstacle.cylinder(200, 120, 25, 0, 150), Obstacle.box(300, 170, 0, 340, 210, 130)))
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    result = plan_mission(snapshots, config)
    assert result.validated, (result.status, result.diagnostics)
    return config, snapshots, result


def test_four_coupled_aircraft_avoid_two_obstacles_and_obey_flight_limits(obstacle_mission):
    config, snapshots, result = obstacle_mission
    for plan, snapshot in zip(result.plans, snapshots):
        assert validate_individual(plan, snapshot, config)
        assert plan_clearance(plan, config) >= nominal_clearance_required(plan.speed_bound, config)
    evaluation = simulate(result.plans, config)
    for key in ("all_arrived", "physical_limits_verified", "vertical_limits_verified", "tube_conditions_verified",
                "separation_verified_in_execution_model", "approach_sectors_verified_in_execution",
                "descent_angle_verified_in_execution", "obstacle_clearance_verified_in_execution_model"):
        assert evaluation.report[key], key
    assert evaluation.report["minimum_obstacle_distance_lower_bound_m"] > 40
    assert not evaluation.report["sampled_obstacle_collision"]


def test_obstacles_are_checked_after_arrival_not_just_on_approach(obstacle_mission):
    config, snapshots, result = obstacle_mission
    plan = result.plans[0]
    point = plan.sample(plan.arrival_time + 10)
    obstacle = Obstacle.cylinder(*point[:2], 1, -100, 200)
    diagnostic = {}
    assert not validate_individual(plan, snapshots[0], replace(config, obstacles=(obstacle,)), diagnostic)
    assert diagnostic["rejection"] == "obstacle_clearance"


def test_existing_plan_that_would_hit_new_building_is_rejected(obstacle_mission):
    config, snapshots, result = obstacle_mission
    plan = result.plans[0]
    point = plan.sample(0.5 * plan.arrival_time)
    diagnostic = {}
    config = replace(config, obstacles=(Obstacle.cylinder(*point[:2], 8, 0, 200),))
    assert not validate_individual(plan, snapshots[0], config, diagnostic)
    assert diagnostic["rejection"] == "obstacle_clearance"


def test_integrated_overflight_is_not_rejected_for_xy_overlap(obstacle_mission):
    config, snapshots, result = obstacle_mission
    plan = result.plans[0]
    point = plan.sample(0.4 * plan.arrival_time)
    obstacle = Obstacle.cylinder(*point[:2], 5, 0, 1)
    assert obstacle.distance(point[:2]) < 0
    config = replace(config, obstacles=(obstacle,))
    assert validate_individual(plan, snapshots[0], config)


def test_obstacle_catalogs_and_diagnostics_are_spawn_reproducible(obstacle_mission):
    config, snapshots, result = obstacle_mission
    parallel = plan_mission(snapshots[::-1], replace(config, planning_workers=2))
    assert parallel.validated
    assert parallel.diagnostics == result.diagnostics
    assert [p.to_dict() for p in parallel.plans] == [p.to_dict() for p in result.plans]
    assert all(not p.flight_states.flags.writeable for p in parallel.plans)


def test_editor_displays_obstacles_and_refuses_initial_overlap():
    import matplotlib.pyplot as plt
    from swarm_sim.trajectory2d.scenario_editor import ScenarioEditor

    snapshots = tuple(replace(s, altitude=80.) for s in separate_loiter_snapshot(4))
    config = Config(dimensions=3, target_altitude=80., obstacles=(Obstacle.cylinder(200, 100, 10, 0, 100),))
    editor = ScenarioEditor(snapshots, config.target, config)
    try:
        editor.set_advanced(True)
        assert any(text.get_text() == 'O0' for text in editor.axis.texts)
        editor.place((200, 100))
        editor.start()
        assert not editor.accepted
        assert 'obstacle' in editor.status.get_text().lower()
        editor.reset()
        assert editor.config.obstacles == config.obstacles
        assert editor.initial_conflict() is None
    finally:
        plt.close(editor.figure)


def test_3d_obstacle_export_uses_finite_scene_bounds(obstacle_mission, tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    from swarm_sim.trajectory2d.visualization3d import visualize

    config, _, result = obstacle_mission
    evaluation = simulate(result.plans, config)
    close = plt.close
    figures = []
    monkeypatch.setattr(plt, 'close', lambda figure: figures.append(figure))
    try:
        visualize(result, config, evaluation, tmp_path / 'overview.png', gui=False)
        figure = next(fig for fig in figures if hasattr(fig, 'axes'))
        axis = figure.axes[0]
        bounds = [*axis.get_xlim(), *axis.get_ylim(), *axis.get_zlim()]
        assert np.all(np.isfinite(bounds)) and max(np.abs(bounds)) < 10000
        assert (tmp_path / 'overview.png').stat().st_size > 10000
    finally:
        for figure in figures:
            if hasattr(figure, 'axes'):
                close(figure)
