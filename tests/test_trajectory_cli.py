"""Verify destination semantics before any expensive planning or execution."""

from pathlib import Path
import json
import runpy
import sys

import pytest

import swarm_sim.trajectory2d as trajectory


@pytest.mark.parametrize("options,expected_altitude,expected_range", [
    ([], 30, (30, 120)), (["--target-altitude", "45"], 45, (30, 120)),
    (["--arrival-mode", "layers"], None, (30, 120)),
    (["--cruise-range", "40", "110"], 30, (40, 110)),
    (["--manual-cruise"], 30, None),
    (["--descent-angle", "10"], 30, (30, 120)),
    (["--descent-angle", "0"], 30, (30, 120)),
    (["--approach-assignment", "id"], 30, (30, 120)),
    (["--workers", "2", "--rollout-check-interval", "1"], 30, (30, 120)),
    (["--planning-strategy", "fixed", "--search-order", "id", "--initial-candidates", "2", "--attempts", "30"], 30, (30, 120)),
    (["--conflict-guidance", "none"], 30, (30, 120)),
    (["--candidate-guidance", "rejections"], 30, (30, 120)),
    (["--profile-policy", "adaptive_mixed", "--cruise-levels", "2",
      "--speed-profile", "smooth", "--speed-profile-range", "10", "14"], 30, (30, 120)),
    (["--reservation-policy", "grid", "--reservation-cell-size", "10"], 30, (30, 120)),
    (["--reservation-policy", "grid", "--interval-guidance", "safe_windows", "--cruise-levels", "2",
      "--speed-profile", "smooth", "--speed-profile-range", "10", "14"], 30, (30, 120)),
    (["--selection-policy", "first", "--margin-search-nodes", "50", "--margin-expansion-rounds", "0"], 30, (30, 120)),
    (["--arrival-deadline", "70", "--post-arrival-time", "10", "--obstacle-routing", "lateral"], 30, (30, 120)),
    (["--obstacle-cylinder", "200", "120", "25", "0", "150", "--obstacle-box", "300", "170", "0", "340", "210", "130",
      "--obstacle-clearance", "7"], 30, (30, 120)),
    (["--cruise-altitude", "40", "--cruise-altitude", "65",
      "--cruise-altitude", "90", "--cruise-altitude", "115"], 30, None),
])
def test_cli_preserves_common_xyz_objective_by_default(monkeypatch, tmp_path, options, expected_altitude, expected_range):
    class PlanningReached(Exception):
        pass

    def inspect(snapshots, config):
        assert config.dimensions == 3
        assert config.target_altitude == expected_altitude
        assert config.cruise_altitude_range == expected_range
        expected_angle = (float(options[options.index("--descent-angle") + 1]) if "--descent-angle" in options
                          else 0 if expected_altitude is None else 30)
        assert config.descent_angle_deg == expected_angle
        assert config.flight_model == ("legacy" if "--flight-model" in options else "point_mass")
        assert config.max_climb_rate == 3
        assert config.descent_rate_limit == 10
        assert config.approach_assignment == ("id" if "--approach-assignment" in options else "geometric")
        assert config.planning_workers == (2 if "--workers" in options else
                                            1 if config.numerical_backend == 'cpp' else 0)
        assert config.rollout_check_interval == (1 if "--rollout-check-interval" in options else 16)
        assert config.planning_strategy == ("fixed" if "--planning-strategy" in options else "adaptive")
        assert config.conflict_guidance == ("none" if "--conflict-guidance" in options else "witness")
        assert config.reservation_policy == ("grid" if "--reservation-policy" in options else "none")
        assert config.reservation_cell_size == (10 if "--reservation-cell-size" in options else 20)
        assert config.interval_guidance == ("safe_windows" if "--interval-guidance" in options else "none")
        assert config.cruise_levels == (2 if "--cruise-levels" in options else 1)
        assert config.profile_policy == ("adaptive_mixed" if "--profile-policy" in options else "strict")
        assert config.passage_guidance == ("safe_windows" if "--passage-guidance" in options else "none")
        assert config.candidate_guidance == ("rejections" if "--candidate-guidance" in options else "none")
        assert config.speed_profile == ("smooth" if "--speed-profile" in options else "constant")
        assert config.speed_profile_range == ((10, 14) if "--speed-profile-range" in options else None)
        assert config.search_order == ("id" if "--search-order" in options else "mrv")
        assert config.initial_candidates == (2 if "--initial-candidates" in options else 4)
        assert config.attempts == (30 if "--attempts" in options else 90)
        assert len(config.obstacles) == (2 if "--obstacle-cylinder" in options else 0)
        assert config.obstacle_clearance == (7 if "--obstacle-clearance" in options else 5)
        assert config.selection_policy == ("first" if "--selection-policy" in options else "max_margin")
        assert config.margin_search_nodes == (50 if "--margin-search-nodes" in options else 2000)
        assert config.margin_expansion_rounds == (0 if "--margin-expansion-rounds" in options else 1)
        assert config.arrival_deadline == (70 if "--arrival-deadline" in options else None)
        assert config.post_arrival_time == (10 if "--post-arrival-time" in options else 5)
        assert config.obstacle_routing == ("lateral" if "--obstacle-routing" in options else "spatial")
        raise PlanningReached

    monkeypatch.setattr(trajectory, "plan_mission", inspect)
    script = Path(__file__).resolve().parents[1] / "examples" / "trajectory_2d.py"
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, "argv", [str(script), "--no-gui", "--output", str(tmp_path), *options])
    with pytest.raises(PlanningReached):
        namespace["main"]()


def test_failed_planning_exports_diagnostics_and_timings_separately(monkeypatch, tmp_path):
    result = trajectory.Result("no_candidate", "Synthetic failed planning for export test.",
                               counts=(0, 0, 0, 0), diagnostics={"rounds": [], "catalogs": []},
                               timings={"total_wall_s": 0.25})
    monkeypatch.setattr(trajectory, "plan_mission", lambda *args: result)
    script = Path(__file__).resolve().parents[1] / "examples" / "trajectory_2d.py"
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, "argv", [str(script), "--no-gui", "--output", str(tmp_path)])
    assert namespace["main"]() == 2
    manifest = json.loads((tmp_path / "mission.json").read_text())
    report = json.loads((tmp_path / "planning_report.json").read_text())
    assert manifest["planning_diagnostics"] == result.diagnostics
    assert "timings" not in manifest and "total_wall_s" not in json.dumps(manifest)
    assert report["timings"]["total_wall_s"] == 0.25
    assert report["status"] == "no_candidate"
    assert not (tmp_path / "report.json").exists()
    assert not list(tmp_path.glob('drone_*.json'))


def test_missing_acknowledgement_blocks_cli_execution_and_individual_payloads(monkeypatch, tmp_path):
    import swarm_sim.trajectory2d.planning as planning
    import swarm_sim.trajectory2d.execution as execution
    monkeypatch.setattr(planning, 'plan_mission', lambda *args: trajectory.Result('validated', 'synthetic planning'))
    monkeypatch.setattr(execution, 'simulate', lambda *args, **kwargs: pytest.fail('blocked mission must not execute'))
    script = Path(__file__).resolve().parents[1] / 'examples' / 'trajectory_2d.py'
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, 'argv', [str(script), '--no-gui', '--output', str(tmp_path),
        '--prepare-in-flight', '--activation-time', '30', '--planning-latency', '5', '--missing-ack', '2'])
    assert namespace['main']() == 2
    report = json.loads((tmp_path / 'planning_report.json').read_text())
    assert report['status'] == 'preparation_not_ready'
    assert not report['diagnostics']['preparation']['all_ready']
    assert not list(tmp_path.glob('drone_*.json'))
    assert not (tmp_path / 'report.json').exists()


@pytest.mark.parametrize('options', [
    ['--wind', '-1'], ['--position-noise', 'nan'], ['--position-bias', '-1'],
    ['--clock-drift', '-.01'], ['--wind-heading', 'inf'], ['--disturbance-seed', '-1'],
    ['--upload-delay', '-1'], ['--planning-latency', 'nan'], ['--activation-guard', '-1'],
])
def test_invalid_execution_settings_are_rejected_before_planning(monkeypatch, tmp_path, options):
    monkeypatch.setattr(trajectory, 'plan_mission', lambda *a: pytest.fail('planning must not run'))
    script = Path(__file__).resolve().parents[1] / 'examples' / 'trajectory_2d.py'
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, 'argv', [str(script), '--no-gui', '--output', str(tmp_path), *options])
    with pytest.raises(SystemExit) as error:
        namespace['main']()
    assert error.value.code == 2


def test_cli_passes_disturbances_to_local_execution(monkeypatch, tmp_path):
    import numpy as np
    import swarm_sim.trajectory2d.execution as execution

    class SimulationReached(Exception):
        pass

    plans = tuple(trajectory.Trajectory(k, 0, 12, .2, np.zeros((2, 4)), np.zeros(1), 1, 0,
                                      cruise_altitude=60) for k in range(4))
    monkeypatch.setattr(trajectory, 'plan_mission', lambda *a: trajectory.Result('validated', 'CLI contract test', plans))

    def inspect(plans, config, **options):
        assert options['wind'] == .2 and options['wind_model'] == 'gust'
        assert options['wind_heading_deg'] == 35 and options['controller'] == 'constrained'
        assert options['position_noise'] == .02 and options['disturbance_seed'] == 9
        np.testing.assert_allclose(np.linalg.norm(options['position_biases'], axis=1), .1)
        np.testing.assert_allclose(options['clock_drifts'], np.linspace(-.0002, .0002, 4))
        raise SimulationReached

    monkeypatch.setattr(execution, 'simulate', inspect)
    script = Path(__file__).resolve().parents[1] / 'examples' / 'trajectory_2d.py'
    namespace = runpy.run_path(str(script))
    monkeypatch.setattr(sys, 'argv', [str(script), '--no-gui', '--output', str(tmp_path),
        '--wind', '.2', '--wind-model', 'gust', '--wind-heading', '35', '--tracking-controller', 'constrained',
        '--position-bias', '.1', '--position-noise', '.02', '--disturbance-seed', '9', '--clock-drift', '.0002'])
    with pytest.raises(SimulationReached):
        namespace['main']()
