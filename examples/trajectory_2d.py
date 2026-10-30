"""Generate, verify and replay a coordinated mission from one frozen snapshot."""

import argparse
import csv
from dataclasses import asdict, replace
from datetime import datetime
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from swarm_sim.trajectory2d import Config, Limits, separate_loiter_snapshot, plan_mission
from swarm_sim.trajectory2d.execution import simulate, check_execution_backend
from swarm_sim.trajectory2d.planning import mission_document
from swarm_sim.trajectory2d.scenario_editor import edit_scenario
from swarm_sim.trajectory2d.visualization import pick_target, visualize
from swarm_sim.trajectory2d.flight3d import FlightLimits
from swarm_sim.trajectory2d.parallel import worker_count
from swarm_sim.trajectory2d.obstacles import Obstacle, load_obstacles
from swarm_sim.trajectory2d.perturbations import position_bias_vectors, wind_at
from swarm_sim.trajectory2d.aircraft_profiles import load_aircraft_profile


# None defaults let us distinguish explicit overrides, even when the user
# supplied exactly the historical value. A loaded profile is authoritative.
PHYSICAL_DEFAULTS = {
    'stall_speed': 6, 'stall_margin': 1.15, 'max_load_factor': 2.5,
    'max_bank': 50, 'max_roll_rate': 60, 'max_flight_path_angle': 45,
    'max_flight_path_rate': 8, 'max_thrust_acceleration': 5,
    'max_airbrake_acceleration': 5, 'parasitic_drag': .003, 'induced_drag': 20,
    'max_climb_rate': 3, 'max_descent_rate': 10, 'max_vertical_acceleration': 1,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-drones", type=int, default=4)
    parser.add_argument("--dimensions", type=int, choices=(2, 3), default=3,
                        help="2D legacy mode or coupled 3D point-mass flight (default)")
    parser.add_argument("--flight-model", choices=("point_mass", "legacy"), default="point_mass",
                        help="Coupled physical 3D model (default), or legacy independent XY/Z")
    parser.add_argument("--aircraft-profile", type=Path, action='append',
                        help="Versioned fixed-wing JSON profile, common to all drones; coupled 3D only, no physical-flag overrides")
    parser.add_argument('--drone-profile', nargs=2, action='append', metavar=('ID', 'JSON'),
                        help='Explicit airframe for one drone; repeat for every ID, incompatible with common profile/physical flags')
    parser.add_argument("--stall-speed", type=float, help="Illustrative 1g stall speed (m/s; default 6)")
    parser.add_argument("--stall-margin", type=float, help="Multiplicative margin above loaded stall speed (default 1.15)")
    parser.add_argument("--max-load-factor", type=float, help="Structural load ceiling (g; default 2.5)")
    parser.add_argument("--max-bank", type=float, help="Maximum bank angle (degrees; default 50)")
    parser.add_argument("--max-roll-rate", type=float, help="Maximum bank rate (degrees/s; default 60)")
    parser.add_argument("--max-flight-path-angle", type=float, help="Maximum absolute flight-path angle (degrees; default 45)")
    parser.add_argument("--max-flight-path-rate", type=float, help="Maximum flight-path angle rate (degrees/s; default 8)")
    parser.add_argument("--max-thrust-acceleration", type=float, help="Available thrust divided by mass (m/s²; default 5)")
    parser.add_argument("--max-airbrake-acceleration", type=float, help="Optional extra drag divided by mass (default 5); 0 disables it")
    parser.add_argument("--parasitic-drag", type=float, help="c0 in D/m = c0 V² + ci n²/V² (default .003)")
    parser.add_argument("--induced-drag", type=float, help="ci in D/m = c0 V² + ci n²/V² (default 20)")
    parser.add_argument("--altitude", type=float, default=30, help="Initial altitude of drone 0 (m)")
    parser.add_argument("--altitude-spacing", type=float, default=25, help="Altitude step between drones (m)")
    parser.add_argument("--drone-altitude", type=float, action="append", help="Initial altitude; repeat once per drone")
    cruise_group = parser.add_mutually_exclusive_group()
    cruise_group.add_argument("--cruise-altitude", type=float, action="append", help="Manual cruise altitude; repeat once per drone")
    cruise_group.add_argument("--cruise-range", type=float, nargs=2, metavar=("MIN_M", "MAX_M"),
                              help="Shared automatic cruise range (default 30 120 m in 3D)")
    cruise_group.add_argument("--manual-cruise", action="store_true", help="Disable automatic cruise selection")
    parser.add_argument("--target-altitude", type=float, help="Common XYZ destination altitude (default 30 m in 3D)")
    parser.add_argument("--arrival-mode", choices=("common", "layers"), default="common",
                        help="Common XYZ destination (default); layers is an explicit XY-only comparison mode")
    parser.add_argument("--max-climb-rate", type=float, help="Maximum upward vertical speed (m/s; default 3)")
    parser.add_argument("--max-descent-rate", type=float, help="Maximum downward vertical speed (m/s; default 10)")
    parser.add_argument("--max-vertical-acceleration", type=float, help="Maximum vertical acceleration magnitude (m/s²; default 1)")
    parser.add_argument("--max-altitude", type=float, default=200, help="Altitude ceiling (m)")
    parser.add_argument("--descent-angle", type=float, default=None, metavar="DEG",
                        help="Common downward flight-path angle at XYZ passage, followed by recovery (default 30° in common XYZ mode)")
    parser.add_argument("--airspeed", type=float, default=12, help="Requested TOTAL airspeed for coupled 3D flight (m/s); horizontal speed in legacy mode")
    parser.add_argument("--drone-speed", type=float, action="append", metavar="M_S",
                        help="Individual airspeed; repeat once per drone, in ID order (overrides airspeed)")
    parser.add_argument("--loiter-radius", type=float, default=25, help="Radius of each holding circle (m)")
    parser.add_argument("--loiter-spacing", type=float, default=80, help="Spacing between holding circle centres (m)")
    parser.add_argument("--target", type=float, nargs=2, default=(140, 50), metavar=("X", "Y"))
    parser.add_argument("--drone-position", type=float, nargs=2, action="append", metavar=("X", "Y"),
                        help="Initial aircraft position in metres; repeat once per drone, in ID order")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--horizon", type=float, default=90,
                        help="Total simulated mission duration, including loiter and post-arrival recovery (s)")
    parser.add_argument("--arrival-deadline", type=float,
                        help="Optional absolute latest arrival time (s from snapshot), no later than horizon")
    parser.add_argument("--post-arrival-time", type=float, default=5,
                        help="Minimum simulated time after passage (s, default 5); full recovery must also fit")
    parser.add_argument("--approach-spread", type=float, default=0, metavar="DEG",
                        help="Arrival-side spread around the destination: 0 keeps original routes; 360 distributes all around")
    parser.add_argument("--approach-assignment", choices=("geometric", "id"), default="geometric",
                        help="Assign entry sides by geometry (default); id reproduces the historical allocation")
    parser.add_argument("--candidates", type=int, default=28)
    parser.add_argument("--attempts", type=int, default=90, help="Cumulative proposal limit per drone across all rounds")
    parser.add_argument("--planning-strategy", choices=("adaptive", "fixed"), default="adaptive",
                        help="Start small and expand conflicting catalogs (default), or build full catalogs")
    parser.add_argument("--conflict-guidance", choices=("none", "witness"), default="witness",
                        help="Bias new proposals using sampled pair conflicts (default); none keeps unguided expansion")
    parser.add_argument("--candidate-guidance", choices=("none", "rejections"), default="none",
                        help="Use own rejection history to propose longer level settling or wider obstacle routes; validation unchanged")
    parser.add_argument("--generator-mode", choices=("geometric", "physical_mission", "physical_recovery_seed"),
                        default="geometric", help="Historical geometry or experimental directly integrated physical mission; "
                        "physical_recovery_seed is a compatibility alias for physical_mission")
    parser.add_argument('--numerical-backend', choices=('python', 'cpp'), default='python',
                        help='physical planner backend; cpp includes proposals, construction, independent validation and selection; Python remains the reference')
    parser.add_argument("--initial-candidates", type=int, default=4,
                        help="Initial catalog size and expansion batch in adaptive mode")
    parser.add_argument("--search-order", choices=("mrv", "id"), default="mrv",
                        help="Search the smallest remaining domain first (default), or historical ID order")
    parser.add_argument("--selection-policy", choices=("max_margin", "first"), default="max_margin",
                        help="Prefer combinations with greater robust separation margin (default), or first feasible")
    parser.add_argument("--margin-search-nodes", type=int, default=2000,
                        help="Extra refinement-node cap, also limited by max-search-nodes; retain best feasible plan")
    parser.add_argument("--margin-expansion-rounds", type=int, default=1,
                        help="Maximum targeted catalog extensions after first success; 0 optimizes existing catalogs only")
    parser.add_argument("--workers", type=int, default=0,
                        help="Independent catalogue processes: 0 = auto, 1 = serial (default auto)")
    parser.add_argument("--rollout-check-interval", type=int, default=16,
                        help="Batch size for identical rollout endpoint checks; 1 restores reference checking")
    parser.add_argument("--max-search-nodes", type=int, default=30000,
                        help="Maximum cumulative search branch expansions across all rounds")
    parser.add_argument("--activation-time", type=float, default=3)
    parser.add_argument("--prepare-in-flight", action="store_true",
                        help="Gate activation on simulated in-flight planning/upload/acknowledgements; use a sufficient activation-time")
    parser.add_argument("--upload-delay", type=float, default=.25, help="Seconds per sequential plan upload")
    parser.add_argument("--ack-delay", type=float, default=.1, help="Acknowledgement delay in seconds")
    parser.add_argument("--activation-guard", type=float, default=.5, help="Readiness guard before earliest clock-shifted activation")
    parser.add_argument("--planning-latency", type=float, help="Explicit simulated planning duration; default uses measured wall time")
    parser.add_argument("--missing-ack", type=int, action="append", default=[], help="Simulate missing acknowledgement by drone ID; repeatable")
    parser.add_argument("--reservation-policy", choices=("none", "grid"), default="none",
                        help="Optional conservative full-horizon space/time cell allocation")
    parser.add_argument("--reservation-cell-size", type=float, default=20, help="Reservation grid cell edge in meters (at least 5)")
    parser.add_argument("--reservation-max-entries", type=int, default=1000000,
                        help="Maximum cell/time construction entries per candidate")
    parser.add_argument("--interval-guidance", choices=("none", "safe_windows"), default="none",
                        help="Guide expanded proposals into forecast free departure intervals; requires grid reservations")
    parser.add_argument("--interval-max-constraints", type=int, default=50000,
                        help="Maximum interval constraints per expansion hint; exhaustion falls back to exploration")
    parser.add_argument("--passage-guidance", choices=("none", "safe_windows"), default="none",
                        help="Fit local speed bumps to intermediate cell windows before rollout; requires grid and smooth speed permission")
    parser.add_argument("--profile-policy", choices=("strict", "adaptive_mixed"), default="strict",
                        help="Strict requested profiles (default), or explicitly allow simple-to-rich adaptive families; levels become a maximum")
    parser.add_argument("--cruise-levels", type=int, default=1,
                        help="Distinct cruise levels, 1 to 3 (maximum with adaptive_mixed); multiple levels require automatic cruise in coupled 3D")
    parser.add_argument("--cruise-level-hold", type=float, default=3,
                        help="Proposed hold duration at cruise levels, at least 2 s")
    parser.add_argument("--speed-profile", choices=("constant", "smooth"), default="constant",
                        help="Constant setpoint or smooth pre-approach speed variation in coupled 3D")
    parser.add_argument("--speed-profile-range", type=float, nargs=2, metavar=("MIN_M_S", "MAX_M_S"),
                        help="Speed-reference range for smooth profiles (default 10 14); not new physical limits")
    parser.add_argument("--speed-profile-acceleration", type=float, default=.3,
                        help="Maximum speed-reference acceleration in m/s²; physical controls remain independently checked")
    parser.add_argument("--departure-delay", type=float, nargs=2, default=(0, 22), metavar=("MIN_S", "MAX_S"),
                        help="Extra loiter delay interval after activation (s); use 0 0 for simultaneous departures")
    parser.add_argument("--separation", type=float, default=8)
    parser.add_argument("--obstacle-cylinder", type=float, nargs=5, action="append",
                        metavar=("X", "Y", "RADIUS", "ZMIN", "ZMAX"), help="Static vertical cylinder; repeat for multiple obstacles")
    parser.add_argument("--obstacle-box", type=float, nargs=6, action="append",
                        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"), help="Static axis-aligned box; repeat as needed")
    parser.add_argument("--obstacles", type=Path, help="JSON list of known static obstacles")
    parser.add_argument("--obstacle-clearance", type=float, default=5, help="Minimum drone-center distance to obstacle surface (m)")
    parser.add_argument("--obstacle-routing", choices=("spatial", "lateral"), default="spatial",
                        help="Explore lateral/overflight/underflight cruise proposals (default), or historical XY routing")
    parser.add_argument("--clock-bound", type=float, default=0.15)
    parser.add_argument("--position-bound", type=float, default=2)
    parser.add_argument("--clock-offset", type=float, default=0, help="Offsets spread from -value to +value (s)")
    parser.add_argument("--clock-drift", type=float, default=0, help="Clock drift spread from -value to +value (s/s)")
    parser.add_argument("--wind", type=float, default=0, help="Synthetic wind norm in m/s")
    parser.add_argument("--wind-model", choices=("rotating", "constant", "gust"), default="rotating",
                        help="Bounded horizontal velocity disturbance; not aerodynamic gust forces")
    parser.add_argument("--wind-heading", type=float, default=0, help="Wind direction or profile offset in degrees")
    parser.add_argument("--position-bias", type=float, default=0, help="Constant position-measurement bias norm per drone (m)")
    parser.add_argument("--position-noise", type=float, default=0, help="Bounded position-measurement noise radius (m)")
    parser.add_argument("--disturbance-seed", type=int, default=0, help="Reproducible independent per-drone sensor-noise seed")
    parser.add_argument("--tracking-controller", choices=("constrained", "legacy"), default="constrained",
                        help="Own-state wind estimation and held-command protection (default), or historical feedback")
    parser.add_argument('--execution-backend', choices=('python', 'cpp'), default='python',
                        help='local controller and simulated motion kernel; cpp requires coupled 3D constrained tracking; observer remains Python')
    parser.add_argument("--playback-speed", type=float, default=4)
    parser.add_argument("--output", type=Path, help="Empty output directory; defaults to a new timestamped run")
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--pick-target", action="store_true", help="Select the target with one click on a frozen XY map")
    parser.add_argument("--edit-scenario", action=argparse.BooleanOptionalAction, default=None,
                        help="Place drones and target interactively (default with gui, except pick-target)")
    args = parser.parse_args()
    if args.aircraft_profile is not None:
        if len(args.aircraft_profile) != 1:
            parser.error('only one common aircraft profile is allowed; use drone-profile for explicit per-ID assignments')
        args.aircraft_profile = args.aircraft_profile[0]
    aircraft_profile = None
    aircraft_profiles = ()
    supplied_physical = [name for name in PHYSICAL_DEFAULTS if getattr(args, name) is not None]
    if args.aircraft_profile is not None or args.drone_profile is not None:
        if args.dimensions != 3 or args.flight_model != 'point_mass':
            parser.error('aircraft-profile requires the coupled 3D point-mass backend')
        if supplied_physical:
            parser.error('aircraft-profile cannot be combined with physical overrides: ' +
                         ', '.join('--' + name.replace('_', '-') for name in supplied_physical))
        try:
            if args.drone_profile is not None:
                if args.aircraft_profile is not None:
                    parser.error('choose aircraft-profile or drone-profile, not both')
                aircraft_profiles = tuple((int(i), load_aircraft_profile(path)) for i, path in args.drone_profile)
                ids = [i for i, _ in aircraft_profiles]
                if len(ids) != len(set(ids)) or set(ids) != set(range(args.num_drones)):
                    parser.error('provide exactly one drone-profile per ID from 0 to num-drones minus one')
                aircraft_profiles = tuple(sorted(aircraft_profiles))
            else:
                aircraft_profile = load_aircraft_profile(args.aircraft_profile)
        except (ValueError, OSError) as error:
            parser.error(str(error))
    for name, default in PHYSICAL_DEFAULTS.items():
        if getattr(args, name) is None:
            setattr(args, name, default)
    if aircraft_profile is not None:
        args.max_climb_rate = aircraft_profile.performance.max_climb_rate
        args.max_descent_rate = aircraft_profile.performance.max_descent_rate
        args.max_vertical_acceleration = aircraft_profile.performance.max_vertical_acceleration
    if args.output is None:
        args.output = Path(f"results/trajectory{args.dimensions}d") / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("output must be empty: use a new directory to avoid mixing mission artifacts")
    if not np.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error("playback-speed must be positive")
    perturbation_bounds = [args.wind, args.position_bias, args.position_noise, args.clock_offset, args.clock_drift]
    if (not np.all(np.isfinite(perturbation_bounds)) or min(perturbation_bounds) < 0
            or not np.isfinite(args.wind_heading) or args.disturbance_seed < 0):
        parser.error("disturbance magnitudes and seed must be finite/nonnegative; heading must be finite")
    if args.pick_target and not args.gui:
        parser.error("pick-target requires gui")
    if args.edit_scenario and not args.gui:
        parser.error("edit-scenario requires gui")
    if args.edit_scenario and args.pick_target:
        parser.error("choose edit-scenario or pick-target")
    if args.target_altitude is not None and (args.dimensions != 3 or args.arrival_mode != "common"):
        parser.error("target-altitude requires dimensions 3 and arrival-mode common")
    if args.cruise_range is not None and args.dimensions != 3:
        parser.error("cruise-range requires dimensions 3")
    try:
        limits = Limits(max_bank_deg=args.max_bank, max_roll_rate_deg_s=args.max_roll_rate)
        flight = FlightLimits(stall_speed=args.stall_speed, stall_margin=args.stall_margin,
                              max_load_factor=args.max_load_factor, max_flight_path_deg=args.max_flight_path_angle,
                              max_flight_path_rate_deg_s=args.max_flight_path_rate,
                              max_thrust_acceleration=args.max_thrust_acceleration,
                              max_brake_acceleration=args.max_airbrake_acceleration,
                              parasitic_drag=args.parasitic_drag, induced_drag=args.induced_drag)
        if aircraft_profile is not None:
            limits, flight = aircraft_profile.limits, aircraft_profile.flight
        snapshots = separate_loiter_snapshot(args.num_drones, radius=args.loiter_radius,
                                             spacing=args.loiter_spacing)
        speeds = args.drone_speed if args.drone_speed is not None else [args.airspeed] * len(snapshots)
        if len(speeds) != len(snapshots):
            parser.error("provide one --drone-speed per drone")
        profiles_by_id = dict(aircraft_profiles)
        snapshots = tuple(s.with_speed(speed, profiles_by_id[s.drone_id].limits if aircraft_profiles else limits)
                          for s, speed in zip(snapshots, speeds))
    except ValueError as error:
        parser.error(str(error))
    if args.drone_position is not None:
        if len(args.drone_position) != len(snapshots) or not np.all(np.isfinite(args.drone_position)):
            parser.error("provide one finite --drone-position X Y per drone")
        snapshots = tuple(replace(s, state=(*position, *s.state[2:]))
                          for s, position in zip(snapshots, args.drone_position))
    try:
        obstacles = load_obstacles(args.obstacles) if args.obstacles else ()
        obstacles += tuple(Obstacle.cylinder(*values) for values in (args.obstacle_cylinder or []))
        obstacles += tuple(Obstacle.box(*values) for values in (args.obstacle_box or []))
        config = Config(seed=args.seed, target=tuple(args.target), horizon=args.horizon,
                        arrival_deadline=args.arrival_deadline, post_arrival_time=args.post_arrival_time,
                        obstacles=obstacles, obstacle_clearance=args.obstacle_clearance,
                        obstacle_routing=args.obstacle_routing,
                        flight_model=args.flight_model, flight=flight, limits=limits,
                        aircraft_profiles=aircraft_profiles,
                        candidates=args.candidates, attempts=args.attempts,
                        planning_strategy=args.planning_strategy, initial_candidates=args.initial_candidates,
                        conflict_guidance=args.conflict_guidance,
                        candidate_guidance=args.candidate_guidance,
                        generator_mode=args.generator_mode,
                        numerical_backend=args.numerical_backend,
                        reservation_policy=args.reservation_policy, reservation_cell_size=args.reservation_cell_size,
                        reservation_max_entries=args.reservation_max_entries,
                        interval_guidance=args.interval_guidance, interval_max_constraints=args.interval_max_constraints,
                        passage_guidance=args.passage_guidance,
                        profile_policy=args.profile_policy,
                        cruise_levels=args.cruise_levels, cruise_level_hold=args.cruise_level_hold,
                        speed_profile=args.speed_profile, speed_profile_range=(
                            tuple(args.speed_profile_range) if args.speed_profile_range is not None else
                            (10., 14.) if args.speed_profile == "smooth" else None),
                        speed_profile_acceleration=args.speed_profile_acceleration,
                        search_order=args.search_order,
                        selection_policy=args.selection_policy, margin_search_nodes=args.margin_search_nodes,
                        margin_expansion_rounds=args.margin_expansion_rounds,
                        max_search_nodes=args.max_search_nodes, activation_time=args.activation_time,
                        planning_workers=args.workers, rollout_check_interval=args.rollout_check_interval,
                        separation=args.separation, clock_error=args.clock_bound,
                        position_error=args.position_bound, approach_spread_deg=args.approach_spread,
                        approach_assignment=args.approach_assignment,
                        min_extra_loiter=args.departure_delay[0], max_extra_loiter=args.departure_delay[1],
                        dimensions=args.dimensions, target_altitude=(
                            (30 if args.target_altitude is None else args.target_altitude)
                            if args.dimensions == 3 and args.arrival_mode == "common" else None),
                        max_climb_rate=args.max_climb_rate, max_vertical_acceleration=args.max_vertical_acceleration,
                        max_altitude=args.max_altitude, max_descent_rate=args.max_descent_rate,
                        descent_angle_deg=(30 if args.descent_angle is None and args.dimensions == 3 and args.arrival_mode == "common"
                                           else 0 if args.descent_angle is None else args.descent_angle),
                        cruise_altitude_range=(tuple(args.cruise_range or (30, 120))
                            if args.dimensions == 3 and not args.manual_cruise and args.cruise_altitude is None else None))
    except (ValueError, OSError) as error:
        parser.error(str(error))
    if aircraft_profile is not None:
        config = aircraft_profile.apply(config)
        aircraft_profile.require_matches(config)
        print(f"Profile: {aircraft_profile.name}; declared status: {aircraft_profile.calibration_status}; "
              f"shared envelope for {len(snapshots)} aircraft; no certification.", flush=True)
    for identifier, profile in aircraft_profiles:
        print(f'Drone {identifier}: {profile.name}; declared status {profile.calibration_status}, no certification.', flush=True)
    if args.dimensions == 3:
        altitudes = (args.drone_altitude if args.drone_altitude is not None else
                     [args.altitude + k * args.altitude_spacing for k in range(len(snapshots))])
        cruise = (args.cruise_altitude if args.cruise_altitude is not None else
                  [None] * len(snapshots) if config.cruise_altitude_range is not None else altitudes)
        if len(altitudes) != len(snapshots) or len(cruise) != len(snapshots):
            parser.error("provide one initial/cruise altitude per drone")
        if not all(np.isfinite(z) and config.min_altitude <= z <= config.max_altitude
                   for z in [*altitudes, *[c for c in cruise if c is not None]]):
            parser.error("initial/cruise altitudes must be finite and within altitude limits")
        snapshots = tuple(replace(s, altitude=float(z), cruise_altitude=None if c is None else float(c))
                          for s, z, c in zip(snapshots, altitudes, cruise))
    try:
        check_execution_backend(config, args.tracking_controller, args.execution_backend)
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))
    if args.gui and args.edit_scenario is not False and not args.pick_target:
        selected = edit_scenario(snapshots, config.target, config)
        if selected is None:
            print("Preparation cancelled; no mission launched.")
            return 0
        snapshots, config = selected
        if config.aircraft_profiles:
            aircraft_profile = None  # GUI may replace the initial common profile with explicit per-ID assignments.
    elif args.pick_target:
        try:
            config = replace(config, target=pick_target(snapshots, config.target))
        except ValueError as error:
            print(f"Preparation cancelled: {error}")
            return 0
    worker_limit = worker_count(config.planning_workers, len(snapshots))
    print(f"Planning from the t=0 snapshot… (up to {worker_limit} processes)", flush=True)
    planning_start = perf_counter()
    from swarm_sim.trajectory2d.preparation import Preparation, prepare_mission
    try:
        preparation = Preparation(args.upload_delay, args.ack_delay, args.activation_guard,
                                  args.planning_latency, tuple(args.missing_ack))
        result = prepare_mission(snapshots, config, preparation) if args.prepare_in_flight else plan_mission(snapshots, config)
    except ValueError as error:
        parser.error(str(error))
    planning_elapsed = perf_counter() - planning_start
    print(f"Generation time: {planning_elapsed:.2f} s", flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    document = mission_document(result, config, snapshots, aircraft_profile=aircraft_profile)
    (args.output / "mission.json").write_text(json.dumps(document, indent=2, allow_nan=False))
    planning_report = {"status": result.status, "reason": result.reason,
                       "diagnostics": result.diagnostics, "timings": result.timings}
    if aircraft_profile is not None:
        planning_report['aircraft_profile'] = aircraft_profile.record()
    if config.aircraft_profiles:
        planning_report['aircraft_profiles'] = document['aircraft_profiles']
    (args.output / "planning_report.json").write_text(json.dumps(planning_report, indent=2, allow_nan=False))
    print(f"Status: {result.status} — {result.reason}")
    if result.diagnostics:
        print(f"Time by stage: catalogs {result.timings.get('catalog_wall_s', 0):.2f} s; "
              f"search {result.timings.get('search_wall_s', 0):.3f} s", flush=True)
        print(f"Search: {len(result.diagnostics.get('rounds', []))} round(s), {result.search_nodes} nodes, "
              f"catalogs {result.counts}", flush=True)
        for catalog in result.diagnostics.get("catalogs", []):
            print(f"Drone {catalog['drone_id']}: {catalog['accepted']}/{catalog['attempted']} accepted proposals; "
                  f"rejections {catalog['rejections']}", flush=True)
            if catalog.get("rejection_details"):
                print(f"  Details: {catalog['rejection_details']}", flush=True)
            if catalog.get("guided_proposals"):
                guided = catalog["guided_proposals"]
                accepted = sum(p["accepted"] for p in guided)
                print(f"  Conflict-guided proposals: {accepted}/{len(guided)} "
                      "individually validated", flush=True)
            if catalog.get("interval_proposals"):
                timed = catalog["interval_proposals"]
                print(f"  Free-window-guided proposals: {sum(p['accepted'] for p in timed)}/{len(timed)} "
                      "individually validated", flush=True)
            if catalog.get("profile_proposals"):
                profiles = catalog["profile_proposals"]
                families = sorted({p["family"] for p in profiles})
                print("  Profile families (accepted/proposed): " + ", ".join(
                    f"{family} {sum(p['accepted'] for p in profiles if p['family'] == family)}/"
                    f"{sum(p['family'] == family for p in profiles)}" for family in families), flush=True)
            if catalog.get("passage_proposals"):
                passages = catalog["passage_proposals"]
                print(f"  Intermediate passages: {sum(p['accepted'] for p in passages)}/{len(passages)} "
                      "individually validated proposals with verified windows", flush=True)
            if catalog.get("candidate_proposals"):
                adjusted = catalog["candidate_proposals"]
                print(f"  Rejection-guided proposals: {sum(p['accepted'] for p in adjusted)}/{len(adjusted)} "
                      "individually validated", flush=True)
        selection = result.diagnostics.get("selection", {})
        if selection.get("initial_sorted_slack_m"):
            print(f"Minimum robust margin: {selection['initial_sorted_slack_m'][0]:.2f} → "
                  f"{selection['selected_sorted_slack_m'][0]:.2f} m; "
                  f"{selection['optimization_nodes']} optimization nodes", flush=True)
        timing = result.diagnostics.get("timing", {})
        for row in timing.get("geometric_template", []):
            if row["conservative_template_limitation"]:
                print(f"Drone {row['drone_id']}: the geometric reference requires "
                      f"{row['geometric_recovery_drop_m']:.1f} m for pull-out, versus "
                      f"{row['available_height_m']:.1f} m available. Generator limitation, "
                      "not proof of infeasibility in the physical model.", flush=True)
        for flight in timing.get("schedule", []):
            recovery = flight["recovery_completed_s"]
            recovery_text = "—" if recovery is None else f"{recovery:.1f} s"
            print(f"Drone {flight['drone_id']}: departure {flight['departure_s']:.1f} s; "
                  f"arrival {flight['arrival_s']:.1f} s; cruise recovery {recovery_text}; "
                  f"route {flight['routing_mode']}", flush=True)
        suggested = timing.get("suggested_horizon_s")
        if suggested is not None:
            print(f"Suggestion: try --horizon {suggested:g} (success is not guaranteed; "
                  "no parameter is changed automatically).", flush=True)
        if not result.validated and config.arrival_deadline is not None:
            print(f"The arrival deadline of {config.arrival_deadline:g} s remains enforced even if the horizon is increased.", flush=True)
    print(f"Planning diagnostics: {args.output / 'planning_report.json'}", flush=True)
    if args.prepare_in_flight:
        gate = result.timings["preparation_timeline"]
        print(f"In-flight preparation: activation at {config.activation_time:g} s; "
              f"all plans ready: {gate['all_ready']}", flush=True)
    if not result.validated:
        return 2
    if config.dimensions == 3:
        print("Selected cruise altitudes: " + ", ".join(
            f"drone {p.drone_id}: {p.cruise_altitude:.1f} m" for p in result.plans), flush=True)
        for plan in result.plans:
            if plan.profile_family is not None:
                print(f"Drone {plan.drone_id}: family {plan.profile_family}", flush=True)
            if plan.cruise_levels_m:
                print(f"Drone {plan.drone_id}: levels " + ", ".join(f"{z:.1f} m" for z in plan.cruise_levels_m), flush=True)
    # Individual payloads deliberately contain no other aircraft's plan/state.
    for plan in result.plans:
        payload = {"schema": document["schema"], "mission_sha256": document["sha256"],
                   "config": asdict(config.for_drone(plan.drone_id)), "plan": plan.to_dict()}
        if aircraft_profile is not None:
            payload['aircraft_profile'] = document['aircraft_profile']
        elif config.aircraft_profiles:
            payload['aircraft_profile'] = config.profile_for(plan.drone_id).record()
        if config.reservation_policy == "grid":
            from swarm_sim.trajectory2d.reservations import occupancy
            payload["own_reservations"] = occupancy(plan, config).to_dict()
        (args.output / f"drone_{plan.drone_id}.json").write_text(json.dumps(payload, indent=2))
    evaluation = simulate(result.plans, config,
                          clock_offsets=np.linspace(-args.clock_offset, args.clock_offset, len(result.plans)),
                          clock_drifts=np.linspace(-args.clock_drift, args.clock_drift, len(result.plans)),
                          wind=args.wind, wind_model=args.wind_model, wind_heading_deg=args.wind_heading,
                          controller=args.tracking_controller, position_noise=args.position_noise,
                          position_biases=position_bias_vectors([p.drone_id for p in result.plans], args.position_bias, config.dimensions),
                          disturbance_seed=args.disturbance_seed, execution_backend=args.execution_backend)
    evaluation.report.update({"planning_wall_time_s": planning_elapsed,
                              "planning_diagnostics": result.diagnostics,
                              "planning_timings": result.timings,
                              "planning_workers_requested": config.planning_workers,
                              "planning_worker_limit": worker_limit,
                              "rollout_check_interval": config.rollout_check_interval})
    if aircraft_profile is not None:
        evaluation.report['aircraft_profile'] = document['aircraft_profile']
    (args.output / "report.json").write_text(json.dumps(evaluation.report, indent=2, allow_nan=False))
    with (args.output / "telemetry.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        columns = ["time_s", "drone_id", "x_m", "y_m", "heading_rad", "yaw_rate_rad_s"]
        if config.dimensions == 3:
            columns += ["z_m", "vertical_speed_m_s"]
        flight_columns = ["airspeed_m_s", "flight_path_rad", "bank_rad", "ground_speed_m_s"] if config.coupled_flight else []
        writer.writerow(columns + flight_columns + ["tracking_error_m"])
        for n, t in enumerate(evaluation.times):
            for k, plan in enumerate(result.plans):
                extra = []
                if config.coupled_flight:
                    from swarm_sim.trajectory2d.flight3d import air_velocity
                    full = evaluation.flight_actual[n, k]
                    held_time = np.floor((t - 1e-10) / config.dt) * config.dt if t > 0 else 0
                    wind_vector = wind_at(held_time, plan.drone_id, args.wind, args.wind_model, args.wind_heading)
                    extra = [full[3], full[5], full[6], np.linalg.norm(air_velocity(full) + wind_vector)]
                writer.writerow([t, plan.drone_id, *evaluation.actual[n, k], *extra, evaluation.tracking_errors[n, k]])
    # Detailed conflict graphs/timings are exported above; avoid duplicating
    # hundreds of diagnostic lines in the terminal's execution summary.
    print(json.dumps({key: value for key, value in evaluation.report.items()
                      if key not in ("planning_diagnostics", "planning_timings",
                                     "minimum_flight_envelope_margins_per_drone", "safety_events")}, indent=2))
    visualize(result, config, evaluation, args.output / "overview.png", args.gui, args.playback_speed)
    print(f"Mission, individual plans, report, telemetry and plot: {args.output}")
    return 0 if evaluation.report["execution_validated"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
