"""Compare unguided and witness-guided expansion on identical four-drone scenes."""

import argparse
from dataclasses import asdict, replace
from datetime import datetime
import json
from pathlib import Path
import platform
import os
import statistics

import numpy as np

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate
from swarm_sim.trajectory2d.obstacles import load_obstacles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", default=[7, 19])
    parser.add_argument("--scenarios", nargs="+", choices=("default", "simultaneous", "obstacles"),
                        default=["default", "simultaneous", "obstacles"])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--initial-candidates", type=int, default=1)
    parser.add_argument("--selection-policy", choices=("first", "max_margin"), default="first")
    parser.add_argument("--simulate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--obstacles", type=Path, default=Path("examples/scenarios/static_obstacles.json"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.repeats < 1 or any(seed < 0 for seed in args.seeds):
        parser.error("repeats must be positive and seeds nonnegative")
    output = args.output or Path("results/benchmarks") / (
        "conflicts-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
    if output.exists():
        parser.error("output must be a new file")
    base = Config(dimensions=3, generator_mode='geometric', numerical_backend='python', target_altitude=30, descent_angle_deg=30, max_descent_rate=10,
                  cruise_altitude_range=(40, 110), planning_strategy="adaptive", search_order="mrv",
                  planning_workers=args.workers, initial_candidates=args.initial_candidates,
                  selection_policy=args.selection_policy, obstacle_routing="spatial")
    scenarios = {"default": base,
                 "simultaneous": replace(base, target=(500, 300), horizon=150, approach_spread_deg=180,
                                         activation_time=0, max_extra_loiter=0),
                 "obstacles": replace(base, target=(500, 500), horizon=180, approach_spread_deg=180,
                                      obstacles=load_obstacles(args.obstacles))}
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    document = {"python": platform.python_version(), "numpy": np.__version__,
                "machine": platform.machine(), "processor": platform.processor(),
                "logical_cpus": os.cpu_count(), "num_drones": 4, "base_config": asdict(base),
                "snapshots": [asdict(s) for s in snapshots], "execution_enabled": args.simulate, "runs": []}
    passed = True
    for name in args.scenarios:
        for seed in args.seeds:
            for repeat in range(args.repeats):
                for mode in (("none", "witness") if repeat % 2 == 0 else ("witness", "none")):
                    config = replace(scenarios[name], seed=seed, conflict_guidance=mode)
                    print(f"START {name} seed={seed} guidance={mode} repeat={repeat + 1}", flush=True)
                    result = plan_mission(snapshots, config)
                    record = {"scenario": name, "seed": seed, "repeat": repeat + 1, "mode": mode,
                              "config": asdict(config), "status": result.status, "counts": result.counts,
                              "search_nodes": result.search_nodes, "timings": result.timings,
                              "signature_sha256": signature(result), "diagnostics": result.diagnostics,
                              "minimum_distance_lower_bound": result.minimum_distance_lower_bound}
                    if result.validated and args.simulate:
                        execution = simulate(result.plans, config)
                        record["execution"] = execution.report
                        passed &= execution.report["execution_validated"]
                    passed &= result.validated
                    document["runs"].append(record)
                    print(json.dumps({k: record[k] for k in ("scenario", "seed", "mode", "status", "counts")}
                                     | {"planning_s": result.timings["total_wall_s"],
                                        "attempts": sum(c["attempted"] for c in result.diagnostics.get("catalogs", [])),
                                        "execution_validated": record.get("execution", {}).get("execution_validated")}),
                          flush=True)
    document["all_requested_checks_passed"] = bool(passed)
    document["summary"] = []
    for name in args.scenarios:
        for seed in args.seeds:
            records = [r for r in document["runs"] if r["scenario"] == name and r["seed"] == seed]
            medians = {mode: statistics.median(r["timings"]["total_wall_s"] for r in records if r["mode"] == mode)
                       for mode in ("none", "witness")}
            document["summary"].append({"scenario": name, "seed": seed, "median_planning_s": medians,
                                        "unguided_over_guided_time": medians["none"] / medians["witness"],
                                        "all_signatures_identical": len({r["signature_sha256"] for r in records}) == 1})
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f"SAVED {output}", flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
