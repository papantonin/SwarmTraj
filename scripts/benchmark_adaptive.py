"""Compare fixed and adaptive planning on four-aircraft 3D scenarios.

Planning is timed without profiling. Optional execution checks use the same
wind/clock settings for every mode. Different valid plans are expected when
catalog strategy or search order changes; only fixed mode must match a supplied
historical reference. No scenario uses more than four aircraft.
"""

import argparse
from dataclasses import asdict, replace
from datetime import datetime
import json
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from benchmark_planning import signature
from swarm_sim.trajectory2d import Config, plan_mission, separate_loiter_snapshot
from swarm_sim.trajectory2d.execution import simulate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--modes", nargs="+", choices=("fixed", "adaptive_id", "adaptive_mrv"),
                        default=["fixed", "adaptive_id", "adaptive_mrv"])
    parser.add_argument("--scenarios", nargs="+", choices=("default_90s", "spread180_150s", "spread360_240s"),
                        default=["default_90s", "spread180_150s", "spread360_240s"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[7])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--initial-candidates", type=int, default=4)
    parser.add_argument("--simulate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--wind", type=float, default=0)
    parser.add_argument("--clock-offset", type=float, default=0)
    parser.add_argument("--reference", type=Path, help="Optional historical benchmark JSON, seed 7 fixed-mode signatures")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.repeats < 1 or any(seed < 0 for seed in args.seeds):
        parser.error("repeats must be positive and seeds nonnegative")
    if not np.all(np.isfinite([args.wind, args.clock_offset])) or min(args.wind, args.clock_offset) < 0:
        parser.error("wind and clock-offset must be finite and nonnegative")
    output = args.output or Path("results/benchmarks") / ("adaptive-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
    if output.exists():
        parser.error("output must be a new file")
    expected = {}
    if args.reference:
        previous = json.loads(args.reference.read_text())
        expected = {r["scenario"]: r["signature_sha256"] for r in previous["runs"]}
    base = Config(dimensions=3, target_altitude=30, max_descent_rate=10, descent_angle_deg=30,
                  cruise_altitude_range=(30, 120), planning_workers=args.workers,
                  initial_candidates=args.initial_candidates)
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot(4)))
    scenarios = {"default_90s": base,
                 "spread180_150s": replace(base, approach_spread_deg=180, horizon=150),
                 "spread360_240s": replace(base, approach_spread_deg=360, horizon=240)}
    document = {"python": platform.python_version(), "numpy": np.__version__, "num_drones": 4,
                "base_config": asdict(base), "workers_requested": args.workers,
                "execution_settings": {"enabled": args.simulate, "wind_m_s": args.wind,
                                       "clock_offset_s": args.clock_offset}, "runs": []}
    successful = True
    for scenario in args.scenarios:
        for seed in args.seeds:
            for repeat in range(args.repeats):
                modes = args.modes if repeat % 2 == 0 else args.modes[::-1]
                for mode in modes:
                    config = replace(scenarios[scenario], seed=seed,
                                     planning_strategy="fixed" if mode == "fixed" else "adaptive",
                                     search_order="mrv" if mode == "adaptive_mrv" else "id")
                    print(f"START {scenario} seed={seed} {mode} repeat={repeat + 1}", flush=True)
                    start = perf_counter()
                    result = plan_mission(snapshots, config)
                    elapsed = perf_counter() - start
                    digest = signature(result)
                    record = {"scenario": scenario, "seed": seed, "mode": mode, "repeat": repeat + 1,
                              "planning_s": elapsed, "status": result.status, "counts": result.counts,
                              "search_nodes": result.search_nodes, "signature_sha256": digest,
                              "diagnostics": result.diagnostics, "timings": result.timings}
                    if mode == "fixed" and seed == 7 and scenario in expected:
                        record["matches_historical_reference"] = digest == expected[scenario]
                        successful &= record["matches_historical_reference"]
                    if result.validated and args.simulate:
                        evaluation = simulate(result.plans, config, wind=args.wind,
                                              clock_offsets=np.linspace(-args.clock_offset, args.clock_offset, 4))
                        record["execution"] = evaluation.report
                        record["execution_failures"] = evaluation.report["execution_failures"]
                        record["execution_validated"] = not record["execution_failures"]
                        successful &= record["execution_validated"]
                    successful &= result.validated
                    document["runs"].append(record)
                    print(json.dumps({key: record[key] for key in
                                      ("scenario", "seed", "mode", "planning_s", "status", "counts", "search_nodes")}
                                     | {"rounds": len(result.diagnostics.get("rounds", [])),
                                        "execution_validated": record.get("execution_validated"),
                                        "execution_failures": record.get("execution_failures", [])}), flush=True)
    document["all_requested_checks_passed"] = bool(successful)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f"SAVED {output}", flush=True)
    return 0 if successful else 2


if __name__ == "__main__":
    raise SystemExit(main())
