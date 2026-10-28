"""Compare planning alone on the same three four-aircraft scenarios.

reference = original serial/stepwise checks; serial = batched checks;
parallel2/parallel4 = batched checks plus 2/4 independent processes.
Pool creation/shutdown is included; simulation, GUI and fingerprinting are not.
Run this file, not an interactive stdin cell: multiprocessing uses spawn.
"""

import argparse
from dataclasses import asdict, replace
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
from time import perf_counter

import numpy as np

from swarm_sim.trajectory2d import Config, separate_loiter_snapshot, plan_mission


def signature(result):
    """Exact output equality, independent of the performance options in Config."""
    output = {"status": result.status, "counts": result.counts, "search_nodes": result.search_nodes,
              "minimum_distance_lower_bound": result.minimum_distance_lower_bound,
              "required_distance": result.required_distance, "plans": [p.to_dict() for p in result.plans]}
    return hashlib.sha256(json.dumps(output, sort_keys=True, allow_nan=False).encode()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variants", nargs="+", choices=("reference", "serial", "parallel2", "parallel4"),
                        default=["reference", "serial", "parallel2", "parallel4"])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--reference", type=Path, help="Previously recorded JSON reference for exact-output comparison")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    if not args.reference and "reference" not in args.variants:
        parser.error("include the reference variant or provide --reference to verify unchanged outputs")
    output = args.output or Path("results/benchmarks") / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
    if output.exists():
        parser.error("output must be a new file")
    expected = {}
    if args.reference:
        previous = json.loads(args.reference.read_text())
        expected = {r["scenario"]: r["signature_sha256"] for r in previous["runs"] if r["variant"] == "reference"}
    base = Config(dimensions=3, target_altitude=30, max_descent_rate=10,
                  descent_angle_deg=30, cruise_altitude_range=(30, 120))
    snapshots = tuple(replace(s, altitude=30 + 25 * k) for k, s in enumerate(separate_loiter_snapshot()))
    scenarios = [("default_90s", base),
                 ("spread180_150s", replace(base, approach_spread_deg=180, horizon=150)),
                 ("spread360_240s", replace(base, approach_spread_deg=360, horizon=240))]
    modes = {"reference": (1, 1), "serial": (1, 16), "parallel2": (2, 16), "parallel4": (4, 16)}
    document = {"python": platform.python_version(), "numpy": np.__version__, "machine": platform.machine(),
                "logical_cpus": os.cpu_count(), "base_config": asdict(base),
                "initial_snapshots": [asdict(s) for s in snapshots],
                "scope": "plan_mission only; includes fresh spawn pool startup/shutdown; excludes GUI, execution and hashing",
                "repetitions": args.repeats, "runs": []}
    for name, scenario in scenarios:
        for repeat in range(args.repeats):
            # Alternate ordering between repeats to reduce systematic order bias.
            variants = args.variants if repeat % 2 == 0 else args.variants[::-1]
            for variant in variants:
                workers, interval = modes[variant]
                if "planning_workers" in Config.__dataclass_fields__:
                    config = replace(scenario, planning_workers=workers, rollout_check_interval=interval)
                elif variant == "reference":
                    config = scenario  # Capture the unmodified source before optimisation.
                else:
                    parser.error("this source revision supports only reference")
                print(f"START {name} {variant} repeat={repeat + 1}", flush=True)
                start = perf_counter()
                result = plan_mission(snapshots, config)
                elapsed = perf_counter() - start
                digest = signature(result)
                if variant == "reference":
                    expected.setdefault(name, digest)
                matches = digest == expected[name] if name in expected else None
                record = {"scenario": name, "variant": variant, "repeat": repeat + 1,
                          "seconds": elapsed, "status": result.status, "counts": result.counts,
                          "search_nodes": result.search_nodes, "workers_requested": workers,
                          "rollout_check_interval": interval, "signature_sha256": digest,
                          "matches_reference": matches}
                document["runs"].append(record)
                print(json.dumps(record), flush=True)
    # Resolve comparisons also when reference was requested after other variants.
    for record in document["runs"]:
        record["matches_reference"] = record["signature_sha256"] == expected.get(record["scenario"])
    all_ok = all(r["status"] == "validated" and r["matches_reference"] for r in document["runs"])
    document["summary"] = []
    for name, _ in scenarios:
        for variant in args.variants:
            seconds = [r["seconds"] for r in document["runs"] if r["scenario"] == name and r["variant"] == variant]
            document["summary"].append({"scenario": name, "variant": variant,
                                         "median_seconds": statistics.median(seconds),
                                         "min_seconds": min(seconds), "max_seconds": max(seconds)})
    document["all_validated_and_matching"] = bool(all_ok)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f"SAVED {output}", flush=True)
    return 0 if all_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
