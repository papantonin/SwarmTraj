"""Compare first-feasible and margin-preferring selection on one saved 4-drone scene."""

import argparse
from dataclasses import replace
from datetime import datetime
import json
from itertools import combinations
from pathlib import Path
from time import perf_counter

import numpy as np

from swarm_sim.trajectory2d import Config, FlightLimits, Limits, Obstacle, Snapshot, plan_mission
from swarm_sim.trajectory2d.execution import positions, simulate


def load_scene(path):
    document = json.loads(path.read_text())
    values = document["config"].copy()
    values["target"] = tuple(values["target"])
    values["limits"] = Limits(**values["limits"])
    values["flight"] = FlightLimits(**values["flight"])
    values["obstacles"] = tuple(Obstacle(**o) for o in values.get("obstacles", []))
    if values.get("cruise_altitude_range") is not None:
        values["cruise_altitude_range"] = tuple(values["cruise_altitude_range"])
    snapshots = tuple(Snapshot(**{**s, "state": tuple(s["state"])}) for s in document["snapshots"])
    if len(snapshots) != 4:
        raise ValueError("This comparison requires exactly four aircraft")
    values.setdefault('generator_mode', 'geometric')
    values.setdefault('numerical_backend', 'python')
    if (values['generator_mode'] not in ('geometric', 'physical_mission', 'physical_recovery_seed')
            or values['numerical_backend'] not in ('python', 'cpp')):
        raise ValueError('Saved mission requires explicit planner selections.')
    return snapshots, Config(**values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--margin-search-nodes", type=int, default=2000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    snapshots, base = load_scene(args.mission)
    output = args.output or Path("results/benchmarks") / ("selection-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
    if output.exists():
        parser.error("output must be a new file")
    document = {"source_mission": str(args.mission), "num_drones": 4, "runs": []}
    successful = True
    for name, policy, expansions in (("first", "first", 0), ("existing_catalogs", "max_margin", 0),
                                     ("targeted_extension", "max_margin", 1)):
        config = replace(base, selection_policy=policy, margin_expansion_rounds=expansions,
                         margin_search_nodes=args.margin_search_nodes, planning_workers=args.workers)
        print(f"START {name}", flush=True)
        start = perf_counter()
        result = plan_mission(snapshots, config)
        elapsed = perf_counter() - start
        record = {"variant": name, "status": result.status, "planning_s": elapsed,
                  "counts": result.counts, "search_nodes": result.search_nodes,
                  "diagnostics": result.diagnostics, "timings": result.timings}
        if result.validated:
            evaluation = simulate(result.plans, config)
            report = evaluation.report
            record["execution"] = report
            record["execution_failures"] = report["execution_failures"]
            record["pairs"] = []
            for i, j in combinations(range(4), 2):
                distance = np.linalg.norm(positions(evaluation.actual[:, i]) - positions(evaluation.actual[:, j]), axis=1)
                k = int(np.argmin(distance))
                record["pairs"].append({"drone_ids": [result.plans[i].drone_id, result.plans[j].drone_id],
                                        "sampled_min_m": float(distance[k]), "time_s": float(evaluation.times[k])})
            successful &= not record["execution_failures"]
            print(json.dumps({"variant": name, "planning_s": elapsed,
                              "minimum_distance_m": report["minimum_sampled_distance_m"],
                              "blue_red": record["pairs"][0], "execution_failures": record["execution_failures"],
                              "selection": result.diagnostics["selection"]}), flush=True)
        successful &= result.validated
        document["runs"].append(record)
    document["all_checks_passed"] = bool(successful)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
    print(f"SAVED {output}", flush=True)
    return 0 if successful else 2


if __name__ == "__main__":
    raise SystemExit(main())
