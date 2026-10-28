# Preferring combinations with greater separation margin

The CLI defaults to `--selection-policy max_margin`. The Python library keeps
`Config(selection_policy="first")` for compatibility with existing experiments.
This changes **selection**, not the safety thresholds, flight dynamics or
communication assumptions. Every candidate still passes individual flight and
obstacle checks before selection, and selected pairs are independently checked
after selection.

## Objective: leximin robust pair slack

For each pair of plans `i,j`, define:

```text
lower_ij = sampled_min_distance_ij - (V_i + V_j) * dt/2
required_ij = separation + 2*position_error + (V_i + V_j)*clock_error
slack_ij = lower_ij - required_ij
```

These are conservative full-horizon nominal separation bounds and robust
requirements, not distances measured during execution. All pairs must still
pass the existing hard compatibility criterion. For each feasible combination,
sort its pair slacks from smallest to largest. Prefer the lexicographically
largest vector: first maximize the worst pair's slack, then the second-worst,
and so on. This is a **leximin** objective.

This avoids selecting a combination with one close pair merely because other
pairs are far apart. Secondary comparisons also help when the worst margin is
fixed by an unavoidable initial-state bottleneck. It does **not** guarantee
that every named pair becomes farther apart, nor does it optimize obstacle
clearance, energy, travel time or all continuous-space trajectories. Obstacle
clearance and flight limits remain hard constraints.

## Bounded selection and targeted extension

1. Find an initial feasible combination using the existing adaptive/fixed
   catalog generation and bounded compatibility search.
2. Retain that feasible incumbent and search the generated catalogs using
   deterministic branch-and-bound. Cache pair margins with pair compatibility.
3. A sorted partial margin vector padded with positive infinity bounds what a
   branch could achieve. Prune branches that cannot beat the incumbent.
4. If permitted, extend at most two expandable catalogs, prioritizing endpoints
   of the current lowest-slack pairs. Resume their existing random streams.
5. Refine again while retaining the best feasible incumbent from earlier steps.
6. Independently verify selected inter-drone and obstacle clearances over the
   full horizon. No geometric or timing change is applied after verification.

This is best-so-far selection under finite budgets. If refinement stops at its
budget, the incumbent remains available; a refinement limit does not turn a
successful feasible mission into `search_budget_exhausted`. If no feasible
combination was found in the first place, the original failure statuses remain.

The score is nondecreasing across refinement/extension steps, but optimality is
limited to the explored catalogs. `catalog_optimal=true` means the search proved
its score optimal **within the final generated catalogs**; it says nothing about
unattempted candidates, future catalog extensions or continuous space. Exact
floating-point comparisons are deterministic within the same numerical
environment; they are not a formal numerical proof.

## Settings

```bash
# New CLI default, four-aircraft visualization
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --selection-policy max_margin

# Faster comparison: first compatible combination, no quality refinement
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --selection-policy first

# Prefer margins within existing catalogs, without extra generation
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --selection-policy max_margin --margin-expansion-rounds 0

# Limit refinement work and allow one targeted extension
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --selection-policy max_margin --margin-search-nodes 2000 --margin-expansion-rounds 1
```

- `--margin-search-nodes 2000`: cumulative refinement-node cap across all quality
  passes. Refinement also consumes the remaining global `--max-search-nodes`
  budget; neither budget resets between passes.
- `--margin-expansion-rounds 1`: maximum catalog-extension rounds after the
  first feasible solution. A value of 0 optimizes only existing catalogs.
- An extension adds up to `--initial-candidates` accepted plans to each selected
  catalog, respecting existing per-drone `--candidates` and `--attempts` caps.
  Exhausted catalogs are not regenerated.

These are work budgets, not guaranteed wall-clock deadlines. Extra candidate
generation can dominate refinement time. The policy does not itself enlarge
departure-delay intervals, altitude ranges, approach spread or physical
separation requirements. It selects among permitted plans, which can have
different paths, cruise levels or departure times within the configured ranges.

## Diagnostics

`planning_diagnostics.selection` in the manifest/report includes:

- Policy, objective and initial/selected candidate IDs.
- Initial and selected sorted slack vectors, and pair-labeled slacks.
- `improved`, indicating a strictly better lexicographic score, including
  secondary-margin improvements even if the minimum itself is unchanged.
- Each quality pass's catalog sizes, attempts, expanded IDs, improvement count,
  cumulative optimization nodes and finite-catalog optimality flag.
- Final `catalog_optimal`, `optimization_budget_exhausted` and actual
  `expansion_rounds`.

`optimization_wall_s` is a subset of `search_wall_s`; do not add both when
computing total phase time. Both remain outside the reproducible manifest.
The CLI prints the minimum robust slack before and after refinement.
Individual drone payloads still contain only their own plan and shared
configuration, not the global selection/conflict diagnostics.

## Four-aircraft comparison

```bash
.venv/bin/python scripts/benchmark_selection.py \
  --mission results/trajectory3d/20261025-173039-456145/mission.json
```

This reconstructs a saved four-aircraft scene and compares first-feasible
selection, refinement within initial catalogs, and refinement with one targeted
extension. Each result is executed and checked. Output includes minimum
distances for every named pair, execution failures and planning diagnostics.
An existing output file is never overwritten. Timings exclude execution.

On the saved scene with a 100 m radius cylinder, a 120 m wide box, 180° approach
spread, a 180 s horizon and simultaneous departures:

| Policy | Minimum across all pairs | Blue/red minimum | Generated catalog sizes |
|---|---:|---:|---|
| First feasible | 26.13 m | 27.42 m | `(4,4,4,4)` |
| Prefer margin, existing catalogs | 27.60 m | 27.60 m | `(4,4,4,4)` |
| Prefer margin + targeted extension | 36.51 m | 37.38 m | `(8,8,4,4)` |

All three executions passed arrival, tube, separation, obstacle, approach,
descent-angle and physical-envelope checks. Refinement used 71 nodes in total
with extension, in addition to four feasibility-search nodes. The physical
separation setting remained 8 m; departure delays remained zero.
Recorded planning times were approximately 5.6/5.3/10.3 s, but other tests were
running concurrently, so these are not isolated performance measurements.
Raw data: `results/benchmarks/margin-selection-4-drones.json`.

Tests compare branch-and-bound with exhaustive enumeration on artificial
four-drone compatibility graphs, check budget-limited incumbent retention,
serial/spawn reproducibility, and execute four coupled aircraft with obstacles.
Safety remains conditional on the [flight-model assumptions](flight_model_3d.md),
not certified real-world collision avoidance.
