# Static obstacles and obstacle-aware trajectories

The coordinated trajectory prototype supports known **vertical cylinders** and
**axis-aligned rectangular boxes**. Obstacles are static and expressed in the
same metric world frame as the aircraft. No in-flight communication or reactive
obstacle sensor is introduced.

## Four-aircraft example

```bash
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 500 300 --cruise-range 40 110 --approach-spread 180 --horizon 180 \
  --departure-delay 0 0 --activation-time 0 \
  --obstacles examples/scenarios/static_obstacles.json
```

The preparation map displays obstacle footprints and altitude intervals; the
3D replay shows solid volumes. Drones and target remain movable in the editor,
which prevents launch with initial obstacle conflicts. Select an obstacle to
move its center on the map or edit its center altitude and half-dimensions.
The **+ Cylinder**, **+ Box** and **Remove obstacle** buttons add/remove
solids. CLI and JSON obstacle inputs remain supported.
Orange dashed XY outlines illustrate physical clearance; rectangular outlines
around boxes conservatively approximate the rounded Euclidean offset.

Add `--no-gui` for headless execution. The same scene can be entered directly:

```bash
.venv/bin/python examples/trajectory_3d.py --num-drones 4 --no-gui \
  --target 500 300 --cruise-range 40 110 --approach-spread 180 --horizon 180 \
  --departure-delay 0 0 --activation-time 0 \
  --obstacle-cylinder 200 120 25 0 150 \
  --obstacle-box 300 170 0 340 210 130 --obstacle-clearance 5
```

- `--obstacle-cylinder X Y RADIUS ZMIN ZMAX`: footprint and altitude interval;
  repeat for multiple cylinders.
- `--obstacle-box XMIN YMIN ZMIN XMAX YMAX ZMAX`: opposite bounding corners;
  repeat for multiple boxes.
- `--obstacles FILE`: JSON list. File obstacles are followed by CLI cylinders
  and boxes, which determines printed obstacle indices.
- `--obstacle-clearance METERS`: required drone-center distance from the solid's
  surface, 5 m by default. Include the aircraft's physical extent and desired
  clearance; this is not inferred from a wingspan model.
- `--obstacle-routing spatial` (CLI default): interleave lateral routes and
  obstacle-derived overflight/underflight cruise proposals in 3D.
  `--obstacle-routing lateral` restores the original XY proposal heuristic.
  The library retains `Config(obstacle_routing="lateral")` for compatibility.

JSON uses the same obstacle fields saved in mission configuration:

```json
[
  {"kind": "cylinder", "center": [200, 120, 75], "half_extents": [25, 25, 75]},
  {"kind": "box", "center": [320, 190, 65], "half_extents": [20, 20, 65]}
]
```

Coordinates must be finite, and half-extents positive and finite. Cylinder X/Y
half-extents must equal its radius; the Z half-extent is half its height.
An empty list describes an obstacle-free scene. In Python, use
`Obstacle.cylinder(x, y, radius, zmin, zmax)` or
`Obstacle.box(xmin, ymin, zmin, xmax, ymax, zmax)` in a tuple passed to
`Config(obstacles=(...), obstacle_clearance=5, ...)`.

## Proposals versus safety checks

The generator retains seeded random intermediate points and assigned approach
sides. An XY visibility graph inserts lateral detours around expanded bounding
rectangles. Expansion includes uncertainty margins and room related to turn
radius. Routing ties are deterministic; candidate parity changes corner order
to permit alternate sides. Random intermediate points inside expanded
rectangles may be omitted; assigned gates and destination are retained.

Post-arrival exit is also routed. Aircraft do not disappear or stop at the
target. Initial loiter is not rewritten: obstacle conflicts during loiter reject
candidates. Existing bounded-turn guidance is retained; coupled mode integrates
actual flight dynamics before independent validation.

The graph is **only a proposal heuristic**, not a safety certificate or complete
3D planner. Lateral-only mode may detour around an obstacle that could be
overflown. Either mode may fail to suggest a narrow feasible passage. If an endpoint is inside an expanded XY
footprint, a segment may be retained as a proposal because it could be above
or below the actual solid. Only final XYZ checks can accept it. Overlapping
obstacles can exhaust the finite catalog. No margin, approach sector, departure
window or flight-envelope limit is silently relaxed.

### Altitude-level alternatives

In spatial mode, candidate IDs cycle through lateral / above / below proposal
families. Above/below alternatives derive levels from the roofs/bases of
obstacles near the proposed XY route. Levels must remain inside the supplied
cruise range (or the single manual level) and leave enough height for the
requested final descent. Within each feasible interval, the initial altitude
is preferred to avoid an unnecessary transition. Additional families explore
different obstacle-derived intervals deterministically, without extra RNG draws.
An unavailable vertical alternative falls back to the lateral proposal; the
algorithm never widens altitude bounds or lowers clearance automatically.

Level selection uses a buffer consisting of nominal robust clearance, the
between-sample speed correction and 3 m of proposal-only headroom. This buffer
is not an additional certified tracking bound. Obstacles sufficiently separated
vertically are omitted only from the cruise-route prefix's visibility graph.
Other blockers still induce lateral detours, allowing mixed proposals. Final
approach gates, destination and post-arrival exit retain the full lateral map.

Altitude is not teleported: the existing bounded vertical profile, retiming,
coupled integration and independent validation still apply. A safe cruise level
does **not** imply a safe climb, descent, loiter or pull-out. Any collision or
insufficient transition time invalidates the candidate. The planner explores
one cruise level per proposal, not arbitrary local 3D altitude transitions;
this remains an incomplete heuristic, not a general kinodynamic 3D solver.

`routing_mode` in individual plans and timing schedules describes the proposal
family (`lateral`, `overflight`, `underflight`), not a classification of the
entire executed path. In particular, the historical lateral fallback may itself
produce a valid overflight. Accepted family counts are in each catalog's
`accepted_routing_modes`. Selecting a vertical family is not an objective:
the joint optimizer continues to prefer inter-drone separation margins.

```bash
# Four aircraft can cross this low, wide obstacle above its roof.
.venv/bin/python examples/trajectory_3d.py --num-drones 4 \
  --target 600 0 --target-altitude 80 --descent-angle 0 \
  --altitude 80 --altitude-spacing 25 --cruise-range 80 170 \
  --horizon 120 --activation-time 0 --departure-delay 0 0 \
  --obstacle-box 220 -500 0 260 500 60 --obstacle-routing spatial \
  --margin-expansion-rounds 0 --candidates 8 --attempts 24

# For a suspended obstacle, replace the obstacle option with:
# --obstacle-box 220 -500 180 260 500 250
```

On these two four-aircraft scenes, the lateral-only catalogs exhausted their
24 proposals per drone without finding a compatible combination. Spatial
catalogs succeeded with four accepted candidates per drone, including
overflight/underflight alternatives. All nominal execution checks passed.
Minimum sampled pair distance was 39.88 m (low wall) and 36.13 m (suspended
wall); obstacle-surface distances were 20.00 m and 25.00 m, respectively.
Neither a general completeness claim nor a disturbed-flight guarantee follows.

## Clearance calculation

Let `s_O(p)` be signed Euclidean distance to a solid: positive outside, zero
on its surface and negative inside. Distances are analytic for ideal cylinders
and boxes. In 3D, roofs, bases and side faces participate, so altitude can make
an XY overlap safe. Planar mode uses footprints regardless of height.

Signed distance is 1-Lipschitz. With nominal speed bound `V` and sample spacing
`dt`, the checker uses:

```text
continuous_clearance_lower_bound = min_samples(s_O(position)) - V * dt/2
required_nominal_clearance = obstacle_clearance + position_error + V * clock_error
```

The minimum covers all obstacles and **the full horizon**: loiter, transitions,
approach, pull-out and onward flight. The static obstacle has no timing error
of its own, but `V*clock_error` conservatively covers the drone's phase error.
Spatial-error bounds remain Euclidean.

Failing candidates are rejected as `obstacle_clearance` before joint selection.
Selected trajectories are checked again independently of cached drone-pair
decisions. Execution uses its finer observation grid, a ground-speed bound
including wind, and the physical obstacle-clearance requirement. A violated
execution speed bound invalidates the obstacle-clearance lower bound.

Guarantees remain conditional on numerical paths, speed envelope, tracking and
clock assumptions, and map accuracy. There is no certified integration-error
bound or real-aircraft certification. Include map uncertainty in obstacle
geometry or clearance; it is not estimated automatically.

## Diagnostics and exports

Preflight may return:

- `initial_obstacle_violation`: an initial position violates physical clearance.
- `initial_obstacle_margin_not_certified`: the robust initial margin fails.
- `target_region_obstructed`: the entire nominal arrival region is within an
  obstacle's forbidden clearance zone. Partial overlap is left to trajectory
  validation. In 3D layers mode, this destination check is skipped because no
  common target altitude is assumed.

Other failures remain `no_candidate`, `catalog_exhausted` or
`search_budget_exhausted`, not proofs of infeasibility. Planning diagnostics
include rejection counts and selected plans' obstacle lower bounds and margins.
Obstacle geometry is saved in mission and individual configuration payloads.

`report.json` adds `obstacle_count`, `obstacle_clearance_m`,
`minimum_sampled_obstacle_distance_m`, `minimum_obstacle_distance_lower_bound_m`,
`obstacle_clearance_verified_in_execution_model` and `sampled_obstacle_collision`.
The CLI returns 3 when obstacle clearance fails during execution, even if all
drones arrive. A conservative clearance failure does not necessarily mean an
actual collision. Without obstacles, distances are `null` and the empty obstacle
check is satisfied.

## Validation and limitations

The initial four-aircraft example passed nominal arrival, envelope, inter-drone
and obstacle checks. Minimum sampled obstacle-surface distance was 45.60 m
(45.44 m conservative execution lower bound), against 5 m physical clearance.
Generation took 3.52 s in the latest headless run, not a guaranteed deadline.
Artifacts: `results/trajectory3d/20261025-171507-013067/`.
For comparison, drone 0's previously planned obstacle-unaware path in
`results/trajectory3d/20261025-162801-830040/` would penetrate the new cylinder
by 9.41 m at sampled instants. That older run used a 150 s horizon, so this is
an illustration of why replanning is necessary, not a controlled timing
comparison. The new two-obstacle scene is rerouted and independently checked.

An actual Tk-window smoke test exercised the preparation map, spawn-based
planning and animated 3D replay. Screenshots are stored under
`results/obstacle-gui-smoke/20261025-170828-178787/`.

Tests cover signed distances, overflight versus planar projection, collisions
between samples, initial/target rejection, deterministic detours, spawn
reproducibility, post-arrival checks and four-aircraft coupled flight, including
new spatial alternatives versus lateral-only catalogs and independent rejection
when an obstacle blocks a formerly valid vertical route.

Not included: moving/newly discovered obstacles, terrain or arbitrary building
meshes, reactive sensing, complete routing or emergency maneuvers. Pre-existing
limitations of disturbed tracking remain; see
[adaptive-planning evaluation](adaptive_planning.md).
