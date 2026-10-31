# Ordered trajectory development

This record distinguishes changes to proposal geometry, selection objectives
and the requested speed history. Evidence is local under
`results/development-20261101/`; generated experiments are ignored by Git.

## Stable checkpoint

Commit `7e5bf6f` preserves physical profiles v3, operating-speed selection,
the hypothetical compact 5 kg airframe, four-aircraft 180-degree arrivals and
bounded alignment-rejection feedback. Its integrated suite passed 1,057 tests
and six standalone C++ tests. It requests 175 km/h throughout the dive mission;
it does not switch automatically from the separately declared cruise speed.

## 1. Curvature-bounded approach references

The explicit `approach_routing=tangent` option adds a standalone C++ planar
reference builder. It selects the shortest of four forward circle-line-circle
connections, with radius derived conservatively from speed, bank and stall/load
authority. It is a bounded proposal family, not a global shortest-path solver.
The reference starts at the actual integrated post-loiter/profile state. Heading
and cross-track feedback include nominal curvature feedforward. The physical
integrator retains roll limits and independently checks the resulting flight.

`pursuit` retains the original proposal behavior for comparison and archived
inputs. Shared PX4 editor schemas currently reject tangent-mode export instead
of silently dropping the unsupported field. No clearance, arrival, recovery,
clock, envelope or tracking limit is weakened, and references are never treated
as accepted trajectories. Obstacles still require integrated clearance checks.

| Matched 175 km/h case | Pursuit | Tangent |
|---|---:|---:|
| Drone 0 transit | 124.2 s | 99.6 s |
| Drone 2 transit | 121.8 s | 98.8 s |
| Full-horizon pair lower bound | 195.04 m | 195.04 m |

Other aircraft do not necessarily improve individually under the existing
margin-only selection objective. Seven standalone C++ tests and 60 focused
Python checks passed, including independent selected-plan replay, point-mass
execution and exact serial/spawn plan agreement. The before/after JSON missions
are retained. These measurements describe this scenario, not guaranteed gains.

## Remaining ordered work

2. Rank duration/distance among trajectories with sufficient preferred robust
   slack, preserving the mandatory acceptance bounds.
3. Integrate phase-dependent operating speeds and validate their transitions.
4. Expand numerical and mission qualification, retaining failed trials.
5. Extend PX4/Gazebo only with an explicit native airframe and separately checked
   model/controller assumptions; point-mass claims do not transfer.
