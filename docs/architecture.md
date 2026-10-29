# Planning and execution architecture

The generator choice is now shared by the trajectory editor and the optional
PX4/Gazebo preparation path. The point-mass engine and the native aerodynamic
simulator remain separate models. Physical reference generation does not replace
Gazebo aerodynamics or transfer a point-mass safety claim to native execution.

```mermaid
flowchart TD
    Editor["Python editor / CLI: aircraft, target, obstacles, timing and generator"]
    JSON["Shared single/fleet JSON v2: explicit generator_mode; v1 means geometric"]
    Input["Fixed designation snapshot and own physical profiles; input checks and approach sectors"]
    Mode{"Generator choice: coupled 3D"}
    Geom["geometric: routing/vertical proposals, then coupled physical rollout"]
    Physical["physical_mission: integrated loiter, levels, transit, approach, passage, pull-out, recovery"]
    Individual{"Independent individual validation: forces/envelope, contact, obstacles, full horizon"}
    Catalog["Deterministic per-aircraft catalogs; serial or spawn processes"]
    Joint["Bounded joint search; full-horizon distance and optional reservations"]
    Expand{"Compatible selection? Budgets available?"}
    Margin["Prefer larger robust slack if requested; preserve all required safety margins"]
    Mission["Immutable original mission; selected own plan per aircraft"]
    Fail["Explicit rejection / exhausted finite catalog / budget diagnostic"]
    Own["Own-state, own-clock point-mass tracker; simulated sensors and disturbances"]
    Eval["Observer-only execution checks: envelope, tracking, separation and obstacles"]
    Spatial["Derive own spatial PX4 mission and SDF: timing/separation claims are lost"]
    Native["Isolated pinned runtime: one Gazebo world, one or four independent PX4 instances"]
    Gates["Native feasibility/readiness/arm and bounded warm initialization gates"]
    Fly["Native own-aircraft controllers; Gazebo aerodynamic model, sensors and collisions"]
    Observer["Observer-only spatial, contact, obstacle and pair measurements; free camera"]
    End{"Completion policy"}
    Remove["remove-at-target: own observed contact; own removal acknowledged AND absence verified"]
    Land["land: historical native return, landing and disarm gates"]
    Artifacts["Reports, fingerprints, telemetry and measured limitations"]
    Editor --> JSON --> Input --> Mode
    Mode --> Geom --> Individual
    Mode --> Physical --> Individual
    Individual -->|accepted| Catalog --> Joint --> Expand
    Individual -->|rejected; attempts remain| Mode
    Individual -->|exhausted| Fail
    Expand -->|compatible| Margin --> Mission
    Expand -->|expand within budgets| Mode
    Expand -->|exhausted| Fail
    Fail -. scenario correction .-> Editor
    Mission --> Own --> Eval --> Artifacts
    Mission --> Spatial --> Native --> Gates --> Fly --> Observer --> End
    End --> Remove --> Artifacts
    End --> Land --> Artifacts
```

The full original horizon always includes initial loiter, transitions, target
passage, pull-out and onward flight. In the explicit simulation-only removal
mode, only the derived native/display prefix and runtime completion change.
The original full-horizon validation is retained. Pair evaluation uses active
overlapping intervals after deletion; missing scene replies cannot establish
successful removal.

Catalog/search hints and geometric routes are ground-side proposals. Controllers
receive their own plan, observed state and clock, without peer telemetry or true
disturbance state. Observer outputs do not become live avoidance commands.
Multiple process counts preserve seeded streams, order and acceptance rules.

See [generator migration](generator_migration.md),
[point-mass assumptions](flight_model_3d.md),
[single-aircraft editor](gazebo_editor.md) and [native fleet](gazebo_fleet.md).
The generated local PDF at `results/architecture-diagram/architecture.pdf`
illustrates the same branches; this tracked document is its reviewable source.
