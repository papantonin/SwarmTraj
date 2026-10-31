# SwarmTraj documentation

The coordinated trajectory engine uses native C++ point-mass planning and local
execution, with Python interfaces and independent audit/observation. The
optional PX4/Gazebo experiments are separate execution backends; their spatial
missions do not inherit the planner's timing or separation claims.

## Start here

- [New-machine installation and troubleshooting](installation.md)
- [Required C++ production backend and archive migration](planner_defaults.md)
- [Project overview and examples](../README.md)
- [Coordinated 3D trajectories and scenario preparation](trajectory_3d.md)
- [Archived planar and historical trajectory modes](trajectory_2d.md)
- [Script catalog](../scripts/README.md)

## Physics, aircraft and tracking

- [Coupled 3D equations and assumptions](flight_model_3d.md)
- [Aircraft profile schemas and identity](aircraft_profiles.md)
- [Hypothetical compact 5 kg airframe and 30-degree dive example](compact_5kg.md)
- [Per-aircraft profiles, enclosing geometry and margins](heterogeneous_aircraft.md)
- [Disturbed execution and own-state tracking](robust_tracking.md)
- [Static obstacles and full-horizon clearance](obstacles.md)

## Planning and mission constraints

- [Adaptive candidate catalogs and joint search](adaptive_planning.md)
- [Margin-preferring selection](margin_selection.md)
- [Conflict witnesses and proposal guidance](conflict_guidance.md)
- [Arrival deadlines, recovery and horizon](mission_timing.md)
- [Contact events, preparation and reservations](mission_pipeline.md)
- [Free departure windows and richer profiles](free_windows_profiles.md)
- [Explicit mixed-profile policy](mixed_profiles.md)
- [Intermediate-cell passage guidance](intermediate_passages.md)
- [Physical recovery primitives and direct individual mission construction](phase2_kinodynamic.md)
- [Generator migration and validation gates](generator_migration.md)
- [CPU/memory audit and Git checkpoint before C++](cpp_readiness.md)
- [Required C++20 engine: build, numerical contract and measurements](native_kernel.md)
- [Complete native physical planner: migration and qualification](native_planner.md)
- [Default C++ planner: selection, installation and saved-scenario compatibility](planner_defaults.md)
- [Current architecture: profiles, C++ planning, local execution and PX4/Gazebo boundaries](architecture.md)
- [Mission editor: essential inputs and Advanced settings](scenario_editor.md)

## Measurements and validation

- [Planning performance and reference comparisons](planning_performance.md)
- [Parameter and disturbance campaigns](parameter_campaign.md)
- [One-to-five-aircraft scaling campaign and artifact reports](scaling_campaign.md)
- [Isolated vertical-flight verification](vertical_flight_validation.md)
- [Flight-envelope validation](flight_envelope_validation.md)

Generated reports and raw runs belong under ignored `results/`. The local
`results/README.md` inventory groups them without changing the original mission
records or treating saved reports as independent trajectory revalidation.

## PX4/Gazebo experiments

- [Single-aircraft backend, setup and limitations](px4_gazebo.md)
- [Shared scenario editor and single-aircraft execution](gazebo_editor.md)
- [Four-aircraft shared world, camera and observer](gazebo_fleet.md)
