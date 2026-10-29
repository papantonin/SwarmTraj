# Script catalog

Run Python scripts from the repository root with the existing `.venv`.
Script locations remain unchanged because launchers, tests and saved campaign
fingerprints reference them. Use `--help` for each Python command's options.

## Environment and tests

| Script | Purpose |
|---|---|
| [setup.sh](setup.sh) | Initial environment setup with the pinned simulation dependency |
| [setup_px4_sitl.sh](setup_px4_sitl.sh) | Optional isolated PX4/Gazebo fixture setup |
| [test.sh](test.sh) | Pytest runner isolated from external ROS plugins |

## Planning and execution comparisons

| Script | Purpose |
|---|---|
| [benchmark_planning.py](benchmark_planning.py) | Reference, batched and spawn-worker comparisons |
| [benchmark_adaptive.py](benchmark_adaptive.py) | Fixed versus adaptive catalogs |
| [benchmark_selection.py](benchmark_selection.py) | First-feasible versus margin-preferring selection |
| [benchmark_conflicts.py](benchmark_conflicts.py) | Unguided versus witness-guided proposals |
| [benchmark_intervals_profiles.py](benchmark_intervals_profiles.py) | Departure windows, rich/mixed profiles and passages |
| [benchmark_tracking.py](benchmark_tracking.py) | Identical-plan disturbance and rejection trials |
| [benchmark_physical_missions.py](benchmark_physical_missions.py) | Paired historical/direct-physical individual missions, with optional execution checks |

## Campaigns, audits and reports

| Script | Purpose |
|---|---|
| [validate_scenarios.py](validate_scenarios.py) | Incremental four-aircraft parameter/disturbance campaign |
| [audit_campaigns.py](audit_campaigns.py) | Read-only consistency audit of supported historical journals |
| [benchmark_scaling_campaign.py](benchmark_scaling_campaign.py) | Reproducible one-to-five-aircraft campaign |
| [benchmark_planning_limits.py](benchmark_planning_limits.py) | Budget, search-cap, horizon and profile-policy probes |
| [report_scaling_campaign.py](report_scaling_campaign.py) | Scaling-journal audit, PDF, plots and CSV exports |
| [report_generator_replay.py](report_generator_replay.py) | Saved-input generator comparison, independent saved-plan revalidation and default-promotion blockers |
| [diagnose_campaign_execution.py](diagnose_campaign_execution.py) | Observer-only replay of original envelope checks |
| [visualize_campaign_plans.py](visualize_campaign_plans.py) | Saved-plan galleries with original targets and obstacles |

See the [scaling campaign guide](../docs/scaling_campaign.md). Consistency
audits do not independently revalidate trajectory physics. Failed trials and
original mission documents remain part of the evidence.

To replay the saved 230 exploratory missions plus negative controls and spawn
replays with the physical generator, preserving all other saved inputs and the
original timeout:

```bash
.venv/bin/python scripts/benchmark_scaling_campaign.py \
  --reference results/scaling-campaign-20261028/manifest.json \
  --suites controls core random replay --generator-mode physical_mission \
  --output results/physical-generator-comparison
```

The physical generator supports the strict multilevel/smooth-speed profiles in
all 230 saved configurations. Unsupported requests outside this matrix (such as
geometric passage-window fitting) remain explicit
`unsupported_generator_configuration` records. They never fall back to simpler
profiles. For physical replays, seed-first validated baseline, obstacle,
heterogeneous, strict-rich and grid families receive nominal, gust/sensor and
deliberately out-of-clock execution trials. These extra execution families are
recorded in the manifest; the per-process timeout remains unchanged.
After completion, create its comparative HTML, PDF, CSV and numerical-check
report:

```bash
.venv/bin/python scripts/report_generator_replay.py results/physical-generator-comparison
```

For matched new comparisons, explicitly supply `--planning-workers 1` or `2`
and the same `--execution-families baseline obstacles heterogeneous rich_strict grid`
for each generator. Dedicated serial/spawn replay pairs keep their saved counts.
`--jobs ID ...` records a diagnostic subset without qualifying default promotion.
See the [migration procedure](../docs/generator_migration.md).

The [completed saved-input comparison](../docs/phase2_kinodynamic.md#completed-saved-input-comparison)
records 192/230 physical planning successes against 180/230 historically,
with all previous successes retained. That historical journal contains five
censored planning timeouts. The [migration checks](../docs/generator_migration.md)
now complete those inputs below 120 s, while retaining the original records;
full qualification and a native fleet startup failure still block promotion.
The default remains geometric.

## Isolated physics verification

| Script | Purpose |
|---|---|
| [validate_vertical_flight.py](validate_vertical_flight.py) | Manufactured fixtures and independent unbanked ODE comparison |
| [validate_flight_envelope.py](validate_flight_envelope.py) | Boundary classifications and steady-equilibrium experiments |
| [phase2_pullout_bench.py](phase2_pullout_bench.py) | Historical local pull-out/seed comparison; scalar coefficients are no longer used in mission generation |

## Local result organization

```bash
.venv/bin/python scripts/index_results.py
```

[index_results.py](index_results.py) writes a grouped `results/README.md` and
`results/inventory.json`. It reads directory metadata only, follows no result
directory symlinks and does not move, delete or reinterpret experiments.
Source code, inputs and `.venv` stay outside generated results.
