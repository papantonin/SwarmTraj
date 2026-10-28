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

## Campaigns, audits and reports

| Script | Purpose |
|---|---|
| [validate_scenarios.py](validate_scenarios.py) | Incremental four-aircraft parameter/disturbance campaign |
| [audit_campaigns.py](audit_campaigns.py) | Read-only consistency audit of supported historical journals |
| [benchmark_scaling_campaign.py](benchmark_scaling_campaign.py) | Reproducible one-to-five-aircraft campaign |
| [benchmark_planning_limits.py](benchmark_planning_limits.py) | Budget, search-cap, horizon and profile-policy probes |
| [report_scaling_campaign.py](report_scaling_campaign.py) | Scaling-journal audit, PDF, plots and CSV exports |
| [diagnose_campaign_execution.py](diagnose_campaign_execution.py) | Observer-only replay of original envelope checks |
| [visualize_campaign_plans.py](visualize_campaign_plans.py) | Saved-plan galleries with original targets and obstacles |

See the [scaling campaign guide](../docs/scaling_campaign.md). Consistency
audits do not independently revalidate trajectory physics. Failed trials and
original mission documents remain part of the evidence.

## Isolated physics verification

| Script | Purpose |
|---|---|
| [validate_vertical_flight.py](validate_vertical_flight.py) | Manufactured fixtures and independent unbanked ODE comparison |
| [validate_flight_envelope.py](validate_flight_envelope.py) | Boundary classifications and steady-equilibrium experiments |

## Local result organization

```bash
.venv/bin/python scripts/index_results.py
```

[index_results.py](index_results.py) writes a grouped `results/README.md` and
`results/inventory.json`. It reads directory metadata only, follows no result
directory symlinks and does not move, delete or reinterpret experiments.
Source code, inputs and `.venv` stay outside generated results.
