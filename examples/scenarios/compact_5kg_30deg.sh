#!/usr/bin/env bash
# Native point-mass demonstration; not a PX4/Gazebo airframe configuration.
# Own operating speeds: loiter, cruise, terminal dive, recovery and cruise.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$project_root/.venv/bin/python" "$project_root/examples/trajectory_3d.py" \
  --num-drones 4 \
  --aircraft-profile "$project_root/examples/aircraft/compact_5kg.json" \
  --phase-speeds --speed-profile-acceleration 1.5 \
  --flight-regime dive --descent-angle 30 \
  --approach-spread 180 --candidate-guidance rejections \
  --approach-routing tangent \
  --selection-policy efficient --efficiency-slack-cap 20 \
  --altitude 500 --cruise-range 500 550 --max-altitude 700 \
  --target 3000 0 --target-altitude 200 \
  --loiter-radius 400 --loiter-spacing 1000 \
  --horizon 420 --candidates 4 --attempts 24 "$@"
