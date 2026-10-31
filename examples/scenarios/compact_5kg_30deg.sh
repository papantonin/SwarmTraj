#!/usr/bin/env bash
# Native point-mass demonstration; not a PX4/Gazebo airframe configuration.
# All legs use the chosen regime; cruise-to-dive switching is not implemented.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "$project_root/.venv/bin/python" "$project_root/examples/trajectory_3d.py" \
  --num-drones 4 \
  --aircraft-profile "$project_root/examples/aircraft/compact_5kg.json" \
  --flight-regime dive --descent-angle 30 \
  --approach-spread 180 --candidate-guidance rejections \
  --approach-routing tangent \
  --altitude 500 --cruise-range 500 550 --max-altitude 700 \
  --target 3000 0 --target-altitude 200 \
  --loiter-radius 400 --loiter-spacing 1000 \
  --horizon 240 --candidates 4 --attempts 24 "$@"
