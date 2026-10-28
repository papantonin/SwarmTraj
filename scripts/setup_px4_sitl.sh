#!/usr/bin/env bash
# Optional, isolated single-aircraft simulator. Does not install host Gazebo/ROS.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ ! -x .venv/bin/python ]]; then
    echo 'Create the project .venv first using scripts/setup.sh.' >&2
    exit 2
fi
command -v docker >/dev/null
docker info --format '{{.ServerVersion}}'
.venv/bin/python -m pip install -e '.[trajectory2d,px4-sitl]'
docker pull px4io/px4-sitl-gazebo@sha256:5ace9dbca657e385cc5c529432fd6d09db6165e482de9df5ab129a941a36aa54
