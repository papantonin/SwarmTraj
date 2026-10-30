#!/usr/bin/env bash
# Optional isolated simulator. Does not install host Gazebo/ROS or change Docker permissions.
set -euo pipefail

# Ignore externally sourced ROS/Conda Python paths during installation.
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [[ ! -x .venv/bin/python ]]; then
    echo 'Create the project .venv first: bash scripts/setup.sh --trajectory' >&2
    exit 2
fi
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
    echo 'The PX4 fixture requires a Linux x86-64 host; see docs/installation.md.' >&2
    exit 2
fi
.venv/bin/python -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 12) else "The PX4 container requires CPython 3.12 (host .venv is mounted inside it).")'
command -v docker >/dev/null || { echo 'Install Docker Engine first; see docs/installation.md.' >&2; exit 2; }
docker info --format '{{.ServerVersion}}' || {
    echo 'Docker is unavailable to this user. See docs/installation.md; do not run setup with sudo.' >&2
    exit 2
}
.venv/bin/python -m pip install -e '.[dev,trajectory2d,px4-sitl]'
IMAGE="$(.venv/bin/python -c 'from swarm_sim.px4_gazebo import IMAGE; print(IMAGE)')"
docker pull "${IMAGE}"
.venv/bin/python scripts/doctor.py --px4
