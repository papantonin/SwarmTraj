#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"
UPSTREAM_COMMIT="f83fbdd4ee1c07e1afa52694963f3a9a889a0f56"

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/python" -m pip install -r "${PROJECT_DIR}/requirements-sim.txt"
"${VENV_DIR}/bin/python" -m pip install --no-deps \
  "gym-pybullet-drones @ git+https://github.com/learnsyslab/gym-pybullet-drones.git@${UPSTREAM_COMMIT}"
"${VENV_DIR}/bin/python" -m pip install -e "${PROJECT_DIR}[dev]"

echo "Environment ready. Activate it with: source .venv/bin/activate"
