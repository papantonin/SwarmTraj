#!/usr/bin/env bash
# Install into the repository-local environment; never modify system Python.
set -euo pipefail

# Ignore externally sourced ROS/Conda Python paths during installation.
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"
UPSTREAM_COMMIT="f83fbdd4ee1c07e1afa52694963f3a9a889a0f56"
MODE=full
case "${1:-}" in
    '') ;;
    --trajectory) MODE=trajectory ;;
    --full) ;;
    -h|--help)
        echo 'Usage: bash scripts/setup.sh [--trajectory|--full]'
        echo 'Default: full (trajectory tools + PyBullet examples + pytest).'
        echo 'Set PYTHON=/path/to/python3.12 to select the initial interpreter.'
        exit 0 ;;
    *) echo "Unknown option: $1 (use --help)" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo 'Expected at most one option.' >&2; exit 2; }

if [[ -e "${VENV_DIR}" || -L "${VENV_DIR}" ]]; then
    [[ -x "${VENV_DIR}/bin/python" && -f "${VENV_DIR}/pyvenv.cfg" ]] || {
        echo '.venv exists but is not a usable virtual environment; move it aside and retry.' >&2
        exit 2
    }
else
    PYTHON_BIN="${PYTHON:-python3}"
    command -v "${PYTHON_BIN}" >/dev/null || { echo "Python not found: ${PYTHON_BIN}" >&2; exit 2; }
    "${PYTHON_BIN}" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 12) else "Python 3.12 or newer is required.")'
    # Copies also allow the optional PX4 container to use a standard CPython
    # executable without following a host-specific interpreter symlink.
    "${PYTHON_BIN}" -m venv --copies "${VENV_DIR}" || {
        echo 'venv creation failed. On Ubuntu 24.04 install python3-venv; see docs/installation.md.' >&2
        exit 2
    }
fi
"${VENV_DIR}/bin/python" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 12) else "Existing .venv requires Python 3.12 or newer; move it aside and retry.")'
if [[ "${MODE}" == full ]]; then
    command -v git >/dev/null || { echo 'Git is required for the pinned PyBullet backend.' >&2; exit 2; }
fi
"${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
"${VENV_DIR}/bin/python" -m pip install -e "${PROJECT_DIR}[dev,trajectory2d]"
if [[ "${MODE}" == full ]]; then
    "${VENV_DIR}/bin/python" -m pip install -r "${PROJECT_DIR}/requirements-sim.txt"
    "${VENV_DIR}/bin/python" -m pip install --no-deps \
      "gym-pybullet-drones @ git+https://github.com/learnsyslab/gym-pybullet-drones.git@${UPSTREAM_COMMIT}"
fi
bash "${PROJECT_DIR}/scripts/build_native.sh"
"${VENV_DIR}/bin/python" "${PROJECT_DIR}/scripts/doctor.py" "--${MODE}"
echo "Environment ready. Activate it with: source \"${VENV_DIR}/bin/activate\""
