#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# A sourced ROS installation may expose unrelated pytest plugins through
# PYTHONPATH. Loading them makes an otherwise isolated venv depend on ROS test
# packages, so run only this project's explicitly installed plugins.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "${PROJECT_DIR}/.venv/bin/python" -m pytest "$@"
