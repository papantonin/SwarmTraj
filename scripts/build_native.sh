#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$project_root/native" -B "$project_root/build/native" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$project_root/.venv/bin/python" \
  -DSWARM_PYTHON_OUTPUT_DIRECTORY="$project_root/src/swarm_sim"
cmake --build "$project_root/build/native" --parallel 2
ctest --test-dir "$project_root/build/native" --output-on-failure
