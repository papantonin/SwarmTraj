#!/usr/bin/env bash
# A separate build image supplies compiler tools; the pinned runtime is unchanged.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
physics_build="$project_root/build/compact-gazebo"
mkdir -p "$physics_build"
docker build -t swarm-compact-physics-builder \
  -f "$project_root/examples/gazebo/compact_physics/Dockerfile.build" \
  "$project_root/examples/gazebo/compact_physics"
docker image inspect swarm-compact-physics-builder --format '{{.Id}}' > "$physics_build/builder-image.txt"
docker run --rm --network none --user "$(id -u):$(id -g)" \
  --mount "type=bind,src=$project_root/examples/gazebo/compact_physics,dst=/source,readonly" \
  --mount "type=bind,src=$physics_build,dst=/build" \
  --entrypoint /bin/sh swarm-compact-physics-builder -c \
  'g++ --version > /build/compiler.txt && cmake -S /source -B /build -DCMAKE_BUILD_TYPE=Release && cmake --build /build --parallel 2'
