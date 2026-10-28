#!/bin/sh
# Container-only fixture. EGL supports the downward range sensor without X11.
set -eu
test "${SWARM_MANAGED_SIM:-}" = 1
test -f /.dockerenv
gz sim --headless-rendering --verbose=1 -r -s /runresults/scene.sdf &
if [ -z "${HEADLESS:-}" ] && [ "${SWARM_AIRBORNE_START:-}" != 1 ]; then
    gz sim -g &
fi
export PX4_GZ_STANDALONE=1
exec /opt/px4-gazebo/bin/px4 -d
