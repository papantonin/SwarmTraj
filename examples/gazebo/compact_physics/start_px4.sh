#!/bin/sh
# Dedicated simulation-only startup qualification, never a hardware connection.
set -eu
test "${SWARM_COMPACT_PX4_STARTUP:-}" = 1
test -f /.dockerenv
gz sim --headless-rendering -r -s /runresults/world.sdf &
export PX4_GZ_STANDALONE=1
exec /opt/px4-gazebo/bin/px4 -d
