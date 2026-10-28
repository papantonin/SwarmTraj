#!/bin/sh
# One isolated world, four independent native autopilots and parameter stores.
set -eu
test "${SWARM_MANAGED_SIM:-}" = 1
test "${SWARM_FLEET_SIZE:-}" = 4
test -f /.dockerenv
gz sim --headless-rendering --verbose=1 -r -s /runresults/scene.sdf &
export PX4_GZ_STANDALONE=1
for fleet_id in 0 1 2 3; do
    mkdir -p "/runresults/drone-$fleet_id/px4-rootfs"
    ln -s /opt/px4-gazebo/etc "/runresults/drone-$fleet_id/px4-rootfs/etc"
    PX4_GZ_MODEL_NAME="rc_cessna_lidar_$fleet_id" \
      /opt/px4-gazebo/bin/px4 -i "$fleet_id" -d \
      -w "/runresults/drone-$fleet_id/px4-rootfs" \
      >"/runresults/drone-$fleet_id/px4-console.log" 2>&1 &
done
wait
