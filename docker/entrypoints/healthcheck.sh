#!/usr/bin/env bash
# Liveness probe for the runtime image: healthy when THIS container's driver node is
# reachable and holds the lifecycle `active` state. We deliberately do NOT sample
# /diagnostics here — diagnostic_updater publishes it on the ABSOLUTE topic, so on a
# host-networked vehicle every node shares one stream and a per-container probe could
# read a neighbor's health. Data-quality (staleness/ERROR) belongs to a fleet-level
# /diagnostics aggregator, not the container probe.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
if [[ -f "${SBG_DRIVER_WORKSPACE:-/opt/sbg_driver}/setup.bash" ]]; then
  # shellcheck disable=SC1090,SC1091
  source "${SBG_DRIVER_WORKSPACE:-/opt/sbg_driver}/setup.bash" 2>/dev/null || true
fi

# The launcher exports SBG_NAMESPACE (e.g. /front) and the deploy compose passes it
# through, so the probe asks its own instance; bare `docker run` defaults to the root namespace.
NODE="${SBG_NAMESPACE:-}/sbg_driver"
state="$(timeout 8 ros2 lifecycle get "$NODE" 2>/dev/null)" || exit 1
[[ "$state" == active* ]] || exit 1
exit 0
