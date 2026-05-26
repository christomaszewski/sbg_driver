#!/usr/bin/env bash
# Liveness check for compose.hil.yaml / Dockerfile.runtime.
# Listens for one DiagnosticArray message on /diagnostics with overall level OK.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

timeout 5 ros2 topic echo --once --field status \
  /diagnostics diagnostic_msgs/msg/DiagnosticArray 2>/dev/null \
  | grep -q "level: 0" && exit 0

exit 1
