#!/usr/bin/env bash
# Liveness probe for the runtime image: healthy when /diagnostics is flowing and no
# task reports ERROR (2) or STALE (3). WARN still counts as alive — a sensor waiting
# for its first fix must not flap the container.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-lyrical}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
if [[ -f "${SBG_DRIVER_WORKSPACE:-/opt/sbg_driver}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${SBG_DRIVER_WORKSPACE:-/opt/sbg_driver}/setup.bash" 2>/dev/null || true
fi

# `ros2 topic echo` renders the byte-typed `level` as a quoted escape (`level: "\x01"`),
# NOT as an integer — match both renderings to stay CLI-version-tolerant.
out="$(timeout 5 ros2 topic echo --once --field status \
  /diagnostics diagnostic_msgs/msg/DiagnosticArray 2>/dev/null)" || exit 1
if ! grep -q 'level:' <<<"$out"; then exit 1; fi
if grep -qE 'level: (2|3|"\\x0[23]")' <<<"$out"; then exit 1; fi
exit 0
