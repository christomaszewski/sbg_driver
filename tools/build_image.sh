#!/usr/bin/env bash
# rig's BUILD-phase entrypoint (rigging.yaml `build.command`): build the self-contained runtime
# image and push it to the fleet registry. rig invokes it as
#   tools/build_image.sh <registry> [tag]        (cwd = repo root, tag defaults to latest)
# and the deploy compose pulls the result as ${RIG_IMAGE_REGISTRY}/sbg_driver:${RIG_IMAGE_TAG}.
# Works standalone too:  tools/build_image.sh 192.168.1.10:5000 mytag
#
# rig build-env contract (rig exports these; scripts opt in; all rig-owned vars are set-or-popped,
# so the ${VAR:+...}/${VAR:-...} expansions below are safe under set -u):
#   RIG_BUILD_NO_CACHE  set => --no-cache --pull. --pull matters: a deliberate `rig build
#                       --no-cache` re-pulls the parent image, so the fleet's ros-* version
#                       authority advances for all images together (see docker/Dockerfile.runtime).
#   ROS_DISTRO          fleet distro (vehicle.yaml ros.distro); default lyrical.
#   RIG_BASE_IMAGE      the deployment's one runtime base image (fleet-ros), forwarded to every
#                       dependent build; empty/absent falls back to the standalone ros-core parent.
#   RIG_BUILD_PLATFORMS comma-separated Docker platforms; buildx publishes one image index.
#   RIG_BUILD_RESULT    optional JSONL path for the published image/index digest.
#
# A plain-HTTP local registry needs the daemon's insecure-registries on this host (with the :port).
# Cross-arch vehicle: export SBG_BUILD_PLATFORM=linux/<arch> (default: host-native).
# RIG_BUILD_PLATFORMS takes precedence over SBG_BUILD_PLATFORM under rig.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=tools/rig-build.sh
. "$REPO/tools/rig-build.sh"

REGISTRY="${1:?usage: build_image.sh <registry> [tag]}"
TAG="${2:-latest}"
IMAGE="${REGISTRY}/sbg_driver:${TAG}"

# Capture identity before .dockerignore strips .git from the build context. A clean checkout
# supplies both the actual ref (exact tag, else SHA) and its commit SHA for rig's SHA-tier audit.
# Dirty trees keep the -dirty ref but cannot claim HEAD describes the built content. Include
# untracked, non-ignored files: git describe --dirty alone misses newly added source files.
# Require this root's own .git (directory OR worktree/submodule file), so a tarball unpacked
# inside some other checkout cannot borrow that checkout's identity.
SBG_SRC_REF=unknown
if [ -e "$REPO/.git" ] && SBG_SRC_REV="$(git -C "$REPO" rev-parse --verify HEAD 2>/dev/null)"; then
  SBG_SRC_REF="$(git -C "$REPO" describe --tags --exact-match "$SBG_SRC_REV" 2>/dev/null \
    || printf '%s\n' "$SBG_SRC_REV")"
  if ! SBG_SRC_STATUS="$(git -C "$REPO" status --porcelain --untracked-files=normal \
      --ignore-submodules=none 2>/dev/null)" || [ -n "$SBG_SRC_STATUS" ]; then
    SBG_SRC_REF="${SBG_SRC_REF}-dirty"
    SBG_SRC_REV=unknown
  fi
else
  SBG_SRC_REV=unknown   # source export, missing git, or no commit yet
fi

set --
if [ -z "${RIG_BUILD_PLATFORMS:-}" ] && [ -n "${SBG_BUILD_PLATFORM:-}" ]; then
  set -- --platform "$SBG_BUILD_PLATFORM"
fi
rig_build "$IMAGE" "$@" \
  --build-arg "ROS_DISTRO=${ROS_DISTRO:-lyrical}" \
  --build-arg "BASE_IMAGE=${RIG_BASE_IMAGE:-ros:${ROS_DISTRO:-lyrical}-ros-core}" \
  --build-arg "SBG_SRC_REF=${SBG_SRC_REF}" \
  --build-arg "SBG_SRC_REV=${SBG_SRC_REV}" \
  -f "$REPO/docker/Dockerfile.runtime" "$REPO"
