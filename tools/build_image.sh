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
#
# A plain-HTTP local registry needs the daemon's insecure-registries on this host (with the :port).
# Cross-arch vehicle: export SBG_BUILD_PLATFORM=linux/<arch> (default: host-native).
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"

REGISTRY="${1:?usage: build_image.sh <registry> [tag]}"
TAG="${2:-latest}"
IMAGE="${REGISTRY}/sbg_driver:${TAG}"

# fleet-msgs provenance REF (rigging.yaml msgs.source + docker/provenance-record.sh): the build
# context's ACTUAL git identity — exact-match tag when HEAD is a release, else the commit SHA,
# -dirty appended when the tree is unclean. Computed here because the context ships without .git
# (.dockerignore). rig image audit compares this against rigging.yaml's declared msgs ref, so a
# build from an untagged or dirty tree shows up as the pin skew it is.
if SBG_SRC_REF="$(git -C "$REPO" describe --tags --exact-match --dirty 2>/dev/null)"; then
  :
elif SBG_SRC_REF="$(git -C "$REPO" rev-parse HEAD 2>/dev/null)"; then
  git -C "$REPO" diff-index --quiet HEAD -- || SBG_SRC_REF="${SBG_SRC_REF}-dirty"
else
  SBG_SRC_REF="unknown"   # not a git checkout (tarball export) — same marker the doctrine uses
fi

docker build ${SBG_BUILD_PLATFORM:+--platform "${SBG_BUILD_PLATFORM}"} \
  ${RIG_BUILD_NO_CACHE:+--no-cache} ${RIG_BUILD_NO_CACHE:+--pull} \
  --build-arg "ROS_DISTRO=${ROS_DISTRO:-lyrical}" \
  --build-arg "BASE_IMAGE=${RIG_BASE_IMAGE:-ros:${ROS_DISTRO:-lyrical}-ros-core}" \
  --build-arg "SBG_SRC_REF=${SBG_SRC_REF}" \
  -f "$REPO/docker/Dockerfile.runtime" -t "$IMAGE" "$REPO"
docker push "$IMAGE"
echo "build_image.sh: pushed $IMAGE" >&2
