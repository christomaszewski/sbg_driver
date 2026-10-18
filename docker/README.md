# Docker images for sbg_driver

Three images. Local repro of CI and a production-ready runtime, all sharing the
same ROS 2 Lyrical base.

| Image | What | When to use |
|---|---|---|
| `Dockerfile.dev` | Full dev environment: compilers, linters, sanitizers, debuggers, ROS desktop. Non-root user with `dialout`/`sudo`. | Day-to-day development. Also drives `.devcontainer.json` for VS Code "Reopen in Container". |
| `Dockerfile.ci` | Slimmed CI runner: same toolchain, runs as root, no editors/docs. | Pulled by `.github/workflows/ci.yml`. Pushed to GHCR on `main`. |
| `Dockerfile.runtime` | Multi-stage build. ~150 MB final image with the driver's `install/` overlaid on the runtime parent (`BASE_IMAGE` build-arg: `ros:lyrical-ros-core` standalone; under rig, the fleet's shared `fleet-ros` base via `RIG_BASE_IMAGE`). No compilers. | Production deployment, HIL testing via `compose.hil.yaml`. |

## Dev container quickstart

```bash
# Option A: docker compose
docker compose -f docker/compose/compose.dev.yaml up -d
docker compose -f docker/compose/compose.dev.yaml exec dev bash

# Option B: VS Code devcontainer
code .
# F1 → "Dev Containers: Reopen in Container"
```

Inside the container:

```bash
cd /workspace
colcon build --symlink-install --cmake-args --preset dev
source install/setup.bash
colcon test
```

## Replay setup

Read a captured `.bin` log file, publish to ROS, visualize in rviz2:

```bash
docker compose -f docker/compose/compose.replay.yaml up
```

## Hardware-in-the-loop

Driver against a real SBG device:

```bash
# One-time host setup:
sudo cp docker/udev/99-sbg.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Plug in the SBG, verify: ls -l /dev/sbg_imu

docker compose -f docker/compose/compose.hil.yaml up
```

## Building images manually

```bash
# Dev (with current user's UID/GID baked in):
docker build -t sbg_driver:dev \
  --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
  -f docker/Dockerfile.dev docker/

# CI (what GitHub Actions pulls):
docker build -t sbg_driver:ci -f docker/Dockerfile.ci docker/

# Runtime (multi-stage; build from repo root):
docker build -t sbg_driver:runtime -f docker/Dockerfile.runtime .
```

## Deployment / `rig` integration

This driver plugs into the vehicle-level `rig` orchestrator as a first-class service — one-way: the
driver never depends on or knows about rig. Per-sensor deployment is driven by one generic config
(the single source of truth, uniform across all rig services; start from
`sensors/sbg.example.yaml`):

```yaml
# sensors/sbg_front.yaml
service: sbg
name: front
connection:                  # exactly one sub-block matching `type` (serial | udp | file)
  type: serial
  serial: { by_id: /dev/serial/by-id/usb-YOUR_DEVICE-if00, baud: 921600 }
ros: { namespace: front }    # default == name
health: { stale_after_s: 5 } # optional fleet-health settings (see sensors/sbg.example.yaml)
driver_params: {}            # OPAQUE -> passed verbatim into ros__parameters (minus transport)
```

Bring it up with the launcher (it *selects + parameterizes* static compose files — never generates one):

```bash
./sbg-up sensors/sbg_front.yaml up -d     # detached
./sbg-up sensors/sbg_front.yaml status    # docker compose ps
./sbg-up sensors/sbg_front.yaml logs -f
./sbg-up sensors/sbg_front.yaml config    # render the merged compose (no run)
./sbg-up sensors/sbg_front.yaml down
```

Each sensor becomes its own compose project (the rig-injected `COMPOSE_PROJECT_NAME`, or
`sbg_<name>` standalone) under ROS namespace `/<name>`, so multiple instances never
collide. Needs the Docker Compose v2 plugin and host PyYAML (`apt install python3-yaml`). "Not
live" is just a config choice (`connection.type: file`) — no separate launcher flag.

| File | Role |
|------|------|
| `sbg-up` | Per-sensor launcher (verbs up/down/status/logs/config; forwards extra args to compose). |
| `tools/render_params.py` | Generic config -> this driver's ROS 2 params (`/**:`-keyed), including fleet health identity + the `health:` block; `--env` emits the instance identity. |
| `tools/build_image.sh` | Build + push the runtime image: `build_image.sh <registry> [tag]` (rig's `build:` entrypoint; forwards `RIG_BUILD_NO_CACHE`/`ROS_DISTRO`/`RIG_BASE_IMAGE` from the rig build env). |
| `sensors/sbg.example.yaml` | Example sensor config (copy + edit per instance; CI certifies against it). |
| `docker/compose/compose.deploy.yaml` | Deployment compose: host net/ipc, params bind-mount, `bringup.launch.py` + namespace. |
| `docker/compose/compose.deploy.serial.yaml` | Serial overlay (adds `--device` + `dialout`); added automatically for `connection.type: serial`. |
| `rigging.yaml` | rig descriptor: service / launcher / verb map / build phase / launch surface (metadata only). |

Publish the runtime image to your registry (`ghcr.io/your-org`):

```bash
tools/build_image.sh ghcr.io/your-org            # build + push :latest
tools/build_image.sh ghcr.io/your-org v1.2.3     # or a pinned tag (rig passes the fleet tag)
```

The build script records the source identity for `rig image audit` in
`/opt/fleet-msgs/provenance.yaml`. Both architectures retain the same source identity.
It passes both the source ref (an exact tag, otherwise the
commit SHA) and the full commit SHA into Docker before `.git` is excluded from the context.
Clean builds can therefore be compared with the fleet message overlay at the SHA level.
Tracked edits or untracked, non-ignored files append `-dirty` to the ref and record
`rev: unknown`; source exports without their own Git metadata record both as `unknown`.
Ignored build outputs do not mark the checkout dirty. Keep the source tree unchanged during
the build so the captured identity describes the context Docker receives.

Use `tools/build_image.sh` (also invoked by `rig build`) for automatic provenance. A direct
`docker build` without `SBG_SRC_REF` and `SBG_SRC_REV` build arguments records unknown values;
only supply a SHA manually when it identifies the clean source tree actually being built.

With Rig v0.2.61 or later, `rig build` publishes one `sbg_driver` image index for
`linux/arm64` and `linux/amd64`. This CPU driver needs no `jp6m`, `jp7`, or `dev`
runtime variants. Use `rig build --arch arm64 --tag <new-tag>` (or `amd64`) for a
faster single-architecture build; use a new tag when narrowing an existing multiarch
release. CI compiles and checks the runtime image on both native architectures.

Standalone builds remain host-native. Set `SBG_BUILD_PLATFORM=linux/<arch>` for
one architecture, or publish both with:

```bash
RIG_BUILD_PLATFORMS=linux/arm64,linux/amd64 tools/build_image.sh ghcr.io/your-org mytag
```

Multiarch publication needs a Docker Buildx builder supporting both architectures
and registry push access. `RIG_BUILD_PLATFORMS` takes precedence over
`SBG_BUILD_PLATFORM`. Both build paths honor `RIG_BUILD_NO_CACHE=1` with
`--no-cache --pull`; `RIG_BUILD_RESULT` receives the published image/index digest.

At deploy time the compose resolves the image as `SBG_IMAGE` (full per-service override)
-> `RIG_IMAGE_REGISTRY`-prefixed `sbg_driver:${RIG_IMAGE_TAG:-latest}` (rig injects both from
fleet policy; `rig build` pushes the same ref) -> bare local `sbg_driver:latest`. Point
`RIG_IMAGE_REGISTRY` at wherever you pushed.

The launcher contract (project name honored, registry/tag agreement, fleet ROS env pass-through,
deterministic config output) is executable: `rig certify --repo . --config
sensors/sbg.example.yaml --require-build-check` — CI runs it on every push, including
an isolated probe that checks the build script's no-cache behavior.

Ordinary Rig bag replay republishes recorded SBG topics through the bag player;
it does not start this sensor-tier driver by default. Explicitly selecting the
live driver still requires its configured sensor connection. Raw SBG file replay
remains a separate driver workflow.

rig (and the fleet) export `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION`; the defaults (`0` /
`rmw_fastrtps_cpp`) are also correct standalone.

### Fleet health identity

The node publishes its health on `/diagnostics` (see `src/sbg_driver/doc/health.md`). The launcher
gives it the fleet identity the dashboard groups by: `health.instance` is the config's `name` and
`health.vehicle_id` is the `VEHICLE_ID` environment variable, both rendered into the params file by
`tools/render_params.py` (rig exports `VEHICLE_ID`; for a standalone run, `VEHICLE_ID=uav6 ./sbg-up
...`). The optional `health:` block of the sensor config is validated at the same point, before a
container is started.

For the dashboard to see the diagnostics the node has to be on the vehicle's Zenoh router:
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`, with the router (`rmw_zenohd`, not this container's job)
reachable at the session default `tcp/localhost:7447` — or wherever `ZENOH_CONFIG_OVERRIDE` points
the session (e.g. `connect/endpoints=["tcp/192.168.1.1:7447"]`; the deploy compose passes it
through).

