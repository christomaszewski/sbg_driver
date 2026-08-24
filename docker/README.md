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
| `tools/render_params.py` | Generic config -> this driver's ROS 2 params (`/**:`-keyed); `--env` emits the instance identity. |
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

At deploy time the compose resolves the image as `SBG_IMAGE` (full per-service override)
-> `RIG_IMAGE_REGISTRY`-prefixed `sbg_driver:${RIG_IMAGE_TAG:-latest}` (rig injects both from
fleet policy; `rig build` pushes the same ref) -> bare local `sbg_driver:latest`. Point
`RIG_IMAGE_REGISTRY` at wherever you pushed.

The launcher contract (project name honored, registry/tag agreement, fleet ROS env pass-through,
deterministic config output) is executable: `rig certify --repo . --config
sensors/sbg.example.yaml` — CI runs it on every push.

rig (and the fleet) export `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION`; the defaults (`0` /
`rmw_fastrtps_cpp`) are also correct standalone.

