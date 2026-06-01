# Docker images for sbg_driver

Three images. Local repro of CI and a production-ready runtime, all sharing the
same ROS 2 Lyrical base.

| Image | What | When to use |
|---|---|---|
| `Dockerfile.dev` | Full dev environment: compilers, linters, sanitizers, debuggers, ROS desktop. Non-root user with `dialout`/`sudo`. | Day-to-day development. Also drives `.devcontainer.json` for VS Code "Reopen in Container". |
| `Dockerfile.ci` | Slimmed CI runner: same toolchain, runs as root, no editors/docs. | Pulled by `.github/workflows/ci.yml`. Pushed to GHCR on `main`. |
| `Dockerfile.runtime` | Multi-stage build. ~150 MB final image with the driver's `install/` overlaid on `ros:lyrical-ros-core`. No compilers. | Production deployment, HIL testing via `compose.hil.yaml`. |

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
(the single source of truth, uniform across all rig services):

```yaml
# sbg_front.yaml
service: sbg
name: sbg_front
connection:                  # exactly one sub-block matching `type`
  type: tcp                  # serial | tcp | udp | file
  tcp: { host: 192.168.1.10, port: 3000 }
ros: { namespace: sbg_front }   # default == name
driver_params:               # OPAQUE -> passed verbatim into ros__parameters (minus transport)
  frames: { data: sbg_link }
```

Bring it up with the launcher (it *selects + parameterizes* static compose files — never generates one):

```bash
./sbg-up sbg_front.yaml up -d     # detached
./sbg-up sbg_front.yaml status    # docker compose ps
./sbg-up sbg_front.yaml logs -f
./sbg-up sbg_front.yaml config    # render the merged compose (no run)
./sbg-up sbg_front.yaml down
```

Each sensor becomes its own compose project (`sbg_<name>`) under ROS namespace
`/<name>`, so multiple instances never collide. Needs the Docker Compose v2 plugin and host PyYAML
(`apt install python3-yaml`). "Not live" is just a config choice (`connection.type: file`) — no
separate launcher flag.

| File | Role |
|------|------|
| `sbg-up` | Per-sensor launcher (verbs up/down/status/logs/config; forwards extra args to compose). |
| `tools/render_params.py` | Generic config -> this driver's ROS 2 params (`/**:`-keyed); `--env` emits the instance identity. |
| `docker/compose/compose.deploy.yaml` | Deployment compose: host net/ipc, params bind-mount, `bringup.launch.py` + namespace. |
| `docker/compose/compose.deploy.serial.yaml` | Serial overlay (adds `--device` + `dialout`); added automatically for `connection.type: serial`. |
| `deploy.yaml` | rig descriptor: service / launcher / verb map / ros_distro (metadata only). |

Publish the runtime image where the deploy compose expects it (override with `SBG_IMAGE`):

```bash
docker build -f docker/Dockerfile.runtime -t ghcr.io/your-org/sbg_driver:latest .
docker push ghcr.io/your-org/sbg_driver:latest
```

rig (and the fleet) export `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION`; the defaults (`0` /
`rmw_fastrtps_cpp`) are also correct standalone.

