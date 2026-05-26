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
