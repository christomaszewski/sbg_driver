# sbg_driver — modern ROS2 driver for SBG Systems IMU/AHRS/INS

A clean, modern replacement for the official [`sbg_ros2_driver`](https://github.com/SBG-Systems/sbg_ros2_driver),
targeting **ROS 2 Lyrical Luth** (May 2026 LTS) with **C++23**, lifecycle management, composable
components, and **standard ROS message types** on default topics (`sensor_msgs/Imu`,
`sensor_msgs/NavSatFix`, `nav_msgs/Odometry`, …) so existing nav stacks just work.

## Workspace layout

```
sbg_driver/
├── src/
│   ├── sbg_msgs/            # 7 SBG-specific custom messages (Status, EkfStatus, GpsRaw, ...)
│   ├── sbg_driver_core/     # ROS-agnostic C++23 wrapper around the sbgECom C SDK
│   └── sbg_driver/          # LifecycleNode + composable component (rclcpp)
├── docker/                  # Dockerfile.{dev,ci,runtime} + compose setups + udev rules
├── .github/workflows/       # CI: build/test/lint/asan/coverage inside Dockerfile.ci
└── reference/               # cloned upstream drivers (read-only design reference, gitignored)
```

## Status

Phase 0: skeleton + build plumbing. See [the plan](../../.claude/plans/iridescent-singing-patterson.md)
for the full design and implementation phasing.

## Quick start (once phase 2+ lands)

```bash
# Dev container (recommended)
docker compose -f docker/compose/compose.dev.yaml up -d
docker compose -f docker/compose/compose.dev.yaml exec dev bash
# inside container:
colcon build
ros2 launch sbg_driver replay.launch.py bag:=data/replay/sample.bin
```

## License

Apache License 2.0 — see [LICENSE](LICENSE). Note: this differs from the
upstream `sbg_ros2_driver` (MIT) — Apache 2.0 is the ROS 2 ecosystem
standard and integrates cleanly with `ament_copyright` linting.
