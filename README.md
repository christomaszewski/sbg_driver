# sbg_driver — modern ROS 2 driver for SBG Systems IMU/AHRS/INS

A clean-room replacement for the official [`sbg_ros2_driver`](https://github.com/SBG-Systems/sbg_ros2_driver),
targeting **ROS 2 Lyrical Luth** (May 2026 LTS) with **C++23**, lifecycle
management, composable components, and **standard ROS message types** on
default topics so existing nav stacks (`nav2`, `robot_localization`) ingest
the driver's outputs without remapping.

## Status

In active development. The driver is end-to-end functional inside the
provided Docker dev container — see [PROGRESS.md](PROGRESS.md) for the
execution journal (commits done, work remaining, known issues).

## Architecture

Three-package workspace:

```
sbg_driver/
├── src/
│   ├── sbg_msgs/            # 7 SBG-specific custom messages (Status, EkfStatus,
│   │                          # GpsRaw, MagCalib, ShipMotion, Event, AirDataStatus)
│   ├── sbg_driver_core/     # ROS-agnostic C++23 wrapper around sbgECom 5.6.2730
│   │                          # RAII Device, std::variant Transport, std::expected
│   │                          # Result<T>, LogView, Configurator façade
│   └── sbg_driver/          # rclcpp_lifecycle::LifecycleNode + composable
│                              # component. Converts log views to ROS messages,
│                              # publishes /tf, /diagnostics, /rtcm subscription,
│                              # /sbg/*_mag_calibration services
├── docker/                  # Dockerfile.{dev,ci,runtime} + compose + udev rules
├── .github/workflows/       # CI: image build → colcon build/test → lint → asan
└── reference/               # cloned upstream drivers (read-only design ref,
                               # gitignored)
```

### Why a core/wrapper split?

- **Testability** — `sbg_driver_core` builds without `rclcpp`, so unit tests
  link in milliseconds without any ROS infrastructure.
- **Reusability** — a modern C++ wrapper around the sbgECom C SDK has
  standalone value (CLI configurators, replay/analysis tools, etc.) outside
  the ROS ecosystem.
- **Separation of concerns** — core handles "talk to sensor, parse logs,
  manage device state"; wrapper handles "publish to ROS, lifecycle
  transitions, parameter loading, diagnostics".

## Features

### Topic graph

```
sensor_msgs/Imu             /imu/data           (covariance from EkfQuat.eulerStdDev)
sensor_msgs/Temperature     /imu/temperature
sensor_msgs/MagneticField   /imu/mag            (NED/ENU aware)
sensor_msgs/NavSatFix       /gps/fix            (covariance, RTK status)
sensor_msgs/TimeReference   /time_reference     (sensor UTC clock)
nav_msgs/Odometry           /odom               (composed from EKF Nav+Quat+VelBody)
geometry_msgs/Transform     /tf                 (odom → base_link)
diagnostic_msgs/DiagnosticArray  /diagnostics   (Device / EKF Solution / IMU Temp)

sbg_msgs/Status             /sbg/status         (general/com/aiding bitmasks)
sbg_msgs/EkfStatus          /sbg/ekf_status     (decoded solution mode + aiding)
sbg_msgs/GpsRaw             /sbg/gps_raw        (raw GNSS observables)
sbg_msgs/MagCalib           /sbg/mag_calib      (mag-cal snapshot)
sbg_msgs/ShipMotion         /sbg/ship_motion    (marine surge/sway/heave)
sbg_msgs/Event              /sbg/event          (GPIO sync-in markers)
sbg_msgs/AirDataStatus      /sbg/air_data_status

# Inputs
std_msgs/UInt8MultiArray    /rtcm  →  Device::write_rtcm  (DGPS/RTK corrections)
std_srvs/Trigger            /sbg/start_mag_calibration
std_srvs/Trigger            /sbg/save_mag_calibration
```

All topic names are **relative** — they pick up the node's namespace at
launch. Override individually via `topics.<name>` params.

### Frame convention

Defaults to **sensor-native NED** to match what the hardware actually emits.
Most ROS users (anything fusing through `robot_localization` or `nav2`)
should set `convention.use_enu: true` to get REP-103-compliant ENU output.
The conversion sign-flips Y/Z on body-frame triplets (accel, gyro, body
velocity, mag) and rotates the orientation quaternion accordingly.

### Modern C++23 features used

- `std::expected<T, Error>` for error propagation (no exceptions on the hot path)
- `std::variant` for the Transport config (Serial / Udp / FileReplay)
- `std::jthread` + `std::stop_token` for the I/O thread
- `std::span` for buffer passing in `write_rtcm`
- `std::chrono::year_month_day` + `sys_days` for UTC time composition
- Designated initializers, `[[nodiscard]]`, `[[maybe_unused]]`,
  `constexpr` lookup tables
- `use_intra_process_comms(true)` + `std::unique_ptr<MessageT>` publishing
  for zero-copy intra-process delivery to co-tenant nodes

## Quick start

### Dev container

```bash
docker compose -f docker/compose/compose.dev.yaml up -d
docker compose -f docker/compose/compose.dev.yaml exec dev bash
```

Or via VS Code: `F1` → "Dev Containers: Reopen in Container".

### Build + test

```bash
# inside the container
cd /workspace
colcon build --base-paths src
source install/setup.bash
colcon test --base-paths src --return-code-on-test-failure
```

### Run with a captured `.bin` file

```bash
ros2 launch sbg_driver replay.launch.py bag:=/path/to/sample.bin
# topics visible immediately after `configure → activate` transitions
ros2 topic list
ros2 topic echo /imu/data
```

### Run against a real device

```bash
# example config files at install/sbg_driver/share/sbg_driver/config/
ros2 run sbg_driver sbg_driver_node --ros-args \
  --params-file install/sbg_driver/share/sbg_driver/config/serial.example.yaml

# drive the lifecycle
ros2 lifecycle set /sbg_driver configure
ros2 lifecycle set /sbg_driver activate
```

### Mag calibration workflow

```bash
ros2 service call /sbg/start_mag_calibration std_srvs/srv/Trigger
# drive the vehicle through full 3D rotation for ~30 seconds
ros2 service call /sbg/save_mag_calibration std_srvs/srv/Trigger
# device reboots; reconnect:
ros2 lifecycle set /sbg_driver deactivate
ros2 lifecycle set /sbg_driver activate
```

### Hardware-in-the-loop

```bash
# one-time host setup for stable /dev/sbg_imu symlink
sudo cp docker/udev/99-sbg.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

docker compose -f docker/compose/compose.hil.yaml up
```

## Configuration

All parameters declared via [`generate_parameter_library`](src/sbg_driver/params/sbg_driver_params.yaml)
— typed, validated, dynamically reconfigurable. Key groups:

- `transport.{type, serial, udp, file}` — connection method + per-method options
- `frames.{imu, gps, base, odom, map, time_reference}` — TF frame IDs
- `convention.use_enu` — sensor-native NED (default) vs REP-103 ENU
- `topics.<name>` — remap every output topic individually
- `tf.broadcast_{odom_to_base, map_to_odom, base_to_imu}` — TF policy

Two example configs ship at install time:
- [`config/replay.example.yaml`](src/sbg_driver/config/replay.example.yaml)
- [`config/serial.example.yaml`](src/sbg_driver/config/serial.example.yaml)

## Migration from the upstream `sbg_ros2_driver`

Topic-by-topic mapping table lives in [`src/sbg_driver/doc/migration.md`](src/sbg_driver/doc/migration.md)
(populated as part of Phase 3j). Short version:

| Upstream `sbg_driver/*`            | This driver                              |
|------------------------------------|------------------------------------------|
| `SbgImuData` on `imu_data`         | `sensor_msgs/Imu` on `/imu/data`         |
| `SbgGpsPos` on `gps_pos`           | `sensor_msgs/NavSatFix` on `/gps/fix`    |
| `SbgMag` on `mag`                  | `sensor_msgs/MagneticField` on `/imu/mag`|
| `SbgEkfNav`+`SbgEkfQuat`+`SbgEkfVelBody`  | `nav_msgs/Odometry` on `/odom`    |
| `SbgStatus` family                 | `sbg_msgs/Status` on `/sbg/status`       |

(Topic names are remappable; the old names can be reproduced with `--remap`.)

## sbgECom SDK integration

CMake `FetchContent` pulls the SDK from upstream at a pinned tag (currently
`5.6.2730-stable`). The headers are installed alongside `sbg_driver_core`
so the export config resolves at install time. To swap to a different tag:

```bash
colcon build --cmake-args -DSBG_DRIVER_SBGECOM_GIT_TAG=5.x.x-stable
```

Distro packagers can use a system-installed sbgECom instead with
`-DSBG_DRIVER_USE_SYSTEM_SBGECOM=ON`.

## License

[Apache License 2.0](LICENSE). The upstream `sbg_ros2_driver` is MIT —
we use Apache 2.0 because it's the ROS 2 ecosystem standard, integrates
cleanly with `ament_copyright`, and the two licenses are practically
equivalent for permissive open-source use.
