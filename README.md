# sbg_driver — modern ROS 2 driver for SBG Systems IMU/AHRS/INS

Release **0.7.0** adds GNSS heading and sensor-output reliability fixes. See [release notes](CHANGELOG.md) and
[upgrade instructions](src/sbg_driver/doc/migration.md#upgrading-061-to-070), especially
TF ownership, namespaced frame defaults, and the updated `sbg_msgs/Event` schema.


[![CI](https://github.com/christomaszewski/sbg_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/christomaszewski/sbg_driver/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

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
│   ├── sbg_driver_core/     # ROS-agnostic C++23 wrapper around sbgECom 5.8.935
│   │                          # RAII Device, std::variant Transport, std::expected
│   │                          # Result<T>, LogView, Configurator façade
│   └── sbg_driver/          # rclcpp_lifecycle::LifecycleNode + composable
│                              # component. Converts log views to ROS messages,
│                              # publishes /tf, /diagnostics, /rtcm subscription,
│                              # /sbg/*_mag_calibration services
├── docker/                  # Dockerfile.{dev,ci,runtime} + compose + udev rules
├── .github/workflows/       # CI: image build → build-test → lint → asan → tsan
├── reference.repos          # vcstool pin of the upstream SBG drivers (design ref)
└── reference/               # populated via `vcs import reference < reference.repos`
                               # (gitignored + COLCON_IGNORE; not a build dep)
```

> The `reference/` upstream drivers are a read-only design/correctness
> reference, not dependencies. Populate them reproducibly with
> `vcs import reference < reference.repos`.

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
sensor_msgs/Imu             /imu/data           (IMU_SHORT when the device streams it, else IMU_DATA — see imu.source; orientation + covariance from EkfQuat)
sensor_msgs/Imu             /raw/orientation    (GPS1_HDT yaw + matching native accel/gyro; opt-in, see outputs.publish_raw_orientation)
sensor_msgs/Temperature     /imu/temperature
sensor_msgs/MagneticField   /imu/mag            (NED/ENU aware)
sensor_msgs/NavSatFix       /gps/fix            (raw GNSS fix; covariance, RTK status)
sensor_msgs/NavSatFix       /ekf/fix            (fused INS position; opt-in, see outputs.publish_ekf_nav_sat_fix)
nmea_msgs/Sentence          /nmea               ($GPGGA @ ~1 Hz; opt-in, for NTRIP VRS upload — outputs.publish_nmea_gga)
sensor_msgs/TimeReference   /time_reference     (sensor UTC clock)
nav_msgs/Odometry           /odom               (composed from EKF Nav+Quat+VelBody)
geometry_msgs/Transform     /tf                 (opt-in; odom → base_link)
diagnostic_msgs/DiagnosticArray  /diagnostics   (Device / EKF Solution / IMU Temp)

sbg_msgs/Status             /sbg/status         (general/com/aiding bitmasks)
sbg_msgs/EkfStatus          /sbg/ekf_status     (decoded solution mode + aiding)
sbg_msgs/GpsRaw             /sbg/gps_raw        (raw GNSS observables)
sbg_msgs/MagCalib           /sbg/mag_calib      (mag-cal snapshot)
sbg_msgs/ShipMotion         /sbg/ship_motion    (marine surge/sway/heave)
sbg_msgs/Event              /sbg/event          (GPIO sync-in markers)
sbg_msgs/AirDataStatus      /sbg/air_data_status

# Inputs
rtcm_msgs/Message           /rtcm  →  Device::write_rtcm  (RTCM3 from an NTRIP client; DGPS/RTK)
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

### GPS heading with native IMU measurements

Enable `outputs.publish_raw_orientation: true` to publish `sensor_msgs/Imu`
on `raw/orientation` (remappable with `topics.raw_orientation`). This carries
**primary-receiver GPS1_HDT heading**, independently of the fused EKF orientation
on `imu/data`. A dual-antenna GNSS heading solution and GPS1_HDT device output
are required. This is true heading, not course over ground.

```yaml
sbg_driver:
  ros__parameters:
    outputs.publish_raw_orientation: true
    topics.raw_orientation: raw/orientation
    raw_orientation.imu_tolerance_us: 10000
    raw_orientation.heading_offset_deg: 0.0
    convention.use_enu: true
```

The message is published immediately for each new GNSS heading timestamp, at
the **GNSS heading rate**. Accelerometer and gyro values come from the nearest
already-received sample selected by `imu.source`, within
`raw_orientation.imu_tolerance_us` (default ±10 ms, zero for exact matching).
A history of up to 2048 IMU samples, retained for at most two seconds of receipt
time, handles GNSS logs arriving after newer IMU samples. If the matching IMU
has not arrived yet, or lies outside that window, the heading is still published
with acceleration and angular velocity marked unavailable. A later IMU does
not cause the heading to be republished. Duplicate and older GNSS epochs are
ignored; lifecycle reactivation clears history. `imu/data` remains the full
IMU-rate stream.

`header.stamp` follows `time.source` for the **heading log**. Matching always
uses device timestamps, even under `receive_time`. Use `device_utc` with a
valid UTC_TIME stream when consumers need acquisition timestamps. An attached
IMU sample can differ by the configured tolerance; the message has one header,
so it cannot expose both acquisition times. No gyro integration, interpolation,
EKF roll/pitch, gravity removal or driver-side attitude fusion is applied.

Only yaw is measured in the quaternion. Roll and pitch are zero placeholders
with variance `1e6`; yaw variance is the GNSS heading accuracy squared after
conversion to radians. GPS status and accuracy must both be valid. Invalid
heading uses `orientation_covariance[0] = -1`. Missing, nonfinite or unhealthy
IMU data uses `-1` in the affected covariance's first element. Healthy accel/gyro
with unknown noise has all-zero covariance; configured noise supplies its
diagonal. These distinguish unavailable measurements from unknown uncertainty.

The message uses `frames.imu`: the GPS bearing must describe that frame's
forward X axis. `raw_orientation.heading_offset_deg` adds a clockwise planar
correction **before** NED/ENU conversion. For example, if the reported antenna
bearing points 90° clockwise from IMU forward, use `-90`. Check whether your
device already compensates its antenna alignment before adding a correction.
This offset does not implement a full 3D mounting transform. GNSS antenna pitch
is deliberately not treated as IMU-body pitch. Consumers should use this
orientation as a yaw observation and respect covariance.

For ELLIPSE devices provisioned by this driver, the additional device output
setting is `configure_device.output.gps_hdt: new_data` (requires the existing
`configure_device.enable` and `configure_device.output.apply` switches).
For High Performance INS, enable GPS1_HDT using the device configuration UI/API,
as with the other output logs. The new ROS output switch alone does not change
device configuration.

For the dashboard map, set `orientation_topic: /<namespace>/raw/orientation`
and `orientation_frame: enu` for the example above. With the driver's default
`convention.use_enu: false`, use `orientation_frame: ned`. A GNSS invalid-state
message makes the current map display its plain marker; if the topic stops
entirely, the dashboard currently retains the last heading until its topic state
changes, so it still needs heading freshness handling for that case.

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

`save_mag_calibration` asks the device to compute the calibration and inspects
its verdict before uploading anything. If the device reports `quality=invalid`
— the usual outcome of a short sweep or one taken near magnetic interference —
the service returns `success: false`, uploads nothing, writes nothing to NVRAM,
and leaves the previous calibration in place; re-run `start_mag_calibration`
and cover more of the rotation range. A `quality=poor` result is accepted (you
asked to save it) but logged as a warning. Either way the response `message`
carries the full summary — quality, confidence, point count and the resulting
1σ/max heading error in degrees — so the trade-off is explicit:

```
quality=good confidence=high points=284/1000 advanced_status=0x0000 \
heading_err(mean/std/max)=0.41/0.19/1.02deg
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
— typed and validated. Parameters are read when the lifecycle node **configures**
(`configure_device.*` again on each activate): set them in the params file or
launch overrides, or change them while the node is inactive and recycle
`cleanup → configure`. A `ros2 param set` on a configured node is accepted by
the parameter server but takes no effect until that recycle (`transport.*` is
`read_only` — fixed once first configured). Key groups:

- `transport.{type, serial, udp, file}` — connection method + per-method options
- `frames.{imu, gps, base, odom, time_reference}` — TF frame IDs
- `convention.use_enu` — sensor-native NED (default) vs REP-103 ENU
- `time.source` — `receive_time` (default) stamps outputs with the host clock
  at dispatch; `device_utc` stamps each log from its device timestamp mapped
  through the INS UTC clock. Use `device_utc` when the host (and other
  sensors) sync to the INS's NTP/PTP server: stamps become jitter-free while
  staying in the system timebase, and replayed captures keep their original
  recording times. Requires the `UTC_TIME` log; falls back to receive time
  until the first GNSS-synced UTC log arrives.
- `topics.<name>` — remap every output topic individually
- `imu.{sensor_model, *_noise_stddev, mag_scale}` — covariance defaults + mag units
- `configure_device.*` — optional device provisioning at activation (RAM only;
  the `save_settings` service persists). Uses the legacy ELLIPSE command set,
  so it is **ELLIPSE-only**: the driver identifies the product at activation
  (`SBG_ECOM_CMD_INFO`) and skips provisioning with a warning on High
  Performance INS units (QUANTA / EKINOX / APOGEE / NAVSIGHT) — configure
  those on the device itself (web UI / [sbgInsRestApi]).
- `outputs.epoch_tolerance_us` — device-timestamp window for composing
  `/imu/data` orientation and the `/odom` triple. Default `-1` auto-resolves
  at activation: exact match on ELLIPSE (same-tick logs share a timestamp),
  ±10 ms on High Performance INS, whose IMU clock is asynchronous to the INS
  main loop so exact matches never occur.
- `time.utc_timeout_s` — maximum UTC-anchor age in steady seconds (default 5); expiry or invalid UTC switches to receipt time until lock returns.
- `tf.broadcast_odom_to_base` — opt-in TF policy, default false (map→odom belongs to your
  localization stack; base→imu mount geometry belongs in the URDF)

[sbgInsRestApi]: https://developer.sbg-systems.com/sbgInsRestApi/

The full generated reference lives at
[`src/sbg_driver/doc/parameters.md`](src/sbg_driver/doc/parameters.md).

Two example configs ship at install time:
- [`config/replay.example.yaml`](src/sbg_driver/config/replay.example.yaml)
- [`config/serial.example.yaml`](src/sbg_driver/config/serial.example.yaml)

## Migration from the upstream `sbg_ros2_driver`

Full guide — topic/type/param mapping, `--remap` snippets, lifecycle notes, and
honest gaps vs. upstream — in
[`src/sbg_driver/doc/migration.md`](src/sbg_driver/doc/migration.md). Short version:

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
`5.8.935-stable`). The headers are installed alongside `sbg_driver_core`
so the export config resolves at install time. To swap to a different tag:

```bash
colcon build --cmake-args -DSBG_DRIVER_SBGECOM_GIT_TAG=5.x.x-stable
```

Distro packagers can use a system-installed sbgECom instead with
`-DSBG_DRIVER_USE_SYSTEM_SBGECOM=ON`.

### Where the SDK is (and isn't) visible

The SDK boundary is deliberately **asymmetric** — abstract the control plane,
pass through the data plane:

- **Control plane — fully abstracted, no SDK leak.** Errors (`sbg::Error`),
  transport (`sbg::transport::{Serial,Udp,FileReplay}`), and all `Configurator`
  commands (`sbg::MotionProfile`, `PortAssignment`, `ImuAlignment`, …) use our
  own types, mapped to/from sbgECom at the boundary. These public headers pull
  in **no** SDK headers.
- **Data plane — intentional zero-copy pass-through.** `sbg::LogView`'s
  accessors (`as_imu_data()`, `as_ekf_nav()`, …) return **raw `const SbgEComLog*`
  structs**, so `log_view.hpp` includes the SDK headers and `sbgECom` is linked
  `PUBLIC`. The wrapper reads log fields directly — no re-wrapping of ~18
  multi-field log structs, no per-field copy on the hot path.

Consequence: a translation unit that reads log payloads (e.g. the conversion
layer, or downstream user code) depends on the sbgECom headers; everything else
stays SDK-agnostic. If you need to read a log field without the SDK dependency,
forward-declare `LogView` and keep the include out of that TU. This is a
conscious trade-off, not an accident — see the note in `sbg_driver_core/CMakeLists.txt`.

## License

[Apache License 2.0](LICENSE). The upstream `sbg_ros2_driver` is MIT —
we use Apache 2.0 because it's the ROS 2 ecosystem standard, integrates
cleanly with `ament_copyright`, and the two licenses are practically
equivalent for permissive open-source use.
