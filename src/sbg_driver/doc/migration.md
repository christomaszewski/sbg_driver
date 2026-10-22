# Upgrading 0.9.1 to 0.10.0

- `imu.mag_scale` now defaults to `0`, disabling `/imu/mag` until a finite positive Tesla-per-normalized-unit scale is supplied. The old default published arbitrary units in a Tesla-defined field. Calibrate the scale for the local magnetic field; `5e-5` is an approximate example, not a universal calibration.
- Delayed `SHIP_MOTION_HP` measurements publish separately on `/sbg/ship_motion_hp` (remappable with `topics.sbg_ship_motion_hp`). `/sbg/ship_motion` carries real-time measurements. Both retain the existing `sbg_msgs/ShipMotion` schema.
- IMU and odometry assembly can wait up to 20 ms for independently arriving measurements. `outputs.epoch_tolerance_us` still permits asynchronous timestamps; buffered matching avoids choosing an old sample before a closer one arrives. Duplicate and backward completed NAV epochs are ignored within a device session.
- Optional `raw/orientation` also waits up to 20 ms for a closer selected IMU sample once an IMU stream exists, using `raw_orientation.imu_tolerance_us`. It still publishes heading-only when no IMU stream is available, and preserves the heading's original timestamp.
- RTCM writes are bounded and queued independently of ROS lifecycle callbacks. A full correction queue rejects new input with a warning, and lifecycle transitions cancel queued work before releasing or reconfiguring the device.

# Upgrading 0.7.0 to 0.8.0

- `/diagnostics` now reports fleet health components named `<instance>: <component>`. Configure `health.vehicle_id` and `health.instance` (or use `sbg-up` with `VEHICLE_ID` and the deployment `name`). See [health.md](health.md) for freshness, alerts, unsupported metrics, and dashboard selection.
- Temperature thresholds are opt-in, chosen for the actual device and installation. The old unconditional operating-range thresholds no longer apply.
- Container stop uses SIGINT and invokes lifecycle shutdown before ending the ROS context. Standby continues health heartbeats; shutdown stops the reporter and releases device resources.
- `/gps/heading` is an orientation-only GNSS output. Existing `raw/orientation`, TF opt-in, frame defaults, UTC expiry, replay mounts, and Event channels keep their v0.7.0 behavior.
- Rebuild the driver image and use the matching `v0.8.0` fleet message overlay pin. There is no message wire-schema change from v0.7.0.

# Upgrading 0.6.1 to 0.7.0

- Rebuild the driver and all `sbg_msgs` consumers together. `Event` now includes `channel` (A=1 through E=5); recordings made with the previous schema require their original type definition or conversion. The fleet message overlay must use the matching `v0.7.0` pin.
- Explicitly set `tf.broadcast_odom_to_base: true` only on the node that should own that transform. The new default is `false`; valid attitude and heading are required before broadcasting.
- `sbg-up` prefixes unspecified IMU, GPS, base and odom frame IDs with the configured ROS namespace. Set `driver_params.frames` explicitly when using existing URDF mounting frames or a shared `base_link`. Direct ROS launch does not derive frame names from namespaces.
- Unknown IMU noise uses zero covariance; failed/unavailable measurements use `-1`. Consumers should distinguish these cases.
- Enable UTC_TIME at a rate comfortably above the `time.utc_timeout_s` deadline (5 seconds by default). Invalid/expired anchors fall back to receipt time. Restart recovery keeps the odometry origin while clearing measurement caches.
- A large backwards timestamp jump (>1 second) is quarantined until a second distinct timestamp advances by at most 5 seconds in the new epoch. This avoids treating a single late packet as a restart. Normal uint32 rollover and short reordering are preserved; the first candidate packet is not published. Data-only streams cannot distinguish arbitrary delayed bursts from resets without an external session identifier.
- File configurations passed to `sbg-up` resolve `connection.file.path` relative to the configuration file. The capture is mounted read-only at `/var/lib/sbg/replay.bin` inside the container.

See [CHANGELOG.md](../../../CHANGELOG.md) for the release notes and publication procedure.

# Migration from the upstream `sbg_ros2_driver`

This driver is a clean-room rewrite of SBG Systems' official
[`sbg_ros2_driver`](https://github.com/SBG-Systems/sbg_ros2_driver). It is **not**
a drop-in replacement — the topic layout, message types, parameters, and node
lifecycle all differ by design. This guide maps the old surface to the new one
so you can port a launch/config setup.

## Philosophy: standard messages first

The upstream driver is **custom-message-first**: it publishes ~22
`sbg_driver/msg/Sbg*` types (one per SBG log) plus a handful of standard
messages, all under an `imu/` namespace (`imu/nav_sat_fix`, `imu/odometry`,
`imu/velocity`, …).

This driver is **standard-message-first**: the primary outputs are
`sensor_msgs` / `nav_msgs` types on semantic topics (`/imu/data`, `/gps/fix`,
`/odom`), so `nav2` / `robot_localization` ingest them with no custom
dependencies. SBG-specific data that has no standard equivalent (health
bitmasks, raw observables, marine motion) is published as **7** `sbg_msgs`
types under `/sbg/*`. The per-log custom messages are dropped — their data now
lives in the standard messages.

## Behavioral change: this is a LifecycleNode

The upstream driver is a plain `rclcpp::Node` that starts streaming as soon as
it spins. **This driver is a `rclcpp_lifecycle::LifecycleNode`** (and a
composable component). It publishes nothing until you drive it through
`configure` → `activate`:

```bash
ros2 lifecycle set /sbg_driver configure
ros2 lifecycle set /sbg_driver activate
```

(Use a lifecycle manager — e.g. nav2's, or your own — in production.) See the
launch files for an example that auto-transitions on startup.

Stopping the `sbg_driver_node` process with SIGINT or SIGTERM (Ctrl-C, `ros2
launch` teardown, a container stop) runs the `shutdown` transition before the
process exits: the device is closed by `on_shutdown`, `transition_event`
reports `finalized`, and the last `/diagnostics` report says `shutdown`.

## Standard-message topic mapping

| Upstream topic (type) | This driver (type) | Notes |
|---|---|---|
| `imu/data` (`sensor_msgs/Imu`) | `/imu/data` (`sensor_msgs/Imu`) | + covariance from `EkfQuat`/noise params |
| `imu/temp` (`sensor_msgs/Temperature`) | `/imu/temperature` (`sensor_msgs/Temperature`) | IMU board temp |
| `imu/mag` (`sensor_msgs/MagneticField`) | `/imu/mag` (`sensor_msgs/MagneticField`) | NED/ENU aware |
| `imu/utc_ref` (`sensor_msgs/TimeReference`) | `/time_reference` (`sensor_msgs/TimeReference`) | |
| `imu/nav_sat_fix` (`sensor_msgs/NavSatFix`) | `/gps/fix` (`sensor_msgs/NavSatFix`) | raw GNSS fix (both derive from the GNSS-position log) |
| `imu/odometry` (`nav_msgs/Odometry`) | `/odom` (`nav_msgs/Odometry`) | upstream uses UTM; we use a local tangent plane anchored at first fix |
| — | `/ekf/fix` (`sensor_msgs/NavSatFix`) | **new**, opt-in: the *fused* INS geodetic position (`outputs.publish_ekf_nav_sat_fix`) |
| — | `/nmea` (`nmea_msgs/Sentence`) | **new**, opt-in: `$GPGGA` for NTRIP VRS upload (`outputs.publish_nmea_gga`) |
| `SbgGpsHdt` on `gps_hdt` | `/gps/heading` (`sensor_msgs/Imu`, orientation only) | GNSS dual-antenna true heading; yaw variance from the receiver's accuracy. Pitch, baseline and satellite counts are not in the message — baseline and accuracy are in the `gnss heading` health component |
| `/diagnostics` | `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) | fleet health components named `<instance>: <component>` — see [health.md](health.md) |

## SBG custom messages: `sbg_driver/msg/*` → `sbg_msgs/*`

The kept custom messages (no standard equivalent):

| Upstream | This driver |
|---|---|
| `SbgStatus` (status family) | `sbg_msgs/Status` on `/sbg/status` |
| `SbgEkfNav.status` (decoded) | `sbg_msgs/EkfStatus` on `/sbg/ekf_status` |
| `SbgGpsRaw` | `sbg_msgs/GpsRaw` on `/sbg/gps_raw` |
| `SbgMagCalib` | `sbg_msgs/MagCalib` on `/sbg/mag_calib` |
| `SbgShipMotion` | `sbg_msgs/ShipMotion` on `/sbg/ship_motion` |
| `SbgEvent` (A–E) | `sbg_msgs/Event` on `/sbg/event` |
| `SbgAirData` (status bits) | `sbg_msgs/AirDataStatus` on `/sbg/air_data_status` |

## Dropped per-log custom messages

These upstream `sbg_driver/msg` types are **not** republished — the data is in
the standard messages:

| Dropped upstream message | Where the data is now |
|---|---|
| `SbgImuData`, `SbgImuShort` | `/imu/data` (`sensor_msgs/Imu`) |
| `SbgEkfQuat`, `SbgEkfEuler` | `/imu/data` orientation; `/odom` pose |
| `SbgEkfNav`, `SbgEkfVelBody` | `/odom` (pose + twist); status → `/sbg/ekf_status` |
| `SbgGpsPos` | `/gps/fix` (`sensor_msgs/NavSatFix`) |
| `SbgGpsHdt` | `/raw/orientation` (`sensor_msgs/Imu`, opt-in): primary GNSS yaw/variance plus matching native accel/gyro; roll/pitch unmeasured |
| `SbgUtcTime` | `/time_reference` |
| `SbgMag` | `/imu/mag` (`sensor_msgs/MagneticField`) |

If you depend on a raw SBG log field that isn't surfaced, open an issue — the
intent is that standard messages + `/sbg/*` status cover the real needs.

## Not yet published (gaps vs. upstream)

Honest gaps — upstream publishes these; this driver does not (yet):

| Upstream topic | Status here |
|---|---|
| `imu/velocity` (`geometry_msgs/TwistStamped`, GNSS velocity) | not standalone — velocity is in `/odom` `twist`; a dedicated `/gps/vel` is a planned add |
| `imu/pres` (`sensor_msgs/FluidPressure`, air data) | not published — only `/sbg/air_data_status` (validity bits); standard `FluidPressure` is a planned add |
| `imu/pos_ecef` (`geometry_msgs/PointStamped`, ECEF) | not published — geodetic via `/gps/fix`, local Cartesian via `/odom` |
| `SbgEkfRotAccel`, `SbgOdoVel` | not surfaced as standalone topics |

## Reproducing upstream topic names

Every output topic is remappable (defaults live in
[`params/sbg_driver_params.yaml`](../params/sbg_driver_params.yaml) under
`topics:`). To make this driver advertise the old names, either set the
`topics.*` params or pass `--ros-args --remap`:

```bash
ros2 run sbg_driver sbg_driver_node --ros-args \
  --remap /imu/data:=imu/data \
  --remap /gps/fix:=imu/nav_sat_fix \
  --remap /odom:=imu/odometry \
  --remap /time_reference:=imu/utc_ref
```

Or, in a launch file, via the params file:

```yaml
sbg_driver:
  ros__parameters:
    topics:
      nav_sat_fix: "imu/nav_sat_fix"
      odom: "imu/odometry"
      time_reference: "imu/utc_ref"
```

## Parameters

Upstream uses ~50 hand-rolled `get_parameter_or<>` calls with a flat layout.
This driver uses [`generate_parameter_library`](https://github.com/PickNikRobotics/generate_parameter_library)
with a validated, nested schema — see
[`params/sbg_driver_params.yaml`](../params/sbg_driver_params.yaml). Rough map:

| Upstream | This driver |
|---|---|
| `uartConf.portName`, `uartConf.baudRate` | `transport.type: serial`, `transport.serial.{port,baud}` |
| (udp config) | `transport.type: udp`, `transport.udp.{remote_ip,in_port,out_port}` |
| (file replay) | `transport.type: file`, `transport.file.path` |
| `confWithRos` | `configure_device.enable` (+ per-section `apply` flags) |
| frame ids | `frames.{imu,gps,odom,base}` |
| ENU/NED selection | `convention.use_enu` (default `false` = sensor-native NED) |
| per-output rates | `configure_device.output.*` (per-log device-side rates: main_loop/div_N/new_data/pps/disabled) |

## Frame convention

This driver defaults to **sensor-native NED** (`convention.use_enu: false`).
Most ROS 2 fusion stacks (`nav2`, `robot_localization`) assume REP-103 **ENU**,
so set `convention.use_enu: true` for a typical ROS pipeline. The conversion is
a real data transform (orientation quaternion axis-swap, body y/z sign flips,
velocity reordering) — it is applied consistently across `/imu/data`, `/odom`,
and `/imu/mag`.

## RTCM / NTRIP (DGPS / RTK)

Identical model to upstream: both subscribe to **`rtcm_msgs/Message`** and
forward the bytes to the INS; neither embeds an NTRIP client (pair with a
third-party one, e.g. `ntrip_client`). The subscription topic here defaults to
`/rtcm` (remap to your NTRIP client's output). For VRS / network-RTK, enable
`/nmea` (`outputs.publish_nmea_gga`) so the client can upload position.

> If your INS has Ethernet, it may run its own NTRIP client or accept RTCM over
> UDP directly (configured on the unit's web UI) — in which case no ROS-side
> RTCM plumbing is needed at all.

## Running as a composable component

Unlike the upstream standalone node, this driver also registers as a component
(`sbg_driver::SbgDriverNode`) and is built with intra-process comms, so it can
share a process (zero-copy) with `robot_localization` / `nav2` in a
`component_container`.
