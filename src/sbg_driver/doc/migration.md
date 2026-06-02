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
| `SbgEkfRotAccel`, `SbgOdoVel`, `SbgGpsHdt` | not surfaced as standalone topics |

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
| frame ids | `frames.{imu,gps,odom,base,map}` |
| ENU/NED selection | `convention.use_enu` (default `false` = sensor-native NED) |
| per-output rates | `configure_device.*` for device-side config (output-rate config: see backlog) |

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
