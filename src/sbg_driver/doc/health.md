# Fleet health on `/diagnostics`

The driver reports its own and the device's health in the form the vehicle
dashboard's Health tab and Home health widget consume. The contract is the
dashboard's `docs/SERVICE_HEALTH.md`
([canonical copy](https://github.com/christomaszewski/dashboard/blob/feat/health/docs/SERVICE_HEALTH.md));
this page says how this driver fills it in.

- **Transport:** standard `diagnostic_msgs/msg/DiagnosticArray` on the absolute
  topic `/diagnostics`, reliable + volatile, `header.stamp` from the system
  clock. No custom message, no native Zenoh keys: the dashboard finds the topic
  through its rmw_zenoh graph (see [Reaching the dashboard](#reaching-the-dashboard)).
- **One reporter per instance:** every publication carries the instance's
  complete component inventory under one `health.publisher_id`.
- **Observational:** nothing here sends a request to the device or changes its
  configuration. Every device observation is a log the unit already streams.

## Identity

Statuses are named `<instance>: <component>` and each carries

| Value | Source |
|---|---|
| `health.vehicle_id` | `health.vehicle_id` param; `sbg-up` renders it from the `VEHICLE_ID` environment variable (rig exports it). Falls back to `VEHICLE_ID` in the node's own environment. |
| `health.instance` | `health.instance` param; `sbg-up` renders it from the sensor config's `name`. |
| `health.service` | `sbg` (the `service:` of `rigging.yaml`). |

The instance is the deployment name, not the ROS namespace: `name: ins` with
`ros.namespace: front` reports as `ins`. Only a node started without the
launcher falls back to its namespace (or node name) for the instance, and says
so in its log. With no vehicle ID at all the statuses are still published; a
dashboard lists them under the ROS domain instead of a vehicle.

## Components

| Component | Observed from | What it says |
|---|---|---|
| `service` | the process itself | Operational state (`state`: `unconfigured`, `standby`, `transitioning`, `active`, `shutdown`), `transport`, `callback_exceptions`. |
| `stream` | arrival of any device log | Whether logs are flowing while active: `logs.received`, `frame_age_s`, plus the composition counters `composition_drops`, `imu_orientation_misses`, `imu_source`. |
| `device` | `SBG_ECOM_LOG_STATUS` | `uptime_s`, `cpu.usage_pct`, the raw status words (`device.general_status`, `device.com_status`, `device.com_status_2`, `device.aiding_status`) and, when the unit answered identification, `device.product_code`, `device.serial_number`, `device.firmware`. |
| `sensor alerts` | `SBG_ECOM_LOG_STATUS` | Summary: `alerts.active`, level = worst active alert. |
| `alert/<CODE>` | `SBG_ECOM_LOG_STATUS` | One per device fault flag that has been seen active — see [Alerts](#alerts). |
| `imu` | `IMU_SHORT` / `IMU_DATA` | Raw `imu.status`; ERROR for failed communication, built-in tests, axis tests or measurement ranges. `imu.failed_checks` preserves the vendor flag names. |
| `temperature` | `IMU_SHORT` / `IMU_DATA` | `temp.imu_c`, °C. |
| `ekf` | `SBG_ECOM_LOG_EKF_NAV` | `ekf.solution_mode`, raw `ekf.status`, validity and aiding flags. |
| `gnss` | `SBG_ECOM_LOG_GPS1_POS` | Fix grade: `gnss.fix_type`, `gnss.rtk` (`fixed` / `float` / `none`), satellites, differential age, 1σ accuracies. *Optional.* |
| `gnss heading` | `SBG_ECOM_LOG_GPS1_HDT` | Dual-antenna heading solution: `gnss.heading_deg`, `gnss.heading_accuracy_deg`, `gnss.baseline_m`. *Optional.* |

`hardware_id` is `"<product code> <serial>"` on every device-side component and
empty on `service` and `stream`. It is also empty on device components when the
unit could not be asked (file replay, a data-only HPINS interface).

*Optional* components join the inventory with the first log that feeds them and
stay for the life of the process. A single-antenna unit therefore never shows a
`gnss heading` component, rather than a permanently missing one. `device`,
`imu`, `temperature` and `ekf` are always present: a unit that does not stream their
log is told so (see below).

### Levels

Levels are the standard `DiagnosticStatus` ones (0 OK, 1 WARN, 2 ERROR,
3 STALE). Operational states are values, never levels: a confirmed `standby`
is OK.

| Component | WARN | ERROR |
|---|---|---|
| `service` | device link released after a settings save (unit rebooting: deactivate, then activate); the log callback is throwing | the last activation failed (`activation failed: <reason>`); lifecycle error |
| `stream` | active, first log not yet received; `/odom` epochs being dropped in the last `stale_after_s` | active and silent for more than `stream_timeout_s` |
| `imu` | — | communication, self-test or range flags failed |
| `temperature` | no valid reading yet; a configured `warn_*` limit crossed | a configured `error_*` limit crossed |
| `ekf` | `VERTICAL_GYRO`, `AHRS` | `UNINITIALIZED` |
| `gnss` | fix grade below `health.required_gnss_fix` | — |
| `gnss heading` | no valid heading solution | — |
| `alert/<CODE>` | see [Alerts](#alerts) | see [Alerts](#alerts) |

`device` itself stays OK while its log arrives: device faults are the alerts.

## Freshness

Each component carries the reserved metadata:

| Value | Here |
|---|---|
| `health.publisher_id` | UUID generated once per process. A lifecycle `cleanup → configure` keeps it (and the sequence); a process restart changes it. |
| `health.sequence` | Increases by one per publication. |
| `health.publish_interval_s` | `health.publish_interval_s`. |
| `health.sample_id` | `<publisher prefix>:<source>:<count>`; changes only on a new observation (temperature requires a finite reading). |
| `health.sample_age_s` | Age of that log on the process's monotonic clock. |
| `health.stale_after_s` | `health.stale_after_s`. |
| `health.availability` | below |
| `health.last_level` | The level a component had when it went STALE. |
| `health.error` | Why the current observation could not be obtained. |

A heartbeat republishes cached observations with the same `sample_id` and a
larger `sample_age_s`; it never makes them new.

| `health.availability` | When | Level and values |
|---|---|---|
| `current` | the source log arrived within `stale_after_s` and the stream is alive | as observed |
| `retrying` | the whole stream has been silent for more than `stream_timeout_s`, the cached observation is still within `stale_after_s` | last-known level and values, `health.error` set |
| `stale` | the source log is older than `stale_after_s`, dates from before the current activation, or a temperature read failed after a valid observation | STALE, `health.last_level`, last-known values kept, `health.error` set |
| `unavailable` | active, and the source log has never arrived | STALE; after `stale_after_s` also `health.error` naming the log to enable on the device, and `temp.imu_c` = `null` with `health.metric.temp.imu_c.state` = `unavailable` |
| `paused` | not active (standby, transition, shutdown), device link released, or a device command (mag calibration, settings save) owns the link | last-known level and values, nothing marked missing |

`service` and `stream` are evaluated at publication time, so they are always
`current` (or `paused` for `stream`); a silent stream is a *current* ERROR.

Faults survive loss of data: a fault flag is only ever cleared by a newer
STATUS log, so through a silent link `alert/<CODE>` goes `retrying`, then STALE
with `health.last_level` 2, and in standby stays ERROR / `paused` until the
next activation refreshes it.

## Measurements

| Metric | Source | Notes |
|---|---|---|
| `temp.imu_c` | IMU log temperature | Rounded to 0.001 °C. `IMU_SHORT`'s 1/256 °C fixed point is converted by the SDK. Zero is published as `0`; a non-finite reading retains the last finite value, sample ID, age and severity, marked STALE with `health.last_level`, `health.error` and `health.metric.temp.imu_c.state: unavailable`. With no valid reading yet it reports `null`/WARN/unavailable. Recovery requires a finite reading. |
| `supply.voltage_v`, `supply.current_a`, `supply.power_w` | — | **Not reported.** sbgECom exposes no voltage or current. The `device` component declares all three `health.metric.<key>.state: unsupported`; supply health arrives as the `*_POWER` alerts instead. |

No temperature limits apply unless configured (`health.limits.temp_imu_c.*`).
The previous built-in thresholds (WARN above 70 °C, ERROR outside −40…85 °C)
were this driver's guess, not a property of every SBG product; to keep them, in
the sensor config:

```yaml
health:
  limits:
    temp.imu_c: {warn_above: 70, error_above: 85, error_below: -40}
```

(or `health.limits.temp_imu_c.warn_above: 70.0` … as node parameters).

The device's own verdict on its temperature is separate and always reported:
the `TEMPERATURE` alert.

## Alerts

The seven `generalStatus` flags of `SBG_ECOM_LOG_STATUS` are reported as
`alert/<CODE>` while their OK bit is cleared, with `alert.code`,
`alert.category`, `alert.active`, `alert.severity`, `alert.flag` (the vendor's
flag name) and `alert.detail`.

| Code | Vendor flag | Severity | Category |
|---|---|---|---|
| `MAIN_POWER` | `SBG_ECOM_GENERAL_MAIN_POWER_OK` | ERROR | power |
| `IMU_POWER` | `SBG_ECOM_GENERAL_IMU_POWER_OK` | ERROR | power |
| `GPS_POWER` | `SBG_ECOM_GENERAL_GPS_POWER_OK` | ERROR | power |
| `SETTINGS` | `SBG_ECOM_GENERAL_SETTINGS_OK` | ERROR | settings |
| `TEMPERATURE` | `SBG_ECOM_GENERAL_TEMPERATURE_OK` | ERROR | temperature |
| `DATALOGGER` | `SBG_ECOM_GENERAL_DATALOGGER_OK` | WARNING | datalogger |
| `CPU` | `SBG_ECOM_GENERAL_CPU_OK` | WARNING | cpu |

The vendor gives these flags no severity; the mapping above is this driver's:
a lost supply, unloaded settings or out-of-limits temperature invalidate the
outputs, the other two degrade them.

- A code enters the inventory the first time it is active and is then kept for
  the life of the process; when the flag recovers it is published explicitly as
  OK, `alert.active: false`, message `cleared`.
- `health.ignored_alerts` lists codes that must never raise an alert, for a
  product that does not implement a flag. The raw word stays visible as
  `device.general_status`.
- The communication and aiding words are published raw on `device` and **not**
  turned into alerts: the vendor documents a cleared interface bit as "closed
  or in error", so an unused port is indistinguishable from a faulty one
  without knowing the installation.
- The device has no alert history, so `alerts.log`, `alerts.epoch` and
  `alerts.history_gap` are not published.

## GNSS fix grade and heading

`gnss` always reports the primary receiver's actual grade (`gnss.fix_type` is
the vendor's position type: `SINGLE`, `SBAS`, `RTK_FLOAT`, `RTK_INT`, …;
`gnss.rtk` condenses it to `fixed` / `float` / `none`). `health.required_gnss_fix`
is what the deployment expects, and the component is a WARN below it:

| `required_gnss_fix` | OK when |
|---|---|
| `none` | always |
| `fix` (default) | any computed position |
| `rtk_float` | RTK float or fixed |
| `rtk_fixed` | RTK fixed |

`gnss heading` reflects the dual-antenna solution published on `/gps/heading`
(see the README). Its validity is independent of the position grade above: it
is a baseline solution between the unit's own two antennas and needs no RTK
corrections.

## Operational state

| Lifecycle | `service` | Device components |
|---|---|---|
| unconfigured | *not publishing* | — |
| inactive (configured) | `state: standby`, OK | `paused` |
| activating / deactivating | `state: transitioning`, OK | `paused` |
| activation failed | `state: standby`, ERROR `activation failed: …` | `paused` |
| active | `state: active`, OK | live |
| active, device released after a settings save | `state: active`, WARN | `paused` |
| lifecycle shutdown, or the process stopped by SIGINT / SIGTERM | one last `state: shutdown`, then silence | `paused` |

The publisher exists from `configure` to `cleanup`. Stopping the process —
`docker stop`, `rig down`, Ctrl-C — runs the lifecycle shutdown first (see
[Runtime behaviour](#runtime-behaviour)), so the last report says `shutdown`.
Only a process that dies without the chance (SIGKILL, a crash, power loss)
simply goes silent; consumers detect that from the missing heartbeat.

## Configuration

Node parameters (all `read_only`: applied when the node is configured).

| Parameter | Default | Meaning |
|---|---|---|
| `health.vehicle_id` | `""` | Fleet vehicle identity. |
| `health.instance` | `""` | Deployment instance name. |
| `health.publish_interval_s` | `1.0` | Heartbeat period (0.1 … 60 s). |
| `health.stale_after_s` | `5.0` | Validity of a device observation; at least twice the heartbeat. Cover a few periods of the slowest health-bearing log (STATUS is usually 1 Hz). |
| `health.stream_timeout_s` | `2.0` | Silence on the whole stream, while active, that is an ERROR. |
| `health.required_gnss_fix` | `fix` | `none`, `fix`, `rtk_float`, `rtk_fixed`. |
| `health.ignored_alerts` | `[]` | Alert codes never to raise. |
| `health.limits.temp_imu_c.{warn,error}_{above,below}` | unset (NaN) | Producer-side limits, °C. |

There is no polling interval: the driver polls nothing. How often an
observation is renewed is the device's output rate for that log.

Invalid values fail the `configure` transition with the reason in the log
(non-positive or non-finite timings, `stale_after_s` below two heartbeats,
inverted or non-finite limits, an identity that is not a single path segment,
unknown alert codes).

In a rig sensor config the same settings go in the top-level `health:` block,
which `tools/render_params.py` validates before the container is started and
maps onto the parameters above (see [`sensors/sbg.example.yaml`](../../../sensors/sbg.example.yaml)):

```yaml
name: ins                       # -> health.instance
health:
  interval_s: 1                 # -> health.publish_interval_s
  stale_after_s: 5
  stream_timeout_s: 2
  required_gnss_fix: rtk_fixed
  ignored_alerts: []
  limits:
    temp.imu_c: {warn_above: 70, error_above: 85}
```

`driver_params.health.*` still works and wins over the block, without dropping
the rendered identity.

## Runtime behaviour

- **Collection** happens on the device I/O thread, as each log is dispatched:
  a mutex-guarded copy of a few fields. It issues no device request and does no
  I/O, so there is nothing to time out.
- **Publication** runs on its own thread, not on the node's executor. Lifecycle
  transitions and the device-command services block the executor for seconds
  (identification, Configurator round-trips); the heartbeat keeps its cadence
  through them and can report `transitioning`. State changes an operator should
  not wait for (failed activation, standby, device released) are published
  immediately.
- **Standby:** the publisher is a plain publisher, not a lifecycle one, so the
  inactive node keeps reporting. The device link is closed in standby, so device
  components are `paused` with their last-known values.
- **Shutdown:** `cleanup`, `shutdown` and node destruction stop the thread at
  once (it does not sit out its interval) and publish one final snapshot.
- **Stop signals:** the `sbg_driver_node` executable handles SIGINT and SIGTERM
  itself (`src/main.cpp`) and runs the node's `shutdown` transition before
  shutting ROS down. rclcpp's own handler would invalidate the context first,
  after which neither the transition nor a final report can go out. The deploy
  compose stops the container with SIGINT (`stop_signal`), which `ros2 launch`
  passes on. This covers the standalone executable only: loaded as a component,
  the container process owns the signals and the node is destroyed as before.
- A failure inside health reporting is logged (throttled) and never propagates
  into the driver.

## Reaching the dashboard

The dashboard reads ROS topics through a Zenoh router, so the node must run
with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (the runtime image ships it; the deploy
compose passes the variable through) and reach the vehicle's `rmw_zenohd`
router — `tcp/localhost:7447` by default, which host networking provides;
`ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<host>:7447"]'` points the
session elsewhere. Under Fast DDS the diagnostics are published just the same
but only DDS peers see them; a DDS-only vehicle needs a supported bridge.

The driver never constructs a Zenoh key: rmw_zenoh advertises the publisher
with its own liveliness token, from which the dashboard derives the data key.

The dashboard subscribes to every `DiagnosticArray` topic it discovers (or the
`health.ros_topics` allowlist of its instance config) and attributes statuses
by the `health.*` identity above. A Home widget finds this reporter by
auto-discovery, or explicitly:

```yaml
home:
  widgets:
    - type: health
      services: [uav6/ins]      # <vehicle>/<instance>, or just the instance
```

## Verification

| What | Where | Run |
|---|---|---|
| Reporter logic against an injected clock: valid, missing, invalid, unsupported and zero readings; silent stream and recovery; stale cached values; publisher restart; fault/clear; standby, shutdown; limits; settings validation | `test/health_test.cpp` | `build/sbg_driver/test_health` |
| Log → reporter routing, generated level constants, real CDR round trip, heartbeat thread (QoS, cadence, prompt stop, final state) | `test/health_publisher_test.cpp` | `build/sbg_driver/test_health_publisher` |
| The real node process replaying real sbgECom frames, read back across the RMW in another process | `test/health_test.launch.py` | `colcon test --packages-select sbg_driver` |
| Launcher: identity and `health:` block rendering | `tools/test_render_params.py` | `python3 -m unittest discover -s tools -p 'test_*.py'` |
| The dashboard's own decoder, store, verdict/issue logic and Home widget on this driver's serialized reports | `tools/dashboard_compat/` | `tools/check_dashboard_health.sh <dashboard checkout>` |

The last one decodes `test/fixtures/health_diagnostics.json` — real CDR written
by `test_health_publisher`, which also fails if the fixture no longer matches
what the code produces. After changing what is reported, regenerate it and
re-run the dashboard check:

```bash
SBG_HEALTH_FIXTURE_UPDATE=1 build/sbg_driver/test_health_publisher
tools/check_dashboard_health.sh ~/ws/dashboard
```

## Changes from the earlier diagnostics

The three `diagnostic_updater` tasks are replaced by the components above. A
`diagnostic_aggregator` or other consumer keyed on the old names needs
updating:

| Before | Now |
|---|---|
| `sbg_driver: Device` | `<instance>: service` (state, `callback_exceptions`), `<instance>: stream` (`frame_age_s` replaces `seconds_since_last_log`; `composition_drops`, `imu_orientation_misses`, `imu_source`), `<instance>: device` (`device.general_status` replaces `device_status_general`) |
| `sbg_driver: EKF Solution` | `<instance>: ekf` (`ekf.solution_mode` is now the mode name, `ekf.status` replaces `raw_status`) |
| `sbg_driver: IMU Temperature` | `<instance>: temperature` (`temp.imu_c` replaces `temperature_c`) |
| `hardware_id: sbg_systems_ins` | the unit's product code and serial, or empty |

Behaviour that changed with it:

- No built-in temperature thresholds (see [Measurements](#measurements)).
- The composition-drop warning now decays `stale_after_s` after the last drop
  instead of latching until the next activation.
- A failed activation is an ERROR on `service`; previously nothing was
  published about it.
- Diagnostics keep flowing while the node is inactive.
