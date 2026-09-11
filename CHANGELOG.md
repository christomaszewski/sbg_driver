# Changelog

## 0.7.0 — 2026-09-11

### Added

- Optional `raw/orientation` (`sensor_msgs/Imu`): primary GNSS true heading with timestamp-matched native accelerometer and gyro data, heading offset, and field validity/covariance handling. Enable `outputs.publish_raw_orientation` and the device's GPS1_HDT output.
- `sbg_msgs/Event.channel` identifies GPIO inputs A–E (`CHANNEL_A` through `CHANNEL_E`).
- File replay through `sbg-up`: host captures are mounted read-only, relative paths resolve beside the sensor configuration, and ROS parameters reference the container path.
- Regression coverage for sensor faults, restart recovery, UTC expiry, TF gating, diagnostics, replay timing, event routing, launcher input handling, and release-version agreement.

### Fixed

- Healthy IMU measurements with unknown noise now use all-zero covariance. `-1` is reserved for unavailable measurements.
- IMU communication, self-test, axis-range flags and nonfinite values are checked consistently in `imu/data`, `raw/orientation`, and odometry angular velocity.
- Confirmed device timestamp restarts clear heading deduplication, cached measurements, source-selection state and UTC lock while preserving the local geodetic origin. Short packet reordering and normal rollover do not reset the session.
- Invalid UTC messages revoke the cached anchor. Valid anchors expire after `time.utc_timeout_s` (default 5 seconds of steady time), followed by receipt-time stamping until a valid anchor returns.
- TF requires valid position, attitude and heading plus a finite, normalized pose. Invalid positions cannot initialize the local origin.
- Diagnostics evaluate device health and EKF validity, age each required stream separately using steady time, and clear composition warnings after drops stop.
- Replay preserves the full recorded delay, unwraps adjacent timestamps through rollover, handles a timestamp of zero at boot, and remains interruptible while waiting.
- The launcher reads literal NUL-delimited configuration values, eliminating shell evaluation and propagating renderer failures before staging/launch.

### Migration required

- **TF is opt-in:** `tf.broadcast_odom_to_base` now defaults to `false`. Enable it only for the node that owns that transform; leave it disabled when a localization node owns it.
- **Generic launcher frame defaults are namespaced:** `sbg-up` defaults to `<namespace>/imu_link`, `<namespace>/gps_link`, `<namespace>/base_link`, and `<namespace>/odom`. Explicit `driver_params.frames` overrides are preserved. Direct ROS launch keeps the schema's frame defaults. Update mounting transforms or supply the intended vehicle frame names explicitly.
- **Event wire schema changed:** rebuild/redeploy `sbg_msgs` consumers, including the fleet recorder overlay, together with the driver. The `rigging.yaml` message pin is `v0.7.0`; old recordings retain their original schema and need compatible decoding or conversion.
- Driver/core/message package versions and the core version constant are aligned at `0.7.0`. The sbgECom SDK remains pinned to `5.8.935-stable`.

### Validation in the preparation workspace

- ROS Lyrical release build and functional tests: `colcon test-result` reports 167 tests, zero failures/errors/skips. This includes the launcher test executable, whose eight Python cases also pass individually.
- CI sanitizer configurations: ASan/UBSan suite and core TSan suite passed. As in the existing CI configuration, compile instrumentation is scoped to `sbg_driver_core`; the wrapper links the sanitizer runtime for the ASan run.
- A fresh merged production build with `BUILD_TESTING=OFF` passed, matching the runtime Docker build configuration.
- CI clang-format 21 check, shellcheck, launcher/build contract checks, and all eight `rig certify` checks passed.
- Docker Compose resolves the new replay overlay as a read-only bind mount with automatic directory creation disabled.
- Physical-device validation remains outstanding. Rebuild and deploy the driver image and matching message overlay before validating on the vehicle.

### Release procedure

1. Build and run the functional suites and clang-format checks in the ROS Lyrical development/CI environment. The launcher tests also run with `python3 tools/tests/test_launcher.py` (PyYAML required).
2. Validate on the target device: GPS1_HDT heading/offset, IMU field availability, sensor restart and UTC reacquisition, and the vehicle's intended TF ownership/mounting frames.
3. Commit the release changes, tag that commit `v0.7.0`, and publish its GitHub release using these notes. Publishing triggers `.github/workflows/registry-release.yml` to update the rig registry; it is triggered when the GitHub release is published.
4. Rebuild and deploy the driver image and fleet message overlay from the same tag before recording the new Event schema.
