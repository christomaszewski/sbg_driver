# Execution Progress

Project journal for the `sbg_driver` modern ROS 2 rewrite. Tracks what's
been built, what's pending, and how to pick the work back up cold.

The original architecture document — the plan we agreed on before any
code landed — lives at `~/.claude/plans/iridescent-singing-patterson.md`
(outside the repo, in Claude tooling state). The plan is the design
intent; this file is the execution journal.

## Build/test sanity

The driver builds and tests cleanly inside the dev Docker image
(`Dockerfile.dev`, ROS 2 Lyrical on Ubuntu 26.04, multi-arch). To
re-verify after picking the work back up:

```bash
docker compose -f docker/compose/compose.dev.yaml up -d
docker compose -f docker/compose/compose.dev.yaml exec dev bash
# inside container:
cd /workspace
rm -rf build install log
colcon build --base-paths src
source install/setup.bash
colcon test --base-paths src --event-handlers console_direct-
colcon test-result | tail -15
```

Expected: 3 packages build clean. Functional tests (gtest, launch_testing)
pass. Stylistic linter complaints remain (~37 across `cpplint` +
`uncrustify`) — addressed at hardening time in Phase 3j, not blocking.

## Commits done

| Commit | Phase | LOC | Topic graph delta |
|--------|-------|-----|-------------------|
| `dda4d25` | 0+1+2 | +4296 | Skeleton, core lib, `/imu/data` |
| `04c9a00` | 3a    | +463  | `/imu/temperature`, `/imu/mag`, `/gps/fix`, `/time_reference` |
| `b34d617` | 3b    | +419  | `/odom`, TF `odom→base_link` |
| `9ea1ed5` | 3c    | +261  | `sbg_msgs` package + `/sbg/status`, `/sbg/ekf_status` |
| `15566bd` | 3d    | +281  | `/sbg/ship_motion`, `/event`, `/mag_calib`, `/gps_raw`, `/air_data_status` |
| `5f7cce3` | 3e+f+g| +207  | `/diagnostics`, `/rtcm`, mag-cal services (stub) |
| `0c7f54f` | 3h    | +348  | `Configurator` + mag-cal services (real) |
| `7e8b06e` | docs  | +442  | README refresh + this PROGRESS.md journal |
| `b140c38` | 3i    | +207  | IMU accel/gyro covariance from noise params |
| `def3537` | review| +90   | correctness fixes: ENU quaternion, SBAS, EkfStatus |
| `0eeed33` | review| ~     | modernization: unique_ptr pImpl, std::format |
| `0a62c82` | bp #1 | ~     | transport.* params read_only (set-at-launch-only) |
| `c7e8145` | bp #2 | ~     | qos_overrides on all publishers |
| `ad284ed` | bp #4 | +220  | sbg_decode CLI (ROS-free replay/analysis, --json) |
| `d89c7b9` | bp #5 | ~     | TSan CI job (core-scoped) |
| `323bc2a` | bp #6 | ~     | reference.repos vcstool pin |
| `a324ed5` | bp #3 | ~     | monadic ready().and_then() Configurator chaining |
| `75ad9d3` | feat  | +130  | optional `/ekf/fix` NavSatFix from fused EkfNav (off by default) |

(Plus post-push CI hardening + an authorship rewrite; see `git log`. SHAs
above are post-rewrite. "bp" = reviewed back-port improvements from a sibling
NovAtel driver project.)

### Back-port improvements (`0a62c82`..`a324ed5`)
Six independent reviewed back-ports, each its own commit, each verified in the
dev container, all green in CI:
- #1 transport.* params `read_only` (device opened once in on_activate).
- #2 every publisher gets QosOverridingOptions::with_default_policies() →
  qos_overrides.<topic>.* tunable at launch; base profiles unchanged.
- #4 sbg_decode CLI (src/sbg_driver_core/tools/) — open FileReplay/Serial via
  sbg::Device, print each LogView (text or --json), poll to EOF. ROS-free;
  for replay analysis + offline golden-vector generation.
- #5 TSan CI job. SCOPED TO sbg_driver_core ON PURPOSE: a TSan-instrumented
  full ROS node SIGILLs (dlopen of uninstrumented rmw/DDS plugins — known
  rclcpp+TSan limitation). Core has the jthread concurrency and runs clean.
- #6 reference.repos: vcstool pin of upstream drivers (3.3.2 / 3.2.1);
  `vcs import reference < reference.repos`. reference/ stays gitignored.
- #3 Configurator: private ready() precondition + .and_then()/.transform()
  chaining + detail::check(). driver_node service chain LEFT explicit so
  restart_io_thread() side effect is preserved.

CI is now image → build-test → lint → asan → tsan (5 jobs), all green.

### Review + modernization (`def3537`, `0eeed33`)
An independent correctness review (subagent, cross-checked vs the reference
SBG driver + SDK headers) found ONE critical bug: the ENU orientation
quaternion only flipped y/z and omitted the NED→ENU axis-swap rotation —
fixed (q_enu = Q ⊗ (w,x,-y,-z), Q = (√2/2,0,0,√2/2)), affecting /imu/data +
/odom in ENU mode (NED default was always correct). Also: SBAS→STATUS_SBAS_FIX
+ PSRDIFF→GBAS_FIX in NavSatFix; EkfStatus gained the 6 missing aiding bits;
save_mag_calibration resets device_ on the reboot path. Modernization was
small (review confirmed the code was already clean): unique_ptr pImpl in
Transport (removed the only raw new/delete), std::format for service messages.
Review confirmed-correct (do NOT change): the EkfVelBody body-frame y/z flip
(the reference's nav-swap there is the actual bug).

Workspace state: ~6500 LOC across 65 files.

### Phase 0+1+2 (`dda4d25`)
3-package skeleton, Docker images, CI, `sbg_driver_core` (Device,
Transport, LogView, Error/Result), `sbg_driver` LifecycleNode publishing
`sensor_msgs/Imu` on `/imu/data`. Bugs surfaced and fixed during Docker
verification: 14 issues (osrf-vs-ros image names, base-image user
collision, ament_target_dependencies removed in Lyrical, message-package
typesupport linking, etc.). See commit message for the full list.

### Phase 3a (`04c9a00`)
4 standard-message publishers: `sensor_msgs/Temperature` (IMU),
`MagneticField` (NED→ENU aware), `NavSatFix` (covariance from per-axis
accuracy, RTK status mapped from SBG GnssPosType), `TimeReference` (UNIX
time composed from sensor UTC via C++20 `std::chrono::year_month_day`).
8 new gtest cases.

### Phase 3b (`b34d617`)
EKF triplet matcher (caches latest `EkfQuat` + `EkfVelBody`, emits on
`EkfNav` arrival). `nav_msgs/Odometry` composition. Local Cartesian
projection from a sticky `GeodeticOrigin` set on first valid fix
(small-angle equidistant, accurate to a few cm within ~1 km).
`tf2_ros::TransformBroadcaster` for `odom → base_link`. 5 new gtests
(GeodeticToLocal x3, Odometry x2).

### Phase 3c (`9ea1ed5`)
All 7 `sbg_msgs` `.msg` files defined and wired through
`rosidl_generate_interfaces`. `/sbg/status` and `/sbg/ekf_status`
publishers (the most-used diagnostic messages). EkfStatus decodes 17
aiding bits from `EkfNav.status` via the SDK's `SBG_ECOM_SOL_*` defs.

### Phase 3d (`15566bd`)
The remaining 5 `/sbg/*` publishers: `ShipMotion`, `Event`, `MagCalib`,
`GpsRaw`, `AirDataStatus`. Required adding 4 new `LogView::Kind`s and
their accessors. AirDataStatus decodes the validity bits from
`SbgEComLogAirData.status`.

### Phase 3e+f+g (`5f7cce3`)
Three production-readiness features bundled:
- **3e**: `diagnostic_updater` with three tasks (Device, EKF Solution,
  IMU Temperature). Driven by a lock-free `DiagSnapshot` of atomic
  fields on `Publishers`, written by the I/O thread and read by the
  updater on the executor thread.
- **3f**: `/rtcm` subscription (`std_msgs/UInt8MultiArray`, not
  `rtcm_msgs/Message` — avoids a Docker image rebuild for the
  rtcm_msgs apt dep). Forwards to `Device::write_rtcm` (added in this
  commit). Thread-safe relative to the I/O thread.
- **3g**: `/sbg/start_mag_calibration` + `/sbg/save_mag_calibration`
  services with std_srvs/Trigger interface — callbacks stubbed (return
  failure) pending the Configurator façade.

### Phase 3h (`0c7f54f`)
`sbg::Configurator` façade — typed wrapper around the sbgECom command
set. This commit scopes it to the mag-cal workflow + settings
persistence:
- `start_mag_calibration(MagCalibMode)`
- `compute_mag_calibration() → Result<MagCalibResults>` (caches
  offset[3] + matrix[9] inside `Device::Impl`)
- `save_mag_calibration_results()` (uploads the cached offset/matrix)
- `save_settings()` (persist to NVRAM + device reboot)

Returned by value from `Device::configurator()` — Configurator is a
move-only pointer wrapper (~16 bytes), no allocation. Methods return
`Error::DeviceBusy` if `Device::run()`'s I/O loop is active.

Mag-cal services in `driver_node.cpp` now do the real work: stop
io_thread → call Configurator → restart io_thread (except after
`save_settings` which reboots the device).

### Phase 3i (`b140c38`)
IMU accel/gyro covariance from noise params — `/imu/data` previously had
sentinel `-1` (unknown), which blocked `robot_localization` ingestion.
Added `ImuCovariance` + `resolve_imu_covariance(sensor_model, accel_stddev,
gyro_stddev)`: explicit stddev wins, else per-model datasheet default,
else unknown. `to_imu()` writes variance to all three diagonal entries.
New params `imu.{sensor_model, accel_noise_stddev, gyro_noise_stddev}`,
resolved once in `on_configure`. Per-model defaults (ellipse/pulse/ekinox/
apogee/quanta) are approximate datasheet starting points — refine per unit.
6 new gtests (25/25 conversion tests pass).

### Optional `/ekf/fix` NavSatFix (`75ad9d3`)
Prompted by a design question: should `EkfNav` be a `NavSatFix` instead of an
`Odometry`? Answer: no — `EkfNav` is the *fused* INS solution and belongs in
`Odometry` (full pose+twist+6×6 covariance), and `/gps/fix` already carries the
*raw* GNSS fix from `GnssPos` (the semantically correct NavSatFix source; its
`NavSatStatus` is a GNSS fix-type enum that's meaningless for a fused solution).
This matches the reference driver (`createRosNavSatFixMessage` takes `SbgGpsPos`,
not EKF nav). The real gap: neither `/gps/fix` (raw) nor `/odom` (local Cartesian,
sticky first-fix origin) exposes the fused *global* lat/lon. So added an opt-in
second NavSatFix from `EkfNav` on `/ekf/fix`:
`to_ekf_navsat()` (MSL→ellipsoid via undulation, ENU [E,N,U] covariance from
`positionStdDev²`, coarse status from the position-valid bit). Gated publisher
created only when `outputs.publish_ekf_nav_sat_fix=true`; topic
`topics.ekf_nav_sat_fix` (default `ekf/fix`). 2 new gtests; suite 65/0. Verified
in dev container (build + ctest, clang-format clean).

## Pending work

Listed in roughly preferred order. Each is sized to be a single
focused commit; pick whichever has highest current value to you.

3i (IMU covariance) is **done** as of `b140c38` — remaining is 3h-2 and 3j.

### Phase 3h-2: broader Configurator surface (~450 LOC)
Add the remaining device-side command wrappers so `configure_through_ros=true`
in `on_activate` can fully provision a device from launch params:
- `set_motion_profile(MotionProfile)` — SbgEComCmdSensorSetMotionProfile
- `set_imu_alignment(...)` — alignment + lever-arm
- `set_lever_arms_primary/secondary(...)`
- `set_aiding_assignment(...)` — which port for GNSS1/2, RTCM, etc.
- `set_magnetometer_model(MagModel)`
- `set_output_log(class, msg_id, rate)` — per-log output divider

Each method follows the pattern in `Configurator::save_settings()` —
check `run_active`, call the sbgECom command, map error code via
`detail::from_sbg`. Add a `configure_device.*` block to the params
YAML with `enable: false` default. In `on_activate` between `Device::open`
and starting the io_thread, if `configure_device.enable` is true, walk
the configurator commands.

Reference: `reference/sbg_ros2_driver/src/config_applier.cpp` (~456 LOC
of boilerplate) shows the SbgEComCmd* surface area we need to cover.

### Phase 3j: hardening (docs + debian, ~minimal code)
Polish for first tagged release:
- `src/sbg_driver/doc/migration.md` — full topic-by-topic remap table
  from the upstream `sbg_ros2_driver` to this driver, with `--remap`
  launch snippets.
- Doxygen on `sbg_driver_core/include/sbg/*.hpp` public API.
  `.github/workflows/doc.yml` → deploy to `gh-pages` on `main`.
- `bloom-generate` recipe for debian packaging. Test publishing to a
  staging apt repo.
- `release.yml` GitHub Action that runs on tag: bloom-generate + push
  `Dockerfile.runtime` to GHCR with the released `.deb` baked in.
- Remove `reference/` directory before first tag (currently gitignored
  but visible on disk).

### Linter cleanup
~37 stylistic complaints remain (`cpplint` formatting + `uncrustify` —
mostly `template<typename T>` vs `template <typename T>` disagreements
between the two tools). Two paths:
1. Manual `// NOLINT` comments on the contentious lines, plus a final
   `ament_uncrustify --reformat` pass on settled code.
2. Configure cpplint filters to drop the specific complaints that fight
   uncrustify, via `ament_cpplint_filters.txt`.

Not blocking — functional tests are green, the failures are formatter
disagreements not real correctness issues.

### Launch test colcon-only flake — FIXED (CI hardening commit)
Root cause: the launch description fired `configure` AND `activate` as two
back-to-back `EmitEvent`s on `OnProcessStart`. `activate` could arrive
before `configure` finished, get rejected (invalid transition from
unconfigured/configuring), and the node would sit at `INACTIVE` — so the
ACTIVE assertion timed out.

Fix: `test_replay_test.launch.py` now drives the transitions from the test
body via `change_state` service calls in `setUpClass`, awaiting each
transition's result before issuing the next. No EmitEvent race. Verified
green across 3 consecutive `colcon test ... --ctest-args -LE linter` runs
in the dev container (RC=0 each).

## Architectural decisions (worth preserving)

These are decisions made during the session that future Claude/future
us might second-guess:

1. **Apache 2.0 license** (not MIT). ROS 2 ecosystem standard.
   `ament_copyright` has first-class support — MIT would need 17-line
   per-file headers, Apache 2.0 has 13. Practical equivalence for
   permissive open-source.
2. **Sensor-native NED default** (not ENU). User overrode my initial
   ENU recommendation: principle of least surprise for someone who
   knows the hardware. README/migration doc loudly notes that most
   ROS users want `use_enu: true`.
3. **C++23, not C++20.** Lyrical's GCC 14/15 supports it cleanly.
   Buys us `std::expected` natively (no `tl::expected` shim).
4. **Relative topic names, not absolute.** Picks up node namespace at
   launch — supports multi-instance deployments (e.g. two SBG units
   on one robot via two namespaces).
5. **Single-process composition over multi-package wrapper.** The
   `sbg_driver_core/sbg_driver` split is internal to one repo. Single
   PR can change all three packages coherently.
6. **`FetchContent` for sbgECom**, not vendored source copy. Pinned
   tag `5.6.2730-stable` (the bundled-in-reference 3.3.2 version was
   too old). System-install override via
   `-DSBG_DRIVER_USE_SYSTEM_SBGECOM=ON` for distro packagers.
7. **`std_msgs/UInt8MultiArray` for `/rtcm`**, not `rtcm_msgs/Message`.
   Avoided pulling in an extra apt dep for `rtcm_msgs`. Users with
   an `rtcm_msgs` source need a 5-line adapter node — documented in
   the README/migration guide.
8. **Configurator returned by value**, move-only, ~16 bytes — not a
   reference into a `unique_ptr` inside `Device::Impl`. Simpler than
   the friend-of-pImpl pattern.
9. **Lock-free atomic `DiagSnapshot`** in Publishers for
   `diagnostic_updater` tasks — no mutex needed since each field is
   independently atomic and the snapshot is eventually consistent
   (fine for /diagnostics).

## Things I'd try first if picking up cold

1. Make sure Docker dev container still boots and `colcon build` is
   clean. The image is `sbg_driver:dev` locally; rebuild via
   `docker build -t sbg_driver:dev -f docker/Dockerfile.dev docker/`
   if it's been pruned.
2. Read PROGRESS.md and the latest commit message — most recent
   architectural decisions are documented in commit messages.
3. Check what's in the working tree (`git status`) and `git log
   --oneline` — should be on `main`, no uncommitted changes, latest
   commit `0c7f54f` (Phase 3h).
4. Skim the design doc at `~/.claude/plans/iridescent-singing-patterson.md`
   for original intent — the plan is the source of truth on
   architectural decisions, this file is execution status.
