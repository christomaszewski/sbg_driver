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
| `6abcc0a` | fix   | ~     | WGS84 M/N radii of curvature in geodetic→local (kills ~0.2-0.7%/km scale bias) |
| `0890301` | feat  | ~     | `/rtcm` typed as `rtcm_msgs/Message` (NTRIP-client compat) + write_rtcm threading-doc fix |
| `c669e0a` | feat  | +120  | opt-in NMEA GGA publisher (`nmea_msgs/Sentence`) for NTRIP VRS upload |
| `5da886b` | 3h-2  | +180  | Configurator device-provisioning wrappers (core lib) |
| `95d0168` | 3h-2  | +200  | configure_device.* params + on_activate provisioning walk |
| `57cb84b` | 3j    | +150  | migration guide (upstream sbg_ros2_driver → this driver) |
| `823e32c` | 3j    | ~     | Doxygen API-doc setup + docs workflow (artifact-only) |
| `9748540` | 3h-3  | +180  | Configurator::set_output (per-log output rates, core) |
| `ef2bd9d` | 3h-3  | +130  | configure_device.output.* params + provisioning walk |
| `45946a6` | test  | +110  | exhaustive LogView accessor coverage (28%→99%) |
| `4f688a3` | test  | +180  | extract param→enum mappers to param_conversions.* + 9 tests |
| `f8bd711` | test  | +45   | transport open_serial/open_udp/move-assign (transport.cpp 68%→95%) |
| `fa3da10` | docker| ~     | make Dockerfile.runtime actually build+run (latent-bug fixes, hand-patched) |
| `a240bbc` | docker| ~     | re-adopt template's self-contained Dockerfile.runtime (drop CI-image dep) |

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

### Geodetic→local projection accuracy (`6abcc0a`)
Follow-up to a design question about how `/odom` is populated from `EkfNav`:
position is a tangent-plane projection of lat/lon to local Cartesian (ENU or
NED per `use_enu`), relative to the sticky first-fix origin — `twist` comes from
`EkfVelBody` (unaffected), orientation from `EkfQuat`. The projection used a
single equatorial radius for both axes → ~0.2-0.7%/km scale bias growing with
distance. Fixed by using the WGS84 **meridional (north)** and **prime-vertical
(east)** radii of curvature at the origin latitude. New `make_geodetic_origin()`
factory bakes the M/N per-degree scales into `GeodeticOrigin` (replaced the
`cos_lat0` cache); `geodetic_to_local()` is now a pure affine map. Equator check:
east unchanged at 111319.49 m/° (N(0)=a), north corrected 111319.49→110574.3 m/°
(M(0)=a(1−e²)). Tests updated; suite 65/0; verified in dev container.

### `/rtcm` → rtcm_msgs/Message + write_rtcm threading doc (`0890301`)
Prompted by an NTRIP-caster question. Our driver has no NTRIP client — it's the
rover-side sink: `/rtcm` subscription → `Device::write_rtcm` → `sbgInterfaceWrite`
pushes RTCM3 down the same serial/UDP link; the SBG's GNSS does the RTK fixing.
The input was typed `std_msgs/UInt8MultiArray`, which no off-the-shelf ROS 2
NTRIP client (ntrip_client, …) publishes — they emit `rtcm_msgs/Message`. Retyped
the subscription to `rtcm_msgs/Message` (reads `msg->message`) so the driver is
drop-in downstream of a standard NTRIP client. Added `rtcm_msgs` dep:
package.xml + CMake find_package/link/export, and `ros-${ROS_DISTRO}-rtcm-msgs`
in Dockerfile.dev/ci (images hardcode the ROS dep list; CI also rosdeps it).
`rtcm_msgs` 1.1.6 IS released for Lyrical via apt (an earlier check hit stale
apt lists baked into the dev image — rebuilt the image to fix). Also corrected a
stale device.hpp threading comment: `write_rtcm()` is concurrency-safe with
`run()`/`poll_once()` (independent write path), not "must not call during run()".
Verified: dev image rebuilt, build + ctest 65/0, clang-format clean.

### NMEA GGA publisher (`c669e0a`)
Follow-up after comparing RTCM handling to the reference: the official driver
also publishes NMEA GGA so an NTRIP client can upload the rover position to a
VRS / network-RTK caster (the position-up half of the NTRIP loop). We had no
NMEA out. Added opt-in `to_nmea_gga()` — locale-independent std::format $GPGGA
builder: ddmm.mmmm + hemisphere, SBG-type→GGA quality digit (matches the
reference), sats/HDOP/alt/geoid/diff-age/base-id, XOR checksum. UTC time-of-day
from the receive-time stamp (avoids GPS-ToW leap-second tracking; fine for VRS,
which keys off lat/lon). Gated publisher emitted from the GnssPos handler,
rate-limited to ~1 Hz, only for a computed fix (to_nmea_gga returns nullptr
otherwise). New `nmea_msgs` dep (2.1.0 on Lyrical; package.xml + CMake + both
Dockerfiles), params `topics.nmea` + `outputs.publish_nmea_gga`. 2 gtests;
suite 67/0. GOTCHA: generate_parameter_library does NOT escape embedded
double-quotes in param descriptions — a `default "nmea"` in a description
emitted `"..."nmea` → an `operator""nmea` UDL compile error. Keep description
text quote-free. The remaining reference-parity gap is device-side RTCM port
routing (Configurator 3h-2).

### Phase 3h-2: device-provisioning Configurator (`5da886b` core, `95d0168` wiring) — DONE
Core (`5da886b`): typed Configurator wrappers `set_motion_profile`,
`set_imu_alignment` (axis dirs + misalignment + lever arm), `set_aiding_assignment`
(gps1/rtcm/dvl/airdata ports; get-modify-set preserves sync + odometer pins),
`set_gnss_installation` (primary/secondary lever arms + dual mode),
`set_magnetometer_model`. Public `sbg::` enums/structs; sbgECom translated in
device.cpp via `to_sbg()` (`using enum`), so the C SDK never leaks. Wiring
(`95d0168`): `configure_device.*` params (master `enable:false` + per-section
`apply` flags) walked by `apply_device_configuration()` in `on_activate`, after
open / before the I/O thread starts (so commands are permitted); fail-fast
aborts activation + drops the handle. RAM-only — does NOT save to NVRAM (avoids
reboot mid-activation); persist via the save-settings service. Per-command
behaviour is HIL-verified (no host-side unit test, like the mag-cal wrappers).

### Test-coverage pass (`45946a6`, `4f688a3`)
Real gcovr numbers (only `sbg_driver_core` is coverage-instrumented). Before:
core 24.6% line — dragged down by `device.cpp` 10% (the Configurator command
surface is HIL-only by design) and `log_view.cpp` 28% (only ~4 of 17 accessors
tested). Cheap wins done: (`45946a6`) exhaustive LogView accessor test →
log_view 28%→**99%**; (`4f688a3`) extracted the configure_device param→enum
mappers into `param_conversions.{hpp,cpp}` + 9 unit tests (were untested);
(`f8bd711`) transport open_serial(error)/open_udp/move-assign → transport.cpp
68%→**95%**. Core total now 24.6% → **42%** (the rest is `device.cpp`'s 10%
HIL-only Configurator surface). Still pending coverage-wise: instrument `sbg_driver`
so `conversions.cpp` (30 tests, the bug hotspot) shows its number — right now
it's a metric blind spot; and the device.cpp config surface stays HIL-only
(would need a mock SbgInterface to unit-test).

### Runtime Docker image (`fa3da10` build/run fixes, `a240bbc` re-adoption)
`docker/Dockerfile.runtime` had never been built and had latent bugs; `fa3da10`
fixed them but as hand-patches to a diverged `FROM ${CI_IMAGE}` (ghcr.io)
structure. `a240bbc` then **re-adopted the boilerplate template's self-contained
two-stage build**: compile in `ros:lyrical-ros-base`, ship on `ros:lyrical-ros-core`
— no published CI image required, so it builds anywhere with just docker and a
future `copier update` of this file merges cleanly. The only divergence from the
template render is the dependency lists: the build stage adds `git` +
`ca-certificates` (sbgECom is FetchContent'd over git) plus the driver's extra ROS
deps (nav-msgs, tf2-ros, std-srvs, rtcm-msgs, nmea-msgs); the runtime stage adds
the same on top of the template's generic list (which already carries `rsl` for the
gpl-generated `librsl.so`). Build + smoke verified on arm64:
`docker buildx build --platform linux/arm64 -f docker/Dockerfile.runtime -t
sbg_driver:test --load .` compiles all 3 packages from scratch, and file-replay
drives the node to lifecycle `active [3]` with every topic advertised (/ekf/fix,
/nmea, /rtcm, /odom, /tf, /sbg/*). No CI job builds this file (it's release-time);
CI's `image` job builds Dockerfile.ci, which is untouched. The generic `rsl` gap
was upstreamed to the template earlier (`ee5dfd0`, tag v0.2.9).

## Pending work

Listed in roughly preferred order. Each is sized to be a single
focused commit; pick whichever has highest current value to you.

3i, 3h-2, 3h-3, and 3j docs are **done** — remaining is the release-time 3j
items (debian/release.yml, gh-pages flip) and the non-gating linter cleanup.

### Phase 3h-3: output-log rate config (`9748540` core, `ef2bd9d` wiring) — DONE
Per-log output-rate config via `sbgEComCmdOutputSetConf`. Core (`9748540`):
`sbg::OutputLog` / `OutputRate` / `OutputPort` enums + `Configurator::set_output`
— raw `SBG_ECOM_LOG_*`/`OUTPUT_MODE_*` ids stay in device.cpp (`to_sbg_msg_id`/
`to_sbg`), never leak. Wiring (`ef2bd9d`): `configure_device.output.*` (master
`apply`, `port`, and per-log rate strings for the 9 logs that feed our published
topics; `"unchanged"` default = leave device setting). Walked in
`apply_device_configuration()` (RAM-only, fail-fast). The OutputLog enum carries
extra logs (events/raw/imu_short/ekf_euler/gps_hdt) for direct-API callers; the
param surface stays curated.

### Phase 3j: hardening (docs + debian)
Polish for first tagged release. Docs DONE:
- ✅ `src/sbg_driver/doc/migration.md` — full upstream→this-driver topic/type/
  param mapping + `--remap` snippets + honest gaps (`57cb84b`).
- ✅ Doxygen API docs: `Doxyfile` + `.github/workflows/docs.yml` (builds HTML,
  uploads artifact; always-green, separate from ci.yml) (`823e32c`). To publish
  on gh-pages: enable Pages (Settings→Pages→GitHub Actions) then switch docs.yml
  to upload-pages-artifact + deploy-pages (commented in the workflow).

Remaining (release-time — need a tag / rosdistro entry / repo settings, so
deferred until first release rather than writing unverifiable boilerplate now):
- `bloom-generate` debian recipe + staging apt repo test.
- `release.yml` (on tag): bloom-generate + push `Dockerfile.runtime` to GHCR
  with the `.deb` baked in.
- Remove `reference/` before first tag — NOTE it is already gitignored (not in
  the repo), so it won't appear in a tag; this is just an on-disk cleanup.

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
