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
| `b89c830` | copier| ~     | template v0.2.10: deploy image via RIG_IMAGE_REGISTRY |
| `e3bcc5e` | copier| ~     | template v0.2.11: rig descriptor deploy.yaml → rigging.yaml |
| `9f51460` | copier| ~     | template v0.2.12: rig launcher contract (COMPOSE_PROJECT_NAME, RIG_IMAGE_TAG, build phase, certify CI gate, no tcp branch) |
| `e3e4c89` | copier| ~     | template v0.2.13: healthcheck level-byte fix; README example de-tcp'd |
| `3d6238f` | copier| ~     | template v0.2.14: healthcheck = own node's lifecycle state (shared-/diagnostics flaw); SBG_NAMESPACE passthrough |
| `3f2d7a4` | fix   | +320  | review sweep 1/4 — core: UDP connected mode, mag-cal mode fidelity, replay pacing, honest poll/run contracts, LogView GNSS µs + instance + RtcmRaw, synthetic-frame tests |
| `03d679a` | fix   | +250  | review sweep 2/4 — driver: EKF/UTC validity gating (CRITICAL origin latch), GGA sentinels, GPS2 split, mag_scale, activate-reset, save_settings service |
| `0171e61` | fix   | +180  | review sweep 3/4 — launch one-shot auto-activate, dead-param removal, honest docs, parameters.md |
| `dae8a52` | copier| ~     | review sweep 4/4 — template v0.2.15 (render_params validation/defaults, fork-PR gating, shellcheck) + stale-render re-adoption (compose.hil/replay, hil_params) |

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

### ROS apt partial-upgrade ABI trap → dist-upgrade in all Dockerfiles (2026-08-12)
Fresh rebuilds of the runtime image crashed under default `rmw_fastrtps_cpp`
(node SIGSEGV during lifecycle activation, rclpy CLI SIGBUS on arm64 / SIGSEGV
on amd64). Bisected to the 2026-07-30/31 rebuild wave, but the root cause is
NOT an upstream code regression: the `ros:lyrical-*` Docker Hub bases (last
built 2026-08-04) predate the 2026-08-07 Lyrical sync, and ROS debs declare
unversioned deps between lock-step packages, so `apt-get install` of a named
subset upgraded `rmw-fastrtps-cpp` to 9.4.9 while leaving the base's
`rmw-fastrtps-shared-cpp` 9.4.8 in place — and 9.4.9 changed
`CustomSubscriberInfo`'s layout (removed `buffer_data_guard_`,
ros2/rmw_fastrtps#902). Mixed-ABI install → crashes; the coherent 9.4.9 set is
fine (verified). Reported upstream: **ros2/rmw_fastrtps#905** (minimal 3-line
docker repro, both arches). Fix: `apt-get dist-upgrade -y` between `update` and
`install` in Dockerfile.runtime (both stages), Dockerfile.ci, and
Dockerfile.dev, keeping every ros-* package on one coherent sync regardless of
base-image staleness. Verified: fresh arm64 build from the stale base compiles
clean, ships both rmw packages at 9.4.9, and file-replay drives the node to
lifecycle `active [3]` under default fastrtps (healthcheck green). Note: the
GHCR CI image rebuilt post-sync on 2026-08-11 and is ALSO mixed (rclcpp 32.0.2
over June rcl/rmw) — it stays green only because Dockerfile.ci never names
`rmw-fastrtps-cpp`, so the rmw pair stayed coherently at 9.4.8. The same
dist-upgrade fix closes that hole. Trap + fix apply to every repo rendered from
the boilerplate template (Dockerfile.runtime.jinja / .ci / .dev) — upstream it.
**SUPERSEDED 2026-08-24** by the parent-sync doctrine (see "rig build/base-image
contract" below): dist-upgrade pinned each image to its build date's archive
sync, which the rig image audit flagged as cross-image ros-* skew.

### Public rig registry publication (2026-08-13)
Registered the driver in **rig-registry-public** as service **`public/sbg`**
(manifest mirrors camera-service: instantiable template service,
`linux/arm64`, `ros_distro: lyrical`, source pinned by full commit SHA).
The package name is READ FROM `rigging.yaml`'s `service:` — that one name
keys the services.yaml route, the row's `service:` field, and the registry's
`services/<name>/` dir, so they must never diverge (initially published as
`sbg-driver`; renamed to `sbg` same day for exactly this reason — a future
`service:` rename requires deleting the old registry dir by hand).
`.github/workflows/registry-release.yml` re-pins the registry on every
published GitHub release (tag vX.Y.Z): rewrite `services/sbg/manifest.yaml`
(name from rigging.yaml, version from the tag, rev = `GITHUB_SHA`, the
tagged commit), `tools/gen-index`, validate, push — the manifest is
workflow-OWNED (rewritten wholesale; registry-side hand edits don't
survive). Mirrors ouster's `registry-release.yml`, the fleet convention,
plus a `workflow_dispatch` (version input) to seed/repair without a
release. Auth is `RIG_REGISTRY_TOKEN`, a fine-grained PAT scoped to
rig-registry-public Contents:read/write only — the same PAT ouster uses
works (it's scoped to the registry, not the caller). Verified end-to-end
via a dispatch run (idempotent no-op against the seed). rigging.yaml also
declares `examples: [sensors/sbg.example.yaml]` so a bare
`rig add public/sbg` materializes the editable instance config from the
pinned rev (declared examples auto-vendor; certify stays 8/8).

### rig build/base-image contract: parent-sync replaces dist-upgrade (2026-08-24)
Adopted the fleet convergence rule from rig-infra/camera-service after rig
image audit found cross-image ros-* version skew: **no image upgrades past the
shared parent image** — the locally-pulled `ros:lyrical-*` (or fleet-ros)
parent is the version authority, advanced deliberately via `rig build
--no-cache` → `docker build --pull`. The 2026-08-12 dist-upgrade fix (above)
was internally consistent but pinned every rebuild to its build date's archive
sync, guaranteeing disagreement with fleet images built on other dates.
Changes: (1) `tools/build_image.sh` adopts the rig build-env contract —
`RIG_BUILD_NO_CACHE` ⇒ `--no-cache --pull`, forwards `ROS_DISTRO` and
`RIG_BASE_IMAGE` as build-args (fallbacks: lyrical, `ros:<distro>-ros-core`);
(2) Dockerfile.runtime grew `ARG BASE_IMAGE` so rig re-parents the runtime
stage onto fleet-ros (shared ros-* layers with router/bag-logger; vehicle pull
cost drops), and every `dist-upgrade -y` in .runtime/.ci/.dev became
`--no-upgrade` on the install (parent-carried packages hold at parent
versions; new ones install normally). Install lists unchanged on purpose —
under fleet-ros apt no-ops/holds them, under ros-core they're needed.
Accepted residual: `--no-upgrade` holds only listed packages, dependency-driven
upgrades can still drift (rig v0.2.24 consumer-mandate doctrine). CI gained a
`build-contract` job (stub `docker` on PATH) asserting the argv both with and
without the rig env. Verified with real builds: against
`localhost:5000/fleet-ros:v1.3.0`, `dpkg-query -W` over the FULL shared ros-*
set matches the parent exactly (0 skew), fleet-ros's 12 layers are a strict
prefix of the image's 17, node binary has 0 unresolved libs, and rcl inits
under both rmw_fastrtps_cpp and rmw_zenoh_cpp; standalone ros-core build
verifies the same way. Mixed-ABI protection is preserved by construction
(nothing upgrades past the parent ⇒ one sync per image) and now also holds
ACROSS images sharing the parent.

### Device identification + HPINS support (2026-08-26)
Field debugging on a QUANTA-USG (uav6, UDP) exposed two ELLIPSE-isms. (1)
`configure_device.*` sends the legacy `sbgEComCmd*` set, which High Performance
INS firmware ignores (only ACK/SETTINGS_ACTION/INFO/API_GET/API_POST exist
there) — every command burned its 3×500 ms ACK timeout and activation failed
opaquely. (2) The HPINS IMU clocks independently of the INS main loop, so the
exact same-timestamp composition contract (/imu orientation attach, /odom
triple) can never match on that hardware. Fix: `Device::query_info()`
(SBG_ECOM_CMD_INFO — both families answer it) + `classify_device_family()` in
core; `on_activate` identifies the product and gates provisioning (HPINS /
unknown products skip with a warning; fails only when `enable=true` and the
device answers nothing; file replay skips), then resolves the new
`outputs.epoch_tolerance_us` (-1 auto: 0 ELLIPSE, 10 ms HPINS) feeding
wrap-safe `device_timestamp_distance()` matching in Publishers. Verified in
the dev container: build + functional suite + replay launch test green,
clang-format clean. (cpplint/uncrustify show ~45 pre-existing findings from
newer linters in the rebuilt dev image — non-gating, see Linter cleanup.)

### Fleet msgs declaration + provenance (2026-08-27)
rigging.yaml gained the top-level `msgs:` block (schema: rig-infra
`msgs/msgs-manifest.example.yaml`; first fleet driver to carry one): the
interface packages our published types come from, consumed by rig ≥ v0.2.28 to
build the deployment's fleet-ros-msgs overlay so the shared bag logger can
subscribe to (and thus record) our topics. Declares `apt: [nmea_msgs]` and
`source:` sbg_msgs pinned to THIS repo at the release tag — everything else we
publish is common_interfaces (not declared); rtcm_msgs is subscribe-only, so
declaring it belongs to the corrections publisher. Because sbg_msgs is built
from the docker context (no pinned clone), the pin the deployment consumes is
the registry entry's release tag: `msgs.source.ref` MUST be bumped to the new
tag as part of cutting each release (comment in rigging.yaml; skew = silent
schema mismatch in recorded bags). For `rig image audit` (≥ v0.2.30) the image
bakes /opt/fleet-msgs/provenance.yaml via docker/provenance-record.sh (vendored
verbatim from rig-infra) in the build stage, copied into the runtime stage; REF
is computed by tools/build_image.sh from the context's REAL git state
(exact-match tag → SHA fallback, `-dirty` when unclean — never a re-echo of the
rigging declaration, which would make the audit circular), and `rev:` is the
doctrine's explicit `unknown` since .dockerignore excludes .git. Verified: rig
0.2.30 certify 8/8 + strict Descriptor parse of the block; full runtime image
build green; baked provenance read back from the image and schema-checked;
shellcheck clean; CI's build-contract stub greps args by presence so the new
`--build-arg SBG_SRC_REF` passes.

### `/ekf/fix` carries the GNSS fix grade (2026-09-01)
Field report: a bag recorded with RTK up showed `/gps/fix` status=2
(GBAS) while every `/ekf/fix` message had status=0. By design until now —
`EkfNav.status` carries no GNSS fix type (only solution mode + aiding bits),
so `to_ekf_navsat()` mapped position-valid → STATUS_FIX and nothing finer.
Now `Publishers` caches the status word of every PRIMARY-receiver `GPS1_POS`
log (`last_gnss_pos_status_`, set whether or not `/gps/fix` is enabled; GPS2
never feeds it) and `to_ekf_navsat()` borrows `to_navsat()`'s grade from it
(RTK/PPP/DGPS → GBAS, SBAS, single → FIX) ONLY while `EkfNav.status` has
`GPS1_POS_USED` set. Position valid but dead-reckoning (bit clear) → plain
STATUS_FIX, so a stale RTK grade is never carried through a dropout — the
aiding bit clearing IS the staleness signal, no timer. A cached word that
decodes below FIX (receiver dropped its solution a log before the EKF
noticed) never drags a valid EKF position down; position-valid clear stays
STATUS_NO_FIX regardless. NavSatStatus cannot tell RTK float from fixed (both
GBAS=2) — same limit as `/gps/fix`. The sticky `/odom` origin still latches on
the first EKF-valid fix regardless of RTK grade (separate decision). 5 new
gtests (`test_conversions` 51/51); verified in the dev container: clean build,
only the known cpplint/uncrustify nits fail, clang-format clean.

### IMU_SHORT source, hold-and-pair orientation, EKF validity gating (2026-09-01)
Fallout of a full correctness review (report kept outside the repo). Three
related /imu/data fixes in one change:
1. **`imu.source` param (auto | imu_data | imu_short).** The SDK deprecates
   SBG_ECOM_LOG_IMU_DATA (log 3): on every product except ELLIPSE it is
   extrapolated to the EKF loop ("artificial noise ... up to 5 ms"), and
   recommends IMU_SHORT (log 44, the IMU's own async stream at the IMU rate).
   LogView already decoded it; Publishers dropped it. `imu_from_short()` packs
   it into the legacy struct via the SDK accessors (fixed point: 1048576 LSB
   per m/s², 67108864 or 12304174 LSB per rad/s keyed off the GYROS_USE_HIGH_SCALE
   bit, 256 LSB per °C — the delta* fields are RATES despite the name), so
   to_imu / to_temperature / the odom gyro twist take either log unchanged.
   `auto` (default) switches to IMU_SHORT on its first log of a session and
   ignores IMU_DATA from then on (latch reset on activate); the fleet will
   enable IMU_SHORT on the units and later disable IMU_DATA. Accepted: /imu/data
   then runs at the IMU rate, above the EKF rate.
2. **Hold-and-pair for /imu/data orientation.** Pairing only attached a quat
   already cached on IMU arrival; IMU_DATA is id 3, EKF_QUAT id 7, so a device
   emitting IMU before quat each tick never attached at tolerance 0 (upstream
   pairs on whichever arrives second). Now: same-epoch quat cached → publish at
   once; else if a quat stream exists and the cached quat is OLDER than the
   sample (signed wrap-aware) → hold; the matching quat publishes it paired, a
   quat that moves past it or the next IMU sample retires it with the
   orientation-unknown sentinel. No quat stream at all → immediate, unknown.
   Async IMU_SHORT within tolerance never waits. Misses are a NEW counter
   (`imu_orientation_misses`, diag KV + throttled WARN naming
   outputs.epoch_tolerance_us); `composition_drops` is /odom-only now (it used
   to count every IMU log published without orientation — 100/s forever with
   IMU faster than EKF — and its WARN blamed the device config).
3. **EKF validity gating.** to_imu emits the orientation sentinel when the
   quat's status is mode UNINITIALIZED or ATTITUDE_VALID clear (SDK: data
   invalid); HEADING_VALID clear floors the yaw variance at
   `k_unknown_yaw_variance` (π²/3, uniform over ±π). to_odometry mirrors it with
   `k_unavailable_variance` on the orientation block / linear-twist block
   (VELOCITY_VALID on EkfVelBody) — values still published, Odometry has no
   sentinel. Also: reset_stream_state() now clears `last_gnss_pos_status_`.
New `test/publishers_test.cpp` — the first Publishers harness: lifecycle node
with intra-process comms so delivery is deterministic (no discovery), feeds
synthetic LogViews and reads /imu/data back; 10 scenarios (hold → pair,
no-quat-stream immediate, retire-by-next-sample + miss count, quat moving past,
async within tolerance, disowned quat, auto prefers short, high-range gyro
scale, imu_data ignores short, activate resets latch). doc/parameters.md
regenerated with `generate_parameter_library_markdown` (also picks up the v0.5.0
/ekf/fix text it had missed). Verified in the dev container: build clean,
test_conversions 55/55, test_publishers 10/10, test_param_conversions 9/9,
replay launch test green, clang-format clean.

### rig launcher-contract sync (template v0.2.12–v0.2.14, 2026-06-10)
rig v0.1.17/18 (at `~/ws/bringup`, public github.com/christomaszewski/rig) made
the launcher contract executable (`rig certify`) and its STATE.md named sbg-up's
`-p` override as a known violation. Implemented template-first, then copier-updated:
**v0.2.12** — sbg-up honors rig's injected `COMPOSE_PROJECT_NAME`
(`<name>-vehicle-<id>`; standalone fallback `sbg_<name>`, never passes `-p`);
compose pulls `:${RIG_IMAGE_TAG:-latest}`; rigging.yaml declares the BUILD phase
(`tools/build_image.sh` + `images: [sbg_driver]`) so `rig build` covers sbg;
in-repo `sensors/sbg.example.yaml`; ci.yml `certify` job (checks out the public
rig repo, certifies on every push — the same gate camera-service got via its
PR #36); render_params.py's phantom `tcp` branch removed (transports now gated
per scaffold; unsupported types fail loudly). **v0.2.13** — docker/README's
inline example no longer says `type: tcp` / `frames.data`. **v0.2.14** — the
baked healthcheck had NEVER worked (three stacked bugs: `grep "level: 0"` vs the
byte rendering `level: "\x01"`; `--field status` printing Python repr, not YAML;
sbg's diverged copy crashing on `set -u` + setup.bash) and was conceptually
wrong anyway: `/diagnostics` is an ABSOLUTE topic, so on a host-networked
vehicle a per-container probe can sample a NEIGHBOR's health (observed live —
the sbg probe read a running novatel container's diagnostics). Redesigned:
healthy iff `ros2 lifecycle get ${SBG_NAMESPACE}/sbg_driver` reports `active`;
compose passes `SBG_NAMESPACE` through; HEALTHCHECK gets
`--timeout=15s --start-period=30s`. Verified: `rig certify` 8/8 (baseline before:
1 error project-name, tag check silently skipped for lack of `build:`);
standalone + rig-injected project naming both live-tested; rebuilt arm64 image
3-case healthcheck test (no node → 1, active `/front` node → 0, wrong ns → 1 —
the probe's first-ever pass). When container-testing on this Mac, set a random
`ROS_DOMAIN_ID` — sibling project containers cross-talk on domain 0.

## 2026-06 deep-review backlog (4-angle, @ e3e4c89) — FIXED 2026-06-10

Four parallel reviewers (conversions math; concurrency/lifecycle; core lib vs
sbgECom SDK; params/launch/deploy) + personal spot-checks of every item below.
Deploy-surface findings were fixed same-day (v0.2.12–14 above).

**RESOLUTION (commits `3f2d7a4` core, `03d679a` driver, `0171e61` launch/docs,
`dae8a52` template v0.2.15 + stale-render re-adoption): every numbered item
below is fixed.** Notes on the judgment calls:
- Mag units (#3): `imu.mag_scale` param, default 1.0 = raw a.u. (upstream-
  compatible) rather than guessing a Tesla factor; documented at schema/header/
  topic level.
- Stale-state reset (#8): cached EkfQuat/EkfVelBody/GGA-stamp/diag latches
  reset on activate; the geodetic origin deliberately SURVIVES the cycle (it
  anchors the local /odom frame; resetting would step every pose consumer).
- Dead TF params (#9): removed (not implemented) — map→odom belongs to the
  localization stack, base→imu to the URDF; `/sbg/save_settings` implemented.
- poll_once (#5): the SDK's absorb-everything loop is now the documented
  contract; link health = data-flow staleness (already in /diagnostics) +
  new callback_exception_count surfaced in the Device diagnostic. Dead
  backoff path kept only as a future-SDK guard.
- Runtime-reconfig claim (M9): documented the real contract (params read at
  configure; recycle to apply) instead of marking every group read_only,
  which would break the legitimate set-while-inactive → recycle workflow.
- Fork-PR CI (M7): container jobs skip on fork PRs (certify + shellcheck
  still run) — honest skip instead of guaranteed manifest-unknown failures.
- render_params fallback defaults (m1): sub-keys render only when user-
  provided, so the driver's schema defaults are the single source of truth.

Original findings list (kept for the record):

**Critical**
1. `publishers.cpp:~347` — geodetic origin latched from the FIRST EkfNav log
   with no validity gating (`if (!geodetic_origin_)`), but on cold start the
   INS streams EKF_NAV before alignment (solution mode 0 = UNINITIALIZED, data
   invalid) → origin latches at firmware-default position and, being sticky by
   design, permanently corrupts every later /odom + TF. conversions.hpp:162
   says "first **valid** fix" — the "valid" isn't implemented. Gate on
   `status & SBG_ECOM_SOL_POSITION_VALID` (bit 7) / solution mode ≥ NAV.

**Major — correctness**
2. `conversions.cpp:~411` — TimeReference published from the UTC log without
   checking the UTC-status nibble; before GNSS sync SBG sends the firmware
   default date flagged INVALID, and we publish it as authoritative.
3. `conversions.cpp:~211` — magnetometer is published as Tesla but
   sbgEComLogMag is "arbitrary units" (~1.0 ≈ Earth field) → reads as ~20,000×
   Earth's field. Scale (configurable, e.g. 5e-5 T/a.u.) or document loudly.
4. `device.cpp:~500` — save_mag_calibration uploads hardcoded
   `SBG_ECOM_MAG_CALIB_MODE_3D` even when the session was started TwoD (the
   comment acknowledges the assumption). Cache the mode from
   start_mag_calibration and pass it through.
5. `device.cpp:~140` — `poll_once` can never fail: SDK `sbgEComHandle` only
   ever exits with SBG_NOT_READY (per-frame errors absorbed), so our error
   branch is dead code, run()'s backoff never fires, and an unplugged serial
   device polls "Ok" forever. Needs independent link-health (consecutive
   zero-data polls or counters) surfaced to diagnostics. Related: the callback
   trampoline's catch-all returns SBG_ERROR which the SDK ignores — exceptions
   vanish silently (device.hpp claims "catches and logs"; nothing logs).
6. `transport.cpp:~69` — transport.hpp documents UDP "connected mode by
   default" (filter to remote_ip) but `sbgInterfaceUdpSetConnectedMode` is
   never called — SDK default accepts frames from ANY host.
7. `transport.cpp:~83` — `FileReplay::real_time_pace` is plumbed end-to-end
   (param, launch arg, examples) but implemented nowhere — replay always runs
   flat-out. Implement pacing or return Unsupported.
8. `publishers.hpp/.cpp` — stale stream state across deactivate→activate:
   `last_quat_`, `last_vel_body_`, `geodetic_origin_`, GGA stamp, and the diag
   atomics survive the cycle (the exact cycle the mag-cal flow prescribes), so
   reactivation composes odometry from pre-reboot orientation/velocity. Add a
   `reset_stream_state()` in `activate()`.
9. Schema/docs vs reality: `frames.map`, `tf.broadcast_map_to_odom`,
   `tf.broadcast_base_to_imu` declared + README'd but consumed NOWHERE; the
   `configure_device.enable` description and the provisioning log tell users to
   call `/sbg/save_settings`, a service that doesn't exist (only the two
   mag-cal services do). Implement or remove.
10. `bringup.launch.py:~70` — the auto-activate `OnStateTransition(goal_state=
    'inactive')` handler has no start_state filter and never unregisters: a
    failed activate retries forever with no backoff, and a deliberate
    `lifecycle set deactivate` is instantly re-activated.
    `replay.launch.py` meanwhile still uses the configure+activate-on-start
    pattern whose race the test launcher explicitly fixed.

**Minor (selection — see review transcript)**
- GGA renders SDK "not available" sentinels (numSvUsed 0xFF → "99" sats,
  diffAge 0xFFFF → 99.9 s, station 0xFFFF → "9999") instead of empty fields;
  no NO_SOLUTION gate.
- GPS1_POS and GPS2_POS both map to Kind::GnssPos → dual-antenna units
  interleave two positions on /gps/fix and alternate the VRS GGA position.
- `geodetic_to_local` doesn't wrap Δlon across the antimeridian.
- `LogView::time_stamp_us()` returns GPS time-of-week **ms** for the 3 GNSS
  kinds (every other kind: µs-since-power-up); sbg_decode prints it as "us".
  RTCM_RAW logs classify as Unknown though the header comment claims otherwise.
- Relaxed-ordering diag atomics can expose flag/value inconsistency for one
  updater tick (cosmetic); `run_active` Configurator guard is advisory and set
  too late to enforce (in-tree call sites are safe; document or fix).
- `compose.hil.yaml`: `ghcr.io/ckt/...` image (never published), mounts a
  `hil_params/` dir that doesn't exist, dead `SBG_DRIVER_CONFIG` env.
- CI: fork PRs can't pass (image `load:`ed but containers pull the unpushed
  sha tag); lint job runs only clang-format while build-test claims the rest
  is "checked in the dedicated lint job".
- README's "dynamically reconfigurable" overstates: params are read at
  configure only; runtime sets silently no-op until cleanup→configure.
- package.xml maintainer is `me@example.com` placeholder; launch files need an
  `exec_depend` on ros2launch; migration.md says output-rate config is backlog
  (it shipped); compose.replay.yaml references a sample .bin + rviz config not
  in the repo; render_params/sbg-up identity tokens unvalidated (`eval` safety).

**Reviewer-verified clean (high-value confirmations)**: NED→ENU quaternion
(axis-swap correct, bit-identical to reference, algebraically re-derived);
covariance ENU ordering [lon²,lat²,alt²] (fixes a reference-driver bug); WGS84
M/N radii exact; UTC→UNIX chrono composition; all 17 LogView union accesses
match the SDK dispatch; every to_sbg() enum value-for-value vs SDK headers;
write_rtcm concurrency safe at the SDK level for serial+UDP (rx path never
writes); teardown/destruction ordering safe (jthread joined before publishers/
device die); on_shutdown/on_error idempotent; one_of↔mapper parity exact in
both directions for all 7 param families; rigging launch_surface paths all
exist; QoS/intra-process wiring sound.

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
