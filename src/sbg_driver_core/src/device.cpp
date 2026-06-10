// Copyright 2026 sbg_driver maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sbg/device.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <optional>
#include <thread>
#include <utility>
#include <variant>

#include "sbg/configurator.hpp"
#include "sbg/detail/c_api.hpp"
#include "sbg/log_view.hpp"

namespace sbg
{

// ---------------------------------------------------------------------------
// Device::Impl
//
// Holds the SbgEComHandle and the owned Transport (so Device's move semantics
// are trivial: just move the unique_ptr).
// ---------------------------------------------------------------------------

struct Device::Impl
{
  Transport transport;
  SbgEComHandle handle{};
  LogCallback log_callback{};
  std::atomic_flag run_active = ATOMIC_FLAG_INIT;
  bool initialized = false;

  // Count of user-callback exceptions swallowed at the C boundary. The SDK's
  // dispatch loop ignores per-log return codes, so this counter (surfaced via
  // Device::callback_exception_count()) is the only way a failing callback
  // becomes observable.
  std::atomic<std::uint64_t> callback_exceptions{0};

  // Cache of the last compute_mag_calibration() result — save_mag_calibration_results()
  // uploads the offset[3] + matrix[9] stored here. Reset when start_mag_calibration() is
  // called or when the cached results have been uploaded once.
  std::optional<SbgEComMagCalibResults> last_mag_calib;
  // Collection mode of the in-progress / last-started calibration session.
  // save_mag_calibration_results() must upload the SAME mode the samples were
  // collected in — a 2D-collected compensation tagged 3D misleads the filter.
  SbgEComMagCalibMode last_mag_calib_mode = SBG_ECOM_MAG_CALIB_MODE_3D;

  // File-replay pacing (transport is FileReplay with real_time_pace=true):
  // the first timestamped log anchors (log time → wall clock); each later log
  // sleeps until its offset from that anchor. Re-anchors if timestamps jump
  // backwards (uint32 µs wrap at ~71.6 min, or a restarted capture).
  bool pace_replay = false;
  std::optional<std::uint32_t> pace_anchor_ts;
  std::chrono::steady_clock::time_point pace_anchor_wall{};

  explicit Impl(Transport t) noexcept : transport(std::move(t)) {}

  ~Impl()
  {
    if (initialized) {
      sbgEComClose(&handle);
    }
  }

  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

  void pace(std::uint32_t ts_us) noexcept
  {
    if (ts_us == 0) {
      return;  // log kind without a payload timestamp — nothing to pace on
    }
    const auto now = std::chrono::steady_clock::now();
    if (!pace_anchor_ts || ts_us < *pace_anchor_ts) {
      pace_anchor_ts = ts_us;
      pace_anchor_wall = now;
      return;
    }
    const auto offset = std::chrono::microseconds{ts_us - *pace_anchor_ts};
    const auto target = pace_anchor_wall + offset;
    if (target > now) {
      // Cap each sleep so a sparse capture can't stall the I/O thread (and
      // its stop/join) for more than ~250 ms per log.
      std::this_thread::sleep_for(
        std::min<std::chrono::steady_clock::duration>(
          target - now, std::chrono::milliseconds{250}));
    }
  }

  // C trampoline. The SDK calls this with `pUserArg = this`.
  static SbgErrorCode on_log(
    SbgEComHandle * /*handle*/, SbgEComClass msg_class, SbgEComMsgId msg_id,
    const SbgEComLogUnion * log_data, void * user_arg) noexcept
  {
    auto * self = static_cast<Impl *>(user_arg);
    if (self == nullptr || !self->log_callback) {
      return SBG_NO_ERROR;
    }
    LogView view{
      static_cast<std::uint8_t>(msg_class), static_cast<std::uint16_t>(msg_id), log_data};
    if (self->pace_replay) {
      self->pace(view.time_stamp_us());
    }
    try {
      self->log_callback(view);
    } catch (...) {
      // Never let exceptions cross back into the C SDK. The SDK's dispatch
      // loop ignores this return code (it drains until SBG_NOT_READY), so the
      // counter — not the return value — is what makes the failure visible.
      self->callback_exceptions.fetch_add(1, std::memory_order_relaxed);
      return SBG_ERROR;
    }
    return SBG_NO_ERROR;
  }
};

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

Result<Device> Device::open(TransportConfig cfg)
{
  auto transport_result = Transport::open(std::move(cfg));
  if (!transport_result) {
    return std::unexpected(transport_result.error());
  }

  auto impl = std::make_unique<Impl>(std::move(*transport_result));
  auto * iface = static_cast<SbgInterface *>(impl->transport.native_handle());
  if (iface == nullptr) {
    return std::unexpected(Error::Internal);
  }

  if (auto code = sbgEComInit(&impl->handle, iface); code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  impl->initialized = true;

  // Wire the trampoline now so callbacks set later via set_log_callback work
  // without needing to re-register.
  sbgEComSetReceiveLogCallback(&impl->handle, &Impl::on_log, impl.get());

  // Real-time pacing is a replay-only concern; live transports deliver at
  // the device's own cadence.
  if (const auto * replay = std::get_if<transport::FileReplay>(&impl->transport.config());
      replay != nullptr && replay->real_time_pace) {
    impl->pace_replay = true;
  }

  Device dev{std::move(impl)};
  // DeviceInfo population is deferred (querying it needs a live responder,
  // which file replay never has) — info() returns a default-constructed
  // struct for now; see device.hpp.
  return dev;
}

Device::Device(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}

Device::~Device() = default;
Device::Device(Device &&) noexcept = default;
Device & Device::operator=(Device &&) noexcept = default;

void Device::set_log_callback(LogCallback cb)
{
  if (impl_ == nullptr) {
    return;
  }
  // Lightweight guard: a run() loop sets this flag; refusing to swap the
  // callback under it avoids a race where the trampoline reads a half-moved
  // std::function. Rejecting silently for now — a future phase can return
  // Result<void> instead.
  if (impl_->run_active.test()) {
    return;
  }
  impl_->log_callback = std::move(cb);
}

Result<void> Device::poll_once(std::chrono::milliseconds /*budget*/)
{
  if (impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  // The C SDK's sbgEComHandle drains all available frames synchronously and
  // — by its own loop structure — only ever exits with SBG_NOT_READY:
  // per-frame errors (CRC, parse, read failures) are absorbed inside the
  // SDK. The error branch below is therefore unreachable with sbgECom 5.6;
  // it is kept as a guard for future SDK versions. Link health must be
  // judged by the caller from data flow (see header).
  auto code = sbgEComHandle(&impl_->handle);
  if (code == SBG_NO_ERROR || code == SBG_NOT_READY) {
    return {};
  }
  return std::unexpected(detail::from_sbg(code));
}

std::uint64_t Device::callback_exception_count() const noexcept
{
  return impl_ != nullptr ? impl_->callback_exceptions.load(std::memory_order_relaxed) : 0;
}

Result<void> Device::write_rtcm(std::span<const std::byte> data)
{
  if (impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (data.empty()) {
    return {};  // nothing to send is not an error
  }
  // sbgInterfaceWrite goes through the transport's write_func, which on
  // POSIX serial / UDP is independent of the read path used by the I/O
  // thread - safe to call concurrently with poll_once().
  auto * iface = static_cast<SbgInterface *>(impl_->transport.native_handle());
  if (iface == nullptr) {
    return std::unexpected(Error::Internal);
  }
  const auto code = sbgInterfaceWrite(iface, data.data(), data.size());
  if (code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return {};
}

Configurator Device::configurator() noexcept { return Configurator{*this}; }

// ---------------------------------------------------------------------------
// Configurator implementations
//
// All methods refuse to run while the I/O thread is active (run_active flag
// set). Calling them after a Device::run loop completes — or before it
// starts — is safe.
// ---------------------------------------------------------------------------

namespace
{
[[nodiscard]] MagCalibQuality from_sbg_quality(SbgEComMagCalibQuality q) noexcept
{
  switch (q) {
    case SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL:
      return MagCalibQuality::Optimal;
    case SBG_ECOM_MAG_CALIB_QUAL_GOOD:
      return MagCalibQuality::Good;
    case SBG_ECOM_MAG_CALIB_QUAL_POOR:
      return MagCalibQuality::Poor;
    case SBG_ECOM_MAG_CALIB_QUAL_INVALID:
      return MagCalibQuality::Invalid;
  }
  return MagCalibQuality::Invalid;
}

[[nodiscard]] MagCalibConfidence from_sbg_confidence(SbgEComMagCalibConfidence c) noexcept
{
  switch (c) {
    case SBG_ECOM_MAG_CALIB_TRUST_HIGH:
      return MagCalibConfidence::High;
    case SBG_ECOM_MAG_CALIB_TRUST_MEDIUM:
      return MagCalibConfidence::Medium;
    case SBG_ECOM_MAG_CALIB_TRUST_LOW:
      return MagCalibConfidence::Low;
  }
  return MagCalibConfidence::Low;
}

[[nodiscard]] MagCalibResults to_results(const SbgEComMagCalibResults & raw) noexcept
{
  return MagCalibResults{
    .quality = from_sbg_quality(raw.quality),
    .confidence = from_sbg_confidence(raw.confidence),
    .advanced_status = raw.advancedStatus,
    .num_points = raw.numPoints,
    .max_num_points = raw.maxNumPoints,
    .mean_accuracy_rad = raw.meanAccuracy,
    .std_accuracy_rad = raw.stdAccuracy,
    .max_accuracy_rad = raw.maxAccuracy,
    .before_mean_error = raw.beforeMeanError,
    .before_std_error = raw.beforeStdError,
    .before_max_error = raw.beforeMaxError,
    .after_mean_error = raw.afterMeanError,
    .after_std_error = raw.afterStdError,
    .after_max_error = raw.afterMaxError,
  };
}

// ---- Provisioning enum translations (sbg:: → sbgECom) ----

[[nodiscard]] constexpr SbgEComMotionProfileStdIds to_sbg(MotionProfile p) noexcept
{
  using enum MotionProfile;
  switch (p) {
    case GeneralPurpose:
      return SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE;
    case Automotive:
      return SBG_ECOM_MOTION_PROFILE_AUTOMOTIVE;
    case Marine:
      return SBG_ECOM_MOTION_PROFILE_MARINE;
    case Airplane:
      return SBG_ECOM_MOTION_PROFILE_AIRPLANE;
    case Helicopter:
      return SBG_ECOM_MOTION_PROFILE_HELICOPTER;
    case Pedestrian:
      return SBG_ECOM_MOTION_PROFILE_PEDESTRIAN;
    case UavRotaryWing:
      return SBG_ECOM_MOTION_PROFILE_UAV_ROTARY_WING;
    case HeavyMachinery:
      return SBG_ECOM_MOTION_PROFILE_HEAVY_MACHINERY;
    case Static:
      return SBG_ECOM_MOTION_PROFILE_STATIC;
    case Truck:
      return SBG_ECOM_MOTION_PROFILE_TRUCK;
    case Railway:
      return SBG_ECOM_MOTION_PROFILE_RAILWAY;
    case OffRoadVehicle:
      return SBG_ECOM_MOTION_PROFILE_OFF_ROAD_VEHICLE;
    case Underwater:
      return SBG_ECOM_MOTION_PROFILE_UNDERWATER;
  }
  return SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE;
}

[[nodiscard]] constexpr SbgEComModulePortAssignment to_sbg(PortAssignment p) noexcept
{
  using enum PortAssignment;
  switch (p) {
    case PortA:
      return SBG_ECOM_MODULE_PORT_A;
    case PortB:
      return SBG_ECOM_MODULE_PORT_B;
    case PortC:
      return SBG_ECOM_MODULE_PORT_C;
    case PortD:
      return SBG_ECOM_MODULE_PORT_D;
    case PortE:
      return SBG_ECOM_MODULE_PORT_E;
    case Internal:
      return SBG_ECOM_MODULE_INTERNAL;
    case Disabled:
      return SBG_ECOM_MODULE_DISABLED;
  }
  return SBG_ECOM_MODULE_DISABLED;
}

[[nodiscard]] constexpr SbgEComAxisDirection to_sbg(AxisDirection d) noexcept
{
  using enum AxisDirection;
  switch (d) {
    case Forward:
      return SBG_ECOM_ALIGNMENT_FORWARD;
    case Backward:
      return SBG_ECOM_ALIGNMENT_BACKWARD;
    case Left:
      return SBG_ECOM_ALIGNMENT_LEFT;
    case Right:
      return SBG_ECOM_ALIGNMENT_RIGHT;
    case Up:
      return SBG_ECOM_ALIGNMENT_UP;
    case Down:
      return SBG_ECOM_ALIGNMENT_DOWN;
  }
  return SBG_ECOM_ALIGNMENT_FORWARD;
}

[[nodiscard]] constexpr SbgEComMagModelsStdId to_sbg(MagModel m) noexcept
{
  switch (m) {
    case MagModel::InternalNormal:
      return SBG_ECOM_MAG_MODEL_INTERNAL_NORMAL;
    case MagModel::ExternalEcom:
      return SBG_ECOM_MAG_MODEL_ECOM_NORMAL;
  }
  return SBG_ECOM_MAG_MODEL_INTERNAL_NORMAL;
}

[[nodiscard]] constexpr SbgEComGnssInstallationMode to_sbg(GnssInstallationMode m) noexcept
{
  using enum GnssInstallationMode;
  switch (m) {
    case Single:
      return SBG_ECOM_GNSS_INSTALLATION_MODE_SINGLE;
    case DualAuto:
      return SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_AUTO;
    case DualRough:
      return SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_ROUGH;
    case DualPrecise:
      return SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE;
  }
  return SBG_ECOM_GNSS_INSTALLATION_MODE_SINGLE;
}

[[nodiscard]] constexpr SbgEComMsgId to_sbg_msg_id(OutputLog log) noexcept
{
  using enum OutputLog;
  // All exposed output logs live in the standard ECOM_0 class; the caller
  // (set_output) supplies SBG_ECOM_CLASS_LOG_ECOM_0. SbgEComMsgId is uint8, so
  // cast the SbgEComLog enumerator once at the end.
  SbgEComLog id = SBG_ECOM_LOG_IMU_DATA;
  switch (log) {
    case ImuShort:
      id = SBG_ECOM_LOG_IMU_SHORT;
      break;
    case ImuData:
      id = SBG_ECOM_LOG_IMU_DATA;
      break;
    case EkfEuler:
      id = SBG_ECOM_LOG_EKF_EULER;
      break;
    case EkfQuat:
      id = SBG_ECOM_LOG_EKF_QUAT;
      break;
    case EkfNav:
      id = SBG_ECOM_LOG_EKF_NAV;
      break;
    case EkfVelBody:
      id = SBG_ECOM_LOG_EKF_VEL_BODY;
      break;
    case Mag:
      id = SBG_ECOM_LOG_MAG;
      break;
    case GnssPos:
      id = SBG_ECOM_LOG_GPS1_POS;
      break;
    case GnssVel:
      id = SBG_ECOM_LOG_GPS1_VEL;
      break;
    case GnssHdt:
      id = SBG_ECOM_LOG_GPS1_HDT;
      break;
    case GnssRaw:
      id = SBG_ECOM_LOG_GPS1_RAW;
      break;
    case Utc:
      id = SBG_ECOM_LOG_UTC_TIME;
      break;
    case Status:
      id = SBG_ECOM_LOG_STATUS;
      break;
    case AirData:
      id = SBG_ECOM_LOG_AIR_DATA;
      break;
    case ShipMotion:
      id = SBG_ECOM_LOG_SHIP_MOTION;
      break;
    case EventA:
      id = SBG_ECOM_LOG_EVENT_A;
      break;
    case EventB:
      id = SBG_ECOM_LOG_EVENT_B;
      break;
    case EventC:
      id = SBG_ECOM_LOG_EVENT_C;
      break;
    case EventD:
      id = SBG_ECOM_LOG_EVENT_D;
      break;
    case EventE:
      id = SBG_ECOM_LOG_EVENT_E;
      break;
  }
  return static_cast<SbgEComMsgId>(id);
}

[[nodiscard]] constexpr SbgEComOutputMode to_sbg(OutputRate r) noexcept
{
  using enum OutputRate;
  switch (r) {
    case Disabled:
      return SBG_ECOM_OUTPUT_MODE_DISABLED;
    case MainLoop:
      return SBG_ECOM_OUTPUT_MODE_MAIN_LOOP;
    case Div2:
      return SBG_ECOM_OUTPUT_MODE_DIV_2;
    case Div4:
      return SBG_ECOM_OUTPUT_MODE_DIV_4;
    case Div5:
      return SBG_ECOM_OUTPUT_MODE_DIV_5;
    case Div8:
      return SBG_ECOM_OUTPUT_MODE_DIV_8;
    case Div10:
      return SBG_ECOM_OUTPUT_MODE_DIV_10;
    case Div20:
      return SBG_ECOM_OUTPUT_MODE_DIV_20;
    case Div40:
      return SBG_ECOM_OUTPUT_MODE_DIV_40;
    case Div100:
      return SBG_ECOM_OUTPUT_MODE_DIV_100;
    case Div200:
      return SBG_ECOM_OUTPUT_MODE_DIV_200;
    case NewData:
      return SBG_ECOM_OUTPUT_MODE_NEW_DATA;
    case Pps:
      return SBG_ECOM_OUTPUT_MODE_PPS;
  }
  return SBG_ECOM_OUTPUT_MODE_DISABLED;
}

[[nodiscard]] constexpr SbgEComOutputPort to_sbg(OutputPort p) noexcept
{
  using enum OutputPort;
  switch (p) {
    case Main:
      return SBG_ECOM_OUTPUT_PORT_A;
    case PortC:
      return SBG_ECOM_OUTPUT_PORT_C;
    case PortE:
      return SBG_ECOM_OUTPUT_PORT_E;
  }
  return SBG_ECOM_OUTPUT_PORT_A;
}
}  // namespace

Result<void> Configurator::ready() const noexcept
{
  if (device_->impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (device_->impl_->run_active.test()) {
    return std::unexpected(Error::DeviceBusy);
  }
  return {};
}

Result<void> Configurator::start_mag_calibration(MagCalibMode mode)
{
  return ready().and_then([&]() -> Result<void> {
    device_->impl_->last_mag_calib.reset();  // drop any stale results
    // Remember the collection mode: the upload step must tag the computed
    // compensation with the SAME mode the samples were gathered in.
    device_->impl_->last_mag_calib_mode = static_cast<SbgEComMagCalibMode>(mode);
    // Bandwidth was deprecated in SDK v3.x; HIGH is the modern recommended value.
    return detail::check(sbgEComCmdMagStartCalib(
      &device_->impl_->handle, static_cast<SbgEComMagCalibMode>(mode), SBG_ECOM_MAG_CALIB_HIGH_BW));
  });
}

Result<MagCalibResults> Configurator::compute_mag_calibration()
{
  return ready().and_then([&]() -> Result<MagCalibResults> {
    SbgEComMagCalibResults raw{};
    return detail::check(sbgEComCmdMagComputeCalib(&device_->impl_->handle, &raw)).transform([&] {
      device_->impl_->last_mag_calib = raw;  // cache for upload step
      return to_results(raw);
    });
  });
}

Result<void> Configurator::save_mag_calibration_results()
{
  return ready().and_then([&]() -> Result<void> {
    if (!device_->impl_->last_mag_calib) {
      return std::unexpected(Error::NotReady);  // must compute_mag_calibration() first
    }
    const auto & raw = *device_->impl_->last_mag_calib;
    // Upload tagged with the mode the session was started in (2D compensation
    // is only valid near-level; mis-tagging it 3D misleads the heading filter).
    return detail::check(sbgEComCmdMagSetCalibData2(
      &device_->impl_->handle, raw.offset, raw.matrix, device_->impl_->last_mag_calib_mode));
  });
}

Result<void> Configurator::save_settings()
{
  return ready().and_then([&]() -> Result<void> {
    return detail::check(sbgEComCmdSettingsAction(&device_->impl_->handle, SBG_ECOM_SAVE_SETTINGS));
  });
}

Result<void> Configurator::set_motion_profile(MotionProfile profile)
{
  return ready().and_then([&]() -> Result<void> {
    return detail::check(
      sbgEComCmdSensorSetMotionProfileId(&device_->impl_->handle, to_sbg(profile)));
  });
}

Result<void> Configurator::set_imu_alignment(const ImuAlignment & alignment)
{
  return ready().and_then([&]() -> Result<void> {
    const SbgEComSensorAlignmentInfo info{
      .axisDirectionX = to_sbg(alignment.axis_x),
      .axisDirectionY = to_sbg(alignment.axis_y),
      .misRoll = alignment.mis_roll,
      .misPitch = alignment.mis_pitch,
      .misYaw = alignment.mis_yaw,
    };
    return detail::check(sbgEComCmdSensorSetAlignmentAndLeverArm(
      &device_->impl_->handle, &info, alignment.lever_arm.data()));
  });
}

Result<void> Configurator::set_aiding_assignment(const AidingAssignment & assignment)
{
  return ready().and_then([&]() -> Result<void> {
    // Get-modify-set: preserve the sync-signal and odometer-pin fields we do
    // not expose, so this never clobbers unrelated device configuration.
    SbgEComAidingAssignConf conf{};
    if (auto r = detail::check(sbgEComCmdSensorGetAidingAssignment(&device_->impl_->handle, &conf));
        !r) {
      return r;
    }
    conf.gps1Port = to_sbg(assignment.gps1_port);
    conf.rtcmPort = to_sbg(assignment.rtcm_port);
    conf.dvlPort = to_sbg(assignment.dvl_port);
    conf.airDataPort = to_sbg(assignment.air_data_port);
    return detail::check(sbgEComCmdSensorSetAidingAssignment(&device_->impl_->handle, &conf));
  });
}

Result<void> Configurator::set_gnss_installation(const GnssInstallation & installation)
{
  return ready().and_then([&]() -> Result<void> {
    SbgEComGnssInstallation inst{};
    inst.leverArmPrimary[0] = installation.lever_arm_primary[0];
    inst.leverArmPrimary[1] = installation.lever_arm_primary[1];
    inst.leverArmPrimary[2] = installation.lever_arm_primary[2];
    inst.leverArmPrimaryPrecise = true;  // caller specified it explicitly
    inst.leverArmSecondary[0] = installation.lever_arm_secondary[0];
    inst.leverArmSecondary[1] = installation.lever_arm_secondary[1];
    inst.leverArmSecondary[2] = installation.lever_arm_secondary[2];
    inst.leverArmSecondaryMode = to_sbg(installation.secondary_mode);
    return detail::check(sbgEComCmdGnss1InstallationSet(&device_->impl_->handle, &inst));
  });
}

Result<void> Configurator::set_magnetometer_model(MagModel model)
{
  return ready().and_then([&]() -> Result<void> {
    return detail::check(sbgEComCmdMagSetModelId(&device_->impl_->handle, to_sbg(model)));
  });
}

Result<void> Configurator::set_output(OutputLog log, OutputRate rate, OutputPort port)
{
  return ready().and_then([&]() -> Result<void> {
    return detail::check(sbgEComCmdOutputSetConf(
      &device_->impl_->handle, to_sbg(port), SBG_ECOM_CLASS_LOG_ECOM_0, to_sbg_msg_id(log),
      to_sbg(rate)));
  });
}

void Device::run(std::stop_token stop, std::chrono::milliseconds budget)
{
  if (impl_ == nullptr) {
    return;
  }
  impl_->run_active.test_and_set();
  while (!stop.stop_requested()) {
    if (!poll_once(budget)) {
      // Unreachable with sbgECom 5.6 (the SDK absorbs per-frame errors and
      // sbgEComHandle always exits SBG_NOT_READY) — kept as a guard for
      // future SDKs so a genuinely failing poll can't hot-spin.
      std::this_thread::sleep_for(budget);
    }
    // Yield to scheduler regardless to ensure stop_token is observed.
    std::this_thread::sleep_for(std::chrono::microseconds{200});
  }
  impl_->run_active.clear();
}

}  // namespace sbg
