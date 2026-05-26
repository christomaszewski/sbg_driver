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

#include <atomic>
#include <cstring>
#include <optional>
#include <thread>
#include <utility>

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

  // Cache of the last compute_mag_calibration() result — save_mag_calibration_results()
  // uploads the offset[3] + matrix[9] stored here. Reset when start_mag_calibration() is
  // called or when the cached results have been uploaded once.
  std::optional<SbgEComMagCalibResults> last_mag_calib;

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

  // C trampoline. The SDK calls this with `pUserArg = this`.
  static SbgErrorCode on_log(
    SbgEComHandle * /*handle*/, SbgEComClass msg_class, SbgEComMsgId msg_id,
    const SbgEComLogUnion * log_data, void * user_arg) noexcept
  {
    auto * self = static_cast<Impl *>(user_arg);
    if (self == nullptr || !self->log_callback) {
      return SBG_NO_ERROR;
    }
    try {
      LogView view{
        static_cast<std::uint8_t>(msg_class), static_cast<std::uint16_t>(msg_id), log_data};
      self->log_callback(view);
    } catch (...) {
      // Never let exceptions cross back into the C SDK. The callback is
      // user-supplied; we eat the exception and report ProtocolError so the
      // caller's outer loop can decide what to do.
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

  Device dev{std::move(impl)};
  // DeviceInfo population deferred to phase 6 (Configurator). For phase 1
  // we leave the struct default-constructed so the field types are correct.
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
  // The C SDK's sbgEComHandle drains all available frames synchronously.
  // We don't use the budget parameter yet — see header note.
  auto code = sbgEComHandle(&impl_->handle);
  if (code == SBG_NO_ERROR || code == SBG_NOT_READY) {
    return {};
  }
  return std::unexpected(detail::from_sbg(code));
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

Configurator Device::configurator() noexcept
{
  return Configurator{*this};
}

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
}  // namespace

Result<void> Configurator::start_mag_calibration(MagCalibMode mode)
{
  if (device_->impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (device_->impl_->run_active.test()) {
    return std::unexpected(Error::DeviceBusy);
  }
  device_->impl_->last_mag_calib.reset();  // drop any stale results
  const auto code = sbgEComCmdMagStartCalib(
    &device_->impl_->handle,
    static_cast<SbgEComMagCalibMode>(mode),
    // Bandwidth was deprecated in SDK v3.x; HIGH is the modern recommended value.
    SBG_ECOM_MAG_CALIB_HIGH_BW);
  if (code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return {};
}

Result<MagCalibResults> Configurator::compute_mag_calibration()
{
  if (device_->impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (device_->impl_->run_active.test()) {
    return std::unexpected(Error::DeviceBusy);
  }
  SbgEComMagCalibResults raw{};
  const auto code = sbgEComCmdMagComputeCalib(&device_->impl_->handle, &raw);
  if (code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  device_->impl_->last_mag_calib = raw;  // cache for upload step

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

Result<void> Configurator::save_mag_calibration_results()
{
  if (device_->impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (device_->impl_->run_active.test()) {
    return std::unexpected(Error::DeviceBusy);
  }
  if (!device_->impl_->last_mag_calib) {
    // Must call compute_mag_calibration() first.
    return std::unexpected(Error::NotReady);
  }
  const auto & raw = *device_->impl_->last_mag_calib;
  // The currently-active calibration mode is implicit in the device state;
  // we always upload as ThreeD which is the typical hardware default and
  // matches the SDK's recommended path on modern firmware.
  const auto code = sbgEComCmdMagSetCalibData2(
    &device_->impl_->handle, raw.offset, raw.matrix, SBG_ECOM_MAG_CALIB_MODE_3D);
  if (code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return {};
}

Result<void> Configurator::save_settings()
{
  if (device_->impl_ == nullptr) {
    return std::unexpected(Error::NotReady);
  }
  if (device_->impl_->run_active.test()) {
    return std::unexpected(Error::DeviceBusy);
  }
  const auto code =
    sbgEComCmdSettingsAction(&device_->impl_->handle, SBG_ECOM_SAVE_SETTINGS);
  if (code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return {};
}

void Device::run(std::stop_token stop, std::chrono::milliseconds budget)
{
  if (impl_ == nullptr) {
    return;
  }
  impl_->run_active.test_and_set();
  while (!stop.stop_requested()) {
    auto result = poll_once(budget);
    if (!result) {
      // Transient timeouts are expected when no data is arriving. Anything
      // else, we yield briefly to avoid hot-spinning on a broken transport.
      if (result.error() != Error::Timeout && result.error() != Error::NotReady) {
        std::this_thread::sleep_for(budget);
      }
    }
    // Yield to scheduler regardless to ensure stop_token is observed.
    std::this_thread::sleep_for(std::chrono::microseconds{200});
  }
  impl_->run_active.clear();
}

}  // namespace sbg
