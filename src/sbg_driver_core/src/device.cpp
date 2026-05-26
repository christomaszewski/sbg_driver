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
#include <thread>
#include <utility>

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
