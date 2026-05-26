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

#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <span>
#include <stop_token>
#include <string>

#include "sbg/configurator.hpp"
#include "sbg/error.hpp"
#include "sbg/transport.hpp"

namespace sbg
{

// Forward decl so users who don't touch log payloads don't pull in the
// (heavy) sbgECom headers via log_view.hpp.
class LogView;

// Lightweight device info pulled at open() time.
struct DeviceInfo
{
  std::string product_code;   // e.g. "ELLIPSE-N"
  std::string serial_number;  // hardware serial
  std::uint32_t firmware_rev = 0;
  std::uint32_t hardware_rev = 0;
};

// Device — central RAII object representing one connected SBG sensor.
//
// Owns: one SbgEComHandle, one Transport. Move-only.
//
// Threading contract:
//   * `poll_once()` / `run()` may be called from one I/O thread.
//   * The registered log callback fires on that I/O thread.
//   * `set_log_callback()` and `write_rtcm()` must NOT be called while
//     a `run()` loop is active (enforced by std::atomic_flag).
//   * `info()` is safe to call from any thread after `open()` returns.
class Device
{
public:
  // Construct and open in one step. Validates the transport, opens the
  // SbgInterface, initializes the SbgEComHandle, and queries device info.
  [[nodiscard]] static Result<Device> open(TransportConfig cfg);

  ~Device();
  Device(Device && other) noexcept;
  Device & operator=(Device && other) noexcept;
  Device(const Device &) = delete;
  Device & operator=(const Device &) = delete;

  // ---- Callback registration ---------------------------------------------

  using LogCallback = std::function<void(const LogView &)>;

  // Replaces any previously registered callback. Must NOT be called while
  // a `run()` loop is active (would race with the C trampoline).
  void set_log_callback(LogCallback cb);

  // ---- Polling ------------------------------------------------------------

  // Drain incoming logs up to `budget`. Returns Ok if at least one log was
  // dispatched and the underlying read completed cleanly; Timeout if the
  // budget expired with no logs; other Errors propagate from the C SDK.
  //
  // Implemented as a single `sbgEComHandle()` call — the budget is currently
  // advisory because the C SDK has its own internal timeout. We surface it
  // for API symmetry with future async transports.
  [[nodiscard]] Result<void> poll_once(
    std::chrono::milliseconds budget = std::chrono::milliseconds{4});

  // Block until `stop` requests stop, calling `poll_once(budget)` in a loop.
  // Designed to be driven by a std::jthread:
  //
  //     std::jthread io_thread{[&](std::stop_token st) { device.run(st); }};
  //
  // Catches and logs exceptions from the user callback; does not let them
  // escape into the C SDK.
  void run(std::stop_token stop, std::chrono::milliseconds budget = std::chrono::milliseconds{4});

  // ---- RTCM injection (DGPS/RTK corrections) -----------------------------

  // Push raw RTCM bytes through the underlying transport to the device.
  // Thread-safe relative to poll_once() / run() - the write path is
  // independent of the read path on serial and UDP transports. Empty `data`
  // is a no-op and returns Ok.
  [[nodiscard]] Result<void> write_rtcm(std::span<const std::byte> data);

  // ---- Configuration commands --------------------------------------------

  // Configurator façade for typed device-side commands (mag-cal, settings
  // persistence, future motion-profile / lever-arm / aiding setters).
  // Returned by value — Configurator is just a pointer wrapper.
  // NOT thread-safe relative to run() - stop the I/O thread first.
  [[nodiscard]] Configurator configurator() noexcept;

  // ---- Device info --------------------------------------------------------

  [[nodiscard]] const DeviceInfo & info() const noexcept { return info_; }

private:
  // pImpl: hides the C SDK handle from this header so consumers who don't
  // need log_view don't transitively pull in sbgECom.
  struct Impl;
  std::unique_ptr<Impl> impl_;
  DeviceInfo info_{};

  explicit Device(std::unique_ptr<Impl> impl) noexcept;

  // Configurator dives into impl_ to reach the SbgEComHandle + cached
  // mag-cal results. Methods are defined in device.cpp where Impl is
  // complete.
  friend class Configurator;
};

}  // namespace sbg
