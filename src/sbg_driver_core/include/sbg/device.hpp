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
#include <cstdint>
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
//   * `set_log_callback()` must NOT be called while a `run()` loop is active
//     (it would race the C callback trampoline).
//   * `write_rtcm()` IS safe to call concurrently with `run()` / `poll_once()`:
//     the transport write path is independent of the read path on serial/UDP.
//   * Configurator commands must not run concurrently with `run()` /
//     `poll_once()` (sbgECom is request/response; a concurrent poll steals
//     the reply). Stop and join the I/O thread first — that join is the
//     contract. Configurator's internal `run_active` check is only a
//     best-effort guard: it is set by run() on the I/O thread, so there is
//     a window between spawning the thread and the flag being set where the
//     check passes. Do not rely on Error::DeviceBusy for safety.
//   * `info()` is safe to call from any thread after `open()` returns.
class Device
{
public:
  // Construct and open in one step. Validates the transport, opens the
  // SbgInterface, and initializes the SbgEComHandle. DeviceInfo is NOT
  // queried yet (it needs a live responder, which file replay never has);
  // info() currently returns a default-constructed struct.
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

  // Drain all logs currently available on the transport (one synchronous
  // `sbgEComHandle()` call; the budget is advisory and unused — kept for API
  // symmetry with future async transports).
  //
  // HONEST ERROR CONTRACT: with sbgECom 5.6 this returns Ok in ALL normal
  // operation, including a dead transport — the SDK's dispatch loop absorbs
  // per-frame errors (CRC, parse, read failures) and always exits
  // SBG_NOT_READY. Judge link health from data flow (your callback's last-log
  // timestamp), not from this return value. The only real failure mode is
  // NotReady on a moved-from Device.
  [[nodiscard]] Result<void> poll_once(
    std::chrono::milliseconds budget = std::chrono::milliseconds{4});

  // Block until `stop` requests stop, calling `poll_once(budget)` in a loop.
  // Designed to be driven by a std::jthread:
  //
  //     std::jthread io_thread{[&](std::stop_token st) { device.run(st); }};
  //
  // Exceptions thrown by the user callback never escape into the C SDK: they
  // are swallowed at the boundary and counted — poll the count via
  // callback_exception_count() (e.g. from a diagnostics task). When the
  // transport is FileReplay with `real_time_pace`, dispatch is throttled so
  // logs are delivered at their recorded cadence (per-log payload timestamps,
  // capped at 250 ms of wait per log).
  //
  // Stop latency does NOT depend on the length of the capture. A single
  // poll_once() cannot be interrupted (for FileReplay it returns only at
  // EOF), so instead a requested stop makes the remainder of that drain
  // cheap: pacing waits are served in 10 ms slices and abandon as soon as
  // stop is seen, and no further logs are dispatched to the callback. What
  // remains is parse-only work proportional to the bytes left in the file,
  // not to their recorded duration.
  void run(std::stop_token stop, std::chrono::milliseconds budget = std::chrono::milliseconds{4});

  // Total user-callback exceptions swallowed at the C boundary since open().
  // Monotonic; safe to read from any thread.
  [[nodiscard]] std::uint64_t callback_exception_count() const noexcept;

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
