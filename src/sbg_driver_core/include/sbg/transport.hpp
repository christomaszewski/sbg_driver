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

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "sbg/error.hpp"

namespace sbg
{

// ---------------------------------------------------------------------------
// TransportConfig — discriminated union of supported transports.
// ---------------------------------------------------------------------------

namespace transport
{

// Serial port, e.g. /dev/ttyUSB0 at 921600 baud.
struct Serial
{
  std::string port;             // OS path or COM port name
  std::uint32_t baud = 921600;  // Standard baud rate
  // Optional alternates the driver will try if the primary baud fails.
  // Empty by default; populate when auto-baud-search is desired.
  std::vector<std::uint32_t> fallback_bauds{};
};

// UDP transport. Connected mode (only accept from remote_ip:out_port) by default.
struct Udp
{
  std::string remote_ip;       // dotted-quad, e.g. "192.168.1.20"
  std::uint16_t in_port = 0;   // local listen port
  std::uint16_t out_port = 0;  // remote port
};

// Replay a captured .bin file as if it were a live device. Used by tests
// and by users debugging issues with recorded data.
struct FileReplay
{
  std::filesystem::path path;
  // Pace replay to the original recording rate (sleeps between frames).
  // Disable for fast offline processing.
  bool real_time_pace = true;
};

}  // namespace transport

using TransportConfig = std::variant<transport::Serial, transport::Udp, transport::FileReplay>;

// Validate a config's contents without opening anything (port format, baud
// in allowed set, IP non-empty, file path non-empty, etc.). Returns Ok on
// success or the most informative Error for the offending field.
[[nodiscard]] Result<void> validate(const TransportConfig & cfg) noexcept;

// ---------------------------------------------------------------------------
// Transport — RAII handle owning one initialized SbgInterface.
// ---------------------------------------------------------------------------

// Forward-declared to avoid pulling sbgECom headers into this public header.
struct TransportImpl;

class Transport
{
public:
  // Construct from a validated config. The underlying SbgInterface is opened
  // eagerly; an error here means the device/file/socket couldn't be reached.
  [[nodiscard]] static Result<Transport> open(TransportConfig cfg);

  ~Transport();
  Transport(Transport && other) noexcept;
  Transport & operator=(Transport && other) noexcept;
  Transport(const Transport &) = delete;
  Transport & operator=(const Transport &) = delete;

  // Internal: hand the underlying SbgInterface to the C SDK. Not part of
  // the public API stability promise — kept here so Device can reach it.
  // Lives in transport.cpp; defined in terms of TransportImpl*.
  [[nodiscard]] void * native_handle() noexcept;

  [[nodiscard]] const TransportConfig & config() const noexcept { return cfg_; }

private:
  Transport(std::unique_ptr<TransportImpl> impl, TransportConfig cfg) noexcept;

  // pImpl owns the SbgInterface; TransportImpl's destructor calls
  // sbgInterfaceDestroy, so Transport's special members are all defaulted.
  std::unique_ptr<TransportImpl> impl_;
  TransportConfig cfg_;
};

}  // namespace sbg
