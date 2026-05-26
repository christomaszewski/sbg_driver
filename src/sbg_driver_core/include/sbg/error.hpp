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
#include <expected>
#include <string_view>

namespace sbg
{

// Library error type. Wraps the sbgECom C SDK's SbgErrorCode into a stable
// C++ API surface; mapping happens at the boundary in detail/c_api.hpp.
enum class Error : std::uint8_t
{
  Ok = 0,
  Timeout,            // Read/write operation timed out
  InvalidParameter,   // Argument out of range, null, etc.
  NotReady,           // Device not connected or not yet initialized
  ProtocolError,      // Malformed frame, unexpected response, etc.
  TransportFailure,   // Underlying serial / socket / file I/O failed
  DeviceBusy,         // Command rejected because device is mid-operation
  BadCrc,             // CRC mismatch on a received frame
  Unsupported,        // Feature not supported by this device/firmware
  AllocationFailure,  // Memory allocation failed inside the C SDK
  CalibrationError,   // Magnetometer calibration could not be completed
  Internal,           // Catch-all for anything we haven't mapped
};

[[nodiscard]] std::string_view to_string(Error err) noexcept;

template <typename T>
using Result = std::expected<T, Error>;

}  // namespace sbg
