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

// INTERNAL HEADER — not part of the public C++ API.
//
// Includes the sbgECom C SDK and provides boundary helpers to keep the C
// headers from leaking into our public surface. Only sbg_driver_core's own
// .cpp files and sbg_driver_core's tests should include this.

#include <cstdint>

#include "sbg/error.hpp"

// clang-format off
extern "C" {
// sbgEComLib.h is the umbrella that pulls in everything except the message
// ID enums and the protocol log union. Keep the explicit list so that an
// upstream reorg can't silently break our compilation.
// NOLINTBEGIN(build/include_order) — sbgECom is third-party C, not a C system
// header. Intentionally grouped together inside this extern "C" block.
#include <sbgEComLib.h>
#include <sbgECanId.h>
#include <sbgECom.h>
#include <sbgEComIds.h>
#include <sbgErrorCodes.h>
#include <interfaces/sbgInterface.h>
#include <interfaces/sbgInterfaceFile.h>
#include <interfaces/sbgInterfaceSerial.h>
#include <interfaces/sbgInterfaceUdp.h>
#include <network/sbgNetwork.h>
// NOLINTEND(build/include_order)
}
// clang-format on

namespace sbg::detail
{

// Translate every SbgErrorCode produced by the C SDK into our typed Error.
//
// The mapping is conservative: codes we don't recognize fall through to
// Error::Internal so downstream callers can still surface them.
[[nodiscard]] constexpr Error from_sbg(SbgErrorCode code) noexcept
{
  switch (code) {
    case SBG_NO_ERROR:
      return Error::Ok;

    case SBG_TIME_OUT:
      return Error::Timeout;

    case SBG_INVALID_PARAMETER:
    case SBG_NULL_POINTER:
      return Error::InvalidParameter;

    case SBG_NOT_READY:
    case SBG_DEVICE_NOT_FOUND:
      return Error::NotReady;

    case SBG_INVALID_FRAME:
    case SBG_NOT_CONTINUOUS_FRAME:
    case SBG_BUFFER_OVERFLOW:
    case SBG_ERROR:
      return Error::ProtocolError;

    case SBG_INVALID_CRC:
      return Error::BadCrc;

    case SBG_READ_ERROR:
    case SBG_WRITE_ERROR:
      return Error::TransportFailure;

    case SBG_OPERATION_CANCELLED:
      return Error::Internal;

    case SBG_MALLOC_FAILED:
      return Error::AllocationFailure;

    case SBG_CALIB_MAG_NOT_ENOUGH_POINTS:
    case SBG_CALIB_MAG_INVALID_TAKE:
    case SBG_CALIB_MAG_SATURATION:
    case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE:
      return Error::CalibrationError;

    case SBG_INCOMPATIBLE_HARDWARE:
    case SBG_INVALID_VERSION:
      return Error::Unsupported;
  }
  return Error::Internal;
}

// Convenience: lift a C SDK call returning SbgErrorCode into a Result<void>.
[[nodiscard]] inline Result<void> check(SbgErrorCode code) noexcept
{
  if (code == SBG_NO_ERROR) {
    return {};
  }
  return std::unexpected(from_sbg(code));
}

}  // namespace sbg::detail
