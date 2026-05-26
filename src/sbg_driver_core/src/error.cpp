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

#include "sbg/error.hpp"

namespace sbg
{

std::string_view to_string(Error err) noexcept
{
  using enum Error;
  switch (err) {
    case Ok:
      return "ok";
    case Timeout:
      return "timeout";
    case InvalidParameter:
      return "invalid parameter";
    case NotReady:
      return "not ready";
    case ProtocolError:
      return "protocol error";
    case TransportFailure:
      return "transport failure";
    case DeviceBusy:
      return "device busy";
    case BadCrc:
      return "bad crc";
    case Unsupported:
      return "unsupported";
    case AllocationFailure:
      return "allocation failure";
    case CalibrationError:
      return "calibration error";
    case Internal:
      return "internal error";
  }
  return "unknown error";
}

}  // namespace sbg
