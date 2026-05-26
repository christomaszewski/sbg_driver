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

#include "sbg/detail/c_api.hpp"

namespace sbg
{

// LogView — non-owning typed view over a parsed sbgECom log delivered through
// a Device callback. The underlying payload lives in a SbgEComLogUnion owned
// by the C SDK and remains valid only for the duration of the callback.
// Do NOT cache pointers returned by `as_*()` beyond callback return.
//
// Accessors return nullptr if the stored kind doesn't match. Typical use:
//
//     device.set_log_callback([&](const sbg::LogView& v) {
//       if (const auto* imu = v.as_imu_data()) {
//         /* handle SBG_ECOM_LOG_IMU_DATA */
//       } else if (const auto* quat = v.as_ekf_quat()) {
//         /* handle SBG_ECOM_LOG_EKF_QUAT */
//       }
//     });
//
// This header transitively pulls in the sbgECom C headers via detail/c_api.hpp
// — by design, since user code needs the C struct field names to access
// log payloads. Users who don't care about log contents can forward-declare
// LogView (`namespace sbg { class LogView; }`) and only pull this include
// into TUs that actually read fields.
class LogView
{
public:
  enum class Kind : std::uint16_t
  {
    Unknown = 0,
    ImuShort,
    ImuData,
    EkfEuler,
    EkfQuat,
    EkfNav,
    EkfVelBody,
    Mag,
    GnssPos,
    GnssVel,
    GnssHdt,
    AirData,
    Utc,
    Status,
  };

  // Construct directly from the parameters delivered to the sbgECom callback.
  LogView(std::uint8_t msg_class, std::uint16_t msg_id, const SbgEComLogUnion * log_data) noexcept;

  [[nodiscard]] Kind kind() const noexcept
  {
    return kind_;
  }

  // Microsecond timestamp from the log header (sensor wall-clock-ish; meaning
  // varies by log type — relative time on most IMU logs, GPS time-of-week on
  // GNSS logs, etc.).
  [[nodiscard]] std::uint32_t time_stamp_us() const noexcept
  {
    return time_stamp_us_;
  }

  // Typed accessors. Each returns nullptr if kind() doesn't match.
  [[nodiscard]] const SbgEComLogImuLegacy * as_imu_data() const noexcept;
  [[nodiscard]] const SbgEComLogImuShort * as_imu_short() const noexcept;
  [[nodiscard]] const SbgEComLogEkfEuler * as_ekf_euler() const noexcept;
  [[nodiscard]] const SbgEComLogEkfQuat * as_ekf_quat() const noexcept;
  [[nodiscard]] const SbgEComLogEkfNav * as_ekf_nav() const noexcept;
  [[nodiscard]] const SbgEComLogEkfVelBody * as_ekf_vel_body() const noexcept;
  [[nodiscard]] const SbgEComLogMag * as_mag() const noexcept;
  [[nodiscard]] const SbgEComLogGnssPos * as_gnss_pos() const noexcept;
  [[nodiscard]] const SbgEComLogGnssVel * as_gnss_vel() const noexcept;
  [[nodiscard]] const SbgEComLogGnssHdt * as_gnss_hdt() const noexcept;
  [[nodiscard]] const SbgEComLogAirData * as_air_data() const noexcept;
  [[nodiscard]] const SbgEComLogUtc * as_utc() const noexcept;
  [[nodiscard]] const SbgEComLogStatus * as_status() const noexcept;

private:
  Kind kind_ = Kind::Unknown;
  const SbgEComLogUnion * log_ = nullptr;
  std::uint32_t time_stamp_us_ = 0;
};

}  // namespace sbg
