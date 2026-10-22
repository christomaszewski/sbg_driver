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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace sbg_driver
{

enum class OutputTopic
{
  ImuData,
  RawOrientation,
  ImuTemperature,
  Mag,
  NavSatFix,
  GnssHeading,
  TimeReference,
  Odom,
  EkfNavSatFix,
  Nmea,
  SbgStatus,
  SbgEkfStatus,
  SbgAirDataStatus,
  SbgEvent,
  SbgGpsRaw,
  SbgMagCalib,
  SbgShipMotion,
  SbgShipMotionHp,
  Tf,
  Count
};

struct OutputRates
{
  double imu_data = 0.0;
  double raw_orientation = 0.0;
  double imu_temperature = 0.0;
  double mag = 0.0;
  double nav_sat_fix = 0.0;
  double gnss_heading = 0.0;
  double time_reference = 0.0;
  double odom = 0.0;
  double ekf_nav_sat_fix = 0.0;
  double nmea = 1.0;
  double sbg_status = 0.0;
  double sbg_ekf_status = 0.0;
  double sbg_air_data_status = 0.0;
  double sbg_event = 0.0;
  double sbg_gps_raw = 0.0;
  double sbg_mag_calib = 0.0;
  double sbg_ship_motion = 0.0;
  double sbg_ship_motion_hp = 0.0;
  double tf = 0.0;

  [[nodiscard]] auto as_array() const noexcept
  {
    return std::array{
      imu_data,
      raw_orientation,
      imu_temperature,
      mag,
      nav_sat_fix,
      gnss_heading,
      time_reference,
      odom,
      ekf_nav_sat_fix,
      nmea,
      sbg_status,
      sbg_ekf_status,
      sbg_air_data_status,
      sbg_event,
      sbg_gps_raw,
      sbg_mag_calib,
      sbg_ship_motion,
      sbg_ship_motion_hp,
      tf};
  }

  [[nodiscard]] bool valid() const noexcept
  {
    const auto values = as_array();
    return std::all_of(
      values.begin(), values.end(), [](double hz) { return std::isfinite(hz) && hz >= 0.0; });
  }
};

// A shared device timeline lets even infrequent topics span timestamp wraps.
// Delayed measurements resolve behind the high-water mark without advancing it.
// Only the driver's existing confirmed-reboot logic resets this timeline.
class SampleTimeUnwrapper
{
public:
  void observe(std::uint32_t timestamp_us) noexcept
  {
    if (!high_water_) {
      high_water_ = timestamp_us;
      extended_us_ = timestamp_us;
    } else {
      const auto delta = static_cast<std::int32_t>(timestamp_us - *high_water_);
      if (delta > 0) {
        extended_us_ += delta;
        high_water_ = timestamp_us;
      }
    }
  }

  [[nodiscard]] std::int64_t resolve(std::uint32_t timestamp_us) const noexcept
  {
    return high_water_ ? extended_us_ + static_cast<std::int32_t>(timestamp_us - *high_water_)
                       : static_cast<std::int64_t>(timestamp_us);
  }

  void reset() noexcept { high_water_.reset(); }

private:
  std::optional<std::uint32_t> high_water_;
  std::int64_t extended_us_ = 0;
};

// Sample-time decimation: serial bursts and accelerated replay select the same
// measurements as paced input. A phase deadline and 5% early allowance avoid
// halving a stream whose nominal period has slight timestamp jitter. No timer,
// held output, or catch-up publication is created; original stamps are retained.
class OutputRateLimiter
{
public:
  explicit OutputRateLimiter(double hz = 0.0)
  {
    if (!std::isfinite(hz) || hz < 0.0) {
      throw std::invalid_argument("output rates must be finite and nonnegative");
    }
    // The clock has microsecond resolution. Saturate extreme finite rates
    // rather than overflowing interval arithmetic on platforms where long
    // double has the same range as double.
    period_us_ = hz > 0.0 ? std::clamp(
                              1'000'000.0L / hz, 1.0L,
                              static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
                          : 0.0L;
  }

  bool allow(std::int64_t sample_us) noexcept
  {
    if (period_us_ == 0.0L) {
      return true;
    }
    if (!last_seen_) {
      last_seen_ = sample_us;
      next_due_us_ = sample_us + period_us_;
      return true;
    }
    if (sample_us <= *last_seen_) {
      return false;
    }
    last_seen_ = sample_us;
    if (sample_us + period_us_ * 0.05L < next_due_us_) {
      return false;
    }
    // After a missing interval, resume from this sample instead of catching
    // up with another publication immediately at an old phase boundary.
    next_due_us_ =
      sample_us >= next_due_us_ + period_us_ ? sample_us + period_us_ : next_due_us_ + period_us_;
    return true;
  }

  void reset() noexcept { last_seen_.reset(); }

private:
  long double period_us_ = 0.0L;
  long double next_due_us_ = 0.0L;
  std::optional<std::int64_t> last_seen_;
};

}  // namespace sbg_driver
