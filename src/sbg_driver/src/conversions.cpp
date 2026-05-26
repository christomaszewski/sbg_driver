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

#include "sbg_driver/conversions.hpp"

#include <array>
#include <chrono>
#include <cmath>

namespace sbg_driver
{

namespace
{

// In-place sign-flip on Y and Z components: NED → ENU for body-frame triplets.
constexpr void flip_yz(double & y, double & z) noexcept
{
  y = -y;
  z = -z;
}

// Rotate a quaternion expressed in NED into an ENU frame by applying the
// rotation R = diag(1, -1, -1). w and x are unchanged; only y and z flip.
// w and x kept in the signature for API symmetry with future rotations.
// REP-103 defines body axes as forward-left-up; sensor NED is forward-right-down.
constexpr void ned_quat_to_enu(
  [[maybe_unused]] double & w, [[maybe_unused]] double & x, double & y, double & z) noexcept
{
  y = -y;
  z = -z;
}

}  // namespace

std::unique_ptr<sensor_msgs::msg::Imu> to_imu(
  const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, FrameConvention convention,
  std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);

  msg->linear_acceleration.x = static_cast<double>(imu.accelerometers[0]);
  msg->linear_acceleration.y = static_cast<double>(imu.accelerometers[1]);
  msg->linear_acceleration.z = static_cast<double>(imu.accelerometers[2]);

  msg->angular_velocity.x = static_cast<double>(imu.gyroscopes[0]);
  msg->angular_velocity.y = static_cast<double>(imu.gyroscopes[1]);
  msg->angular_velocity.z = static_cast<double>(imu.gyroscopes[2]);

  if (convention == FrameConvention::Enu) {
    flip_yz(msg->linear_acceleration.y, msg->linear_acceleration.z);
    flip_yz(msg->angular_velocity.y, msg->angular_velocity.z);
  }

  if (quat != nullptr) {
    double qw = static_cast<double>(quat->quaternion[0]);
    double qx = static_cast<double>(quat->quaternion[1]);
    double qy = static_cast<double>(quat->quaternion[2]);
    double qz = static_cast<double>(quat->quaternion[3]);
    if (convention == FrameConvention::Enu) {
      ned_quat_to_enu(qw, qx, qy, qz);
    }
    msg->orientation.w = qw;
    msg->orientation.x = qx;
    msg->orientation.y = qy;
    msg->orientation.z = qz;

    // Orientation covariance: diagonal from EkfQuat.eulerStdDev squared.
    // Rotation between NED and ENU preserves diagonal variances.
    const double sx = static_cast<double>(quat->eulerStdDev[0]);
    const double sy = static_cast<double>(quat->eulerStdDev[1]);
    const double sz = static_cast<double>(quat->eulerStdDev[2]);
    msg->orientation_covariance.fill(0.0);
    msg->orientation_covariance[0] = sx * sx;
    msg->orientation_covariance[4] = sy * sy;
    msg->orientation_covariance[8] = sz * sz;
  } else {
    // No orientation data yet — sentinel per sensor_msgs/Imu.
    msg->orientation.w = 1.0;
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation_covariance.fill(0.0);
    msg->orientation_covariance[0] = -1.0;
  }

  // Phase 2 placeholder: accel/gyro covariance is unknown until phase 3+
  // wires up noise-density params. Mark as unknown (-1) per sensor_msgs.
  msg->linear_acceleration_covariance.fill(0.0);
  msg->linear_acceleration_covariance[0] = -1.0;
  msg->angular_velocity_covariance.fill(0.0);
  msg->angular_velocity_covariance[0] = -1.0;

  return msg;
}

std::unique_ptr<sensor_msgs::msg::MagneticField> to_magnetic_field(
  const SbgEComLogMag & mag, FrameConvention convention, std::string_view frame_id,
  const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::MagneticField>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);

  msg->magnetic_field.x = static_cast<double>(mag.magnetometers[0]);
  msg->magnetic_field.y = static_cast<double>(mag.magnetometers[1]);
  msg->magnetic_field.z = static_cast<double>(mag.magnetometers[2]);
  if (convention == FrameConvention::Enu) {
    flip_yz(msg->magnetic_field.y, msg->magnetic_field.z);
  }

  // SBG doesn't provide per-measurement mag accuracy. Phase 3b will populate
  // from a configurable noise-density parameter; until then mark unknown.
  msg->magnetic_field_covariance.fill(0.0);
  msg->magnetic_field_covariance[0] = -1.0;
  return msg;
}

std::unique_ptr<sensor_msgs::msg::Temperature> to_temperature(
  const SbgEComLogImuLegacy & imu, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::Temperature>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->temperature = static_cast<double>(imu.temperature);
  msg->variance = 0.0;  // unknown
  return msg;
}

namespace
{

// Bit layout: SbgEComLogGnssPos.status packs solution status (low 6 bits) and
// position type (next 6 bits). Constants mirrored from sbgEComLogGnssPos.c
// (the SDK keeps these private to the .c file; we duplicate them here rather
// than call the deprecated extraction helpers).
inline constexpr std::uint32_t k_gnss_status_shift = 0u;
inline constexpr std::uint32_t k_gnss_status_mask = 0x3Fu;
inline constexpr std::uint32_t k_gnss_type_shift = 6u;
inline constexpr std::uint32_t k_gnss_type_mask = 0x3Fu;

constexpr SbgEComGnssPosStatus extract_status(std::uint32_t status) noexcept
{
  return static_cast<SbgEComGnssPosStatus>((status >> k_gnss_status_shift) & k_gnss_status_mask);
}

constexpr SbgEComGnssPosType extract_type(std::uint32_t status) noexcept
{
  return static_cast<SbgEComGnssPosType>((status >> k_gnss_type_shift) & k_gnss_type_mask);
}

// Map SBG position type → ROS NavSatStatus.status code.
constexpr std::int8_t to_nav_sat_status(
  SbgEComGnssPosStatus status, SbgEComGnssPosType type) noexcept
{
  if (status != SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED) {
    return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  switch (type) {
    case SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION:
    case SBG_ECOM_GNSS_POS_TYPE_UNKNOWN:
      return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    case SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT:
    case SBG_ECOM_GNSS_POS_TYPE_RTK_INT:
    case SBG_ECOM_GNSS_POS_TYPE_PPP_FLOAT:
    case SBG_ECOM_GNSS_POS_TYPE_PPP_INT:
    case SBG_ECOM_GNSS_POS_TYPE_FIXED:
      return sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    default:
      // SINGLE and other pseudorange/SBAS-like types
      return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
}

}  // namespace

std::unique_ptr<sensor_msgs::msg::NavSatFix> to_navsat(
  const SbgEComLogGnssPos & gnss, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);

  msg->latitude = gnss.latitude;
  msg->longitude = gnss.longitude;
  // SBG altitude is MSL; ROS NavSatFix wants altitude above the WGS84 ellipsoid.
  // Add undulation (height above ellipsoid = altitude + undulation).
  msg->altitude = gnss.altitude + static_cast<double>(gnss.undulation);

  // Status. We don't have a way to distinguish per-constellation service from
  // GnssPos alone, so report SERVICE_GPS as a sensible default. Phase 3b can
  // refine using SbgEComLogGnssHdt or SbgEComLogSat.
  msg->status.status = to_nav_sat_status(extract_status(gnss.status), extract_type(gnss.status));
  msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Covariance: diag from per-axis accuracy (1σ stddev in metres) squared.
  // Order in ROS is row-major 3×3 = [east, north, up] = [lon, lat, alt].
  const double slat = gnss.latitudeAccuracy;
  const double slon = gnss.longitudeAccuracy;
  const double salt = gnss.altitudeAccuracy;
  msg->position_covariance.fill(0.0);
  msg->position_covariance[0] = slon * slon;
  msg->position_covariance[4] = slat * slat;
  msg->position_covariance[8] = salt * salt;
  msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return msg;
}

std::unique_ptr<sensor_msgs::msg::TimeReference> to_time_reference(
  const SbgEComLogUtc & utc, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::TimeReference>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->source = "sbg_utc";

  // Compose UNIX timestamp from sensor UTC fields using C++20 chrono calendar.
  const std::chrono::year_month_day ymd{
    std::chrono::year{utc.year},
    std::chrono::month{static_cast<unsigned>(utc.month)},
    std::chrono::day{static_cast<unsigned>(utc.day)},
  };
  const auto sys_days = std::chrono::sys_days{ymd};
  const auto since_epoch = sys_days.time_since_epoch() + std::chrono::hours{utc.hour} +
                           std::chrono::minutes{utc.minute} + std::chrono::seconds{utc.second} +
                           std::chrono::nanoseconds{utc.nanoSecond};
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - sec);

  msg->time_ref.sec = static_cast<std::int32_t>(sec.count());
  msg->time_ref.nanosec = static_cast<std::uint32_t>(nsec.count());
  return msg;
}

}  // namespace sbg_driver
