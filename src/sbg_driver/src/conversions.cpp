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

// ---- Geodetic → local Cartesian -------------------------------------------

namespace
{
// WGS84 equatorial radius. Small-angle approximation; metres-per-degree at
// the origin's latitude.
inline constexpr double k_wgs84_equatorial_radius_m = 6378137.0;
inline constexpr double k_pi = 3.14159265358979323846;
}  // namespace

LocalPosition geodetic_to_local(
  double lat, double lon, double alt, const GeodeticOrigin & origin,
  FrameConvention convention) noexcept
{
  const double dlat_rad = (lat - origin.lat) * k_pi / 180.0;
  const double dlon_rad = (lon - origin.lon) * k_pi / 180.0;
  const double east = dlon_rad * k_wgs84_equatorial_radius_m * origin.cos_lat0;
  const double north = dlat_rad * k_wgs84_equatorial_radius_m;
  const double up = alt - origin.alt;
  if (convention == FrameConvention::Enu) {
    return LocalPosition{.x = east, .y = north, .z = up};
  }
  return LocalPosition{.x = north, .y = east, .z = -up};
}

// ---- Odometry composition --------------------------------------------------

std::unique_ptr<nav_msgs::msg::Odometry> to_odometry(
  const SbgEComLogEkfNav & nav, const SbgEComLogEkfQuat & quat,
  const SbgEComLogEkfVelBody & vel_body, const GeodeticOrigin & origin, FrameConvention convention,
  std::string_view header_frame_id, std::string_view child_frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(header_frame_id);
  msg->child_frame_id.assign(child_frame_id);

  // Position: geodetic → local. EkfNav.position[2] is altitude above MSL;
  // promote to ellipsoidal height via undulation so it's consistent with
  // NavSatFix.altitude elsewhere in the driver.
  const double alt_ellipsoid = nav.position[2] + static_cast<double>(nav.undulation);
  const auto local =
    geodetic_to_local(nav.position[0], nav.position[1], alt_ellipsoid, origin, convention);
  msg->pose.pose.position.x = local.x;
  msg->pose.pose.position.y = local.y;
  msg->pose.pose.position.z = local.z;

  // Orientation from EkfQuat.
  double qw = static_cast<double>(quat.quaternion[0]);
  double qx = static_cast<double>(quat.quaternion[1]);
  double qy = static_cast<double>(quat.quaternion[2]);
  double qz = static_cast<double>(quat.quaternion[3]);
  if (convention == FrameConvention::Enu) {
    ned_quat_to_enu(qw, qx, qy, qz);
  }
  msg->pose.pose.orientation.w = qw;
  msg->pose.pose.orientation.x = qx;
  msg->pose.pose.orientation.y = qy;
  msg->pose.pose.orientation.z = qz;

  // Pose covariance — 6×6 row-major. Diagonal: x², y², z², roll², pitch², yaw².
  // EkfNav.positionStdDev is in (lat, lon, alt) order (metres); reorder to match
  // our pose axes per convention.
  const double std_lat_m = static_cast<double>(nav.positionStdDev[0]);
  const double std_lon_m = static_cast<double>(nav.positionStdDev[1]);
  const double std_alt_m = static_cast<double>(nav.positionStdDev[2]);
  double std_x = 0.0;
  double std_y = 0.0;
  double std_z = 0.0;
  if (convention == FrameConvention::Enu) {
    std_x = std_lon_m;  // east
    std_y = std_lat_m;  // north
    std_z = std_alt_m;  // up
  } else {
    std_x = std_lat_m;  // north
    std_y = std_lon_m;  // east
    std_z = std_alt_m;  // down (sign flip preserves variance)
  }
  const double std_roll = static_cast<double>(quat.eulerStdDev[0]);
  const double std_pitch = static_cast<double>(quat.eulerStdDev[1]);
  const double std_yaw = static_cast<double>(quat.eulerStdDev[2]);
  msg->pose.covariance.fill(0.0);
  msg->pose.covariance[0] = std_x * std_x;
  msg->pose.covariance[7] = std_y * std_y;
  msg->pose.covariance[14] = std_z * std_z;
  msg->pose.covariance[21] = std_roll * std_roll;
  msg->pose.covariance[28] = std_pitch * std_pitch;
  msg->pose.covariance[35] = std_yaw * std_yaw;

  // Twist: linear from EkfVelBody (body frame matches child_frame_id by
  // construction). Angular left zero until phase 3c wires in IMU gyros.
  double vbx = static_cast<double>(vel_body.velocity[0]);
  double vby = static_cast<double>(vel_body.velocity[1]);
  double vbz = static_cast<double>(vel_body.velocity[2]);
  if (convention == FrameConvention::Enu) {
    vby = -vby;
    vbz = -vbz;
  }
  msg->twist.twist.linear.x = vbx;
  msg->twist.twist.linear.y = vby;
  msg->twist.twist.linear.z = vbz;
  msg->twist.twist.angular.x = 0.0;
  msg->twist.twist.angular.y = 0.0;
  msg->twist.twist.angular.z = 0.0;

  // Twist covariance: linear diag from EkfVelBody.velocityStdDev squared.
  // Angular diag marked unknown (-1 sentinel in first slot, sensor_msgs convention).
  const double vbsx = static_cast<double>(vel_body.velocityStdDev[0]);
  const double vbsy = static_cast<double>(vel_body.velocityStdDev[1]);
  const double vbsz = static_cast<double>(vel_body.velocityStdDev[2]);
  msg->twist.covariance.fill(0.0);
  msg->twist.covariance[0] = vbsx * vbsx;
  msg->twist.covariance[7] = vbsy * vbsy;
  msg->twist.covariance[14] = vbsz * vbsz;
  msg->twist.covariance[21] = -1.0;

  return msg;
}

// ---- SBG-specific custom messages -----------------------------------------

std::unique_ptr<sbg_msgs::msg::Status> to_status(
  const SbgEComLogStatus & status, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::Status>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->time_stamp_us = status.timeStamp;
  msg->general_status = status.generalStatus;
  msg->com_status = status.comStatus;
  msg->com_status_2 = status.comStatus2;
  msg->aiding_status = status.aidingStatus;
  msg->uptime_seconds = status.uptime;
  msg->cpu_usage_percent = status.cpuUsage;
  return msg;
}

namespace
{
// EkfNav.status bit layout (mirrored from sbgEComLogEkf.h SBG_ECOM_SOL_* defs).
// Low 4 bits = solution mode; remaining bits are aiding/validity flags.
inline constexpr std::uint32_t k_ekf_sol_mode_mask = 0x0Fu;

inline constexpr std::uint32_t k_ekf_attitude_valid = 0x00000001u << 4;
inline constexpr std::uint32_t k_ekf_heading_valid = 0x00000001u << 5;
inline constexpr std::uint32_t k_ekf_velocity_valid = 0x00000001u << 6;
inline constexpr std::uint32_t k_ekf_position_valid = 0x00000001u << 7;
inline constexpr std::uint32_t k_ekf_vert_ref_used = 0x00000001u << 8;
inline constexpr std::uint32_t k_ekf_mag_ref_used = 0x00000001u << 9;
inline constexpr std::uint32_t k_ekf_gps1_vel_used = 0x00000001u << 10;
inline constexpr std::uint32_t k_ekf_gps1_pos_used = 0x00000001u << 11;
inline constexpr std::uint32_t k_ekf_vel_constraints_used = 0x00000001u << 12;
inline constexpr std::uint32_t k_ekf_gps1_hdt_used = 0x00000001u << 13;
inline constexpr std::uint32_t k_ekf_gps2_vel_used = 0x00000001u << 14;
inline constexpr std::uint32_t k_ekf_gps2_pos_used = 0x00000001u << 15;
inline constexpr std::uint32_t k_ekf_gps2_hdt_used = 0x00000001u << 17;
inline constexpr std::uint32_t k_ekf_odo_used = 0x00000001u << 18;
inline constexpr std::uint32_t k_ekf_dvl_bt_used = 0x00000001u << 19;
inline constexpr std::uint32_t k_ekf_dvl_wt_used = 0x00000001u << 20;
inline constexpr std::uint32_t k_ekf_vel1_used = 0x00000001u << 21;
inline constexpr std::uint32_t k_ekf_usbl_used = 0x00000001u << 24;
}  // namespace

std::unique_ptr<sbg_msgs::msg::EkfStatus> to_ekf_status(
  const SbgEComLogEkfNav & nav, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::EkfStatus>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);

  const std::uint32_t s = nav.status;
  msg->solution_mode = static_cast<std::uint8_t>(s & k_ekf_sol_mode_mask);
  msg->attitude_valid = (s & k_ekf_attitude_valid) != 0;
  msg->heading_valid = (s & k_ekf_heading_valid) != 0;
  msg->velocity_valid = (s & k_ekf_velocity_valid) != 0;
  msg->position_valid = (s & k_ekf_position_valid) != 0;
  msg->vert_ref_used = (s & k_ekf_vert_ref_used) != 0;
  msg->mag_ref_used = (s & k_ekf_mag_ref_used) != 0;
  msg->gps1_vel_used = (s & k_ekf_gps1_vel_used) != 0;
  msg->gps1_pos_used = (s & k_ekf_gps1_pos_used) != 0;
  msg->gps1_hdt_used = (s & k_ekf_gps1_hdt_used) != 0;
  msg->gps2_vel_used = (s & k_ekf_gps2_vel_used) != 0;
  msg->gps2_pos_used = (s & k_ekf_gps2_pos_used) != 0;
  msg->gps2_hdt_used = (s & k_ekf_gps2_hdt_used) != 0;
  msg->vel_constraints_used = (s & k_ekf_vel_constraints_used) != 0;
  msg->odometer_used = (s & k_ekf_odo_used) != 0;
  msg->dvl_bottom_track_used = (s & k_ekf_dvl_bt_used) != 0;
  msg->dvl_water_track_used = (s & k_ekf_dvl_wt_used) != 0;
  msg->generic_vel1_used = (s & k_ekf_vel1_used) != 0;
  msg->usbl_used = (s & k_ekf_usbl_used) != 0;
  msg->raw_status = s;
  return msg;
}

std::unique_ptr<sbg_msgs::msg::ShipMotion> to_ship_motion(
  const SbgEComLogShipMotion & ship, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::ShipMotion>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->status = ship.status;
  msg->main_heave_period_s = ship.mainHeavePeriod;
  msg->motion_m.x = static_cast<double>(ship.shipMotion[0]);
  msg->motion_m.y = static_cast<double>(ship.shipMotion[1]);
  msg->motion_m.z = static_cast<double>(ship.shipMotion[2]);
  msg->acceleration_m_per_s2.x = static_cast<double>(ship.shipAccel[0]);
  msg->acceleration_m_per_s2.y = static_cast<double>(ship.shipAccel[1]);
  msg->acceleration_m_per_s2.z = static_cast<double>(ship.shipAccel[2]);
  msg->velocity_m_per_s.x = static_cast<double>(ship.shipVel[0]);
  msg->velocity_m_per_s.y = static_cast<double>(ship.shipVel[1]);
  msg->velocity_m_per_s.z = static_cast<double>(ship.shipVel[2]);
  return msg;
}

std::unique_ptr<sbg_msgs::msg::Event> to_event(
  const SbgEComLogEvent & ev, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::Event>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->time_stamp_us = ev.timeStamp;
  msg->status = ev.status;
  msg->time_offsets_us = {ev.timeOffset0, ev.timeOffset1, ev.timeOffset2, ev.timeOffset3};
  return msg;
}

std::unique_ptr<sbg_msgs::msg::MagCalib> to_mag_calib(
  const SbgEComLogMagCalib & cal, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::MagCalib>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->time_stamp_us = cal.timeStamp;
  for (std::size_t i = 0; i < msg->mag_data.size(); ++i) {
    msg->mag_data[i] = cal.magData[i];
  }
  return msg;
}

std::unique_ptr<sbg_msgs::msg::GpsRaw> to_gps_raw(
  const SbgEComLogRawData & raw, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::GpsRaw>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->raw_data.assign(raw.rawBuffer, raw.rawBuffer + raw.bufferSize);
  return msg;
}

namespace
{
// AirData status bits from sbgEComLogAirData.h. Bit 0 is currently unused
// by the SDK; we decode the standard validity bits 1-5.
inline constexpr std::uint16_t k_air_pressure_abs_valid = 0x0001u << 1;
inline constexpr std::uint16_t k_air_altitude_valid = 0x0001u << 2;
inline constexpr std::uint16_t k_air_airspeed_valid = 0x0001u << 4;
inline constexpr std::uint16_t k_air_temperature_valid = 0x0001u << 5;
}  // namespace

std::unique_ptr<sbg_msgs::msg::AirDataStatus> to_air_data_status(
  const SbgEComLogAirData & air, std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sbg_msgs::msg::AirDataStatus>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);
  msg->status_bits = air.status;
  msg->pressure_valid = (air.status & k_air_pressure_abs_valid) != 0;
  msg->altitude_valid = (air.status & k_air_altitude_valid) != 0;
  msg->airspeed_valid = (air.status & k_air_airspeed_valid) != 0;
  msg->temperature_valid = (air.status & k_air_temperature_valid) != 0;
  return msg;
}

}  // namespace sbg_driver
