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

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <sbg/log_view.hpp>
#include <sbg_msgs/msg/ekf_status.hpp>
#include <sbg_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <string>
#include <string_view>

namespace sbg_driver
{

// Frame convention selector. Passed into conversion functions so each output
// message reflects either sensor-native NED (default) or REP-103 ENU.
enum class FrameConvention
{
  Ned = 0,  // sensor-native, no transform
  Enu = 1,  // y/z sign-flipped, quaternion rotated per REP-103
};

// ---- sensor_msgs/Imu -------------------------------------------------------
//
// Build an Imu message from an IMU log (linear_acceleration, angular_velocity)
// and optionally an EKF Quat log (orientation + orientation_covariance).
//
// `quat` may be null — when missing, orientation is set to identity and
// orientation_covariance[0] = -1 per the sensor_msgs convention (unknown).
//
// Covariance for linear_acceleration / angular_velocity is set to -1 in phase
// 2; phase 3+ populates from configurable noise-density parameters.
//
// Returns std::unique_ptr so callers can move into the publisher's
// `publish(std::unique_ptr<...>)` overload for intra-process zero-copy.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::Imu> to_imu(
  const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, FrameConvention convention,
  std::string_view frame_id, const rclcpp::Time & stamp);

// ---- sensor_msgs/MagneticField --------------------------------------------
//
// Maps the body-frame magnetic field vector. With ENU convention, y and z are
// sign-flipped on top of sensor-native NED.
// Covariance currently marked unknown (diag = -1); will be populated from a
// configurable noise-density param in phase 3b.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::MagneticField> to_magnetic_field(
  const SbgEComLogMag & mag, FrameConvention convention, std::string_view frame_id,
  const rclcpp::Time & stamp);

// ---- sensor_msgs/Temperature ----------------------------------------------
//
// Pulls the IMU board temperature (deg C) from an IMU log. Always populated;
// variance defaults to 0 (Temperature has no covariance type field).
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::Temperature> to_temperature(
  const SbgEComLogImuLegacy & imu, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- sensor_msgs/NavSatFix -------------------------------------------------
//
// Maps GNSS position log → NavSatFix. Status, service bits, and covariance
// are filled from SbgEComLogGnssPos fields:
//   * status.status  ← SBG GPS solution status (NO_SOLUTION, single, RTK, etc.)
//   * status.service ← which GNSS constellations were used
//   * position_covariance ← positionAccuracy.{x,y,z}² on the diagonal
//   * position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::NavSatFix> to_navsat(
  const SbgEComLogGnssPos & gnss, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- sensor_msgs/TimeReference ---------------------------------------------
//
// Wraps the sensor's UTC clock readout into a TimeReference message.
//   * header.stamp = ROS time (when we received the log)
//   * time_ref     = sensor's UTC time
//   * source       = "sbg_utc"
// Phase 3a: builds time_ref from {year, month, day, hour, minute, second, nanoSecond}.
// Phase 5+: incorporates clock bias / scale-factor accuracy fields.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::TimeReference> to_time_reference(
  const SbgEComLogUtc & utc, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- Geodetic origin + local Cartesian conversion --------------------------
//
// Sticky reference frame for converting subsequent geodetic positions to
// metres. Set on the first valid EKF Nav fix, never updated thereafter -
// downstream code (robot_localization, nav2) generally expects a stable origin.
//
// `lat`/`lon` are in degrees, `alt` in metres above the WGS84 ellipsoid.
struct GeodeticOrigin
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  // Cached cos(lat0) for small-angle longitude conversion.
  double cos_lat0 = 1.0;
};

// Small-angle equidistant projection from (lat, lon, alt) to local Cartesian
// metres relative to `origin`. Returns (east, north, up) when convention=Enu;
// (north, east, down) when convention=Ned. Accurate to a few cm within ~1 km
// of the origin; for larger areas use a proper LocalCartesian projection.
struct LocalPosition
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
[[nodiscard]] LocalPosition geodetic_to_local(
  double lat, double lon, double alt, const GeodeticOrigin & origin,
  FrameConvention convention) noexcept;

// ---- nav_msgs/Odometry (composed) ------------------------------------------
//
// Pose is in the parent frame ("odom"), twist is in the child frame ("base_link").
//   * pose.position    = local Cartesian metres relative to `origin`
//                        (geodetic lat/lon/alt → east/north/up or north/east/down).
//   * pose.orientation = from EkfQuat (NED→ENU flip if requested).
//   * pose.covariance  = positionStdDev + eulerStdDev squared on the 6-element diagonal.
//   * twist.linear     = EkfVelBody (body-frame X/Y/Z) - already matches child frame,
//                        just sign-flip y/z if ENU is requested.
//   * twist.angular    = zero (covariance diag = -1, unknown). Phase 3c populates
//                        from the IMU log's gyroscopes when one is cached.
[[nodiscard]] std::unique_ptr<nav_msgs::msg::Odometry> to_odometry(
  const SbgEComLogEkfNav & nav, const SbgEComLogEkfQuat & quat,
  const SbgEComLogEkfVelBody & vel_body, const GeodeticOrigin & origin, FrameConvention convention,
  std::string_view header_frame_id, std::string_view child_frame_id, const rclcpp::Time & stamp);

// ---- SBG-specific custom messages -----------------------------------------

// Device-level status: forwards general/com/aiding bitmasks verbatim. Consult
// the SBG firmware reference for bit decoding (we keep the raw 16/32-bit
// fields so users can decode against the firmware version they actually run).
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::Status> to_status(
  const SbgEComLogStatus & status, std::string_view frame_id, const rclcpp::Time & stamp);

// Decoded EKF solution mode + aiding bits from EkfNav.status. The raw uint32
// is preserved alongside for advanced introspection.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::EkfStatus> to_ekf_status(
  const SbgEComLogEkfNav & nav, std::string_view frame_id, const rclcpp::Time & stamp);

}  // namespace sbg_driver
