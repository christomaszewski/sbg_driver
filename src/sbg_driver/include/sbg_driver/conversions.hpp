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
#include <nmea_msgs/msg/sentence.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <sbg/log_view.hpp>
#include <sbg_msgs/msg/air_data_status.hpp>
#include <sbg_msgs/msg/ekf_status.hpp>
#include <sbg_msgs/msg/event.hpp>
#include <sbg_msgs/msg/gps_raw.hpp>
#include <sbg_msgs/msg/mag_calib.hpp>
#include <sbg_msgs/msg/ship_motion.hpp>
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

// header.stamp source selector.
//   * ReceiveTime: host clock at dispatch (default). Right when the host's
//     clock is the system timebase and cross-sensor consistency matters more
//     than stamp jitter.
//   * DeviceUtc: the log's own device timeStamp mapped through the most
//     recent valid UTC log (see UtcAnchor). Removes transport/scheduling
//     jitter and, when the host disciplines its clock to the INS (the SBG
//     NTP/PTP server setup), stays in the system timebase too. Falls back to
//     receive time until the first valid UTC log and for logs without a
//     payload timestamp (GPS raw / RTCM raw).
enum class TimeSource
{
  ReceiveTime = 0,
  DeviceUtc = 1,
};

// One (device timeStamp → UTC) correspondence taken from a UTC log. The UTC
// log pairs the device's free-running µs timestamp with the UTC instant it
// refers to, so no transport-latency estimation is involved — the pair is
// exact, device-side data.
struct UtcAnchor
{
  std::uint32_t device_ts_us = 0;  // device timeStamp of the UTC log
  std::int64_t utc_epoch_ns = 0;   // same instant as UNIX-epoch nanoseconds
};

// True when the UTC log's status carries a usable clock state (INITIALIZED,
// or NO_LEAP_SEC — initialized with a defaulted leap second). False while
// INVALID: before GNSS sync the INS streams its firmware-default date, which
// must never anchor timestamps or be published as a time reference.
[[nodiscard]] bool utc_time_valid(std::uint16_t utc_status) noexcept;

// Compose the UTC log's calendar fields into UNIX-epoch nanoseconds.
// Callers must have checked utc_time_valid() first — an INVALID log composes
// to the firmware-default date.
[[nodiscard]] std::int64_t utc_epoch_ns(const SbgEComLogUtc & utc) noexcept;

// Map a device timeStamp to UNIX-epoch nanoseconds via `anchor`. The delta is
// computed in signed 32-bit µs, which absorbs the uint32 wrap (~71.6 min)
// transparently as long as the log lies within ±35 min of the anchor; anchors
// refresh on every valid UTC log, so in practice the delta is sub-second.
// Returns nullopt when the delta exceeds a ±30 min sanity horizon (stale
// anchor after the UTC log stopped, or a device reboot mid-session) — the
// caller should fall back to receive time.
[[nodiscard]] std::optional<std::int64_t> device_stamp_ns(
  const UtcAnchor & anchor, std::uint32_t device_ts_us) noexcept;

// Per-axis measurement variance for the IMU's accel + gyro. These land
// directly on the diagonal of sensor_msgs/Imu's covariance matrices, so
// callers supply variance (σ², units (m/s²)² and (rad/s)²) rather than
// raw noise density — no bandwidth conversion happens in the conversion
// layer. A negative value is the sensor_msgs "unknown" sentinel (-1 in
// element [0], the rest zeroed). SBG provides no per-measurement accel/gyro
// accuracy, so these come from configuration (see ImuNoiseModel).
struct ImuCovariance
{
  double accel_variance = -1.0;  // (m/s²)²  per axis; <0 => unknown
  double gyro_variance = -1.0;   // (rad/s)² per axis; <0 => unknown
};

// Resolve the effective accel/gyro variance from a sensor-model name plus
// optional explicit per-axis 1σ standard deviations.
//   * An explicit stddev >= 0 always wins (variance = stddev²).
//   * Otherwise, a recognized `sensor_model` supplies an approximate
//     datasheet-derived default stddev for that axis.
//   * "custom"/unknown model with no explicit stddev leaves the axis unknown.
// `accel_stddev` is in m/s², `gyro_stddev` in rad/s. The per-model defaults
// are rough starting points — users should refine from their unit's datasheet
// or an Allan-variance characterization.
[[nodiscard]] ImuCovariance resolve_imu_covariance(
  std::string_view sensor_model, double accel_stddev, double gyro_stddev) noexcept;

// ---- sensor_msgs/Imu -------------------------------------------------------
//
// Build an Imu message from an IMU log (linear_acceleration, angular_velocity)
// and optionally an EKF Quat log (orientation + orientation_covariance).
//
// `quat` may be null — when missing, orientation is set to identity and
// orientation_covariance[0] = -1 per the sensor_msgs convention (unknown).
//
// `cov` supplies accel/gyro variance for the respective covariance diagonals;
// rotation between NED and ENU preserves the diagonal so `cov` is convention-
// independent.
//
// Returns std::unique_ptr so callers can move into the publisher's
// `publish(std::unique_ptr<...>)` overload for intra-process zero-copy.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::Imu> to_imu(
  const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, FrameConvention convention,
  std::string_view frame_id, const rclcpp::Time & stamp, const ImuCovariance & cov);

// ---- sensor_msgs/MagneticField --------------------------------------------
//
// Maps the body-frame magnetic field vector. With ENU convention, y and z are
// sign-flipped on top of sensor-native NED.
//
// UNITS: SBG magnetometer logs are in arbitrary units (a.u.), normalized so
// ~1.0 ≈ the local Earth field — NOT Tesla, which sensor_msgs/MagneticField
// specifies. `scale` converts a.u. → T; the default 1.0 publishes the raw
// a.u. values (matching the upstream SBG driver). Consumers needing physical
// units should set imu.mag_scale ≈ the local field magnitude in Tesla
// (e.g. 5e-5).
// Covariance currently marked unknown (diag = -1); will be populated from a
// configurable noise-density param in phase 3b.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::MagneticField> to_magnetic_field(
  const SbgEComLogMag & mag, FrameConvention convention, std::string_view frame_id,
  const rclcpp::Time & stamp, double scale = 1.0);

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

// ---- nmea_msgs/Sentence ($GPGGA) -------------------------------------------
//
// Builds an NMEA GGA sentence from the raw GNSS position, for a third-party
// NTRIP client to upload to a VRS / network-RTK caster (the position-up half of
// the NTRIP loop). Returns nullptr unless the fix is a computed solution with a
// real position type (solution status SOL_COMPUTED and type != NO_SOLUTION) - a
// caster needs a real position. UTC time-of-day comes from `stamp` (receive
// time): the GGA timestamp is informational for VRS (casters key off lat/lon),
// so this avoids GPS-ToW leap-second bookkeeping. Fix-quality digit, satellite
// count, HDOP (from position accuracy), altitude, geoid separation, differential
// age and base-station id are filled from the log; the SDK's "not available"
// sentinels (numSvUsed 0xFF, differentialAge/baseStationId 0xFFFF) render as
// EMPTY NMEA fields per convention, not as fabricated values. Locale-independent
// (NMEA mandates '.' as the decimal separator).
[[nodiscard]] std::unique_ptr<nmea_msgs::msg::Sentence> to_nmea_gga(
  const SbgEComLogGnssPos & gnss, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- sensor_msgs/NavSatFix from the fused EKF solution ---------------------
//
// Optional second NavSatFix carrying the *fused* INS geodetic position from
// EkfNav - NOT the raw GNSS fix that to_navsat() above produces. Smoother than
// raw GNSS and available through brief GNSS dropouts via dead-reckoning.
// The fused solution has no GNSS fix-type or constellation, so NavSatStatus is
// necessarily coarse: status.position_valid → STATUS_FIX else STATUS_NO_FIX,
// service left at the conventional SERVICE_GPS. Covariance diagonal is filled
// from EkfNav.positionStdDev² in ENU [east, north, up] order (frame-convention
// independent - NavSatFix is always geographic). Altitude is promoted from MSL
// to ellipsoidal height via undulation, matching to_navsat().
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::NavSatFix> to_ekf_navsat(
  const SbgEComLogEkfNav & nav, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- sensor_msgs/TimeReference ---------------------------------------------
//
// Wraps the sensor's UTC clock readout into a TimeReference message.
//   * header.stamp = the caller-supplied stamp (receive time — kept that way
//                    even under TimeSource::DeviceUtc, so the message stays a
//                    meaningful host↔sensor clock pairing)
//   * time_ref     = sensor's UTC time
//   * source       = "sbg_utc"
// Returns nullptr while the log's UTC status is INVALID — before GNSS sync
// the INS streams its firmware-default initial date/time, which must not be
// published as an authoritative time reference. NO_LEAP_SEC (initialized but
// leap second defaulted) and INITIALIZED both publish.
// Phase 5+: incorporates clock bias / scale-factor accuracy fields.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::TimeReference> to_time_reference(
  const SbgEComLogUtc & utc, std::string_view frame_id, const rclcpp::Time & stamp);

// ---- EKF validity ----------------------------------------------------------
//
// True when EkfNav.status carries the SBG_ECOM_SOL_POSITION_VALID bit — the
// filter vouches for its position estimate. Used to gate the sticky odometry
// origin latch and Odometry composition (an UNINITIALIZED EKF streams the
// firmware-default position, which must never become the local-frame origin).
[[nodiscard]] bool ekf_position_valid(std::uint32_t ekf_nav_status) noexcept;

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
  // Precomputed metres-per-degree scale at the origin latitude, folding in the
  // WGS84 radii of curvature (meridional for north, prime-vertical × cos(lat)
  // for east). Defaults are the equatorial values; always build via
  // make_geodetic_origin() so the scales stay consistent with `lat`.
  double north_metres_per_deg = 110574.27;  // M(0) · π/180
  double east_metres_per_deg = 111319.49;   // N(0) · cos(0) · π/180
};

// Build a GeodeticOrigin with its metres-per-degree scales evaluated at `lat`
// from the WGS84 radii of curvature. Prefer this over aggregate-initialising
// GeodeticOrigin directly, so the scale factors always match the latitude.
[[nodiscard]] GeodeticOrigin make_geodetic_origin(double lat, double lon, double alt) noexcept;

// Local tangent-plane projection from (lat, lon, alt) to Cartesian metres
// relative to `origin`. Returns (east, north, up) when convention=Enu;
// (north, east, down) when convention=Ned. Scales use the WGS84 radii of
// curvature at the origin latitude (via make_geodetic_origin), so they are
// correct to first order; residual error is second-order curvature - roughly
// centimetres within a few km, sub-metre out to tens of km. For global-scale
// work use a full geodetic projection. The longitude difference is wrapped
// into [-180, 180), so an origin near the antimeridian stays continuous.
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

// Marine motion solution (surge/sway/heave) from SBG_ECOM_LOG_SHIP_MOTION{,_HP}.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::ShipMotion> to_ship_motion(
  const SbgEComLogShipMotion & ship, std::string_view frame_id, const rclcpp::Time & stamp);

// GPIO sync-in event marker from SBG_ECOM_LOG_EVENT_{A..E}.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::Event> to_event(
  const SbgEComLogEvent & ev, std::string_view frame_id, const rclcpp::Time & stamp);

// Mag-calibration data snapshot from SBG_ECOM_LOG_MAG_CALIB.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::MagCalib> to_mag_calib(
  const SbgEComLogMagCalib & cal, std::string_view frame_id, const rclcpp::Time & stamp);

// Raw GNSS observable blob from SBG_ECOM_LOG_GPS{1,2}_RAW.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::GpsRaw> to_gps_raw(
  const SbgEComLogRawData & raw, std::string_view frame_id, const rclcpp::Time & stamp);

// Decoded air-data status bits from SbgEComLogAirData.status.
[[nodiscard]] std::unique_ptr<sbg_msgs::msg::AirDataStatus> to_air_data_status(
  const SbgEComLogAirData & air, std::string_view frame_id, const rclcpp::Time & stamp);

}  // namespace sbg_driver
