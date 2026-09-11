// Copyright 2026 sbg_driver maintainers
// SPDX-License-Identifier: Apache-2.0
#include "sbg_driver/diagnostics.hpp"

#include <cmath>

namespace sbg_driver
{
namespace
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
bool stale(std::int64_t stamp, std::int64_t now)
{
  return stamp == 0 || now - stamp > 2'000'000'000LL;
}
}  // namespace

void diagnose_device(
  const Publishers::DiagSnapshot & s, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("device_status_general", s.last_device_status_general);
  stat.add("composition_drops", s.composition_drops);
  stat.add("imu_orientation_misses", s.imu_orientation_misses);
  stat.add("imu_source", s.imu_short_in_use ? "imu_short" : "imu_data");
  if (s.last_log_stamp_ns == 0) {
    stat.summary(Status::WARN, "No logs received yet");
    return;
  }
  stat.add("seconds_since_last_log", static_cast<double>(now - s.last_log_stamp_ns) / 1e9);
  // GPS power and datalogger bits are model/option dependent; require the
  // common device-health flags, not optional capabilities.
  constexpr unsigned required = SBG_ECOM_GENERAL_MAIN_POWER_OK | SBG_ECOM_GENERAL_IMU_POWER_OK |
                                SBG_ECOM_GENERAL_SETTINGS_OK | SBG_ECOM_GENERAL_TEMPERATURE_OK;
  if (stale(s.last_log_stamp_ns, now)) {
    stat.summary(Status::ERROR, "Stale device — no recent logs");
  } else if (!s.has_device_status) {
    stat.summary(Status::WARN, "Receiving data but no SBG_ECOM_LOG_STATUS yet");
  } else if (stale(s.last_status_stamp_ns, now)) {
    stat.summary(Status::ERROR, "Device status is stale");
  } else if ((s.last_device_status_general & required) != required) {
    stat.add("failed_required_health_bits", required & ~s.last_device_status_general);
    stat.summary(Status::ERROR, "Device reports a power, settings or temperature fault");
  } else if (s.last_composition_drop_ns != 0 && !stale(s.last_composition_drop_ns, now)) {
    stat.summary(
      Status::WARN, "Recent /odom composition drops — check EKF rates and epoch tolerance");
  } else {
    stat.summary(Status::OK, "Streaming");
  }
}

void diagnose_ekf(
  const Publishers::DiagSnapshot & s, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("solution_mode", static_cast<int>(s.last_ekf_solution_mode));
  stat.addf("raw_status", "0x%08x", s.last_ekf_status_raw);
  if (!s.has_ekf_status) {
    stat.summary(Status::WARN, "Waiting for first EKF Nav log");
  } else if (stale(s.last_ekf_stamp_ns, now)) {
    stat.summary(Status::ERROR, "EKF Nav is stale");
  } else if (s.last_ekf_solution_mode == 0) {
    stat.summary(Status::ERROR, "Uninitialized");
  } else if (!ekf_attitude_valid(s.last_ekf_status_raw)) {
    stat.summary(Status::ERROR, "EKF attitude is invalid");
  } else if (!ekf_heading_valid(s.last_ekf_status_raw)) {
    stat.summary(Status::WARN, "EKF heading is invalid");
  } else if (s.last_ekf_solution_mode == 4 && !ekf_position_valid(s.last_ekf_status_raw)) {
    stat.summary(Status::ERROR, "EKF position is invalid");
  } else if (s.last_ekf_solution_mode >= 3 && !ekf_velocity_valid(s.last_ekf_status_raw)) {
    stat.summary(Status::ERROR, "EKF velocity is invalid");
  } else if (s.last_ekf_solution_mode == 4) {
    stat.summary(Status::OK, "NAV_POSITION");
  } else if (s.last_ekf_solution_mode == 3) {
    stat.summary(Status::OK, "NAV_VELOCITY");
  } else if (s.last_ekf_solution_mode == 2) {
    stat.summary(Status::WARN, "AHRS only (no nav)");
  } else {
    stat.summary(Status::WARN, "Vertical gyro or unsupported solution mode");
  }
}

void diagnose_temperature(
  const Publishers::DiagSnapshot & s, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double temperature = s.last_imu_temperature_c;
  stat.add("temperature_c", temperature);
  if (!s.has_imu_temperature) {
    stat.summary(Status::WARN, "No IMU data yet");
  } else if (stale(s.last_temperature_stamp_ns, now)) {
    stat.summary(Status::ERROR, "IMU temperature is stale");
  } else if (!std::isfinite(temperature) || temperature < -40.0 || temperature > 85.0) {
    stat.summary(Status::ERROR, "Out of typical operating range");
  } else if (temperature > 70.0) {
    stat.summary(Status::WARN, "Approaching upper operating limit");
  } else {
    stat.summary(Status::OK, "Nominal");
  }
}
}  // namespace sbg_driver
