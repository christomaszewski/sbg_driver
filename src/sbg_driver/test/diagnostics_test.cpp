// Copyright 2026 sbg_driver maintainers
// SPDX-License-Identifier: Apache-2.0
#include "sbg_driver/diagnostics.hpp"

#include <gtest/gtest.h>

namespace
{
using Status = diagnostic_msgs::msg::DiagnosticStatus;
constexpr std::int64_t now = 10'000'000'000LL;
sbg_driver::Publishers::DiagSnapshot healthy()
{
  sbg_driver::Publishers::DiagSnapshot s;
  s.has_device_status = s.has_ekf_status = s.has_imu_temperature = true;
  s.last_log_stamp_ns = s.last_status_stamp_ns = s.last_ekf_stamp_ns = s.last_temperature_stamp_ns =
    now;
  s.last_device_status_general = 0x7F;
  s.last_ekf_solution_mode = 4;
  s.last_ekf_status_raw = 0xF4;
  s.last_imu_temperature_c = 25.0F;
  return s;
}

TEST(Diagnostics, HealthyStreamsReportOk)
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  sbg_driver::diagnose_device(healthy(), now, status);
  EXPECT_EQ(status.level, Status::OK);
  sbg_driver::diagnose_ekf(healthy(), now, status);
  EXPECT_EQ(status.level, Status::OK);
  sbg_driver::diagnose_temperature(healthy(), now, status);
  EXPECT_EQ(status.level, Status::OK);
}

TEST(Diagnostics, RequiredHardwareFaultsCannotBeHiddenByFreshTraffic)
{
  for (unsigned flag :
       {SBG_ECOM_GENERAL_MAIN_POWER_OK, SBG_ECOM_GENERAL_IMU_POWER_OK, SBG_ECOM_GENERAL_SETTINGS_OK,
        SBG_ECOM_GENERAL_TEMPERATURE_OK}) {
    auto s = healthy();
    s.last_device_status_general &= ~flag;
    diagnostic_updater::DiagnosticStatusWrapper status;
    sbg_driver::diagnose_device(s, now, status);
    EXPECT_EQ(status.level, Status::ERROR) << flag;
  }
}

TEST(Diagnostics, NavModeDoesNotOverrideInvalidMeasurements)
{
  for (const unsigned flag : {0x10U, 0x20U, 0x40U, 0x80U}) {
    auto s = healthy();
    s.last_ekf_status_raw &= ~flag;
    diagnostic_updater::DiagnosticStatusWrapper status;
    sbg_driver::diagnose_ekf(s, now, status);
    EXPECT_NE(status.level, Status::OK) << flag;
  }
}

TEST(Diagnostics, EachStreamMustRemainFresh)
{
  auto s = healthy();
  s.last_status_stamp_ns = s.last_ekf_stamp_ns = s.last_temperature_stamp_ns =
    now - 3'000'000'000LL;
  // Other logs continue arriving while these three streams have stopped.
  diagnostic_updater::DiagnosticStatusWrapper status;
  sbg_driver::diagnose_device(s, now, status);
  EXPECT_EQ(status.level, Status::ERROR);
  sbg_driver::diagnose_ekf(s, now, status);
  EXPECT_EQ(status.level, Status::ERROR);
  sbg_driver::diagnose_temperature(s, now, status);
  EXPECT_EQ(status.level, Status::ERROR);
}

TEST(Diagnostics, CompositionWarningsRecoverWithoutResettingLifetimeCounters)
{
  auto s = healthy();
  s.composition_drops = 7;
  s.last_composition_drop_ns = now;
  diagnostic_updater::DiagnosticStatusWrapper status;
  sbg_driver::diagnose_device(s, now, status);
  EXPECT_EQ(status.level, Status::WARN);
  s.last_composition_drop_ns = now - 3'000'000'000LL;
  sbg_driver::diagnose_device(s, now, status);
  EXPECT_EQ(status.level, Status::OK);
}
}  // namespace
