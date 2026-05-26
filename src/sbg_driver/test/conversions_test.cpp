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

#include <gtest/gtest.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace
{

SbgEComLogImuLegacy make_imu(float ax, float ay, float az, float gx, float gy, float gz)
{
  SbgEComLogImuLegacy imu{};
  imu.accelerometers[0] = ax;
  imu.accelerometers[1] = ay;
  imu.accelerometers[2] = az;
  imu.gyroscopes[0] = gx;
  imu.gyroscopes[1] = gy;
  imu.gyroscopes[2] = gz;
  imu.timeStamp = 1000;
  return imu;
}

// IMU log fields are float; ROS Imu message uses double. The cast is lossless
// for representable values but EXPECT_DOUBLE_EQ requires bit-identical doubles
// — we use EXPECT_NEAR with a float-precision-friendly tolerance instead.
constexpr double k_float_tol = 1e-5;

TEST(Conversions, ImuNedPassesThroughUnchanged)
{
  auto imu = make_imu(1.0F, 2.0F, 9.81F, 0.1F, 0.2F, 0.3F);
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "imu_link");
  EXPECT_NEAR(msg->linear_acceleration.x, 1.0, k_float_tol);
  EXPECT_NEAR(msg->linear_acceleration.y, 2.0, k_float_tol);
  EXPECT_NEAR(msg->linear_acceleration.z, 9.81, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.x, 0.1, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.y, 0.2, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.z, 0.3, k_float_tol);
}

TEST(Conversions, ImuEnuFlipsYAndZ)
{
  auto imu = make_imu(1.0F, 2.0F, 9.81F, 0.1F, 0.2F, 0.3F);
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Enu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_NEAR(msg->linear_acceleration.x, 1.0, k_float_tol);
  EXPECT_NEAR(msg->linear_acceleration.y, -2.0, k_float_tol);
  EXPECT_NEAR(msg->linear_acceleration.z, -9.81, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.x, 0.1, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.y, -0.2, k_float_tol);
  EXPECT_NEAR(msg->angular_velocity.z, -0.3, k_float_tol);
}

TEST(Conversions, ImuWithoutQuatSetsUnknownOrientation)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_DOUBLE_EQ(msg->orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(msg->orientation_covariance[0], -1.0);
}

TEST(Conversions, ImuWithQuatPopulatesOrientationAndCovariance)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 1.0F;
  quat.quaternion[1] = 0.0F;
  quat.quaternion[2] = 0.0F;
  quat.quaternion[3] = 0.0F;
  quat.eulerStdDev[0] = 0.01F;
  quat.eulerStdDev[1] = 0.02F;
  quat.eulerStdDev[2] = 0.03F;

  auto msg = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_DOUBLE_EQ(msg->orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(msg->orientation.x, 0.0);
  // Float→double squared: actual tolerance scales with the value's magnitude
  // and the square. ~1e-7 (float ULP) squared at value 0.03 ≈ 6e-11; use 1e-10
  // as a safe envelope.
  EXPECT_NEAR(msg->orientation_covariance[0], 0.01 * 0.01, 1e-10);
  EXPECT_NEAR(msg->orientation_covariance[4], 0.02 * 0.02, 1e-10);
  EXPECT_NEAR(msg->orientation_covariance[8], 0.03 * 0.03, 1e-10);
}

TEST(Conversions, EnuQuatFlipsYAndZComponents)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 0.7071F;  // w
  quat.quaternion[1] = 0.0F;     // x
  quat.quaternion[2] = 0.7071F;  // y
  quat.quaternion[3] = 0.0F;     // z

  auto msg_ned = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  auto msg_enu = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Enu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  EXPECT_NEAR(msg_ned->orientation.y, 0.7071, k_float_tol);
  EXPECT_NEAR(msg_enu->orientation.y, -0.7071, k_float_tol);
  EXPECT_NEAR(msg_ned->orientation.z, 0.0, k_float_tol);
  EXPECT_NEAR(msg_enu->orientation.z, 0.0, k_float_tol);  // -0.0 == 0.0 numerically
  // Covariance diagonal preserved across the frame swap.
  EXPECT_EQ(msg_ned->orientation_covariance, msg_enu->orientation_covariance);
}

TEST(Conversions, AccelGyroCovarianceMarkedUnknown)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  // Phase 2 placeholder — phase 3 fills these from noise-density params.
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(msg->angular_velocity_covariance[0], -1.0);
}

// ---- MagneticField ---------------------------------------------------------

TEST(Conversions, MagFieldNed)
{
  SbgEComLogMag mag{};
  mag.magnetometers[0] = 1.0F;
  mag.magnetometers[1] = 2.0F;
  mag.magnetometers[2] = 3.0F;

  auto msg = sbg_driver::to_magnetic_field(
    mag, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "imu_link");
  EXPECT_NEAR(msg->magnetic_field.x, 1.0, k_float_tol);
  EXPECT_NEAR(msg->magnetic_field.y, 2.0, k_float_tol);
  EXPECT_NEAR(msg->magnetic_field.z, 3.0, k_float_tol);
  // Covariance currently unknown — phase 3b sets from noise-density params.
  EXPECT_DOUBLE_EQ(msg->magnetic_field_covariance[0], -1.0);
}

TEST(Conversions, MagFieldEnuFlipsYZ)
{
  SbgEComLogMag mag{};
  mag.magnetometers[0] = 1.0F;
  mag.magnetometers[1] = 2.0F;
  mag.magnetometers[2] = 3.0F;

  auto msg = sbg_driver::to_magnetic_field(
    mag, sbg_driver::FrameConvention::Enu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  EXPECT_NEAR(msg->magnetic_field.x, 1.0, k_float_tol);
  EXPECT_NEAR(msg->magnetic_field.y, -2.0, k_float_tol);
  EXPECT_NEAR(msg->magnetic_field.z, -3.0, k_float_tol);
}

// ---- Temperature -----------------------------------------------------------

TEST(Conversions, TemperatureFromImu)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  imu.temperature = 32.5F;
  auto msg = sbg_driver::to_temperature(imu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "imu_link");
  EXPECT_NEAR(msg->temperature, 32.5, k_float_tol);
  EXPECT_DOUBLE_EQ(msg->variance, 0.0);
}

// ---- NavSatFix -------------------------------------------------------------

SbgEComLogGnssPos make_gnss(double lat, double lon, double alt, std::uint32_t status)
{
  SbgEComLogGnssPos g{};
  g.latitude = lat;
  g.longitude = lon;
  g.altitude = alt;
  g.undulation = 0.0F;
  g.latitudeAccuracy = 1.0F;
  g.longitudeAccuracy = 2.0F;
  g.altitudeAccuracy = 3.0F;
  g.status = status;
  return g;
}

// Helper to build a packed status field (status low 6 bits | type bits 6-11).
constexpr std::uint32_t pack_gnss_status(SbgEComGnssPosStatus status, SbgEComGnssPosType type)
{
  return (static_cast<std::uint32_t>(status) & 0x3Fu) |
         ((static_cast<std::uint32_t>(type) & 0x3Fu) << 6u);
}

TEST(Conversions, NavSatFixFieldsAndCovariance)
{
  auto gnss = make_gnss(
    47.6062, -122.3321, 56.0,
    pack_gnss_status(SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED, SBG_ECOM_GNSS_POS_TYPE_SINGLE));
  auto msg = sbg_driver::to_navsat(gnss, "gps_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "gps_link");
  EXPECT_DOUBLE_EQ(msg->latitude, 47.6062);
  EXPECT_DOUBLE_EQ(msg->longitude, -122.3321);
  EXPECT_DOUBLE_EQ(msg->altitude, 56.0);
  EXPECT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_EQ(
    msg->position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  EXPECT_NEAR(msg->position_covariance[0], 4.0, k_float_tol);  // long²
  EXPECT_NEAR(msg->position_covariance[4], 1.0, k_float_tol);  // lat²
  EXPECT_NEAR(msg->position_covariance[8], 9.0, k_float_tol);  // alt²
}

TEST(Conversions, NavSatFixNoSolutionMapsToNoFix)
{
  auto gnss = make_gnss(
    0.0, 0.0, 0.0,
    pack_gnss_status(
      SBG_ECOM_GNSS_POS_STATUS_INSUFFICIENT_OBS, SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION));
  auto msg = sbg_driver::to_navsat(gnss, "gps_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  EXPECT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST(Conversions, NavSatFixRtkMapsToGbasFix)
{
  auto gnss = make_gnss(
    47.6, -122.3, 50.0,
    pack_gnss_status(SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED, SBG_ECOM_GNSS_POS_TYPE_RTK_INT));
  auto msg = sbg_driver::to_navsat(gnss, "gps_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  EXPECT_EQ(msg->status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}

TEST(Conversions, NavSatFixAddsUndulationToAltitude)
{
  auto gnss = make_gnss(
    0.0, 0.0, 100.0,
    pack_gnss_status(SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED, SBG_ECOM_GNSS_POS_TYPE_SINGLE));
  gnss.undulation = -33.5F;  // typical geoid undulation
  auto msg = sbg_driver::to_navsat(gnss, "gps_link", rclcpp::Clock{RCL_ROS_TIME}.now());
  EXPECT_NEAR(msg->altitude, 100.0 - 33.5, k_float_tol);
}

// ---- TimeReference ---------------------------------------------------------

TEST(Conversions, TimeReferenceComposeFromUtc)
{
  SbgEComLogUtc utc{};
  utc.year = 2026;
  utc.month = 5;
  utc.day = 22;
  utc.hour = 12;
  utc.minute = 30;
  utc.second = 45;
  utc.nanoSecond = 123'456'789;
  auto msg = sbg_driver::to_time_reference(utc, "utc", rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "utc");
  EXPECT_EQ(msg->source, "sbg_utc");

  // 2026-05-22 12:30:45.123456789 UTC = 1779453045 + 0.123456789 s since epoch
  // (20595 days from 1970-01-01 plus 45045 s of day → 1779453045).
  EXPECT_EQ(msg->time_ref.sec, 1779453045);
  EXPECT_EQ(msg->time_ref.nanosec, 123456789U);
}

}  // namespace
