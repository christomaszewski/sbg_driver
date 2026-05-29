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
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
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
    imu, nullptr, sbg_driver::FrameConvention::Enu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
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
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
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
    imu, &quat, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
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

TEST(Conversions, NedOrientationPassesThrough)
{
  // In NED mode the orientation quaternion is the SBG quaternion verbatim
  // (just reordered w,x,y,z -> ROS x,y,z,w by the message field layout).
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 0.7071F;  // w
  quat.quaternion[1] = 0.0F;     // x
  quat.quaternion[2] = 0.7071F;  // y
  quat.quaternion[3] = 0.0F;     // z

  auto msg = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
  EXPECT_NEAR(msg->orientation.w, 0.7071, k_float_tol);
  EXPECT_NEAR(msg->orientation.x, 0.0, k_float_tol);
  EXPECT_NEAR(msg->orientation.y, 0.7071, k_float_tol);
  EXPECT_NEAR(msg->orientation.z, 0.0, k_float_tol);
}

TEST(Conversions, EnuOrientationAppliesAxisSwapNotJustFlip)
{
  // NED→ENU is NOT a y/z flip alone: it composes the body FRD→FLU flip
  // (w,x,-y,-z) with a +90°-about-Z reference-axis swap Q=(√2/2,0,0,√2/2),
  // i.e. q_enu = Q ⊗ (w,x,-y,-z). For SBG (w,x,y,z)=(0.7071,0,0.7071,0) the
  // hand-computed result is (w,x,y,z)=(0.5,0.5,-0.5,0.5). A flip-only
  // implementation would (wrongly) give (0.7071,0,-0.7071,0).
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 0.7071F;  // w
  quat.quaternion[1] = 0.0F;     // x
  quat.quaternion[2] = 0.7071F;  // y
  quat.quaternion[3] = 0.0F;     // z

  auto msg = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Enu, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
  EXPECT_NEAR(msg->orientation.w, 0.5, k_float_tol);
  EXPECT_NEAR(msg->orientation.x, 0.5, k_float_tol);
  EXPECT_NEAR(msg->orientation.y, -0.5, k_float_tol);
  EXPECT_NEAR(msg->orientation.z, 0.5, k_float_tol);
  // Resulting quaternion stays normalized. The transform is norm-preserving;
  // the envelope (1e-4) only accommodates the rounded 0.7071F input, whose own
  // norm is ~0.99998, not the transform itself.
  const double n =
    msg->orientation.w * msg->orientation.w + msg->orientation.x * msg->orientation.x +
    msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z;
  EXPECT_NEAR(n, 1.0, 1e-4);
  // Orientation covariance diagonal is preserved regardless of convention.
  auto msg_ned = sbg_driver::to_imu(
    imu, &quat, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
  EXPECT_EQ(msg_ned->orientation_covariance, msg->orientation_covariance);
}

TEST(Conversions, AccelGyroCovarianceUnknownWhenDefault)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  // Default ImuCovariance{} has negative variances => unknown sentinel.
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    sbg_driver::ImuCovariance{});
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(msg->angular_velocity_covariance[0], -1.0);
}

TEST(Conversions, AccelGyroCovariancePopulatedWhenKnown)
{
  auto imu = make_imu(0, 0, 9.81F, 0, 0, 0);
  sbg_driver::ImuCovariance cov{.accel_variance = 4.0e-4, .gyro_variance = 9.0e-6};
  auto msg = sbg_driver::to_imu(
    imu, nullptr, sbg_driver::FrameConvention::Ned, "imu_link", rclcpp::Clock{RCL_ROS_TIME}.now(),
    cov);
  // Variance lands on all three diagonal entries; off-diagonal stays zero.
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[0], 4.0e-4);
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[4], 4.0e-4);
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[8], 4.0e-4);
  EXPECT_DOUBLE_EQ(msg->linear_acceleration_covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(msg->angular_velocity_covariance[0], 9.0e-6);
  EXPECT_DOUBLE_EQ(msg->angular_velocity_covariance[4], 9.0e-6);
  EXPECT_DOUBLE_EQ(msg->angular_velocity_covariance[8], 9.0e-6);
}

// ---- resolve_imu_covariance ------------------------------------------------

TEST(ResolveImuCovariance, CustomWithNoStddevIsUnknown)
{
  auto cov = sbg_driver::resolve_imu_covariance("custom", -1.0, -1.0);
  EXPECT_LT(cov.accel_variance, 0.0);
  EXPECT_LT(cov.gyro_variance, 0.0);
}

TEST(ResolveImuCovariance, ExplicitStddevSquared)
{
  auto cov = sbg_driver::resolve_imu_covariance("custom", 0.02, 0.003);
  EXPECT_NEAR(cov.accel_variance, 0.02 * 0.02, 1e-12);
  EXPECT_NEAR(cov.gyro_variance, 0.003 * 0.003, 1e-12);
}

TEST(ResolveImuCovariance, ModelDefaultUsedWhenStddevNegative)
{
  // "ekinox" tier has known defaults; both should resolve to a positive variance.
  auto cov = sbg_driver::resolve_imu_covariance("ekinox", -1.0, -1.0);
  EXPECT_GT(cov.accel_variance, 0.0);
  EXPECT_GT(cov.gyro_variance, 0.0);
}

TEST(ResolveImuCovariance, ExplicitOverridesModel)
{
  auto model = sbg_driver::resolve_imu_covariance("ellipse", -1.0, -1.0);
  auto override_accel = sbg_driver::resolve_imu_covariance("ellipse", 0.1, -1.0);
  // Explicit accel stddev wins over the model default; gyro still model default.
  EXPECT_NEAR(override_accel.accel_variance, 0.1 * 0.1, 1e-12);
  EXPECT_NEAR(override_accel.gyro_variance, model.gyro_variance, 1e-12);
}

TEST(ResolveImuCovariance, UnknownModelNameIsUnknown)
{
  auto cov = sbg_driver::resolve_imu_covariance("not_a_real_model", -1.0, -1.0);
  EXPECT_LT(cov.accel_variance, 0.0);
  EXPECT_LT(cov.gyro_variance, 0.0);
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

// ---- Geodetic → local Cartesian --------------------------------------------

TEST(GeodeticToLocal, OriginIsZero)
{
  sbg_driver::GeodeticOrigin origin{
    .lat = 47.6062,
    .lon = -122.3321,
    .alt = 56.0,
    .cos_lat0 = std::cos(47.6062 * 3.14159265358979323846 / 180.0)};
  auto local = sbg_driver::geodetic_to_local(
    47.6062, -122.3321, 56.0, origin, sbg_driver::FrameConvention::Enu);
  EXPECT_NEAR(local.x, 0.0, 1e-6);
  EXPECT_NEAR(local.y, 0.0, 1e-6);
  EXPECT_NEAR(local.z, 0.0, 1e-6);
}

TEST(GeodeticToLocal, EnuFrameAxes)
{
  // Origin at equator, lon 0, alt 0 → 1° east ≈ 111 320 m, 1° north ≈ 111 320 m.
  sbg_driver::GeodeticOrigin origin{.lat = 0.0, .lon = 0.0, .alt = 0.0, .cos_lat0 = 1.0};
  auto east_only =
    sbg_driver::geodetic_to_local(0.0, 1.0, 0.0, origin, sbg_driver::FrameConvention::Enu);
  EXPECT_NEAR(east_only.x, 111319.49, 1.0);  // east
  EXPECT_NEAR(east_only.y, 0.0, 1e-3);       // north
  EXPECT_NEAR(east_only.z, 0.0, 1e-9);

  auto north_only =
    sbg_driver::geodetic_to_local(1.0, 0.0, 0.0, origin, sbg_driver::FrameConvention::Enu);
  EXPECT_NEAR(north_only.x, 0.0, 1e-3);
  EXPECT_NEAR(north_only.y, 111319.49, 1.0);

  auto up_only =
    sbg_driver::geodetic_to_local(0.0, 0.0, 100.0, origin, sbg_driver::FrameConvention::Enu);
  EXPECT_NEAR(up_only.z, 100.0, 1e-9);
}

TEST(GeodeticToLocal, NedFrameSwap)
{
  // NED: x=north, y=east, z=down. Test 1° north + 1° east + +100 m up.
  sbg_driver::GeodeticOrigin origin{.lat = 0.0, .lon = 0.0, .alt = 0.0, .cos_lat0 = 1.0};
  auto p = sbg_driver::geodetic_to_local(1.0, 1.0, 100.0, origin, sbg_driver::FrameConvention::Ned);
  EXPECT_NEAR(p.x, 111319.49, 1.0);  // north
  EXPECT_NEAR(p.y, 111319.49, 1.0);  // east
  EXPECT_NEAR(p.z, -100.0, 1e-9);    // down (negative of up)
}

// ---- Odometry composition --------------------------------------------------

TEST(Conversions, OdometryFromTripletEnu)
{
  sbg_driver::GeodeticOrigin origin{
    .lat = 47.6062,
    .lon = -122.3321,
    .alt = 56.0,
    .cos_lat0 = std::cos(47.6062 * 3.14159265358979323846 / 180.0)};

  SbgEComLogEkfNav nav{};
  nav.position[0] = 47.6062;    // lat (unchanged → x/y should be ~0)
  nav.position[1] = -122.3321;  // lon
  nav.position[2] = 56.0;       // altitude MSL
  nav.undulation = 0.0F;
  nav.velocity[0] = 1.0F;  // north (unused — body twist takes priority)
  nav.velocity[1] = 2.0F;
  nav.velocity[2] = 0.0F;
  nav.positionStdDev[0] = 0.5F;  // lat std
  nav.positionStdDev[1] = 0.6F;  // lon std
  nav.positionStdDev[2] = 1.5F;  // alt std

  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 1.0F;  // identity
  quat.eulerStdDev[0] = 0.01F;
  quat.eulerStdDev[1] = 0.02F;
  quat.eulerStdDev[2] = 0.03F;

  SbgEComLogEkfVelBody vel{};
  vel.velocity[0] = 5.0F;  // body forward
  vel.velocity[1] = 0.5F;
  vel.velocity[2] = 0.1F;
  vel.velocityStdDev[0] = 0.1F;
  vel.velocityStdDev[1] = 0.2F;
  vel.velocityStdDev[2] = 0.3F;

  auto msg = sbg_driver::to_odometry(
    nav, quat, vel, origin, sbg_driver::FrameConvention::Enu, "odom", "base_link",
    rclcpp::Clock{RCL_ROS_TIME}.now());
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->header.frame_id, "odom");
  EXPECT_EQ(msg->child_frame_id, "base_link");
  EXPECT_NEAR(msg->pose.pose.position.x, 0.0, 1e-3);  // at origin → 0
  EXPECT_NEAR(msg->pose.pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(msg->pose.pose.position.z, 0.0, 1e-3);
  // ENU(identity NED orientation) = +90°-about-Z quaternion (√2/2, 0, 0, √2/2).
  EXPECT_NEAR(msg->pose.pose.orientation.w, 0.70710678, k_float_tol);
  EXPECT_NEAR(msg->pose.pose.orientation.x, 0.0, k_float_tol);
  EXPECT_NEAR(msg->pose.pose.orientation.y, 0.0, k_float_tol);
  EXPECT_NEAR(msg->pose.pose.orientation.z, 0.70710678, k_float_tol);

  // ENU: pose.x = east = lon_std; pose.y = north = lat_std; pose.z = alt_std.
  // Tolerance scales with value² × float-ulp; 1e-6 envelope covers 0..2 m std range.
  EXPECT_NEAR(msg->pose.covariance[0], 0.6 * 0.6, 1e-6);      // east  std²
  EXPECT_NEAR(msg->pose.covariance[7], 0.5 * 0.5, 1e-6);      // north std²
  EXPECT_NEAR(msg->pose.covariance[14], 1.5 * 1.5, 1e-6);     // up    std²
  EXPECT_NEAR(msg->pose.covariance[21], 0.01 * 0.01, 1e-10);  // roll
  EXPECT_NEAR(msg->pose.covariance[28], 0.02 * 0.02, 1e-10);  // pitch
  EXPECT_NEAR(msg->pose.covariance[35], 0.03 * 0.03, 1e-10);  // yaw

  // Body-frame twist with ENU sign flip on y/z.
  EXPECT_NEAR(msg->twist.twist.linear.x, 5.0, k_float_tol);
  EXPECT_NEAR(msg->twist.twist.linear.y, -0.5, k_float_tol);
  EXPECT_NEAR(msg->twist.twist.linear.z, -0.1, k_float_tol);
  EXPECT_DOUBLE_EQ(msg->twist.covariance[21], -1.0);  // angular unknown
}

TEST(Conversions, OdometryNedPreservesAxes)
{
  sbg_driver::GeodeticOrigin origin{.lat = 0.0, .lon = 0.0, .alt = 0.0, .cos_lat0 = 1.0};
  SbgEComLogEkfNav nav{};
  nav.position[0] = 1.0;  // 1° north
  nav.position[1] = 0.0;
  nav.position[2] = 10.0;
  nav.positionStdDev[0] = 0.5F;
  nav.positionStdDev[1] = 0.6F;
  nav.positionStdDev[2] = 1.5F;
  SbgEComLogEkfQuat quat{};
  quat.quaternion[0] = 1.0F;
  SbgEComLogEkfVelBody vel{};
  vel.velocity[0] = 5.0F;
  vel.velocity[1] = 0.5F;
  vel.velocity[2] = 0.1F;

  auto msg = sbg_driver::to_odometry(
    nav, quat, vel, origin, sbg_driver::FrameConvention::Ned, "odom", "base_link",
    rclcpp::Clock{RCL_ROS_TIME}.now());
  // NED: pose.x = north, pose.y = east, pose.z = -up.
  EXPECT_NEAR(msg->pose.pose.position.x, 111319.49, 1.0);
  EXPECT_NEAR(msg->pose.pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(msg->pose.pose.position.z, -10.0, 1e-9);
  // Body twist unchanged in NED.
  EXPECT_NEAR(msg->twist.twist.linear.x, 5.0, k_float_tol);
  EXPECT_NEAR(msg->twist.twist.linear.y, 0.5, k_float_tol);
  EXPECT_NEAR(msg->twist.twist.linear.z, 0.1, k_float_tol);
}

}  // namespace
