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

#include "sbg/log_view.hpp"

#include <gtest/gtest.h>

#include "sbg/detail/c_api.hpp"

namespace
{

TEST(LogView, UnknownClassIsUnknown)
{
  SbgEComLogUnion u{};
  sbg::LogView v{0xFF, SBG_ECOM_LOG_IMU_DATA, &u};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::Unknown);
  EXPECT_EQ(v.as_imu_data(), nullptr);
}

TEST(LogView, ImuDataAccessor)
{
  SbgEComLogUnion u{};
  u.imuData.timeStamp = 12345;
  u.imuData.accelerometers[0] = 1.0F;
  u.imuData.accelerometers[1] = 2.0F;
  u.imuData.accelerometers[2] = 9.81F;

  sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, &u};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::ImuData);
  EXPECT_EQ(v.time_stamp_us(), 12345U);
  ASSERT_NE(v.as_imu_data(), nullptr);
  EXPECT_FLOAT_EQ(v.as_imu_data()->accelerometers[0], 1.0F);
  EXPECT_FLOAT_EQ(v.as_imu_data()->accelerometers[2], 9.81F);

  // Wrong-kind accessors return nullptr.
  EXPECT_EQ(v.as_ekf_quat(), nullptr);
  EXPECT_EQ(v.as_gnss_pos(), nullptr);
}

TEST(LogView, EkfQuatAccessor)
{
  SbgEComLogUnion u{};
  u.ekfQuatData.timeStamp = 999;
  u.ekfQuatData.quaternion[0] = 1.0F;  // w
  u.ekfQuatData.quaternion[1] = 0.0F;  // x
  u.ekfQuatData.quaternion[2] = 0.0F;  // y
  u.ekfQuatData.quaternion[3] = 0.0F;  // z
  u.ekfQuatData.eulerStdDev[0] = 0.01F;
  u.ekfQuatData.eulerStdDev[1] = 0.01F;
  u.ekfQuatData.eulerStdDev[2] = 0.02F;

  sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &u};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::EkfQuat);
  ASSERT_NE(v.as_ekf_quat(), nullptr);
  EXPECT_FLOAT_EQ(v.as_ekf_quat()->quaternion[0], 1.0F);
  EXPECT_FLOAT_EQ(v.as_ekf_quat()->eulerStdDev[2], 0.02F);
}

TEST(LogView, GnssPosAccessorBothChannels)
{
  SbgEComLogUnion u{};
  u.gpsPosData.timeStamp = 777;      // µs since power-up — what time_stamp_us() returns
  u.gpsPosData.timeOfWeek = 600000;  // GPS ToW (ms) — reachable via the struct, not the header
  u.gpsPosData.latitude = 47.6062;
  u.gpsPosData.longitude = -122.3321;

  for (std::uint16_t msg : {SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_LOG_GPS2_POS}) {
    sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, msg, &u};
    EXPECT_EQ(v.kind(), sbg::LogView::Kind::GnssPos) << "msg=" << msg;
    EXPECT_EQ(v.time_stamp_us(), 777U) << "msg=" << msg;
    ASSERT_NE(v.as_gnss_pos(), nullptr);
    EXPECT_DOUBLE_EQ(v.as_gnss_pos()->latitude, 47.6062);
  }
  EXPECT_EQ(
    (sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, &u}).gnss_instance(), 1);
  EXPECT_EQ(
    (sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS2_POS, &u}).gnss_instance(), 2);
}

// Every per-receiver GNSS id carries its receiver index; everything else is 0.
TEST(LogView, GnssInstanceTagging)
{
  SbgEComLogUnion u{};
  struct Case
  {
    std::uint16_t msg_id;
    std::uint8_t instance;
  };
  const Case cases[] = {
    {SBG_ECOM_LOG_GPS1_VEL, 1}, {SBG_ECOM_LOG_GPS2_VEL, 2}, {SBG_ECOM_LOG_GPS1_HDT, 1},
    {SBG_ECOM_LOG_GPS2_HDT, 2}, {SBG_ECOM_LOG_GPS1_RAW, 1}, {SBG_ECOM_LOG_GPS2_RAW, 2},
    {SBG_ECOM_LOG_MAG, 0},      {SBG_ECOM_LOG_EKF_NAV, 0},
  };
  for (const auto & c : cases) {
    sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, c.msg_id, &u};
    EXPECT_EQ(v.gnss_instance(), c.instance) << "msg_id=" << c.msg_id;
  }
}

TEST(LogView, NullLogUnionGivesNullAccessors)
{
  sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, nullptr};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::ImuData);
  EXPECT_EQ(v.as_imu_data(), nullptr);
  EXPECT_EQ(v.time_stamp_us(), 0U);
}

// Calls every accessor and asserts exactly the one matching `expected` is
// non-null (and all others null). Exercises both branches of all 18 accessors.
void expect_only_accessor(const sbg::LogView & v, sbg::LogView::Kind expected)
{
  using K = sbg::LogView::Kind;
  EXPECT_EQ(v.kind(), expected);
  EXPECT_EQ(v.as_imu_data() != nullptr, expected == K::ImuData);
  EXPECT_EQ(v.as_imu_short() != nullptr, expected == K::ImuShort);
  EXPECT_EQ(v.as_ekf_euler() != nullptr, expected == K::EkfEuler);
  EXPECT_EQ(v.as_ekf_quat() != nullptr, expected == K::EkfQuat);
  EXPECT_EQ(v.as_ekf_nav() != nullptr, expected == K::EkfNav);
  EXPECT_EQ(v.as_ekf_vel_body() != nullptr, expected == K::EkfVelBody);
  EXPECT_EQ(v.as_mag() != nullptr, expected == K::Mag);
  EXPECT_EQ(v.as_gnss_pos() != nullptr, expected == K::GnssPos);
  EXPECT_EQ(v.as_gnss_vel() != nullptr, expected == K::GnssVel);
  EXPECT_EQ(v.as_gnss_hdt() != nullptr, expected == K::GnssHdt);
  EXPECT_EQ(v.as_air_data() != nullptr, expected == K::AirData);
  EXPECT_EQ(v.as_utc() != nullptr, expected == K::Utc);
  EXPECT_EQ(v.as_status() != nullptr, expected == K::Status);
  EXPECT_EQ(v.as_ship_motion() != nullptr, expected == K::ShipMotion);
  EXPECT_EQ(v.as_event() != nullptr, expected == K::Event);
  EXPECT_EQ(v.as_mag_calib() != nullptr, expected == K::MagCalib);
  EXPECT_EQ(v.as_gps_raw() != nullptr, expected == K::GpsRawData);
  EXPECT_EQ(v.as_rtcm_raw() != nullptr, expected == K::RtcmRaw);
}

TEST(LogView, EveryAccessorMatchesItsKind)
{
  using K = sbg::LogView::Kind;
  struct Case
  {
    std::uint16_t msg_id;
    K kind;
  };
  const Case cases[] = {
    {SBG_ECOM_LOG_IMU_SHORT, K::ImuShort},
    {SBG_ECOM_LOG_IMU_DATA, K::ImuData},
    {SBG_ECOM_LOG_EKF_EULER, K::EkfEuler},
    {SBG_ECOM_LOG_EKF_QUAT, K::EkfQuat},
    {SBG_ECOM_LOG_EKF_NAV, K::EkfNav},
    {SBG_ECOM_LOG_EKF_VEL_BODY, K::EkfVelBody},
    {SBG_ECOM_LOG_MAG, K::Mag},
    {SBG_ECOM_LOG_MAG_CALIB, K::MagCalib},
    {SBG_ECOM_LOG_GPS1_POS, K::GnssPos},
    {SBG_ECOM_LOG_GPS1_VEL, K::GnssVel},
    {SBG_ECOM_LOG_GPS1_HDT, K::GnssHdt},
    {SBG_ECOM_LOG_GPS1_RAW, K::GpsRawData},
    {SBG_ECOM_LOG_RTCM_RAW, K::RtcmRaw},
    {SBG_ECOM_LOG_AIR_DATA, K::AirData},
    {SBG_ECOM_LOG_UTC_TIME, K::Utc},
    {SBG_ECOM_LOG_STATUS, K::Status},
    {SBG_ECOM_LOG_SHIP_MOTION, K::ShipMotion},
    {SBG_ECOM_LOG_EVENT_A, K::Event},
  };
  SbgEComLogUnion u{};
  for (const auto & c : cases) {
    sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, c.msg_id, &u};
    expect_only_accessor(v, c.kind);
  }
}

// A valid ECOM_0 class but an unrecognized msg id → Unknown (classify default).
TEST(LogView, UnmappedMsgIdInEcomClassIsUnknown)
{
  SbgEComLogUnion u{};
  sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, 0xFFFF, &u};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::Unknown);
  EXPECT_EQ(v.as_imu_data(), nullptr);
}

// Secondary-channel / high-precision / B-E event ids fold into the same kind.
TEST(LogView, AliasMsgIdsMapToExpectedKind)
{
  using K = sbg::LogView::Kind;
  struct Case
  {
    std::uint16_t msg_id;
    K kind;
  };
  const Case cases[] = {
    {SBG_ECOM_LOG_GPS2_POS, K::GnssPos},
    {SBG_ECOM_LOG_GPS2_VEL, K::GnssVel},
    {SBG_ECOM_LOG_GPS2_HDT, K::GnssHdt},
    {SBG_ECOM_LOG_GPS2_RAW, K::GpsRawData},
    {SBG_ECOM_LOG_SHIP_MOTION_HP, K::ShipMotion},
    {SBG_ECOM_LOG_EVENT_B, K::Event},
    {SBG_ECOM_LOG_EVENT_C, K::Event},
    {SBG_ECOM_LOG_EVENT_D, K::Event},
    {SBG_ECOM_LOG_EVENT_E, K::Event},
  };
  SbgEComLogUnion u{};
  for (const auto & c : cases) {
    sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, c.msg_id, &u};
    EXPECT_EQ(v.kind(), c.kind) << "msg_id=" << c.msg_id;
  }
}

}  // namespace
