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
  u.gpsPosData.timeOfWeek = 600000;
  u.gpsPosData.latitude = 47.6062;
  u.gpsPosData.longitude = -122.3321;

  for (std::uint16_t msg : {SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_LOG_GPS2_POS}) {
    sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, msg, &u};
    EXPECT_EQ(v.kind(), sbg::LogView::Kind::GnssPos) << "msg=" << msg;
    ASSERT_NE(v.as_gnss_pos(), nullptr);
    EXPECT_DOUBLE_EQ(v.as_gnss_pos()->latitude, 47.6062);
  }
}

TEST(LogView, NullLogUnionGivesNullAccessors)
{
  sbg::LogView v{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, nullptr};
  EXPECT_EQ(v.kind(), sbg::LogView::Kind::ImuData);
  EXPECT_EQ(v.as_imu_data(), nullptr);
  EXPECT_EQ(v.time_stamp_us(), 0U);
}

}  // namespace
