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

#include "sbg_driver/param_conversions.hpp"

#include <gtest/gtest.h>

#include <string_view>
#include <vector>

namespace
{
using namespace sbg_driver;  // NOLINT(build/namespaces) — test-local convenience

TEST(ParamConversions, MotionProfile)
{
  EXPECT_EQ(motion_profile_from_string("general_purpose"), sbg::MotionProfile::GeneralPurpose);
  EXPECT_EQ(motion_profile_from_string("automotive"), sbg::MotionProfile::Automotive);
  EXPECT_EQ(motion_profile_from_string("underwater"), sbg::MotionProfile::Underwater);
  // Unknown input falls back to the safe default.
  EXPECT_EQ(motion_profile_from_string("nonsense"), sbg::MotionProfile::GeneralPurpose);
}

TEST(ParamConversions, Port)
{
  EXPECT_EQ(port_from_string("internal"), sbg::PortAssignment::Internal);
  EXPECT_EQ(port_from_string("port_b"), sbg::PortAssignment::PortB);
  EXPECT_EQ(port_from_string("port_e"), sbg::PortAssignment::PortE);
  EXPECT_EQ(port_from_string("disabled"), sbg::PortAssignment::Disabled);
  EXPECT_EQ(port_from_string("???"), sbg::PortAssignment::Disabled);
}

TEST(ParamConversions, Axis)
{
  EXPECT_EQ(axis_from_string("forward"), sbg::AxisDirection::Forward);
  EXPECT_EQ(axis_from_string("left"), sbg::AxisDirection::Left);
  EXPECT_EQ(axis_from_string("down"), sbg::AxisDirection::Down);
}

TEST(ParamConversions, MagModel)
{
  EXPECT_EQ(mag_model_from_string("internal_normal"), sbg::MagModel::InternalNormal);
  EXPECT_EQ(mag_model_from_string("external_ecom"), sbg::MagModel::ExternalEcom);
}

TEST(ParamConversions, GnssMode)
{
  EXPECT_EQ(gnss_mode_from_string("single"), sbg::GnssInstallationMode::Single);
  EXPECT_EQ(gnss_mode_from_string("dual_precise"), sbg::GnssInstallationMode::DualPrecise);
}

TEST(ParamConversions, OutputRateUnchangedIsNullopt)
{
  EXPECT_FALSE(output_rate_from_string("unchanged").has_value());
}

TEST(ParamConversions, OutputRateMappings)
{
  EXPECT_EQ(output_rate_from_string("disabled").value(), sbg::OutputRate::Disabled);
  EXPECT_EQ(output_rate_from_string("div_8").value(), sbg::OutputRate::Div8);
  EXPECT_EQ(output_rate_from_string("div_200").value(), sbg::OutputRate::Div200);
  EXPECT_EQ(output_rate_from_string("pps").value(), sbg::OutputRate::Pps);
  // Every non-sentinel rate string resolves to a value.
  for (std::string_view r :
       {"disabled", "main_loop", "div_2", "div_4", "div_5", "div_8", "div_10", "div_20", "div_40",
        "div_100", "div_200", "new_data", "pps"}) {
    EXPECT_TRUE(output_rate_from_string(r).has_value()) << r;
  }
}

TEST(ParamConversions, OutputPort)
{
  EXPECT_EQ(output_port_from_string("main"), sbg::OutputPort::Main);
  EXPECT_EQ(output_port_from_string("port_c"), sbg::OutputPort::PortC);
  EXPECT_EQ(output_port_from_string("port_e"), sbg::OutputPort::PortE);
}

TEST(ParamConversions, LeverArm)
{
  const std::vector<double> v{1.5, -2.0, 0.25};
  const auto a = to_lever_arm(v);
  EXPECT_FLOAT_EQ(a[0], 1.5F);
  EXPECT_FLOAT_EQ(a[1], -2.0F);
  EXPECT_FLOAT_EQ(a[2], 0.25F);
}

}  // namespace
