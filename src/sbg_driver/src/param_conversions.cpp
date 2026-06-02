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

#include <utility>

namespace sbg_driver
{

sbg::MotionProfile motion_profile_from_string(std::string_view s)
{
  using enum sbg::MotionProfile;
  constexpr std::array<std::pair<std::string_view, sbg::MotionProfile>, 13> table{{
    {"general_purpose", GeneralPurpose},
    {"automotive", Automotive},
    {"marine", Marine},
    {"airplane", Airplane},
    {"helicopter", Helicopter},
    {"pedestrian", Pedestrian},
    {"uav_rotary_wing", UavRotaryWing},
    {"heavy_machinery", HeavyMachinery},
    {"static", Static},
    {"truck", Truck},
    {"railway", Railway},
    {"off_road_vehicle", OffRoadVehicle},
    {"underwater", Underwater},
  }};
  for (const auto & [name, value] : table) {
    if (name == s) {
      return value;
    }
  }
  return GeneralPurpose;
}

sbg::PortAssignment port_from_string(std::string_view s)
{
  using enum sbg::PortAssignment;
  constexpr std::array<std::pair<std::string_view, sbg::PortAssignment>, 7> table{{
    {"port_a", PortA},
    {"port_b", PortB},
    {"port_c", PortC},
    {"port_d", PortD},
    {"port_e", PortE},
    {"internal", Internal},
    {"disabled", Disabled},
  }};
  for (const auto & [name, value] : table) {
    if (name == s) {
      return value;
    }
  }
  return Disabled;
}

sbg::AxisDirection axis_from_string(std::string_view s)
{
  using enum sbg::AxisDirection;
  constexpr std::array<std::pair<std::string_view, sbg::AxisDirection>, 6> table{{
    {"forward", Forward},
    {"backward", Backward},
    {"left", Left},
    {"right", Right},
    {"up", Up},
    {"down", Down},
  }};
  for (const auto & [name, value] : table) {
    if (name == s) {
      return value;
    }
  }
  return Forward;
}

sbg::MagModel mag_model_from_string(std::string_view s)
{
  return s == "external_ecom" ? sbg::MagModel::ExternalEcom : sbg::MagModel::InternalNormal;
}

sbg::GnssInstallationMode gnss_mode_from_string(std::string_view s)
{
  using enum sbg::GnssInstallationMode;
  constexpr std::array<std::pair<std::string_view, sbg::GnssInstallationMode>, 4> table{{
    {"single", Single},
    {"dual_auto", DualAuto},
    {"dual_rough", DualRough},
    {"dual_precise", DualPrecise},
  }};
  for (const auto & [name, value] : table) {
    if (name == s) {
      return value;
    }
  }
  return Single;
}

std::optional<sbg::OutputRate> output_rate_from_string(std::string_view s)
{
  using enum sbg::OutputRate;
  if (s == "unchanged") {
    return std::nullopt;
  }
  constexpr std::array<std::pair<std::string_view, sbg::OutputRate>, 13> table{{
    {"disabled", Disabled},
    {"main_loop", MainLoop},
    {"div_2", Div2},
    {"div_4", Div4},
    {"div_5", Div5},
    {"div_8", Div8},
    {"div_10", Div10},
    {"div_20", Div20},
    {"div_40", Div40},
    {"div_100", Div100},
    {"div_200", Div200},
    {"new_data", NewData},
    {"pps", Pps},
  }};
  for (const auto & [name, value] : table) {
    if (name == s) {
      return value;
    }
  }
  return std::nullopt;
}

sbg::OutputPort output_port_from_string(std::string_view s)
{
  if (s == "port_c") {
    return sbg::OutputPort::PortC;
  }
  if (s == "port_e") {
    return sbg::OutputPort::PortE;
  }
  return sbg::OutputPort::Main;
}

std::array<float, 3> to_lever_arm(const std::vector<double> & v)
{
  return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

}  // namespace sbg_driver
