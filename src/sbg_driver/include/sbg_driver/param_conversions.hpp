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

#include <array>
#include <optional>
#include <sbg/configurator.hpp>
#include <string_view>
#include <vector>

namespace sbg_driver
{

// Map validated configure_device.* parameter strings to the typed sbg:: config
// enums used by the Configurator. generate_parameter_library's one_of<>
// validation guarantees the input is one of the listed strings, so each
// mapper's trailing return is only the (unreached) default branch.
//
// Extracted from driver_node.cpp so they are unit-testable without bringing up
// a node.

[[nodiscard]] sbg::MotionProfile motion_profile_from_string(std::string_view s);
[[nodiscard]] sbg::PortAssignment port_from_string(std::string_view s);
[[nodiscard]] sbg::AxisDirection axis_from_string(std::string_view s);
[[nodiscard]] sbg::MagModel mag_model_from_string(std::string_view s);
[[nodiscard]] sbg::GnssInstallationMode gnss_mode_from_string(std::string_view s);

// Returns nullopt for the "unchanged" sentinel (leave the device's setting).
[[nodiscard]] std::optional<sbg::OutputRate> output_rate_from_string(std::string_view s);
[[nodiscard]] sbg::OutputPort output_port_from_string(std::string_view s);

// gpl fixed_size<>:[3] guarantees the vector is length 3.
[[nodiscard]] std::array<float, 3> to_lever_arm(const std::vector<double> & v);

}  // namespace sbg_driver
