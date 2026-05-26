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
#include <rclcpp/time.hpp>
#include <sbg/log_view.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

// ---- sensor_msgs/Imu -------------------------------------------------------
//
// Build an Imu message from an IMU log (linear_acceleration, angular_velocity)
// and optionally an EKF Quat log (orientation + orientation_covariance).
//
// `quat` may be null — when missing, orientation is set to identity and
// orientation_covariance[0] = -1 per the sensor_msgs convention (unknown).
//
// Covariance for linear_acceleration / angular_velocity is set to -1 in phase
// 2; phase 3+ populates from configurable noise-density parameters.
//
// Returns std::unique_ptr so callers can move into the publisher's
// `publish(std::unique_ptr<...>)` overload for intra-process zero-copy.
[[nodiscard]] std::unique_ptr<sensor_msgs::msg::Imu> to_imu(
  const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, FrameConvention convention,
  std::string_view frame_id, const rclcpp::Time & stamp);

}  // namespace sbg_driver
