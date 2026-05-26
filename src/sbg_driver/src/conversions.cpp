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

#include <array>
#include <cmath>

namespace sbg_driver
{

namespace
{

// In-place sign-flip on Y and Z components: NED → ENU for body-frame triplets.
constexpr void flip_yz(double & y, double & z) noexcept
{
  y = -y;
  z = -z;
}

// Rotate a quaternion expressed in NED into an ENU frame by applying the
// rotation R = diag(1, -1, -1). w and x are unchanged; only y and z flip.
// w and x kept in the signature for API symmetry with future rotations.
// REP-103 defines body axes as forward-left-up; sensor NED is forward-right-down.
constexpr void ned_quat_to_enu(
  [[maybe_unused]] double & w, [[maybe_unused]] double & x, double & y, double & z) noexcept
{
  y = -y;
  z = -z;
}

}  // namespace

std::unique_ptr<sensor_msgs::msg::Imu> to_imu(
  const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, FrameConvention convention,
  std::string_view frame_id, const rclcpp::Time & stamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = stamp;
  msg->header.frame_id.assign(frame_id);

  msg->linear_acceleration.x = static_cast<double>(imu.accelerometers[0]);
  msg->linear_acceleration.y = static_cast<double>(imu.accelerometers[1]);
  msg->linear_acceleration.z = static_cast<double>(imu.accelerometers[2]);

  msg->angular_velocity.x = static_cast<double>(imu.gyroscopes[0]);
  msg->angular_velocity.y = static_cast<double>(imu.gyroscopes[1]);
  msg->angular_velocity.z = static_cast<double>(imu.gyroscopes[2]);

  if (convention == FrameConvention::Enu) {
    flip_yz(msg->linear_acceleration.y, msg->linear_acceleration.z);
    flip_yz(msg->angular_velocity.y, msg->angular_velocity.z);
  }

  if (quat != nullptr) {
    double qw = static_cast<double>(quat->quaternion[0]);
    double qx = static_cast<double>(quat->quaternion[1]);
    double qy = static_cast<double>(quat->quaternion[2]);
    double qz = static_cast<double>(quat->quaternion[3]);
    if (convention == FrameConvention::Enu) {
      ned_quat_to_enu(qw, qx, qy, qz);
    }
    msg->orientation.w = qw;
    msg->orientation.x = qx;
    msg->orientation.y = qy;
    msg->orientation.z = qz;

    // Orientation covariance: diagonal from EkfQuat.eulerStdDev squared.
    // Rotation between NED and ENU preserves diagonal variances.
    const double sx = static_cast<double>(quat->eulerStdDev[0]);
    const double sy = static_cast<double>(quat->eulerStdDev[1]);
    const double sz = static_cast<double>(quat->eulerStdDev[2]);
    msg->orientation_covariance.fill(0.0);
    msg->orientation_covariance[0] = sx * sx;
    msg->orientation_covariance[4] = sy * sy;
    msg->orientation_covariance[8] = sz * sz;
  } else {
    // No orientation data yet — sentinel per sensor_msgs/Imu.
    msg->orientation.w = 1.0;
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation_covariance.fill(0.0);
    msg->orientation_covariance[0] = -1.0;
  }

  // Phase 2 placeholder: accel/gyro covariance is unknown until phase 3+
  // wires up noise-density params. Mark as unknown (-1) per sensor_msgs.
  msg->linear_acceleration_covariance.fill(0.0);
  msg->linear_acceleration_covariance[0] = -1.0;
  msg->angular_velocity_covariance.fill(0.0);
  msg->angular_velocity_covariance[0] = -1.0;

  return msg;
}

}  // namespace sbg_driver
