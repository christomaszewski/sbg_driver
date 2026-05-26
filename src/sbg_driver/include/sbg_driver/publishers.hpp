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
#include <optional>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sbg/log_view.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

#include "sbg_driver/conversions.hpp"

namespace sbg_driver
{

// Owns the lifecycle publishers and dispatches log_view -> ROS message.
// Phase 2 covers only sensor_msgs/Imu; subsequent phases add the rest.
class Publishers
{
public:
  struct Config
  {
    std::string imu_data_topic = "imu/data";
    std::string imu_frame_id = "imu_link";
    FrameConvention convention = FrameConvention::Ned;
  };

  Publishers(rclcpp_lifecycle::LifecycleNode & node, Config config);

  // Activate / deactivate matches LifecycleNode state transitions.
  void activate();
  void deactivate();

  // Dispatch one log view. Called from the I/O thread.
  // Rejects gracefully if publishers haven't been activated yet.
  void on_log(const sbg::LogView & view);

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  Config cfg_;
  rclcpp::Clock::SharedPtr clock_;

  // Last EKF Quat — cached so the next IMU log can attach orientation +
  // covariance. Phase 3 will replace this with a proper triplet matcher.
  std::optional<SbgEComLogEkfQuat> last_quat_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
};

}  // namespace sbg_driver
