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

#include "sbg_driver/publishers.hpp"

#include <utility>

namespace sbg_driver
{

Publishers::Publishers(rclcpp_lifecycle::LifecycleNode & node, Config config)
: node_(node), cfg_(std::move(config)), clock_(node.get_clock())
{
  imu_pub_ =
    node_.create_publisher<sensor_msgs::msg::Imu>(cfg_.imu_data_topic, rclcpp::SensorDataQoS());
}

void Publishers::activate()
{
  if (imu_pub_) {
    imu_pub_->on_activate();
  }
}

void Publishers::deactivate()
{
  if (imu_pub_) {
    imu_pub_->on_deactivate();
  }
}

void Publishers::on_log(const sbg::LogView & view)
{
  using Kind = sbg::LogView::Kind;
  switch (view.kind()) {
    case Kind::EkfQuat:
      if (const auto * quat = view.as_ekf_quat()) {
        last_quat_ = *quat;  // cache for next IMU log
      }
      break;

    case Kind::ImuData:
      if (imu_pub_ && imu_pub_->is_activated()) {
        if (const auto * imu = view.as_imu_data()) {
          auto msg = to_imu(
            *imu, last_quat_ ? &*last_quat_ : nullptr, cfg_.convention, cfg_.imu_frame_id,
            clock_->now());
          imu_pub_->publish(std::move(msg));
        }
      }
      break;

    default:
      // Phase 2 only handles IMU + EkfQuat. Other log kinds dropped silently;
      // phases 3+ wire them up.
      break;
  }
}

}  // namespace sbg_driver
