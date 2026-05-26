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
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(10);  // for low-rate aux topics

  imu_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>(cfg_.imu_data_topic, sensor_qos);
  imu_temp_pub_ =
    node_.create_publisher<sensor_msgs::msg::Temperature>(cfg_.imu_temperature_topic, reliable_qos);
  mag_pub_ = node_.create_publisher<sensor_msgs::msg::MagneticField>(cfg_.mag_topic, sensor_qos);
  nav_sat_pub_ =
    node_.create_publisher<sensor_msgs::msg::NavSatFix>(cfg_.nav_sat_fix_topic, sensor_qos);
  time_ref_pub_ = node_.create_publisher<sensor_msgs::msg::TimeReference>(
    cfg_.time_reference_topic, reliable_qos);
}

void Publishers::activate()
{
  if (imu_pub_) {
    imu_pub_->on_activate();
  }
  if (imu_temp_pub_) {
    imu_temp_pub_->on_activate();
  }
  if (mag_pub_) {
    mag_pub_->on_activate();
  }
  if (nav_sat_pub_) {
    nav_sat_pub_->on_activate();
  }
  if (time_ref_pub_) {
    time_ref_pub_->on_activate();
  }
}

void Publishers::deactivate()
{
  if (imu_pub_) {
    imu_pub_->on_deactivate();
  }
  if (imu_temp_pub_) {
    imu_temp_pub_->on_deactivate();
  }
  if (mag_pub_) {
    mag_pub_->on_deactivate();
  }
  if (nav_sat_pub_) {
    nav_sat_pub_->on_deactivate();
  }
  if (time_ref_pub_) {
    time_ref_pub_->on_deactivate();
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
      if (const auto * imu = view.as_imu_data()) {
        const auto stamp = clock_->now();
        if (imu_pub_ && imu_pub_->is_activated()) {
          auto msg = to_imu(
            *imu, last_quat_ ? &*last_quat_ : nullptr, cfg_.convention, cfg_.imu_frame_id, stamp);
          imu_pub_->publish(std::move(msg));
        }
        if (imu_temp_pub_ && imu_temp_pub_->is_activated()) {
          auto temp_msg = to_temperature(*imu, cfg_.imu_frame_id, stamp);
          imu_temp_pub_->publish(std::move(temp_msg));
        }
      }
      break;

    case Kind::Mag:
      if (mag_pub_ && mag_pub_->is_activated()) {
        if (const auto * mag = view.as_mag()) {
          auto msg = to_magnetic_field(*mag, cfg_.convention, cfg_.imu_frame_id, clock_->now());
          mag_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::GnssPos:
      if (nav_sat_pub_ && nav_sat_pub_->is_activated()) {
        if (const auto * gnss = view.as_gnss_pos()) {
          auto msg = to_navsat(*gnss, cfg_.gps_frame_id, clock_->now());
          nav_sat_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::Utc:
      if (time_ref_pub_ && time_ref_pub_->is_activated()) {
        if (const auto * utc = view.as_utc()) {
          auto msg = to_time_reference(*utc, cfg_.time_reference_frame_id, clock_->now());
          time_ref_pub_->publish(std::move(msg));
        }
      }
      break;

    default:
      // Other log kinds dropped silently; phase 3b+ wires them up.
      break;
  }
}

}  // namespace sbg_driver
