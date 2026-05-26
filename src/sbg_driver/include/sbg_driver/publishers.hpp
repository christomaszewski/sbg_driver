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
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sbg/log_view.hpp>
#include <sbg_msgs/msg/ekf_status.hpp>
#include <sbg_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.hpp>

#include "sbg_driver/conversions.hpp"

namespace sbg_driver
{

// Owns the lifecycle publishers and dispatches log_view -> ROS message.
// Phase 3a covers IMU + MagneticField + Temperature + NavSatFix + TimeReference.
// Phase 3b+ adds Odometry composition, GPS velocity, custom sbg_msgs, etc.
class Publishers
{
public:
  struct Config
  {
    // Topic names (remappable via params)
    std::string imu_data_topic = "imu/data";
    std::string imu_temperature_topic = "imu/temperature";
    std::string mag_topic = "imu/mag";
    std::string nav_sat_fix_topic = "gps/fix";
    std::string time_reference_topic = "time_reference";
    std::string odom_topic = "odom";

    // SBG-specific custom-message topics
    std::string sbg_status_topic = "sbg/status";
    std::string sbg_ekf_status_topic = "sbg/ekf_status";

    // Frame IDs
    std::string imu_frame_id = "imu_link";
    std::string gps_frame_id = "gps_link";
    std::string time_reference_frame_id = "";  // empty = global
    std::string odom_frame_id = "odom";
    std::string base_frame_id = "base_link";

    // TF policy
    bool broadcast_odom_to_base = true;

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

  // Cached latest EKF logs. EkfNav is the "trigger" log - on its arrival, if
  // we have a recent EkfQuat AND EkfVelBody, compose an Odometry message.
  // EkfQuat is also used to attach orientation + covariance to /imu/data.
  std::optional<SbgEComLogEkfQuat> last_quat_;
  std::optional<SbgEComLogEkfVelBody> last_vel_body_;

  // Sticky origin set on the first EkfNav arrival - locks the local frame
  // so downstream odom poses are stable.
  std::optional<GeodeticOrigin> geodetic_origin_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>>
    imu_temp_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>> mag_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>> nav_sat_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::TimeReference>>
    time_ref_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::Status>> sbg_status_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::EkfStatus>>
    sbg_ekf_status_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace sbg_driver
