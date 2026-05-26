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

#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
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
  odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(cfg_.odom_topic, sensor_qos);

  // Construct the TF broadcaster unconditionally; we gate emission on the
  // per-link broadcast_* flags in on_log.
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
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
  if (odom_pub_) {
    odom_pub_->on_activate();
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
  if (odom_pub_) {
    odom_pub_->on_deactivate();
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

    case Kind::EkfVelBody:
      if (const auto * vel = view.as_ekf_vel_body()) {
        last_vel_body_ = *vel;  // cache for next EKF Nav
      }
      break;

    case Kind::EkfNav:
      if (const auto * nav = view.as_ekf_nav()) {
        // Set sticky origin on first arrival. EkfNav.position[0..1] are lat/lon
        // in degrees; promote altitude to ellipsoid height via undulation so
        // it lines up with /gps/fix.
        if (!geodetic_origin_) {
          const double lat0 = nav->position[0];
          const double lon0 = nav->position[1];
          const double alt0 = nav->position[2] + static_cast<double>(nav->undulation);
          constexpr double k_pi = 3.14159265358979323846;
          geodetic_origin_ = GeodeticOrigin{
            .lat = lat0,
            .lon = lon0,
            .alt = alt0,
            .cos_lat0 = std::cos(lat0 * k_pi / 180.0),
          };
        }

        // Compose Odometry if we have a recent EkfQuat and EkfVelBody.
        if (last_quat_ && last_vel_body_ && odom_pub_ && odom_pub_->is_activated()) {
          const auto stamp = clock_->now();
          auto msg = to_odometry(
            *nav, *last_quat_, *last_vel_body_, *geodetic_origin_, cfg_.convention,
            cfg_.odom_frame_id, cfg_.base_frame_id, stamp);

          // Optionally broadcast odom -> base_link from the same pose we just
          // published. Done before publish() so the transform is available to
          // subscribers as soon as they get the Odometry message.
          if (cfg_.broadcast_odom_to_base && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf{};
            tf.header.stamp = stamp;
            tf.header.frame_id = cfg_.odom_frame_id;
            tf.child_frame_id = cfg_.base_frame_id;
            tf.transform.translation.x = msg->pose.pose.position.x;
            tf.transform.translation.y = msg->pose.pose.position.y;
            tf.transform.translation.z = msg->pose.pose.position.z;
            tf.transform.rotation = msg->pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
          }

          odom_pub_->publish(std::move(msg));
        }
      }
      break;

    default:
      // Other log kinds dropped silently; phase 3c+ wires them up.
      break;
  }
}

}  // namespace sbg_driver
