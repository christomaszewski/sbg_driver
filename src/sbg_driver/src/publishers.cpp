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
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <utility>

namespace sbg_driver
{

Publishers::Publishers(rclcpp_lifecycle::LifecycleNode & node, Config config)
: node_(node), cfg_(std::move(config)), clock_(node.get_clock())
{
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto reliable_qos = rclcpp::QoS(10);  // for low-rate aux topics

  // Expose qos_overrides.<topic>.* params on every publisher so integrators
  // can retune reliability / durability / depth / history at launch without
  // recompiling. The base profiles above remain the defaults.
  rclcpp::PublisherOptions pub_opts;
  pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  imu_pub_ =
    node_.create_publisher<sensor_msgs::msg::Imu>(cfg_.imu_data_topic, sensor_qos, pub_opts);
  imu_temp_pub_ = node_.create_publisher<sensor_msgs::msg::Temperature>(
    cfg_.imu_temperature_topic, reliable_qos, pub_opts);
  mag_pub_ =
    node_.create_publisher<sensor_msgs::msg::MagneticField>(cfg_.mag_topic, sensor_qos, pub_opts);
  nav_sat_pub_ = node_.create_publisher<sensor_msgs::msg::NavSatFix>(
    cfg_.nav_sat_fix_topic, sensor_qos, pub_opts);
  time_ref_pub_ = node_.create_publisher<sensor_msgs::msg::TimeReference>(
    cfg_.time_reference_topic, reliable_qos, pub_opts);
  odom_pub_ =
    node_.create_publisher<nav_msgs::msg::Odometry>(cfg_.odom_topic, sensor_qos, pub_opts);
  sbg_status_pub_ =
    node_.create_publisher<sbg_msgs::msg::Status>(cfg_.sbg_status_topic, reliable_qos, pub_opts);
  sbg_ekf_status_pub_ = node_.create_publisher<sbg_msgs::msg::EkfStatus>(
    cfg_.sbg_ekf_status_topic, reliable_qos, pub_opts);
  sbg_air_data_status_pub_ = node_.create_publisher<sbg_msgs::msg::AirDataStatus>(
    cfg_.sbg_air_data_status_topic, reliable_qos, pub_opts);
  sbg_event_pub_ =
    node_.create_publisher<sbg_msgs::msg::Event>(cfg_.sbg_event_topic, reliable_qos, pub_opts);
  sbg_gps_raw_pub_ =
    node_.create_publisher<sbg_msgs::msg::GpsRaw>(cfg_.sbg_gps_raw_topic, reliable_qos, pub_opts);
  sbg_mag_calib_pub_ = node_.create_publisher<sbg_msgs::msg::MagCalib>(
    cfg_.sbg_mag_calib_topic, reliable_qos, pub_opts);
  sbg_ship_motion_pub_ = node_.create_publisher<sbg_msgs::msg::ShipMotion>(
    cfg_.sbg_ship_motion_topic, sensor_qos, pub_opts);

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
  if (sbg_status_pub_) {
    sbg_status_pub_->on_activate();
  }
  if (sbg_ekf_status_pub_) {
    sbg_ekf_status_pub_->on_activate();
  }
  if (sbg_air_data_status_pub_) {
    sbg_air_data_status_pub_->on_activate();
  }
  if (sbg_event_pub_) {
    sbg_event_pub_->on_activate();
  }
  if (sbg_gps_raw_pub_) {
    sbg_gps_raw_pub_->on_activate();
  }
  if (sbg_mag_calib_pub_) {
    sbg_mag_calib_pub_->on_activate();
  }
  if (sbg_ship_motion_pub_) {
    sbg_ship_motion_pub_->on_activate();
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
  if (sbg_status_pub_) {
    sbg_status_pub_->on_deactivate();
  }
  if (sbg_ekf_status_pub_) {
    sbg_ekf_status_pub_->on_deactivate();
  }
  if (sbg_air_data_status_pub_) {
    sbg_air_data_status_pub_->on_deactivate();
  }
  if (sbg_event_pub_) {
    sbg_event_pub_->on_deactivate();
  }
  if (sbg_gps_raw_pub_) {
    sbg_gps_raw_pub_->on_deactivate();
  }
  if (sbg_mag_calib_pub_) {
    sbg_mag_calib_pub_->on_deactivate();
  }
  if (sbg_ship_motion_pub_) {
    sbg_ship_motion_pub_->on_deactivate();
  }
}

Publishers::DiagSnapshot Publishers::diag_snapshot() const noexcept
{
  DiagSnapshot s;
  s.last_log_stamp_ns = diag_last_log_stamp_ns_.load(std::memory_order_relaxed);
  s.last_ekf_status_raw = diag_last_ekf_status_raw_.load(std::memory_order_relaxed);
  s.last_ekf_solution_mode = diag_last_ekf_solution_mode_.load(std::memory_order_relaxed);
  s.last_device_status_general = diag_last_device_status_general_.load(std::memory_order_relaxed);
  s.has_ekf_status = diag_has_ekf_status_.load(std::memory_order_relaxed);
  s.has_device_status = diag_has_device_status_.load(std::memory_order_relaxed);
  s.last_imu_temperature_c = diag_last_imu_temperature_c_.load(std::memory_order_relaxed);
  s.has_imu_temperature = diag_has_imu_temperature_.load(std::memory_order_relaxed);
  return s;
}

void Publishers::on_log(const sbg::LogView & view)
{
  using Kind = sbg::LogView::Kind;
  // Update the "any log seen" timestamp on every dispatch.
  diag_last_log_stamp_ns_.store(clock_->now().nanoseconds(), std::memory_order_relaxed);
  switch (view.kind()) {
    case Kind::EkfQuat:
      if (const auto * quat = view.as_ekf_quat()) {
        last_quat_ = *quat;  // cache for next IMU log
      }
      break;

    case Kind::ImuData:
      if (const auto * imu = view.as_imu_data()) {
        diag_last_imu_temperature_c_.store(imu->temperature, std::memory_order_relaxed);
        diag_has_imu_temperature_.store(true, std::memory_order_relaxed);
        const auto stamp = clock_->now();
        if (imu_pub_ && imu_pub_->is_activated()) {
          auto msg = to_imu(
            *imu, last_quat_ ? &*last_quat_ : nullptr, cfg_.convention, cfg_.imu_frame_id, stamp,
            cfg_.imu_covariance);
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

    case Kind::Status:
      if (const auto * status = view.as_status()) {
        diag_last_device_status_general_.store(status->generalStatus, std::memory_order_relaxed);
        diag_has_device_status_.store(true, std::memory_order_relaxed);
        if (sbg_status_pub_ && sbg_status_pub_->is_activated()) {
          auto msg = to_status(*status, cfg_.imu_frame_id, clock_->now());
          sbg_status_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::ShipMotion:
      if (sbg_ship_motion_pub_ && sbg_ship_motion_pub_->is_activated()) {
        if (const auto * ship = view.as_ship_motion()) {
          auto msg = to_ship_motion(*ship, cfg_.base_frame_id, clock_->now());
          sbg_ship_motion_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::Event:
      if (sbg_event_pub_ && sbg_event_pub_->is_activated()) {
        if (const auto * ev = view.as_event()) {
          auto msg = to_event(*ev, cfg_.imu_frame_id, clock_->now());
          sbg_event_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::MagCalib:
      if (sbg_mag_calib_pub_ && sbg_mag_calib_pub_->is_activated()) {
        if (const auto * cal = view.as_mag_calib()) {
          auto msg = to_mag_calib(*cal, cfg_.imu_frame_id, clock_->now());
          sbg_mag_calib_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::GpsRawData:
      if (sbg_gps_raw_pub_ && sbg_gps_raw_pub_->is_activated()) {
        if (const auto * raw = view.as_gps_raw()) {
          auto msg = to_gps_raw(*raw, cfg_.gps_frame_id, clock_->now());
          sbg_gps_raw_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::AirData:
      if (sbg_air_data_status_pub_ && sbg_air_data_status_pub_->is_activated()) {
        if (const auto * air = view.as_air_data()) {
          auto msg = to_air_data_status(*air, cfg_.imu_frame_id, clock_->now());
          sbg_air_data_status_pub_->publish(std::move(msg));
        }
      }
      // Phase 3e will also emit sensor_msgs/FluidPressure + Temperature here.
      break;

    case Kind::EkfNav:
      if (const auto * nav = view.as_ekf_nav()) {
        diag_last_ekf_status_raw_.store(nav->status, std::memory_order_relaxed);
        diag_last_ekf_solution_mode_.store(
          static_cast<std::uint8_t>(nav->status & 0x0Fu), std::memory_order_relaxed);
        diag_has_ekf_status_.store(true, std::memory_order_relaxed);
        const auto stamp_ekf = clock_->now();

        // Publish decoded EKF status alongside Odometry composition.
        if (sbg_ekf_status_pub_ && sbg_ekf_status_pub_->is_activated()) {
          auto status_msg = to_ekf_status(*nav, cfg_.imu_frame_id, stamp_ekf);
          sbg_ekf_status_pub_->publish(std::move(status_msg));
        }

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
          auto msg = to_odometry(
            *nav, *last_quat_, *last_vel_body_, *geodetic_origin_, cfg_.convention,
            cfg_.odom_frame_id, cfg_.base_frame_id, stamp_ekf);

          // Optionally broadcast odom -> base_link from the same pose we just
          // published. Done before publish() so the transform is available to
          // subscribers as soon as they get the Odometry message.
          if (cfg_.broadcast_odom_to_base && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf{};
            tf.header.stamp = stamp_ekf;
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
