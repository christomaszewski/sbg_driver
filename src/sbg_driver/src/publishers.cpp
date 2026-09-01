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
  if (cfg_.publish_ekf_nav_sat_fix) {
    ekf_nav_sat_pub_ = node_.create_publisher<sensor_msgs::msg::NavSatFix>(
      cfg_.ekf_nav_sat_fix_topic, sensor_qos, pub_opts);
  }
  if (cfg_.publish_nmea_gga) {
    nmea_gga_pub_ =
      node_.create_publisher<nmea_msgs::msg::Sentence>(cfg_.nmea_topic, reliable_qos, pub_opts);
  }
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

void Publishers::reset_stream_state()
{
  // A new activation means a new device session (possibly post-reboot, e.g.
  // after mag-cal save_settings): pre-cycle EkfQuat/EkfVelBody must never be
  // composed with fresh EkfNav samples, and the diagnostics latches restart
  // from "nothing seen yet". geodetic_origin_ is deliberately KEPT — it
  // anchors the local /odom frame across the cycle (see header).
  last_quat_.reset();
  last_vel_body_.reset();
  last_imu_.reset();
  last_nav_.reset();
  last_nmea_gga_stamp_.reset();
  utc_anchor_.reset();
  utc_lock_announced_ = false;
  diag_composition_drops_.store(0, std::memory_order_relaxed);
  diag_last_log_stamp_ns_.store(0, std::memory_order_relaxed);
  diag_last_ekf_status_raw_.store(0, std::memory_order_relaxed);
  diag_last_ekf_solution_mode_.store(0xFF, std::memory_order_relaxed);
  diag_last_device_status_general_.store(0, std::memory_order_relaxed);
  diag_last_imu_temperature_c_.store(0.0F, std::memory_order_relaxed);
  // Flags last, released, so a concurrent snapshot never sees a stale value
  // behind a fresh flag.
  diag_has_ekf_status_.store(false, std::memory_order_release);
  diag_has_device_status_.store(false, std::memory_order_release);
  diag_has_imu_temperature_.store(false, std::memory_order_release);
}

void Publishers::activate()
{
  reset_stream_state();
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
  if (ekf_nav_sat_pub_) {
    ekf_nav_sat_pub_->on_activate();
  }
  if (nmea_gga_pub_) {
    nmea_gga_pub_->on_activate();
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
  if (ekf_nav_sat_pub_) {
    ekf_nav_sat_pub_->on_deactivate();
  }
  if (nmea_gga_pub_) {
    nmea_gga_pub_->on_deactivate();
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
  // Load the has_* flags FIRST with acquire: each pairs with the I/O thread's
  // release store (which happens after the corresponding value store), so a
  // true flag guarantees the value read below is at least the one published
  // with that flag — never the pre-init default.
  DiagSnapshot s;
  s.has_ekf_status = diag_has_ekf_status_.load(std::memory_order_acquire);
  s.has_device_status = diag_has_device_status_.load(std::memory_order_acquire);
  s.has_imu_temperature = diag_has_imu_temperature_.load(std::memory_order_acquire);
  s.last_log_stamp_ns = diag_last_log_stamp_ns_.load(std::memory_order_relaxed);
  s.last_ekf_status_raw = diag_last_ekf_status_raw_.load(std::memory_order_relaxed);
  s.last_ekf_solution_mode = diag_last_ekf_solution_mode_.load(std::memory_order_relaxed);
  s.last_device_status_general = diag_last_device_status_general_.load(std::memory_order_relaxed);
  s.last_imu_temperature_c = diag_last_imu_temperature_c_.load(std::memory_order_relaxed);
  s.composition_drops = diag_composition_drops_.load(std::memory_order_relaxed);
  return s;
}

rclcpp::Time Publishers::stamp_for(const sbg::LogView & view, const rclcpp::Time & received)
{
  if (cfg_.time_source != TimeSource::DeviceUtc) {
    return received;
  }
  const auto ts_us = view.time_stamp_us();
  if (ts_us == 0 || !utc_anchor_) {
    // No payload timestamp (GPS raw / RTCM raw) or no UTC lock yet.
    return received;
  }
  if (const auto ns = device_stamp_ns(*utc_anchor_, ts_us)) {
    return rclcpp::Time{*ns, received.get_clock_type()};
  }
  // Anchor went stale (UTC log stopped, or device rebooted mid-session):
  // wrap arithmetic is ambiguous beyond ±30 min, so degrade honestly.
  RCLCPP_WARN_THROTTLE(
    node_.get_logger(), *clock_, 10000,
    "time.source=device_utc: UTC anchor is stale (no recent valid UTC log) — stamping with "
    "receive time. Check the device's UTC_TIME output rate.");
  return received;
}

void Publishers::note_composition_drop(
  const char * what, std::uint32_t epoch_ts, std::optional<std::uint32_t> cached_ts)
{
  diag_composition_drops_.fetch_add(1, std::memory_order_relaxed);
  if (cached_ts) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *clock_, 10000,
      "not composing: cached %s has device timeStamp %u but the epoch needed %u (tolerance "
      "±%u µs). The device's log rates are not on a common epoch — align them via "
      "configure_device.output.* (ELLIPSE) or widen outputs.epoch_tolerance_us (async-clock "
      "devices, e.g. High Performance INS). (total drops: %lu)",
      what, *cached_ts, epoch_ts, cfg_.epoch_tolerance_us,
      static_cast<unsigned long>(diag_composition_drops_.load(std::memory_order_relaxed)));
  } else {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *clock_, 10000,
      "not composing: no %s log has been received for epoch %u — is that log enabled on the "
      "device? (configure_device.output.*). (total drops: %lu)",
      what, epoch_ts,
      static_cast<unsigned long>(diag_composition_drops_.load(std::memory_order_relaxed)));
  }
}

void Publishers::try_compose_odometry()
{
  if (!last_nav_ || last_nav_->composed) {
    return;
  }
  const auto epoch_ts = last_nav_->time_stamp_us;
  const bool quat_ok = last_quat_ && same_epoch(last_quat_->time_stamp_us, epoch_ts);
  const bool vel_ok = last_vel_body_ && same_epoch(last_vel_body_->time_stamp_us, epoch_ts);
  if (!quat_ok || !vel_ok) {
    // Not complete yet. A later member of this epoch may still arrive; if it
    // never does, the NEXT EkfNav retires this epoch with a counted drop.
    return;
  }
  // Consume the epoch even if the publisher is inactive — the triple was
  // complete, there is nothing more to wait for.
  last_nav_->composed = true;
  if (!odom_pub_ || !odom_pub_->is_activated() || !geodetic_origin_) {
    return;
  }

  // Optional enrichment, NOT a gate: a same-epoch IMU log supplies a real
  // angular rate. Without one, to_odometry reports the angular twist as
  // unavailable rather than as a confident zero.
  const SbgEComLogImuLegacy * imu_for_twist =
    (last_imu_ && same_epoch(last_imu_->time_stamp_us, epoch_ts)) ? &last_imu_->log : nullptr;
  auto msg = to_odometry(
    last_nav_->log, last_quat_->log, last_vel_body_->log, imu_for_twist, cfg_.imu_covariance,
    *geodetic_origin_, cfg_.convention, cfg_.odom_frame_id, cfg_.base_frame_id, last_nav_->stamp);

  // Optionally broadcast odom -> base_link from the same pose we just
  // published. Done before publish() so the transform is available to
  // subscribers as soon as they get the Odometry message.
  if (cfg_.broadcast_odom_to_base && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf{};
    tf.header.stamp = last_nav_->stamp;
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

void Publishers::on_log(const sbg::LogView & view)
{
  using Kind = sbg::LogView::Kind;
  const auto received = clock_->now();
  // Update the "any log seen" timestamp on every dispatch. Deliberately
  // receive-time even under TimeSource::DeviceUtc — it feeds the staleness
  // diagnostic, which compares against the host clock.
  diag_last_log_stamp_ns_.store(received.nanoseconds(), std::memory_order_relaxed);
  switch (view.kind()) {
    case Kind::EkfQuat:
      if (const auto * quat = view.as_ekf_quat()) {
        // Cache with its epoch; only same-epoch logs ever compose with it.
        last_quat_ = Stamped<SbgEComLogEkfQuat>{*quat, view.time_stamp_us()};
        // This may be the last member of the pending /odom triple to arrive.
        try_compose_odometry();
      }
      break;

    case Kind::ImuData:
      if (const auto * imu = view.as_imu_data()) {
        // Value first (relaxed), flag second (release): pairs with the
        // acquire flag-load in diag_snapshot().
        diag_last_imu_temperature_c_.store(imu->temperature, std::memory_order_relaxed);
        diag_has_imu_temperature_.store(true, std::memory_order_release);
        const auto trigger_ts = view.time_stamp_us();
        last_imu_ = Stamped<SbgEComLogImuLegacy>{*imu, trigger_ts};
        const auto stamp = stamp_for(view, received);
        if (imu_pub_ && imu_pub_->is_activated()) {
          // Attach orientation only from a same-epoch EkfQuat. A mismatched
          // one is dropped rather than republished: to_imu() then emits the
          // sensor_msgs/Imu "orientation unknown" sentinel, which is honest,
          // instead of a stale attitude under full covariance.
          const SbgEComLogEkfQuat * quat = nullptr;
          if (last_quat_) {
            if (same_epoch(last_quat_->time_stamp_us, trigger_ts)) {
              quat = &last_quat_->log;
            } else {
              note_composition_drop("EkfQuat", trigger_ts, last_quat_->time_stamp_us);
            }
          }
          auto msg =
            to_imu(*imu, quat, cfg_.convention, cfg_.imu_frame_id, stamp, cfg_.imu_covariance);
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
          auto msg = to_magnetic_field(
            *mag, cfg_.convention, cfg_.imu_frame_id, stamp_for(view, received), cfg_.mag_scale);
          mag_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::GnssPos:
      // Dual-receiver units can stream GPS2_POS as well; /gps/fix (and the
      // GGA upload) carry the PRIMARY receiver only, so the two antennas
      // never interleave on one topic. A secondary-receiver topic can be
      // added later if needed.
      if (view.gnss_instance() == 2) {
        break;
      }
      if (const auto * gnss = view.as_gnss_pos()) {
        const auto stamp_gnss = stamp_for(view, received);
        // Cached unconditionally (not just while /gps/fix is active): it is
        // the fix grade /ekf/fix reports while the EKF uses GPS1 position.
        last_gnss_pos_status_ = gnss->status;
        if (nav_sat_pub_ && nav_sat_pub_->is_activated()) {
          nav_sat_pub_->publish(to_navsat(*gnss, cfg_.gps_frame_id, stamp_gnss));
        }
        // Optional NMEA GGA for a third-party NTRIP client to upload to a VRS
        // caster. Rate-limited to ~1 Hz; to_nmea_gga returns nullptr for an
        // invalid fix (so we neither publish nor advance the rate-limit clock).
        if (nmea_gga_pub_ && nmea_gga_pub_->is_activated()) {
          const bool due =
            !last_nmea_gga_stamp_ || (stamp_gnss - *last_nmea_gga_stamp_).seconds() >= 0.95;
          if (due) {
            if (auto gga = to_nmea_gga(*gnss, cfg_.gps_frame_id, stamp_gnss)) {
              nmea_gga_pub_->publish(std::move(gga));
              last_nmea_gga_stamp_ = stamp_gnss;
            }
          }
        }
      }
      break;

    case Kind::Utc:
      if (const auto * utc = view.as_utc()) {
        // Refresh the device-time → UTC anchor on every VALID UTC log; this
        // is what keeps TimeSource::DeviceUtc stamps wrap-safe and current.
        // Invalid logs (pre-GNSS-sync firmware-default date) never anchor.
        if (utc_time_valid(utc->status)) {
          utc_anchor_ = UtcAnchor{
            .device_ts_us = utc->timeStamp,
            .utc_epoch_ns = utc_epoch_ns(*utc),
          };
          if (cfg_.time_source == TimeSource::DeviceUtc && !utc_lock_announced_) {
            utc_lock_announced_ = true;
            RCLCPP_INFO(
              node_.get_logger(),
              "time.source=device_utc: UTC lock acquired — stamping outputs from device time");
          }
        }
        if (time_ref_pub_ && time_ref_pub_->is_activated()) {
          // nullptr while the UTC status is INVALID (firmware-default date).
          // header.stamp stays receive-time in both time-source modes so the
          // message remains a host↔sensor clock pairing.
          if (auto msg = to_time_reference(*utc, cfg_.time_reference_frame_id, received)) {
            time_ref_pub_->publish(std::move(msg));
          }
        }
      }
      break;

    case Kind::EkfVelBody:
      if (const auto * vel = view.as_ekf_vel_body()) {
        // Cache with its epoch; only same-epoch logs ever compose with it.
        last_vel_body_ = Stamped<SbgEComLogEkfVelBody>{*vel, view.time_stamp_us()};
        // This may be the last member of the pending /odom triple to arrive.
        try_compose_odometry();
      }
      break;

    case Kind::Status:
      if (const auto * status = view.as_status()) {
        diag_last_device_status_general_.store(status->generalStatus, std::memory_order_relaxed);
        diag_has_device_status_.store(true, std::memory_order_release);
        if (sbg_status_pub_ && sbg_status_pub_->is_activated()) {
          auto msg = to_status(*status, cfg_.imu_frame_id, stamp_for(view, received));
          sbg_status_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::ShipMotion:
      if (sbg_ship_motion_pub_ && sbg_ship_motion_pub_->is_activated()) {
        if (const auto * ship = view.as_ship_motion()) {
          auto msg = to_ship_motion(*ship, cfg_.base_frame_id, stamp_for(view, received));
          sbg_ship_motion_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::Event:
      if (sbg_event_pub_ && sbg_event_pub_->is_activated()) {
        if (const auto * ev = view.as_event()) {
          auto msg = to_event(*ev, cfg_.imu_frame_id, stamp_for(view, received));
          sbg_event_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::MagCalib:
      if (sbg_mag_calib_pub_ && sbg_mag_calib_pub_->is_activated()) {
        if (const auto * cal = view.as_mag_calib()) {
          auto msg = to_mag_calib(*cal, cfg_.imu_frame_id, stamp_for(view, received));
          sbg_mag_calib_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::GpsRawData:
      // Primary receiver only, mirroring /gps/fix (sbg_msgs/GpsRaw carries no
      // receiver-id field to disambiguate interleaved GPS2 observables).
      if (view.gnss_instance() == 2) {
        break;
      }
      if (sbg_gps_raw_pub_ && sbg_gps_raw_pub_->is_activated()) {
        if (const auto * raw = view.as_gps_raw()) {
          // SbgEComLogRawData carries no timestamp; stamp_for falls back to
          // receive time for it in both modes.
          auto msg = to_gps_raw(*raw, cfg_.gps_frame_id, stamp_for(view, received));
          sbg_gps_raw_pub_->publish(std::move(msg));
        }
      }
      break;

    case Kind::AirData:
      if (sbg_air_data_status_pub_ && sbg_air_data_status_pub_->is_activated()) {
        if (const auto * air = view.as_air_data()) {
          auto msg = to_air_data_status(*air, cfg_.imu_frame_id, stamp_for(view, received));
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
        diag_has_ekf_status_.store(true, std::memory_order_release);
        const auto stamp_ekf = stamp_for(view, received);

        // Publish decoded EKF status alongside Odometry composition.
        if (sbg_ekf_status_pub_ && sbg_ekf_status_pub_->is_activated()) {
          auto status_msg = to_ekf_status(*nav, cfg_.imu_frame_id, stamp_ekf);
          sbg_ekf_status_pub_->publish(std::move(status_msg));
        }

        // Optional: fused INS geodetic position as a second NavSatFix. Global
        // lat/lon (unlike /odom's local Cartesian) and smoother than raw GNSS.
        // Published regardless of validity — its NavSatStatus self-describes
        // (STATUS_NO_FIX while the position-valid bit is clear; the latest
        // GPS1 fix grade while the EKF uses GPS1 position; else STATUS_FIX).
        if (ekf_nav_sat_pub_ && ekf_nav_sat_pub_->is_activated()) {
          auto fix = to_ekf_navsat(*nav, cfg_.imu_frame_id, stamp_ekf, last_gnss_pos_status_);
          ekf_nav_sat_pub_->publish(std::move(fix));
        }

        // Everything below trusts the position estimate, so it is gated on
        // the EKF's own POSITION_VALID bit: on cold start the filter streams
        // EKF_NAV with the firmware-default position while UNINITIALIZED —
        // latching the sticky origin (or composing odometry) from that would
        // permanently corrupt the local frame.
        if (!ekf_position_valid(nav->status)) {
          break;
        }

        // Set sticky origin on the first VALID fix. EkfNav.position[0..1] are
        // lat/lon in degrees; promote altitude to ellipsoid height via
        // undulation so it lines up with /gps/fix.
        if (!geodetic_origin_) {
          const double alt0 = nav->position[2] + static_cast<double>(nav->undulation);
          geodetic_origin_ = make_geodetic_origin(nav->position[0], nav->position[1], alt0);
        }

        // Retire the previous epoch LOUDLY if it never completed: only now —
        // when its successor arrives — do we know no later same-epoch
        // EkfQuat/EkfVelBody is coming. This is also where a log that is
        // disabled on the device (never cached at all) becomes visible.
        const auto trigger_ts = view.time_stamp_us();
        if (last_nav_ && !last_nav_->composed) {
          const auto old_ts = last_nav_->time_stamp_us;
          if (!(last_quat_ && same_epoch(last_quat_->time_stamp_us, old_ts))) {
            note_composition_drop(
              "EkfQuat", old_ts,
              last_quat_ ? std::optional{last_quat_->time_stamp_us} : std::nullopt);
          }
          if (!(last_vel_body_ && same_epoch(last_vel_body_->time_stamp_us, old_ts))) {
            note_composition_drop(
              "EkfVelBody", old_ts,
              last_vel_body_ ? std::optional{last_vel_body_->time_stamp_us} : std::nullopt);
          }
        }

        // Cache this epoch's nav (with its resolved header stamp) and try to
        // compose. Composition is SET-COMPLETION — it also runs on EkfQuat /
        // EkfVelBody arrival — so /odom does not depend on the firmware's
        // within-loop log emission order (see try_compose_odometry()).
        last_nav_ = PendingNav{*nav, trigger_ts, stamp_ekf, false};
        try_compose_odometry();
      }
      break;

    default:
      // Other log kinds dropped silently; phase 3c+ wires them up.
      break;
  }
}

}  // namespace sbg_driver
