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

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sbg/log_view.hpp>
#include <sbg_msgs/msg/air_data_status.hpp>
#include <sbg_msgs/msg/ekf_status.hpp>
#include <sbg_msgs/msg/event.hpp>
#include <sbg_msgs/msg/gps_raw.hpp>
#include <sbg_msgs/msg/mag_calib.hpp>
#include <sbg_msgs/msg/ship_motion.hpp>
#include <sbg_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "sbg_driver/conversions.hpp"

namespace sbg_driver
{

// Wrap-safe distance between two device timestamps (µs since power-up,
// uint32 — wraps every ~71.6 min): the shorter way around the circle.
[[nodiscard]] constexpr std::uint32_t device_timestamp_distance(
  std::uint32_t a, std::uint32_t b) noexcept
{
  const std::uint32_t forward = a - b;
  const std::uint32_t backward = 0U - forward;
  return forward < backward ? forward : backward;
}

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
    std::string raw_orientation_topic = "raw/orientation";
    std::string imu_temperature_topic = "imu/temperature";
    std::string mag_topic = "imu/mag";
    std::string nav_sat_fix_topic = "gps/fix";
    std::string gnss_heading_topic = "gps/heading";
    std::string time_reference_topic = "time_reference";
    std::string odom_topic = "odom";
    std::string ekf_nav_sat_fix_topic = "ekf/fix";
    std::string nmea_topic = "nmea";

    // SBG-specific custom-message topics
    std::string sbg_status_topic = "sbg/status";
    std::string sbg_ekf_status_topic = "sbg/ekf_status";
    std::string sbg_air_data_status_topic = "sbg/air_data_status";
    std::string sbg_event_topic = "sbg/event";
    std::string sbg_gps_raw_topic = "sbg/gps_raw";
    std::string sbg_mag_calib_topic = "sbg/mag_calib";
    std::string sbg_ship_motion_topic = "sbg/ship_motion";
    std::string sbg_ship_motion_hp_topic = "sbg/ship_motion_hp";

    // Frame IDs
    std::string imu_frame_id = "imu_link";
    std::string gps_frame_id = "gps_link";
    std::string time_reference_frame_id = "";  // empty = global
    std::string odom_frame_id = "odom";
    std::string base_frame_id = "base_link";

    // TF policy
    bool broadcast_odom_to_base = false;

    // Optional outputs. Publish the fused INS geodetic position as a second
    // NavSatFix on ekf_nav_sat_fix_topic. Off by default - /gps/fix already
    // carries the raw GNSS fix and /odom the fused local solution.
    bool publish_ekf_nav_sat_fix = false;

    // Added (clockwise positive, degrees) to the GNSS dual-antenna heading
    // before it is published on gnss_heading_topic: the yaw from the
    // primary→secondary antenna baseline to the gps frame's X axis. See
    // to_gnss_heading().
    double gnss_heading_offset_deg = 0.0;

    // Publish NMEA GGA on nmea_topic (~1 Hz) so a third-party NTRIP client can
    // upload position to a VRS / network-RTK caster. Off by default.
    bool publish_nmea_gga = false;

    bool publish_raw_orientation = false;
    std::uint32_t raw_orientation_imu_tolerance_us = 10'000;
    double raw_orientation_heading_offset_deg = 0.0;

    FrameConvention convention = FrameConvention::Ned;

    // header.stamp source. ReceiveTime (default) stamps with the host clock
    // at dispatch; DeviceUtc maps each log's device timeStamp through the
    // latest valid UTC log, removing transport/scheduling jitter — the right
    // choice when the host disciplines its clock to the INS's NTP/PTP server.
    // Falls back to receive time until UTC lock and for logs without a
    // payload timestamp. The TimeReference message keeps a receive-time
    // header in both modes so it remains a host↔sensor clock pairing.
    TimeSource time_source = TimeSource::ReceiveTime;
    double utc_timeout_s = 5.0;
    // False for accelerated replay: elapsed wall time cannot bound device advancement.
    bool device_clock_real_time = true;

    // Accel/gyro variance for /imu/data covariance (resolved at configure
    // time from sensor_model + noise-stddev params). Default unknown.
    ImuCovariance imu_covariance{};

    // Scale applied to SBG magnetometer arbitrary units before publishing
    // (sensor_msgs/MagneticField is specified in Tesla; SBG logs use arbitrary
    // units). Zero disables the topic; publishing requires a finite positive
    // Tesla-per-unit scale calibrated for the sensor/location.
    double mag_scale = 0.0;

    // Which IMU log feeds /imu/data, /imu/temperature and the /odom angular
    // twist (see ImuSource). Auto = IMU_SHORT as soon as the device streams
    // it, IMU_DATA until then; the other log is ignored so the topic never
    // carries both.
    ImuSource imu_source = ImuSource::Auto;

    // Max |device timeStamp| difference (µs) two logs may have and still be
    // composed (/imu/data orientation attach, /odom triple). 0 = exact-epoch
    // matching, the ELLIPSE contract (same-tick logs share a timestamp).
    // Nonzero admits independently timed members. Odometry chooses the nearest
    // received member within this window after stream progress or a bounded
    // wait, rather than consuming an older cache immediately. Resolved
    // at activation from the identified device family — see
    // set_epoch_tolerance_us() and SbgDriverNode::on_activate().
    std::uint32_t epoch_tolerance_us = 0;
  };

  Publishers(rclcpp_lifecycle::LifecycleNode & node, Config config);

  // Activate / deactivate matches LifecycleNode state transitions. activate()
  // also resets per-stream state (cached EkfQuat/EkfVelBody, GGA rate-limit,
  // diagnostics latches) so a deactivate→activate cycle — e.g. the prescribed
  // post-mag-cal reconnect — never composes new logs with pre-cycle ones. The
  // geodetic origin deliberately survives the cycle: it anchors the local
  // /odom frame, and resetting it would step every downstream pose consumer.
  void activate();
  void deactivate();

  // Override Config::epoch_tolerance_us with the value resolved at
  // activation (explicit param, or auto keyed off the identified device
  // family). Must be called before the I/O thread starts — it mutates
  // config that on_log() reads unsynchronized.
  void set_epoch_tolerance_us(std::uint32_t tolerance_us) noexcept
  {
    cfg_.epoch_tolerance_us = tolerance_us;
  }

  // Dispatch one log view. Called from the I/O thread.
  // Rejects gracefully if publishers haven't been activated yet.
  void on_log(const sbg::LogView & view);

  // Call periodically from the same I/O thread, including when input is idle.
  // Retires incomplete epochs and publishes complete epochs after a bounded IMU wait.
  void flush_pending();

  // ---- Lock-free diagnostics snapshot ----
  // Composition counters, written by the I/O thread inside on_log() and read
  // from the health publisher's thread (they ride on the `stream` health
  // component). Device observations themselves go to HealthReporter.
  struct DiagSnapshot
  {
    // /odom epochs retired without composing: an EkfNav arrived whose
    // EkfQuat / EkfVelBody never came within the epoch tolerance. A steadily
    // climbing value means the device's EKF log rates do not share an epoch —
    // see configure_device.output.* (ELLIPSE) or outputs.epoch_tolerance_us
    // (async-clock devices).
    std::uint64_t composition_drops = 0;
    // /imu/data messages published with the orientation-unknown sentinel
    // although an EkfQuat stream exists: no quaternion fell within the epoch
    // tolerance of the IMU sample. Steady growth means the tolerance is below
    // the EKF period (an IMU faster than the EKF is expected and fine as long
    // as the tolerance covers one EKF period).
    std::uint64_t imu_orientation_misses = 0;
    // True once IMU_SHORT has been selected as the /imu/data source.
    bool imu_short_in_use = false;
  };
  [[nodiscard]] DiagSnapshot diag_snapshot() const noexcept;

private:
  // Clear cached stream state + diagnostics latches (NOT the geodetic
  // origin — see activate() docs). Called on activation or a confirmed restart.
  void reset_stream_state();

  // Resolve the header.stamp for one log per cfg_.time_source. `received` is
  // the host-clock dispatch time (always the fallback). Only meaningful on
  // the I/O thread, like the rest of the stream state.
  [[nodiscard]] rclcpp::Time stamp_for(const sbg::LogView & view, const rclcpp::Time & received);

  rclcpp_lifecycle::LifecycleNode & node_;
  Config cfg_;
  rclcpp::Clock::SharedPtr clock_;

  // Latest valid (device timeStamp → UTC) pairing, refreshed on every valid
  // UTC log; used by stamp_for() under TimeSource::DeviceUtc. Reset on
  // activate — a new session (possibly post-reboot) must re-anchor.
  std::optional<UtcAnchor> utc_anchor_;
  bool utc_lock_announced_ = false;
  std::chrono::steady_clock::time_point utc_anchor_received_{};
  std::optional<std::uint32_t> device_high_water_;
  std::optional<std::uint32_t> restart_candidate_;
  std::chrono::steady_clock::time_point last_device_received_{};
  bool current_clock_stream_seen_ = false;
  // Short reordering is allowed. A large backwards jump needs two distinct
  // advancing timestamps before discarding the previous device session.
  bool observe_device_clock(std::uint32_t ts);

  // Increment the composition-drop counter and warn (throttled) that a log
  // needed for composition either carried a different device timeStamp than
  // the epoch `epoch_ts` (cached_ts has a value) or was never received at
  // all (nullopt — the log is probably disabled on the device).
  void note_composition_drop(
    const char * what, std::uint32_t epoch_ts, std::optional<std::uint32_t> cached_ts);

  // Histories retain contributors while pending NAV epochs wait for closer
  // measurements. Odometry preserves NAV order and resolves each constituent
  // within the configured tolerance after stream progress or a 20 ms deadline.
  // Histories are bounded to 2048 samples / two steady seconds.
  //
  // /imu/data also waits for the nearest quaternion when that stream exists.
  // It publishes immediately with unknown orientation before any quaternion
  // is observed, or with an exact/bracketed match. A 20 ms deadline bounds
  // latency when the stream stops or remains behind the IMU.
  template <typename LogT>
  struct Stamped
  {
    LogT log{};
    std::uint32_t time_stamp_us = 0;
    std::chrono::steady_clock::time_point received = std::chrono::steady_clock::now();
  };
  std::optional<Stamped<SbgEComLogEkfQuat>> last_quat_;
  std::deque<Stamped<SbgEComLogEkfQuat>> quat_history_;
  std::deque<Stamped<SbgEComLogEkfVelBody>> vel_history_;

  // Shared IMU history for odometry and raw heading. Both select the nearest
  // eligible sample after stream progress or a bounded wait.
  struct RawImuSample
  {
    SbgEComLogImuLegacy log{};
    std::chrono::steady_clock::time_point received;
  };
  std::deque<RawImuSample> raw_imu_history_;
  std::optional<std::uint32_t> last_raw_heading_ts_;
  void prune_raw_imu_history();

  struct PendingHeading
  {
    SbgEComLogGnssHdt log{};
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point received;
  };
  std::deque<PendingHeading> held_headings_;
  bool imu_seen_ = false;
  void try_publish_raw_orientations(bool force_oldest = false);

  // IMU sample waiting for its quaternion (hold-and-pair). Carries the header
  // stamp resolved at its arrival.
  struct PendingImu
  {
    SbgEComLogImuLegacy log{};
    std::uint32_t time_stamp_us = 0;
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point received;
  };
  std::deque<PendingImu> held_imus_;

  // Auto source selection latch: set on the first IMU_SHORT log of a session,
  // after which IMU_DATA logs are ignored (ImuSource::Auto only).
  bool imu_short_seen_ = false;

  // One IMU sample (either log, already in the legacy layout) through the
  // temperature publish, the odom-twist cache and the hold-and-pair step.
  void handle_imu(
    const SbgEComLogImuLegacy & imu, const sbg::LogView & view, const rclcpp::Time & received);
  void publish_imu(
    const SbgEComLogImuLegacy & imu, const SbgEComLogEkfQuat * quat, const rclcpp::Time & stamp);
  // Resolve held samples when the quaternion stream advances or the deadline expires.
  void try_publish_imus(bool force_oldest = false);
  void note_imu_orientation_miss(std::uint32_t imu_ts, std::uint32_t quat_ts);

  // Bounded, ordered NAV epochs. Separate histories retain asynchronous
  // quaternion, body velocity and IMU measurements. Each is matched to NAV
  // within the configured tolerance; waiting allows closer samples to arrive.
  struct OdomEpoch
  {
    std::uint32_t time_stamp_us = 0;
    std::optional<SbgEComLogEkfNav> nav;
    rclcpp::Time stamp;
    std::chrono::steady_clock::time_point received;
  };
  std::deque<OdomEpoch> odom_epochs_;
  std::optional<std::uint32_t> last_odom_ts_;
  static constexpr auto k_odom_wait = std::chrono::milliseconds{20};
  static constexpr std::size_t k_max_odom_epochs = 64;
  OdomEpoch * odom_epoch(std::uint32_t ts);
  void try_compose_odometry(bool force_oldest = false);
  void publish_odometry(
    const OdomEpoch & epoch, const SbgEComLogEkfQuat & quat, const SbgEComLogEkfVelBody & vel,
    const SbgEComLogImuLegacy * imu);

  // Sticky origin set on the first POSITION_VALID EkfNav - locks the local
  // frame so downstream odom poses are stable. Survives deactivate→activate.
  std::optional<GeodeticOrigin> geodetic_origin_;

  // Last NMEA GGA emission, for ~1 Hz rate-limiting of the optional publisher.
  std::optional<rclcpp::Time> last_nmea_gga_stamp_;

  // Status word of the most recent PRIMARY-receiver GnssPos log. Lends its fix
  // grade (RTK → GBAS, SBAS, single → FIX) to /ekf/fix while the EKF reports
  // GPS1 position aiding; stays empty if the device never streams GPS1_POS, in
  // which case /ekf/fix falls back to the coarse STATUS_FIX.
  std::optional<std::uint32_t> last_gnss_pos_status_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_pub_;
  // GNSS dual-antenna heading; only ever publishes on units that stream GPS1_HDT.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr gnss_heading_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr raw_orientation_pub_;
  // Optional: fused INS geodetic position (only created when enabled in Config).
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr ekf_nav_sat_pub_;
  // Optional: NMEA GGA for NTRIP clients (only created when enabled in Config).
  rclcpp_lifecycle::LifecyclePublisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_gga_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::Status>::SharedPtr sbg_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::EkfStatus>::SharedPtr sbg_ekf_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::AirDataStatus>::SharedPtr
    sbg_air_data_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::Event>::SharedPtr sbg_event_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::GpsRaw>::SharedPtr sbg_gps_raw_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::MagCalib>::SharedPtr sbg_mag_calib_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::ShipMotion>::SharedPtr sbg_ship_motion_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbg_msgs::msg::ShipMotion>::SharedPtr
    sbg_ship_motion_hp_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Lock-free diagnostics fields (see DiagSnapshot).
  std::atomic<std::uint64_t> diag_composition_drops_{0};
  std::atomic<std::uint64_t> diag_imu_orientation_misses_{0};
  std::atomic<bool> diag_imu_short_in_use_{false};
};

}  // namespace sbg_driver
