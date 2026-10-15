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

#include "sbg_driver/health_publisher.hpp"

#include <chrono>
#include <exception>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

namespace sbg_driver
{

namespace
{

// The generated constants, not bare integers: `level` is a ROS `byte`.
constexpr auto to_ros_level(HealthLevel level) noexcept
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  switch (level) {
    case HealthLevel::Ok:
      return DiagnosticStatus::OK;
    case HealthLevel::Warn:
      return DiagnosticStatus::WARN;
    case HealthLevel::Error:
      return DiagnosticStatus::ERROR;
    case HealthLevel::Stale:
      break;
  }
  return DiagnosticStatus::STALE;
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray to_diagnostic_array(
  const std::vector<HealthComponent> & components, const builtin_interfaces::msg::Time & stamp)
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = stamp;
  msg.status.reserve(components.size());
  for (const auto & component : components) {
    auto & status = msg.status.emplace_back();
    status.level = to_ros_level(component.level);
    status.name = component.name;
    status.message = component.message;
    status.hardware_id = component.hardware_id;
    status.values.reserve(component.values.size());
    for (const auto & [key, value] : component.values) {
      auto & kv = status.values.emplace_back();
      kv.key = key;
      kv.value = to_diagnostic_string(value);
    }
  }
  return msg;
}

void observe_log(HealthReporter & reporter, const sbg::LogView & view)
{
  using Kind = sbg::LogView::Kind;
  reporter.note_log();
  switch (view.kind()) {
    case Kind::Status:
      if (const auto * status = view.as_status()) {
        reporter.observe_status(
          HealthReporter::DeviceStatus{
            .general = status->generalStatus,
            .com = status->comStatus,
            .com2 = status->comStatus2,
            .aiding = status->aidingStatus,
            .uptime_s = status->uptime,
            .cpu_usage_pct = status->cpuUsage,
          });
      }
      break;

    // Both IMU logs carry the same sensor's temperature; whichever the device
    // streams feeds the measurement (unlike /imu/data, nothing here needs the
    // two kept apart).
    case Kind::ImuData:
      if (const auto * imu = view.as_imu_data()) {
        reporter.observe_imu_status(imu->status);
        reporter.observe_imu_temperature(static_cast<double>(imu->temperature));
      }
      break;
    case Kind::ImuShort:
      if (const auto * imu = view.as_imu_short()) {
        reporter.observe_imu_status(imu->status);
        reporter.observe_imu_temperature(
          static_cast<double>(sbgEComLogImuShortGetTemperature(imu)));
      }
      break;

    case Kind::EkfNav:
      if (const auto * nav = view.as_ekf_nav()) {
        reporter.observe_ekf(nav->status);
      }
      break;

    // Primary receiver only, like /gps/fix and /gps/heading.
    case Kind::GnssPos:
      if (const auto * pos = view.as_gnss_pos(); pos != nullptr && view.gnss_instance() != 2) {
        reporter.observe_gnss(
          HealthReporter::GnssPosition{
            .solution_status = static_cast<std::uint8_t>(sbgEComLogGnssPosGetStatus(pos)),
            .fix_type = static_cast<std::uint8_t>(sbgEComLogGnssPosGetType(pos)),
            .sv_used = pos->numSvUsed,
            .differential_age_cs = pos->differentialAge,
            .base_station_id = pos->baseStationId,
            .latitude_accuracy_m = static_cast<double>(pos->latitudeAccuracy),
            .longitude_accuracy_m = static_cast<double>(pos->longitudeAccuracy),
            .altitude_accuracy_m = static_cast<double>(pos->altitudeAccuracy),
          });
      }
      break;
    case Kind::GnssHdt:
      if (const auto * hdt = view.as_gnss_hdt(); hdt != nullptr && view.gnss_instance() != 2) {
        reporter.observe_gnss_heading(
          HealthReporter::GnssHeading{
            .solution_status = static_cast<std::uint8_t>(sbgEComLogGnssHdtGetStatus(hdt)),
            .heading_valid = sbgEComLogGnssHdtHeadingIsValid(hdt),
            .heading_deg = static_cast<double>(hdt->heading),
            .accuracy_deg = static_cast<double>(hdt->headingAccuracy),
            .baseline_valid = sbgEComLogGnssHdtBaseLineIsValid(hdt),
            .baseline_m = static_cast<double>(hdt->baseline),
            .sv_used = hdt->numSvUsed,
          });
      }
      break;

    default:
      break;
  }
}

HealthPublisher::HealthPublisher(
  rclcpp_lifecycle::LifecycleNode & node, HealthReporter & reporter, Counters counters)
: reporter_(reporter),
  counters_(std::move(counters)),
  logger_(node.get_logger()),
  // Absolute, as every /diagnostics producer publishes it. Reliable +
  // volatile: a late joiner waits one heartbeat, it is never handed a
  // stored snapshot as if it were current.
  publisher_(
    rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      node, "/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable().durability_volatile())),
  thread_([this](const std::stop_token & stop) { run(stop); })
{
}

HealthPublisher::~HealthPublisher()
{
  thread_.request_stop();
  wake_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void HealthPublisher::publish_soon()
{
  {
    const std::lock_guard lock{mutex_};
    publish_requested_ = true;
  }
  wake_.notify_all();
}

void HealthPublisher::run(const std::stop_token & stop)
{
  using Steady = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<Steady::duration>(
    std::chrono::duration<double>(reporter_.publish_interval_s()));
  auto next = Steady::now();
  while (!stop.stop_requested()) {
    publish_once();
    // Fixed cadence; after a stall (suspended host) resume from now rather
    // than bursting the missed heartbeats.
    next += period;
    if (const auto now = Steady::now(); next <= now) {
      next = now + period;
    }
    std::unique_lock lock{mutex_};
    if (wake_.wait_until(lock, stop, next, [this] { return publish_requested_; })) {
      publish_requested_ = false;
      next = Steady::now();
    }
  }
  publish_once();
}

void HealthPublisher::publish_once()
{
  // Health reporting must never take the driver down with it.
  try {
    const auto counters = counters_ ? counters_() : HealthReporter::StreamCounters{};
    auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>(
      to_diagnostic_array(reporter_.snapshot(counters), system_clock_.now()));
    publisher_->publish(std::move(msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 60000, "health publication failed: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 60000, "health publication failed");
  }
}

}  // namespace sbg_driver
