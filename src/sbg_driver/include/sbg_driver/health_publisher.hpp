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

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>  // NOLINT(build/include_order) -- C++20/23 header
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sbg/log_view.hpp>

#include "sbg_driver/health.hpp"

namespace sbg_driver
{

// Components → the standard diagnostic_msgs/DiagnosticArray. KeyValue is
// string-only, so every value goes through to_diagnostic_string().
[[nodiscard]] diagnostic_msgs::msg::DiagnosticArray to_diagnostic_array(
  const std::vector<HealthComponent> & components, const builtin_interfaces::msg::Time & stamp);

// Hand one device log to the reporter. Called on the device I/O thread for
// every log, before it is published.
void observe_log(HealthReporter & reporter, const sbg::LogView & view);

// Publishes HealthReporter snapshots on /diagnostics from its own thread.
//
// Not a node timer on purpose: this node's executor thread is where lifecycle
// transitions and the device-command services block (device identification
// and Configurator round-trips take seconds), and a heartbeat that stops
// exactly then would read as a dead reporter — and could never show
// "transitioning". The publisher is a plain rclcpp::Publisher, not a
// LifecyclePublisher, so it also keeps reporting while the node is inactive.
class HealthPublisher
{
public:
  using Counters = std::function<HealthReporter::StreamCounters()>;

  // Starts publishing at reporter.publish_interval_s(). `counters` is called
  // on the publisher thread.
  HealthPublisher(
    rclcpp_lifecycle::LifecycleNode & node, HealthReporter & reporter, Counters counters);

  // Stops the thread; it publishes one last snapshot on its way out so the
  // final operational state is on the wire.
  ~HealthPublisher();

  HealthPublisher(const HealthPublisher &) = delete;
  HealthPublisher & operator=(const HealthPublisher &) = delete;

  // Publish now instead of at the next heartbeat (state changes an operator
  // should not wait an interval to see). Safe from any thread.
  void publish_soon();

private:
  void run(const std::stop_token & stop);
  void publish_once();

  HealthReporter & reporter_;
  Counters counters_;
  rclcpp::Logger logger_;
  // Header stamps are wall time whatever the node clock is (sim time, a
  // replayed bag); the throttle clock only has to be monotonic.
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;

  std::mutex mutex_;
  std::condition_variable_any wake_;
  bool publish_requested_ = false;
  std::jthread thread_;  // last: joins before the members it uses go away
};

}  // namespace sbg_driver
