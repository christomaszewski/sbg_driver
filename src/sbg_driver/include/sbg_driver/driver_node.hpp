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
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <sbg/device.hpp>
#include <sbg/transport.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "sbg_driver/health.hpp"
#include "sbg_driver/health_publisher.hpp"
#include "sbg_driver/publishers.hpp"

namespace sbg_driver
{

class SbgDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SbgDriverNode(const rclcpp::NodeOptions & options);
  ~SbgDriverNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  [[nodiscard]] sbg::TransportConfig build_transport_config() const;

  // Apply the configure_device.* params to the device (RAM) during on_activate,
  // before the I/O thread starts. No-op unless configure_device.enable is true.
  // Returns FAILURE (and the caller aborts activation) if a requested command
  // fails. Does not persist to NVRAM — use the save-settings service for that.
  // `failure` receives the operator-facing reason on FAILURE.
  [[nodiscard]] CallbackReturn apply_device_configuration(std::string & failure);

  // on_activate's body: open, identify, provision, start streaming. On
  // FAILURE the device is released again and `failure` says why — it becomes
  // the service health component's message.
  [[nodiscard]] CallbackReturn activate_device(std::string & failure);

  // Join the I/O thread, deactivate the publishers, close the device.
  void stop_streaming();
  // Drop everything on_configure created.
  void release_resources();
  // Bring the next /diagnostics publication forward (no-op when unconfigured).
  void publish_health_soon();

  // Forward-declared param listener struct (definition from
  // generate_parameter_library is included only in driver_node.cpp).
  class ParamHolder;
  std::unique_ptr<ParamHolder> params_;

  // Fleet health (doc/health.md). The reporter lives as long as the process
  // — its publisher id and sequence must not restart on a reconfigure — and
  // is declared before everything that feeds it.
  HealthReporter health_;

  std::unique_ptr<Publishers> publishers_;
  std::optional<sbg::Device> device_;
  std::jthread io_thread_;

  // ---- /rtcm, /sbg/* services ----
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_settings_srv_;

  // /diagnostics heartbeat, alive from configure to cleanup. Declared last so
  // its thread stops before health_ and publishers_, which it reads.
  std::unique_ptr<HealthPublisher> health_publisher_;
};

}  // namespace sbg_driver
