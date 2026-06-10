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
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <sbg/device.hpp>
#include <sbg/transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

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
  [[nodiscard]] CallbackReturn apply_device_configuration();

  // Forward-declared param listener struct (definition from
  // generate_parameter_library is included only in driver_node.cpp).
  class ParamHolder;
  std::unique_ptr<ParamHolder> params_;

  std::unique_ptr<Publishers> publishers_;
  std::optional<sbg::Device> device_;
  std::jthread io_thread_;

  // ---- /diagnostics, /rtcm, /sbg/* services ----
  // Diagnostic state lives on Publishers as lock-free atomics; the updater
  // tasks read them directly from the executor thread.
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_settings_srv_;

  // True after a persistence service released the device (save_settings
  // reboots the unit): the node is still ACTIVE but no longer streaming, and
  // the Device diagnostic reports "reactivation required" instead of decaying
  // into a generic staleness error. Cleared on the next activate.
  std::atomic<bool> device_released_{false};
};

}  // namespace sbg_driver
