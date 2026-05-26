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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sbg/device.hpp>
#include <sbg/transport.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
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

  // Forward-declared param listener struct (definition from
  // generate_parameter_library is included only in driver_node.cpp).
  class ParamHolder;
  std::unique_ptr<ParamHolder> params_;

  std::unique_ptr<Publishers> publishers_;
  std::optional<sbg::Device> device_;
  std::jthread io_thread_;

  // ---- /diagnostics, /rtcm, /sbg/*_mag_calibration ----
  // Diagnostic state lives on Publishers as lock-free atomics; the updater
  // tasks read them directly from the executor thread.
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_mag_cal_srv_;
};

}  // namespace sbg_driver
