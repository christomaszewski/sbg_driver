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

// Standalone executable for sbg_driver. Mirrors the composable component
// load path but instantiates the node directly under a single-threaded
// executor. The I/O thread inside Device::run() does not go through the
// executor, so an STE is sufficient and lower-overhead.

#include <chrono>
#include <csignal>
#include <memory>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sbg_driver/driver_node.hpp"

namespace
{

// Set by the signal handler, polled by the spin loop. Writing a sig_atomic_t
// is all a handler may safely do.
volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void request_stop(int /*signum*/) { g_stop_requested = 1; }

}  // namespace

int main(int argc, char ** argv)
{
  // SIGINT/SIGTERM are taken here rather than by rclcpp. rclcpp's handler
  // shuts the context down from its own thread, and a lifecycle node can do
  // nothing after that: the shutdown transition cannot run on an invalid
  // context and nothing published reaches the wire, so the node would be
  // destroyed while still "active" (rclcpp_lifecycle warns about exactly
  // that) and its last health report would never be sent. Instead the signal
  // only ends the spin loop; the transition then runs on this thread — the
  // one every node callback runs on, so it cannot race one — while the context
  // is still valid.
  //
  // The handler is idempotent on purpose: Ctrl-C under `ros2 launch` delivers
  // SIGINT twice (once from the terminal, once from launch), and the second
  // must not cut the shutdown short. A shutdown that hangs is ended by
  // whoever asked for it (launch escalates to SIGKILL, Docker likewise).
  rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);

  auto node = std::make_shared<sbg_driver::SbgDriverNode>(rclcpp::NodeOptions{});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  // Bounded waits instead of spin(): nothing async-signal-safe can wake the
  // executor, so this is also the latency of reacting to a signal.
  while (rclcpp::ok() && g_stop_requested == 0) {
    executor.spin_once(std::chrono::milliseconds{100});
  }

  if (
    rclcpp::ok() &&
    node->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    RCLCPP_INFO(node->get_logger(), "stop requested — running the lifecycle shutdown");
    node->shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
