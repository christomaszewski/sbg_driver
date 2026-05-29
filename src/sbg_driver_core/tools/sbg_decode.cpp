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

// sbg_decode — minimal offline/live decoder for SBG logs, built on the
// ROS-free sbg_driver_core. Opens a .bin replay file (or a serial port),
// prints each parsed log, and exits at EOF. Useful for replay analysis and
// for generating golden test vectors without any ROS dependency.
//
// Usage:
//   sbg_decode <file.bin> [--json]
//   sbg_decode --serial <port> [--baud N] [--json]

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include "sbg/device.hpp"
#include "sbg/log_view.hpp"

namespace
{

std::atomic<bool> g_run{true};

void handle_sigint(int /*signum*/) { g_run.store(false, std::memory_order_relaxed); }

std::string_view kind_name(sbg::LogView::Kind k) noexcept
{
  using K = sbg::LogView::Kind;
  switch (k) {
    case K::ImuShort:
      return "ImuShort";
    case K::ImuData:
      return "ImuData";
    case K::EkfEuler:
      return "EkfEuler";
    case K::EkfQuat:
      return "EkfQuat";
    case K::EkfNav:
      return "EkfNav";
    case K::EkfVelBody:
      return "EkfVelBody";
    case K::Mag:
      return "Mag";
    case K::MagCalib:
      return "MagCalib";
    case K::GnssPos:
      return "GnssPos";
    case K::GnssVel:
      return "GnssVel";
    case K::GnssHdt:
      return "GnssHdt";
    case K::GpsRawData:
      return "GpsRawData";
    case K::AirData:
      return "AirData";
    case K::Utc:
      return "Utc";
    case K::Status:
      return "Status";
    case K::ShipMotion:
      return "ShipMotion";
    case K::Event:
      return "Event";
    case K::Unknown:
      return "Unknown";
  }
  return "Unknown";
}

void print_text(const sbg::LogView & v)
{
  const auto name = kind_name(v.kind());
  if (const auto * imu = v.as_imu_data()) {
    std::println(
      "[{:>10} us] {:<11} accel=({:.3f}, {:.3f}, {:.3f}) gyro=({:.4f}, {:.4f}, {:.4f}) "
      "temp={:.1f}C",
      v.time_stamp_us(), name, imu->accelerometers[0], imu->accelerometers[1],
      imu->accelerometers[2], imu->gyroscopes[0], imu->gyroscopes[1], imu->gyroscopes[2],
      imu->temperature);
  } else if (const auto * nav = v.as_ekf_nav()) {
    std::println(
      "[{:>10} us] {:<11} lat={:.7f} lon={:.7f} alt={:.2f} status=0x{:08x}", v.time_stamp_us(),
      name, nav->position[0], nav->position[1], nav->position[2], nav->status);
  } else if (const auto * gnss = v.as_gnss_pos()) {
    std::println(
      "[{:>10} us] {:<11} lat={:.7f} lon={:.7f} alt={:.2f} status=0x{:08x}", v.time_stamp_us(),
      name, gnss->latitude, gnss->longitude, gnss->altitude, gnss->status);
  } else if (const auto * quat = v.as_ekf_quat()) {
    std::println(
      "[{:>10} us] {:<11} q=({:.4f}, {:.4f}, {:.4f}, {:.4f})", v.time_stamp_us(), name,
      quat->quaternion[0], quat->quaternion[1], quat->quaternion[2], quat->quaternion[3]);
  } else {
    std::println("[{:>10} us] {:<11}", v.time_stamp_us(), name);
  }
}

void print_json(const sbg::LogView & v)
{
  const auto name = kind_name(v.kind());
  if (const auto * imu = v.as_imu_data()) {
    std::println(
      R"({{"t_us":{},"kind":"{}","accel":[{:.6f},{:.6f},{:.6f}],"gyro":[{:.6f},{:.6f},{:.6f}],"temp":{:.3f}}})",
      v.time_stamp_us(), name, imu->accelerometers[0], imu->accelerometers[1],
      imu->accelerometers[2], imu->gyroscopes[0], imu->gyroscopes[1], imu->gyroscopes[2],
      imu->temperature);
  } else if (const auto * nav = v.as_ekf_nav()) {
    std::println(
      R"({{"t_us":{},"kind":"{}","lat":{:.9f},"lon":{:.9f},"alt":{:.3f},"status":{}}})",
      v.time_stamp_us(), name, nav->position[0], nav->position[1], nav->position[2], nav->status);
  } else if (const auto * gnss = v.as_gnss_pos()) {
    std::println(
      R"({{"t_us":{},"kind":"{}","lat":{:.9f},"lon":{:.9f},"alt":{:.3f},"status":{}}})",
      v.time_stamp_us(), name, gnss->latitude, gnss->longitude, gnss->altitude, gnss->status);
  } else if (const auto * quat = v.as_ekf_quat()) {
    std::println(
      R"({{"t_us":{},"kind":"{}","quat":[{:.6f},{:.6f},{:.6f},{:.6f}]}})", v.time_stamp_us(), name,
      quat->quaternion[0], quat->quaternion[1], quat->quaternion[2], quat->quaternion[3]);
  } else {
    std::println(R"({{"t_us":{},"kind":"{}"}})", v.time_stamp_us(), name);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  bool json = false;
  bool serial = false;
  std::string path;
  std::string port;
  std::uint32_t baud = 921600;

  for (std::size_t i = 0; i < args.size(); ++i) {
    const auto a = args[i];
    if (a == "--json") {
      json = true;
    } else if (a == "--serial" && i + 1 < args.size()) {
      serial = true;
      port = std::string(args[++i]);
    } else if (a == "--baud" && i + 1 < args.size()) {
      baud = static_cast<std::uint32_t>(std::strtoul(std::string(args[++i]).c_str(), nullptr, 10));
    } else if (a == "-h" || a == "--help") {
      std::println("usage: sbg_decode <file.bin> [--json]");
      std::println("       sbg_decode --serial <port> [--baud N] [--json]");
      return 0;
    } else {
      path = std::string(a);
    }
  }

  if (!serial && path.empty()) {
    std::println(stderr, "error: provide a .bin file path or --serial <port> (see --help)");
    return 2;
  }

  sbg::TransportConfig cfg =
    serial
      ? sbg::TransportConfig{sbg::transport::Serial{.port = port, .baud = baud}}
      : sbg::TransportConfig{sbg::transport::FileReplay{.path = path, .real_time_pace = false}};

  auto dev = sbg::Device::open(std::move(cfg));
  if (!dev) {
    std::println(stderr, "error: open failed: {}", sbg::to_string(dev.error()));
    return 1;
  }

  std::uint64_t count = 0;
  dev->set_log_callback([&](const sbg::LogView & v) {
    ++count;
    if (json) {
      print_json(v);
    } else {
      print_text(v);
    }
  });

  std::signal(SIGINT, handle_sigint);

  // Poll until EOF (file: ~400 ms of no new logs) or SIGINT (serial: until ^C).
  int stale = 0;
  while (g_run.load(std::memory_order_relaxed)) {
    const auto before = count;
    if (auto r = dev->poll_once(std::chrono::milliseconds{4}); !r) {
      if (r.error() != sbg::Error::Timeout && r.error() != sbg::Error::NotReady) {
        std::println(stderr, "error: poll failed: {}", sbg::to_string(r.error()));
        break;
      }
    }
    if (count == before) {
      if (!serial && ++stale > 100) {
        break;  // file EOF
      }
    } else {
      stale = 0;
    }
  }

  std::println(stderr, "decoded {} logs", count);
  return 0;
}
