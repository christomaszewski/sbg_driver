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

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <future>
#include <span>  // NOLINT(build/include_order) -- C++20/23 header
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "sbg/detail/c_api.hpp"
#include "sbg/device.hpp"
#include "sbg/transport.hpp"

namespace
{

using namespace std::chrono_literals;

class PseudoTerminal
{
public:
  PseudoTerminal()
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (master_ >= 0 && ::grantpt(master_) == 0 && ::unlockpt(master_) == 0) {
      if (const char * path = ::ptsname(master_)) {
        path_ = path;
      }
    }
  }
  ~PseudoTerminal()
  {
    if (master_ >= 0) {
      ::close(master_);
    }
  }
  [[nodiscard]] const std::string & path() const { return path_; }
  [[nodiscard]] int master() const { return master_; }

private:
  int master_ = -1;
  std::string path_;
};

TEST(SerialBackpressure, CorrectionWriteHasDeadline)
{
  PseudoTerminal tty;
  ASSERT_FALSE(tty.path().empty());
  auto device = sbg::Device::open(sbg::transport::Serial{.port = tty.path()});
  ASSERT_TRUE(device);
  const std::vector<std::byte> payload(4 * 1024 * 1024, std::byte{0xD3});
  const auto start = std::chrono::steady_clock::now();
  const auto result = device->write_rtcm(payload);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error(), sbg::Error::Timeout);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 50ms);
}

TEST(SerialBackpressure, CancellationInterruptsBlockedWrite)
{
  PseudoTerminal tty;
  ASSERT_FALSE(tty.path().empty());
  auto device = sbg::Device::open(sbg::transport::Serial{.port = tty.path()});
  ASSERT_TRUE(device);
  const std::vector<std::byte> payload(4 * 1024 * 1024, std::byte{0xD3});
  std::stop_source stop;
  auto result = std::async(
    std::launch::async, [&]() { return device->write_rtcm(payload, stop.get_token(), 5s); });
  // Wait until bytes actually reach the PTY: cancellation then interrupts an
  // active write under backpressure, not merely a thread that has not begun.
  std::array<std::byte, 1> first{};
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (::read(tty.master(), first.data(), first.size()) != 1 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  const auto start = std::chrono::steady_clock::now();
  stop.request_stop();
  ASSERT_EQ(result.wait_for(500ms), std::future_status::ready);
  const auto written = result.get();
  ASSERT_FALSE(written);
  EXPECT_EQ(written.error(), sbg::Error::NotReady);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST(SerialBackpressure, SdkCommandWritesAlsoHaveDeadline)
{
  PseudoTerminal tty;
  ASSERT_FALSE(tty.path().empty());
  auto transport = sbg::Transport::open(sbg::transport::Serial{.port = tty.path()});
  ASSERT_TRUE(transport);
  const std::vector<std::byte> payload(4 * 1024 * 1024, std::byte{0x5A});
  const auto start = std::chrono::steady_clock::now();
  // Exercise the very callback Configurator's SDK commands use, bypassing
  // Device::write_rtcm so a correction-only fix cannot pass this regression.
  const auto result = sbgInterfaceWrite(
    static_cast<SbgInterface *>(transport->native_handle()), payload.data(), payload.size());
  EXPECT_EQ(result, SBG_TIME_OUT);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST(SerialBackpressure, SerialReadWritePreservesBinaryBytes)
{
  PseudoTerminal tty;
  ASSERT_FALSE(tty.path().empty());
  auto transport = sbg::Transport::open(sbg::transport::Serial{.port = tty.path(), .baud = 115200});
  ASSERT_TRUE(transport);
  auto * iface = static_cast<SbgInterface *>(transport->native_handle());
  EXPECT_EQ(sbgInterfaceGetSpeed(iface), 115200U);
  const std::array<std::byte, 6> payload{std::byte{0},    std::byte{10},   std::byte{13},
                                         std::byte{0x11}, std::byte{0x13}, std::byte{0xFF}};
  ASSERT_TRUE(transport->write(payload));
  std::array<std::byte, 6> received{};
  ssize_t count = -1;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while ((count = ::read(tty.master(), received.data(), received.size())) < 0 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(count, 6);
  EXPECT_EQ(received, payload);
  ASSERT_EQ(::write(tty.master(), payload.data(), payload.size()), 6);
  std::size_t read_count = 0;
  do {
    ASSERT_EQ(sbgInterfaceRead(iface, received.data(), &read_count, received.size()), SBG_NO_ERROR);
  } while (read_count == 0 && std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(read_count, payload.size());
  EXPECT_EQ(received, payload);
}

}  // namespace
