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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <stop_token>
#include <thread>

#include "sbg/device.hpp"
#include "sbg/log_view.hpp"

namespace
{

namespace fs = std::filesystem;

class TempReplayFile
{
public:
  TempReplayFile()
  {
    path_ =
      fs::temp_directory_path() /
      ("sbg_replay_" + std::to_string(static_cast<unsigned long>(std::random_device{}())) + ".bin");
    std::ofstream{path_};  // create empty
  }
  ~TempReplayFile()
  {
    std::error_code ec;
    fs::remove(path_, ec);
  }

  [[nodiscard]] const fs::path & path() const noexcept { return path_; }

private:
  fs::path path_;
};

TEST(DeviceSmoke, OpenWithBadFilePathFails)
{
  auto dev = sbg::Device::open(sbg::transport::FileReplay{.path = "/no/such/path/__nope__.bin"});
  ASSERT_FALSE(dev.has_value());
  EXPECT_NE(dev.error(), sbg::Error::Ok);
}

TEST(DeviceSmoke, OpenWithEmptyFileSucceedsAndPollHitsEof)
{
  TempReplayFile tmp;
  auto dev = sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path()});
  ASSERT_TRUE(dev.has_value()) << "open empty replay failed: " << sbg::to_string(dev.error());

  // Empty file means no logs will be delivered, but poll_once should not
  // error out (Ok or NotReady are both acceptable).
  auto result = dev->poll_once(std::chrono::milliseconds{1});
  EXPECT_TRUE(result.has_value() || result.error() == sbg::Error::NotReady)
    << "poll_once on empty file returned unexpected error: "
    << (result ? std::string_view{} : sbg::to_string(result.error()));
}

TEST(DeviceSmoke, SetLogCallbackAcceptsLambda)
{
  TempReplayFile tmp;
  auto dev = sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path()});
  ASSERT_TRUE(dev.has_value());

  std::atomic<int> call_count = 0;
  dev->set_log_callback([&](const sbg::LogView & /*v*/) { ++call_count; });
  // Empty replay file means callback should never fire.
  (void)dev->poll_once(std::chrono::milliseconds{1});
  EXPECT_EQ(call_count.load(), 0);
}

TEST(DeviceSmoke, RunStopsOnStopToken)
{
  TempReplayFile tmp;
  auto dev = sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path()});
  ASSERT_TRUE(dev.has_value());

  std::stop_source stop;
  std::jthread io_thread{[&](std::stop_token st) { dev->run(st, std::chrono::milliseconds{1}); }};
  std::this_thread::sleep_for(std::chrono::milliseconds{20});
  io_thread.request_stop();
  // Joined automatically by jthread; if run() ignored the token, this test
  // would hang forever and the CI watchdog would fail it.
}

TEST(DeviceSmoke, MoveSemantics)
{
  TempReplayFile tmp;
  auto dev = sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path()});
  ASSERT_TRUE(dev.has_value());
  sbg::Device moved = std::move(*dev);
  // moved is now the active handle; original *dev is moved-from but still
  // destructible. Just check we can call poll_once on the moved-to.
  EXPECT_NO_FATAL_FAILURE((void)moved.poll_once(std::chrono::milliseconds{1}));
}

}  // namespace
