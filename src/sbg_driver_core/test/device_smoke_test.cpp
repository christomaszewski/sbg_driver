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
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <random>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

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

// ---------------------------------------------------------------------------
// Synthetic-frame replay tests
//
// Build valid sbgECom STATUS frames byte-for-byte (sync, id/class/len header,
// payload, CRC-16 poly 0x8408 over id..payload, ETX) so replay tests exercise
// the SDK's real parse + dispatch path without committing binary fixtures.
// ---------------------------------------------------------------------------

[[nodiscard]] std::uint16_t crc16_sbg(const std::uint8_t * data, std::size_t len)
{
  // Reflected CRC-16 poly 0x8408, init 0 — bitwise form of the SDK's
  // table-driven sbgCrc16Compute (see sbgECom common/crc/sbgCrc.c).
  std::uint16_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    crc = static_cast<std::uint16_t>(crc ^ data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0 ? static_cast<std::uint16_t>((crc >> 1) ^ 0x8408U)
                            : static_cast<std::uint16_t>(crc >> 1);
    }
  }
  return crc;
}

void append_u16(std::vector<std::uint8_t> & v, std::uint16_t x)
{
  v.push_back(static_cast<std::uint8_t>(x & 0xFFU));
  v.push_back(static_cast<std::uint8_t>(x >> 8));
}

void append_u32(std::vector<std::uint8_t> & v, std::uint32_t x)
{
  append_u16(v, static_cast<std::uint16_t>(x & 0xFFFFU));
  append_u16(v, static_cast<std::uint16_t>(x >> 16));
}

[[nodiscard]] std::vector<std::uint8_t> make_status_frame(std::uint32_t time_stamp_us)
{
  // SBG_ECOM_LOG_STATUS payload layout per sbgEComLogStatusReadFromStream:
  // timeStamp u32, generalStatus u16, comStatus2 u16, comStatus u32,
  // aidingStatus u32, reserved2 u32, reserved3 u16, uptime u32.
  std::vector<std::uint8_t> payload;
  append_u32(payload, time_stamp_us);
  append_u16(payload, 0x1FU);  // generalStatus: all basic health bits set
  append_u16(payload, 0);      // comStatus2
  append_u32(payload, 0);      // comStatus
  append_u32(payload, 0);      // aidingStatus
  append_u32(payload, 0);      // reserved2
  append_u16(payload, 0);      // reserved3
  append_u32(payload, 42);     // uptime (s)

  std::vector<std::uint8_t> crc_region;
  crc_region.push_back(static_cast<std::uint8_t>(SBG_ECOM_LOG_STATUS));
  crc_region.push_back(static_cast<std::uint8_t>(SBG_ECOM_CLASS_LOG_ECOM_0));
  append_u16(crc_region, static_cast<std::uint16_t>(payload.size()));
  crc_region.insert(crc_region.end(), payload.begin(), payload.end());

  std::vector<std::uint8_t> frame{0xFFU, 0x5AU};  // SBG_ECOM_SYNC_1 / SYNC_2
  frame.insert(frame.end(), crc_region.begin(), crc_region.end());
  append_u16(frame, crc16_sbg(crc_region.data(), crc_region.size()));
  frame.push_back(0x33U);  // SBG_ECOM_ETX
  return frame;
}

void write_frames(const fs::path & path, const std::vector<std::uint32_t> & stamps_us)
{
  std::ofstream out{path, std::ios::binary | std::ios::trunc};
  for (auto ts : stamps_us) {
    const auto frame = make_status_frame(ts);
    out.write(
      reinterpret_cast<const char *>(frame.data()), static_cast<std::streamsize>(frame.size()));
  }
}

TEST(DeviceReplay, DispatchesSyntheticStatusFrames)
{
  TempReplayFile tmp;
  write_frames(tmp.path(), {1000U, 2000U, 3000U});
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = false});
  ASSERT_TRUE(dev.has_value());

  std::vector<std::uint32_t> stamps;
  dev->set_log_callback([&](const sbg::LogView & v) {
    EXPECT_EQ(v.kind(), sbg::LogView::Kind::Status);
    stamps.push_back(v.time_stamp_us());
  });
  (void)dev->poll_once(std::chrono::milliseconds{1});

  // Three dispatches proves the hand-built frames pass the SDK's real CRC +
  // framing checks (a builder bug would silently drop them).
  ASSERT_EQ(stamps.size(), 3U);
  EXPECT_EQ(stamps[0], 1000U);
  EXPECT_EQ(stamps[2], 3000U);
  EXPECT_EQ(dev->callback_exception_count(), 0U);
}

TEST(DeviceReplay, CallbackExceptionsAreCountedAndContained)
{
  TempReplayFile tmp;
  write_frames(tmp.path(), {1000U, 2000U, 3000U});
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = false});
  ASSERT_TRUE(dev.has_value());

  dev->set_log_callback([](const sbg::LogView &) { throw std::runtime_error{"boom"}; });
  auto result = dev->poll_once(std::chrono::milliseconds{1});

  // The SDK absorbs per-log callback failures (documented poll_once
  // contract), so the poll succeeds and the counter is the only signal.
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(dev->callback_exception_count(), 3U);
}

TEST(DeviceReplay, RealTimePaceThrottlesAndReanchorsOnBackwardJump)
{
  TempReplayFile tmp;
  // First frame anchors; second sits +150 ms after it; third jumps backwards
  // (uint32 wrap / restarted capture) and must re-anchor without sleeping.
  write_frames(tmp.path(), {1000U, 151000U, 500U});
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = true});
  ASSERT_TRUE(dev.has_value());

  int count = 0;
  dev->set_log_callback([&](const sbg::LogView &) { ++count; });
  const auto t0 = std::chrono::steady_clock::now();
  (void)dev->poll_once(std::chrono::milliseconds{1});
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_EQ(count, 3);
  EXPECT_GE(elapsed, std::chrono::milliseconds{120});   // paced by the 150 ms gap
  EXPECT_LT(elapsed, std::chrono::milliseconds{1000});  // backward jump didn't sleep
}

TEST(DeviceReplay, RequestStopAbandonsPacingPromptly)
{
  // Regression: pacing used to sleep without consulting any stop signal, so
  // run()'s stop check only got a look-in between poll_once() calls — and a
  // poll_once() over a FileReplay only returns at EOF. A deactivate part-way
  // through a paced capture therefore had to wait out every remaining log.
  //
  // Each individual pace() sleep was already capped at 250 ms, so the stall
  // scales with the number of logs LEFT IN THE FILE, not with the recorded
  // span — which is exactly why a long capture was deadly. 60 frames spaced
  // 300 ms apart (every gap exceeding the cap) therefore cost ~60 × 250 ms =
  // ~15 s before the fix. Post-fix the wait is one 10 ms pacing slice plus
  // the parse of what remains, so a 1 s bound separates the two cleanly.
  TempReplayFile tmp;
  std::vector<std::uint32_t> stamps;
  stamps.reserve(60);
  for (std::uint32_t i = 0; i < 60; ++i) {
    stamps.push_back(1000U + (i * 300000U));  // 300 ms apart
  }
  write_frames(tmp.path(), stamps);
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = true});
  ASSERT_TRUE(dev.has_value());

  std::atomic<int> dispatched{0};
  dev->set_log_callback([&](const sbg::LogView &) { dispatched.fetch_add(1); });

  const auto t0 = std::chrono::steady_clock::now();
  std::jthread io{[&](std::stop_token st) { dev->run(st, std::chrono::milliseconds{4}); }};
  std::this_thread::sleep_for(std::chrono::milliseconds{50});
  io.request_stop();
  io.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::milliseconds{1000});
  // Sanity: we really did stop early rather than the file being consumed
  // before request_stop() landed. The first frame anchors and dispatches
  // immediately; ~50 ms of pacing cannot have reached all 60.
  EXPECT_GE(dispatched.load(), 1);
  EXPECT_LT(dispatched.load(), 60);
}

TEST(DeviceReplay, StoppingFlagIsClearedSoAThreadCanRestart)
{
  // The mag-cal / save-settings services stop and restart the I/O thread on
  // the same Device. A latched stop flag would leave the restarted thread
  // silently dispatching nothing.
  TempReplayFile tmp;
  write_frames(tmp.path(), {1000U, 2000U, 3000U});
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = false});
  ASSERT_TRUE(dev.has_value());

  std::atomic<int> dispatched{0};
  dev->set_log_callback([&](const sbg::LogView &) { dispatched.fetch_add(1); });

  {
    std::jthread io{[&](std::stop_token st) { dev->run(st, std::chrono::milliseconds{4}); }};
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    io.request_stop();
    io.join();
  }
  const int after_first = dispatched.load();
  ASSERT_GT(after_first, 0);

  // Second run: the file is at EOF so nothing new arrives, but the run must
  // start cleanly and exit on request rather than hanging or short-circuiting.
  std::jthread io2{[&](std::stop_token st) { dev->run(st, std::chrono::milliseconds{4}); }};
  std::this_thread::sleep_for(std::chrono::milliseconds{30});
  io2.request_stop();
  io2.join();
  EXPECT_EQ(dev->callback_exception_count(), 0U);
}

TEST(DeviceReplay, NoPaceRunsFlatOut)
{
  TempReplayFile tmp;
  write_frames(tmp.path(), {1000U, 151000U});
  auto dev =
    sbg::Device::open(sbg::transport::FileReplay{.path = tmp.path(), .real_time_pace = false});
  ASSERT_TRUE(dev.has_value());

  int count = 0;
  dev->set_log_callback([&](const sbg::LogView &) { ++count; });
  const auto t0 = std::chrono::steady_clock::now();
  (void)dev->poll_once(std::chrono::milliseconds{1});

  EXPECT_EQ(count, 2);
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds{100});
}

}  // namespace
