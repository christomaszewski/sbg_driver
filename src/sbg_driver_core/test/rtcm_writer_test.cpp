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

#include "sbg/rtcm_writer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <span>  // NOLINT(build/include_order) -- C++20/23 header
#include <vector>

#include "gtest/gtest.h"

namespace
{

using namespace std::chrono_literals;

TEST(RtcmWriter, IdleDestructionStopsPromptly)
{
  const auto start = std::chrono::steady_clock::now();
  {
    sbg::RtcmWriter writer{[](auto, auto) -> sbg::Result<void> { return {}; }};
  }
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
}

TEST(RtcmWriter, QueueIsBoundedAndDestructionCancelsActiveWriteAndDropsQueuedData)
{
  std::promise<void> entered;
  auto ready = entered.get_future();
  std::atomic<int> writes = 0;
  std::atomic<bool> cancelled = false;
  auto writer =
    std::make_unique<sbg::RtcmWriter>([&](auto, std::stop_token stop) -> sbg::Result<void> {
    ++writes;
    entered.set_value();
    std::mutex mutex;
    std::condition_variable_any cv;
    std::unique_lock lock{mutex};
    cv.wait(lock, stop, []() { return false; });
    cancelled = stop.stop_requested();
    return std::unexpected(sbg::Error::NotReady);
  });
  const std::vector<std::byte> message(1, std::byte{0xD3});
  ASSERT_TRUE(writer->enqueue(message));
  ASSERT_EQ(ready.wait_for(1s), std::future_status::ready);
  for (std::size_t i = 0; i < sbg::RtcmWriter::k_max_messages; ++i) {
    EXPECT_TRUE(writer->enqueue(message));
  }
  EXPECT_FALSE(writer->enqueue(message));
  const auto start = std::chrono::steady_clock::now();
  writer.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
  EXPECT_TRUE(cancelled);
  EXPECT_EQ(writes, 1);
}

TEST(RtcmWriter, ByteLimitRejectsWholeNewestMessage)
{
  std::promise<void> entered;
  auto ready = entered.get_future();
  sbg::RtcmWriter writer([&](auto, std::stop_token stop) -> sbg::Result<void> {
    entered.set_value();
    std::mutex mutex;
    std::condition_variable_any cv;
    std::unique_lock lock{mutex};
    cv.wait(lock, stop, []() { return false; });
    return {};
  });
  const std::vector<std::byte> first(1);
  ASSERT_TRUE(writer.enqueue(first));
  ASSERT_EQ(ready.wait_for(1s), std::future_status::ready);
  const std::vector<std::byte> full(sbg::RtcmWriter::k_max_bytes);
  EXPECT_TRUE(writer.enqueue(full));
  EXPECT_FALSE(writer.enqueue(first));
  EXPECT_FALSE(writer.enqueue(std::vector<std::byte>(sbg::RtcmWriter::k_max_bytes + 1)));
}

TEST(RtcmWriter, PreservesMessageOrderAndReportsFailedWrites)
{
  std::promise<void> done;
  auto ready = done.get_future();
  std::vector<std::byte> received;
  std::atomic<int> failures = 0;
  sbg::RtcmWriter writer([&](std::span<const std::byte> data, auto) -> sbg::Result<void> {
    received.push_back(data[0]);
    if (data[0] == std::byte{1}) {
      return std::unexpected(sbg::Error::Timeout);
    }
    if (data[0] == std::byte{3}) {
      done.set_value();
    }
    return {};
  }, [&](sbg::Error error) {
    EXPECT_EQ(error, sbg::Error::Timeout);
    ++failures;
  });
  for (unsigned char i = 1; i <= 3; ++i) {
    const std::byte value{i};
    ASSERT_TRUE(writer.enqueue(std::span{&value, 1}));
  }
  ASSERT_EQ(ready.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(received, (std::vector<std::byte>{std::byte{1}, std::byte{2}, std::byte{3}}));
  EXPECT_EQ(failures, 1);
}

}  // namespace
