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
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <span>        // NOLINT(build/include_order) -- C++20/23 header
#include <stop_token>  // NOLINT(build/include_order) -- C++20/23 header
#include <thread>
#include <vector>

#include "sbg/error.hpp"

namespace sbg
{

// Nonblocking admission for correction traffic. At most 20 messages / 64 KiB
// wait behind the current write. Reject newest on overflow (no silent partial
// message drops). Destruction cancels the current write, discards queued
// traffic, then joins; its sink must honor the stop token. Owners destroy the
// writer before issuing device commands or releasing the device.
class RtcmWriter
{
public:
  using Sink = std::function<Result<void>(std::span<const std::byte>, std::stop_token)>;
  using FailureCallback = std::function<void(Error)>;
  static constexpr std::size_t k_max_messages = 20;
  static constexpr std::size_t k_max_bytes = 64 * 1024;

  explicit RtcmWriter(Sink sink, FailureCallback on_failure = {});
  ~RtcmWriter();
  RtcmWriter(const RtcmWriter &) = delete;
  RtcmWriter & operator=(const RtcmWriter &) = delete;

  // Empty input is a no-op. False means queue full, oversize, or stopping.
  [[nodiscard]] bool enqueue(std::span<const std::byte> data);

private:
  void run(std::stop_token stop);

  Sink sink_;
  FailureCallback on_failure_;
  std::mutex mutex_;
  std::condition_variable_any ready_;
  std::deque<std::vector<std::byte>> queue_;
  std::size_t queued_bytes_ = 0;
  bool stopping_ = false;
  std::jthread thread_;
};

}  // namespace sbg
