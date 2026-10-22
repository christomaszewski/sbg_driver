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

#include <utility>

namespace sbg
{

RtcmWriter::RtcmWriter(Sink sink, FailureCallback on_failure)
: sink_(std::move(sink)),
  on_failure_(std::move(on_failure)),
  thread_([this](std::stop_token stop) { run(stop); })
{
}

RtcmWriter::~RtcmWriter()
{
  {
    const std::lock_guard lock{mutex_};
    stopping_ = true;
    queue_.clear();
    queued_bytes_ = 0;
  }
  thread_.request_stop();
  ready_.notify_all();
  thread_.join();
}

bool RtcmWriter::enqueue(std::span<const std::byte> data)
{
  const std::lock_guard lock{mutex_};
  if (stopping_) {
    return false;
  }
  if (data.empty()) {
    return true;
  }
  if (data.size() > k_max_bytes - queued_bytes_ || queue_.size() >= k_max_messages) {
    return false;
  }
  queue_.emplace_back(data.begin(), data.end());
  queued_bytes_ += data.size();
  ready_.notify_one();
  return true;
}

void RtcmWriter::run(std::stop_token stop)
{
  while (!stop.stop_requested()) {
    std::vector<std::byte> data;
    {
      std::unique_lock lock{mutex_};
      if (
        !ready_.wait(lock, stop, [this]() { return stopping_ || !queue_.empty(); }) || stopping_) {
        return;
      }
      data = std::move(queue_.front());
      queue_.pop_front();
      queued_bytes_ -= data.size();
    }
    Result<void> result;
    try {
      result = sink_(data, stop);
    } catch (...) {
      result = std::unexpected(Error::Internal);
    }
    if (!result && !stop.stop_requested() && on_failure_) {
      try {
        on_failure_(result.error());
      } catch (...) {
        // Reporting errors must not terminate the process from a worker.
      }
    }
  }
}

}  // namespace sbg
