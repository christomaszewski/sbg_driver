// Copyright 2026 sbg_driver maintainers
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <optional>

namespace sbg
{

// Unwrap adjacent timestamps, not timestamps relative to the first record.
// Small backwards steps are reordered logs; large ones start a new capture epoch.
class ReplayTimeline
{
public:
  struct Position
  {
    std::int64_t offset_us;
    bool new_epoch;
  };

  Position observe(std::uint32_t timestamp) noexcept
  {
    if (!last_) {
      last_ = timestamp;
      return {0, true};
    }
    const auto delta = static_cast<std::int32_t>(timestamp - *last_);
    if (delta < -1'000'000) {
      last_ = timestamp;
      offset_ = 0;
      return {0, true};
    }
    if (delta >= 0) {
      offset_ += delta;
      last_ = timestamp;
      return {offset_, false};
    }
    return {offset_ + delta, false};
  }

private:
  std::optional<std::uint32_t> last_;
  std::int64_t offset_ = 0;
};

}  // namespace sbg
