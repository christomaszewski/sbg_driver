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

#include <array>
#include <cstdint>

#include "gtest/gtest.h"
#include "sbg/detail/c_api.hpp"

namespace
{

TEST(SdkStreamBuffer, EmptyCommandPayloadDoesNotCopyANullPointer)
{
  std::array<std::uint8_t, 2> bytes{0xAA, 0xBB};
  SbgStreamBuffer buffer{};
  ASSERT_EQ(sbgStreamBufferInitForWrite(&buffer, bytes.data(), bytes.size()), SBG_NO_ERROR);
  EXPECT_EQ(sbgStreamBufferWriteBuffer(&buffer, nullptr, 0), SBG_NO_ERROR);
  EXPECT_EQ(sbgStreamBufferGetLength(&buffer), 0U);
  EXPECT_EQ(bytes, (std::array<std::uint8_t, 2>{0xAA, 0xBB}));

  const std::array<std::uint8_t, 2> payload{0x12, 0x34};
  ASSERT_EQ(sbgStreamBufferWriteBuffer(&buffer, payload.data(), payload.size()), SBG_NO_ERROR);
  EXPECT_EQ(sbgStreamBufferWriteBuffer(&buffer, nullptr, 0), SBG_NO_ERROR);
  EXPECT_EQ(sbgStreamBufferGetLength(&buffer), 2U);
  EXPECT_EQ(bytes, payload);
}

TEST(SdkStreamBuffer, EmptyPayloadPreservesAnExistingWriteError)
{
  std::array<std::uint8_t, 1> bytes{0xAA};
  SbgStreamBuffer buffer{};
  ASSERT_EQ(sbgStreamBufferInitForWrite(&buffer, bytes.data(), bytes.size()), SBG_NO_ERROR);
  const std::array<std::uint8_t, 2> payload{0x12, 0x34};
  ASSERT_EQ(
    sbgStreamBufferWriteBuffer(&buffer, payload.data(), payload.size()), SBG_BUFFER_OVERFLOW);
  EXPECT_EQ(sbgStreamBufferWriteBuffer(&buffer, nullptr, 0), SBG_BUFFER_OVERFLOW);
  EXPECT_EQ(sbgStreamBufferGetLength(&buffer), 0U);
  EXPECT_EQ(bytes[0], 0xAA);
}

}  // namespace
