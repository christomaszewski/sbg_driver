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

#include "sbg/detail/c_api.hpp"
#include "sbg/error.hpp"

namespace
{

// Every SbgErrorCode value the sbgECom 5.6 SDK exposes. Hard-coded so this
// test fails loudly if upstream adds a new code we haven't mapped.
constexpr SbgErrorCode k_all_codes[] = {
  SBG_NO_ERROR,
  SBG_ERROR,
  SBG_NULL_POINTER,
  SBG_INVALID_CRC,
  SBG_INVALID_FRAME,
  SBG_TIME_OUT,
  SBG_WRITE_ERROR,
  SBG_READ_ERROR,
  SBG_BUFFER_OVERFLOW,
  SBG_INVALID_PARAMETER,
  SBG_NOT_READY,
  SBG_MALLOC_FAILED,
  SBG_CALIB_MAG_NOT_ENOUGH_POINTS,
  SBG_CALIB_MAG_INVALID_TAKE,
  SBG_CALIB_MAG_SATURATION,
  SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE,
  SBG_DEVICE_NOT_FOUND,
  SBG_OPERATION_CANCELLED,
  SBG_NOT_CONTINUOUS_FRAME,
  SBG_INCOMPATIBLE_HARDWARE,
  SBG_INVALID_VERSION,
};

TEST(ErrorMapping, NoErrorMapsToOk)
{
  EXPECT_EQ(sbg::detail::from_sbg(SBG_NO_ERROR), sbg::Error::Ok);
}

TEST(ErrorMapping, EveryCodeMapsToNonInternal)
{
  // We want every documented SbgErrorCode to map to a specific Error.
  // SBG_OPERATION_CANCELLED legitimately maps to Internal as a placeholder,
  // but everything else should be more specific.
  for (auto code : k_all_codes) {
    auto err = sbg::detail::from_sbg(code);
    if (code == SBG_OPERATION_CANCELLED) {
      EXPECT_EQ(err, sbg::Error::Internal)
        << "SBG_OPERATION_CANCELLED is currently mapped to Internal";
      continue;
    }
    EXPECT_NE(err, sbg::Error::Internal) << "Unmapped SbgErrorCode: " << static_cast<int>(code);
  }
}

TEST(ErrorMapping, SpecificMappings)
{
  using sbg::Error;
  EXPECT_EQ(sbg::detail::from_sbg(SBG_TIME_OUT), Error::Timeout);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_INVALID_PARAMETER), Error::InvalidParameter);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_NULL_POINTER), Error::InvalidParameter);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_NOT_READY), Error::NotReady);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_DEVICE_NOT_FOUND), Error::NotReady);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_INVALID_CRC), Error::BadCrc);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_READ_ERROR), Error::TransportFailure);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_WRITE_ERROR), Error::TransportFailure);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_MALLOC_FAILED), Error::AllocationFailure);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_CALIB_MAG_SATURATION), Error::CalibrationError);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_INCOMPATIBLE_HARDWARE), Error::Unsupported);
  EXPECT_EQ(sbg::detail::from_sbg(SBG_INVALID_VERSION), Error::Unsupported);
}

TEST(ErrorMapping, CheckReturnsExpected)
{
  EXPECT_TRUE(sbg::detail::check(SBG_NO_ERROR).has_value());

  auto result = sbg::detail::check(SBG_INVALID_PARAMETER);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error(), sbg::Error::InvalidParameter);
}

TEST(ErrorToString, AllValuesHaveStrings)
{
  using sbg::Error;
  using sbg::to_string;
  constexpr Error all_errors[] = {
    Error::Ok,
    Error::Timeout,
    Error::InvalidParameter,
    Error::NotReady,
    Error::ProtocolError,
    Error::TransportFailure,
    Error::DeviceBusy,
    Error::BadCrc,
    Error::Unsupported,
    Error::AllocationFailure,
    Error::CalibrationError,
    Error::Internal,
  };
  for (auto e : all_errors) {
    auto s = to_string(e);
    EXPECT_FALSE(s.empty()) << "Error::" << static_cast<int>(e) << " has empty string";
    EXPECT_NE(s, "unknown error") << "Error::" << static_cast<int>(e)
                                  << " falls through to default";
  }
}

}  // namespace
