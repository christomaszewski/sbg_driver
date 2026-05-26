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

#include <filesystem>
#include <fstream>
#include <variant>

#include "sbg/error.hpp"
#include "sbg/transport.hpp"

namespace
{

TEST(TransportConfig, SerialDefaults)
{
  sbg::transport::Serial s{.port = "/dev/ttyUSB0"};
  EXPECT_EQ(s.baud, 921600U);
  EXPECT_TRUE(s.fallback_bauds.empty());
}

TEST(TransportConfig, VariantHoldsEachType)
{
  sbg::TransportConfig cfg = sbg::transport::Serial{.port = "/dev/ttyUSB0"};
  EXPECT_TRUE(std::holds_alternative<sbg::transport::Serial>(cfg));

  cfg = sbg::transport::Udp{.remote_ip = "192.168.1.20", .in_port = 1234, .out_port = 5678};
  EXPECT_TRUE(std::holds_alternative<sbg::transport::Udp>(cfg));

  cfg = sbg::transport::FileReplay{.path = "/tmp/sample.bin"};
  EXPECT_TRUE(std::holds_alternative<sbg::transport::FileReplay>(cfg));
}

TEST(Validate, SerialAcceptsStandardBauds)
{
  for (std::uint32_t baud : {9600U, 115200U, 230400U, 460800U, 921600U}) {
    sbg::TransportConfig cfg = sbg::transport::Serial{.port = "/dev/ttyUSB0", .baud = baud};
    auto r = sbg::validate(cfg);
    EXPECT_TRUE(r.has_value()) << "baud=" << baud << " rejected";
  }
}

TEST(Validate, SerialRejectsUnsupportedBaud)
{
  sbg::TransportConfig cfg = sbg::transport::Serial{.port = "/dev/ttyUSB0", .baud = 4096};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), sbg::Error::InvalidParameter);
}

TEST(Validate, SerialRejectsEmptyPort)
{
  sbg::TransportConfig cfg = sbg::transport::Serial{.port = "", .baud = 921600};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), sbg::Error::InvalidParameter);
}

TEST(Validate, SerialRejectsBadFallbackBaud)
{
  sbg::TransportConfig cfg = sbg::transport::Serial{
    .port = "/dev/ttyUSB0", .baud = 921600, .fallback_bauds = {115200, 12345}};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
}

TEST(Validate, UdpAcceptsWellFormed)
{
  sbg::TransportConfig cfg =
    sbg::transport::Udp{.remote_ip = "192.168.1.20", .in_port = 1234, .out_port = 5678};
  EXPECT_TRUE(sbg::validate(cfg).has_value());
}

TEST(Validate, UdpRejectsEmptyIp)
{
  sbg::TransportConfig cfg =
    sbg::transport::Udp{.remote_ip = "", .in_port = 1234, .out_port = 5678};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), sbg::Error::InvalidParameter);
}

TEST(Validate, UdpRejectsZeroPort)
{
  sbg::TransportConfig cfg =
    sbg::transport::Udp{.remote_ip = "192.168.1.20", .in_port = 0, .out_port = 5678};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
}

TEST(Validate, FileReplayAcceptsAnyPath)
{
  sbg::TransportConfig cfg = sbg::transport::FileReplay{.path = "/tmp/sample.bin"};
  EXPECT_TRUE(sbg::validate(cfg).has_value());
}

TEST(Validate, FileReplayRejectsEmptyPath)
{
  sbg::TransportConfig cfg = sbg::transport::FileReplay{.path = ""};
  auto r = sbg::validate(cfg);
  ASSERT_FALSE(r.has_value());
}

TEST(TransportOpen, FileReplayMissingFileFails)
{
  auto r = sbg::Transport::open(
    sbg::transport::FileReplay{.path = "/this/path/does/not/exist/xyz123.bin"});
  ASSERT_FALSE(r.has_value());
  // Specific error depends on the SDK; we just want a non-Ok mapping.
  EXPECT_NE(r.error(), sbg::Error::Ok);
}

TEST(TransportOpen, FileReplayWithEmptyTempFileSucceeds)
{
  namespace fs = std::filesystem;
  fs::path tmp = fs::temp_directory_path() / "sbg_transport_empty.bin";
  std::ofstream{tmp};  // create empty file
  auto r = sbg::Transport::open(sbg::transport::FileReplay{.path = tmp});
  EXPECT_TRUE(r.has_value()) << "open empty file failed: "
                             << (r ? std::string_view{} : sbg::to_string(r.error()));
  fs::remove(tmp);
}

}  // namespace
