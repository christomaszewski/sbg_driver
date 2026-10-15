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

#include "sbg_driver/health_publisher.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "gtest/gtest.h"

// The ROS side of health reporting: device logs → reporter, components →
// DiagnosticArray, the array's real CDR bytes, and the heartbeat thread on a
// lifecycle node. The serialized fixture this file pins is what
// tools/check_dashboard_health.sh feeds to the dashboard's own decoder.

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using sbg_driver::HealthReporter;
using sbg_driver::HealthSettings;
using std::chrono::milliseconds;

const DiagnosticStatus * find_status(const DiagnosticArray & msg, std::string_view name)
{
  for (const auto & status : msg.status) {
    if (status.name == name) {
      return &status;
    }
  }
  return nullptr;
}

std::optional<std::string> value_of(const DiagnosticStatus & status, std::string_view key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return std::nullopt;
}

std::optional<std::string> value_of(
  const DiagnosticArray & msg, std::string_view name, std::string_view key)
{
  const auto * status = find_status(msg, name);
  return status == nullptr ? std::nullopt : value_of(*status, key);
}

HealthSettings settings(double interval_s = 1.0, double stale_after_s = 5.0)
{
  HealthSettings s;
  s.vehicle_id = "uav6";
  s.instance = "ins";
  s.transport = "udp";
  s.publish_interval_s = interval_s;
  s.stale_after_s = stale_after_s;
  return s;
}

sbg::LogView view_of(std::uint16_t msg_id, const SbgEComLogUnion & u)
{
  return sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, msg_id, &u};
}

void feed_status(HealthReporter & r, std::uint16_t general, std::uint32_t uptime = 3600)
{
  SbgEComLogUnion u{};
  u.statusData.generalStatus = general;
  u.statusData.comStatus = 0x0000801FU;
  u.statusData.comStatus2 = 0x0003U;
  u.statusData.aidingStatus = 0x0000000FU;
  u.statusData.uptime = uptime;
  u.statusData.cpuUsage = 23;
  sbg_driver::observe_log(r, view_of(SBG_ECOM_LOG_STATUS, u));
}

void feed_imu_short(HealthReporter & r, std::int16_t temperature_lsb)
{
  SbgEComLogUnion u{};
  u.imuShort.status = 0x03FF;
  u.imuShort.temperature = temperature_lsb;  // 256 LSB per °C
  sbg_driver::observe_log(r, view_of(SBG_ECOM_LOG_IMU_SHORT, u));
}

void feed_imu_data(HealthReporter & r, float celsius)
{
  SbgEComLogUnion u{};
  u.imuData.status = 0x03FF;
  u.imuData.temperature = celsius;
  sbg_driver::observe_log(r, view_of(SBG_ECOM_LOG_IMU_DATA, u));
}

void feed_ekf(HealthReporter & r, std::uint32_t status)
{
  SbgEComLogUnion u{};
  u.ekfNavData.status = status;
  sbg_driver::observe_log(r, view_of(SBG_ECOM_LOG_EKF_NAV, u));
}

// GnssPos.status: solution status in bits 0-5, position type in bits 6-11.
void feed_gnss(HealthReporter & r, std::uint32_t type, std::uint16_t msg_id = SBG_ECOM_LOG_GPS1_POS)
{
  SbgEComLogUnion u{};
  u.gpsPosData.status = SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED | (type << 6U);
  u.gpsPosData.latitudeAccuracy = 0.012F;
  u.gpsPosData.longitudeAccuracy = 0.011F;
  u.gpsPosData.altitudeAccuracy = 0.02F;
  u.gpsPosData.numSvUsed = 21;
  u.gpsPosData.differentialAge = 120;
  u.gpsPosData.baseStationId = 7;
  sbg_driver::observe_log(r, view_of(msg_id, u));
}

void feed_heading(
  HealthReporter & r, float heading, float accuracy, std::uint16_t msg_id = SBG_ECOM_LOG_GPS1_HDT)
{
  SbgEComLogUnion u{};
  u.gpsHdtData.status = SBG_ECOM_GNSS_HDT_STATUS_SOL_COMPUTED | (1U << 6U);  // baseline valid
  u.gpsHdtData.heading = heading;
  u.gpsHdtData.headingAccuracy = accuracy;
  u.gpsHdtData.baseline = 1.25F;
  u.gpsHdtData.numSvUsed = 14;
  sbg_driver::observe_log(r, view_of(msg_id, u));
}

constexpr std::uint16_t k_all_ok = 0x007F;
constexpr std::uint32_t k_ekf_nav_position =
  SBG_ECOM_SOL_MODE_NAV_POSITION | SBG_ECOM_SOL_ATTITUDE_VALID | SBG_ECOM_SOL_HEADING_VALID |
  SBG_ECOM_SOL_VELOCITY_VALID | SBG_ECOM_SOL_POSITION_VALID | SBG_ECOM_SOL_GPS1_POS_USED |
  SBG_ECOM_SOL_GPS1_VEL_USED | SBG_ECOM_SOL_GPS1_HDT_USED | SBG_ECOM_SOL_ALIGN_VALID;

builtin_interfaces::msg::Time stamp(std::int32_t sec = 1791450000)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  return t;
}

std::vector<std::uint8_t> serialize(const DiagnosticArray & msg)
{
  // CDR leaves alignment padding unwritten, i.e. whatever the heap held.
  // Serializing into a zeroed buffer big enough not to be reallocated keeps
  // the fixture reproducible (and free of stray memory).
  constexpr std::size_t capacity = 256U * 1024U;
  rclcpp::SerializedMessage serialized{capacity};
  auto & raw = serialized.get_rcl_serialized_message();
  std::fill_n(raw.buffer, raw.buffer_capacity, std::uint8_t{0});
  rclcpp::Serialization<DiagnosticArray>{}.serialize_message(&msg, &serialized);
  EXPECT_LE(raw.buffer_length, capacity);
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

DiagnosticArray deserialize(const std::vector<std::uint8_t> & bytes)
{
  rclcpp::SerializedMessage serialized{bytes.size()};
  auto & raw = serialized.get_rcl_serialized_message();
  std::copy(bytes.begin(), bytes.end(), raw.buffer);
  raw.buffer_length = bytes.size();
  DiagnosticArray msg;
  rclcpp::Serialization<DiagnosticArray>{}.deserialize_message(&serialized, &msg);
  return msg;
}

std::string base64(const std::vector<std::uint8_t> & bytes)
{
  constexpr std::string_view alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const std::uint32_t chunk = (static_cast<std::uint32_t>(bytes[i]) << 16U) |
                                (i + 1 < bytes.size() ? bytes[i + 1] << 8U : 0U) |
                                (i + 2 < bytes.size() ? bytes[i + 2] : 0U);
    out += alphabet[(chunk >> 18U) & 0x3FU];
    out += alphabet[(chunk >> 12U) & 0x3FU];
    out += i + 1 < bytes.size() ? alphabet[(chunk >> 6U) & 0x3FU] : '=';
    out += i + 2 < bytes.size() ? alphabet[chunk & 0x3FU] : '=';
  }
  return out;
}

std::vector<std::uint8_t> unbase64(std::string_view text)
{
  constexpr std::string_view alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<std::uint8_t> out;
  std::uint32_t chunk = 0;
  int bits = 0;
  for (const char c : text) {
    const auto index = alphabet.find(c);
    if (index == std::string_view::npos) {
      break;  // '=' padding
    }
    chunk = (chunk << 6U) | static_cast<std::uint32_t>(index);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<std::uint8_t>((chunk >> static_cast<unsigned>(bits)) & 0xFFU));
    }
  }
  return out;
}

// ---- device logs → reporter -------------------------------------------------

TEST(ObserveLog, RoutesEachLogKindToItsComponent)
{
  HealthReporter r;
  ASSERT_TRUE(r.configure(settings()));
  r.enter_active({"ELLIPSE-D", "045000123", 0x03000000});

  feed_status(r, k_all_ok & static_cast<std::uint16_t>(~SBG_ECOM_GENERAL_IMU_POWER_OK));
  feed_imu_short(r, 256 * 25 + 128);  // 25.5 °C in the log's fixed point
  feed_ekf(r, k_ekf_nav_position);
  feed_gnss(r, SBG_ECOM_GNSS_POS_TYPE_RTK_INT);
  feed_heading(r, 90.5F, 0.25F);

  const auto msg = sbg_driver::to_diagnostic_array(r.snapshot(), stamp());
  EXPECT_EQ(value_of(msg, "ins: stream", "logs.received"), "5");
  EXPECT_EQ(value_of(msg, "ins: device", "device.general_status"), "0x007d");
  EXPECT_EQ(value_of(msg, "ins: device", "device.com_status"), "0x0000801f");
  EXPECT_EQ(value_of(msg, "ins: device", "device.com_status_2"), "0x0003");
  EXPECT_EQ(value_of(msg, "ins: device", "device.aiding_status"), "0x0000000f");
  EXPECT_EQ(value_of(msg, "ins: device", "uptime_s"), "3600");
  EXPECT_EQ(value_of(msg, "ins: device", "cpu.usage_pct"), "23");
  EXPECT_EQ(value_of(msg, "ins: alert/IMU_POWER", "alert.active"), "true");
  EXPECT_EQ(value_of(msg, "ins: temperature", "temp.imu_c"), "25.5");
  EXPECT_EQ(value_of(msg, "ins: ekf", "ekf.solution_mode"), "NAV_POSITION");
  EXPECT_EQ(value_of(msg, "ins: ekf", "ekf.gps1_hdt_used"), "true");
  EXPECT_EQ(value_of(msg, "ins: gnss", "gnss.rtk"), "fixed");
  EXPECT_EQ(value_of(msg, "ins: gnss", "gnss.sv_used"), "21");
  EXPECT_EQ(value_of(msg, "ins: gnss", "gnss.differential_age_s"), "1.2");
  EXPECT_EQ(value_of(msg, "ins: gnss heading", "gnss.heading_deg"), "90.5");
  EXPECT_EQ(value_of(msg, "ins: gnss heading", "gnss.heading_accuracy_deg"), "0.25");
  EXPECT_EQ(value_of(msg, "ins: gnss heading", "gnss.baseline_m"), "1.25");
  EXPECT_EQ(find_status(msg, "ins: gnss heading")->hardware_id, "ELLIPSE-D 045000123");
}

TEST(ObserveLog, LegacyImuLogFeedsTheSameTemperature)
{
  HealthReporter r;
  ASSERT_TRUE(r.configure(settings()));
  r.enter_active({});
  feed_imu_data(r, 0.0F);  // a real 0 °C
  auto msg = sbg_driver::to_diagnostic_array(r.snapshot(), stamp());
  EXPECT_EQ(value_of(msg, "ins: temperature", "temp.imu_c"), "0");
  EXPECT_EQ(find_status(msg, "ins: temperature")->level, DiagnosticStatus::OK);

  feed_imu_data(r, std::numeric_limits<float>::quiet_NaN());
  msg = sbg_driver::to_diagnostic_array(r.snapshot(), stamp());
  EXPECT_EQ(value_of(msg, "ins: temperature", "temp.imu_c"), "0");
  EXPECT_EQ(find_status(msg, "ins: temperature")->level, DiagnosticStatus::STALE);
  EXPECT_EQ(value_of(msg, "ins: temperature", "health.last_level"), "0");
}

TEST(ObserveLog, ImuFailuresAndRecoverySurviveCdrSerialization)
{
  for (auto id : {SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_LOG_IMU_SHORT}) {
    HealthReporter r;
    ASSERT_TRUE(r.configure(settings()));
    r.enter_active({"QUANTA", "123", 0});
    SbgEComLogUnion u{};
    if (id == SBG_ECOM_LOG_IMU_DATA) {
      u.imuData.status = 0;
      u.imuData.temperature = 25;
    } else {
      u.imuShort.status = 0;
      u.imuShort.temperature = 25 * 256;
    }
    sbg_driver::observe_log(r, view_of(id, u));
    auto msg = deserialize(serialize(sbg_driver::to_diagnostic_array(r.snapshot(), stamp())));
    ASSERT_NE(find_status(msg, "ins: imu"), nullptr);
    EXPECT_EQ(find_status(msg, "ins: imu")->level, DiagnosticStatus::ERROR);
    EXPECT_EQ(value_of(msg, "ins: imu", "imu.status"), "0x0000");
    EXPECT_EQ(value_of(msg, "ins: imu", "health.availability"), "current");
    EXPECT_EQ(value_of(msg, "ins: temperature", "temp.imu_c"), "25");
    if (id == SBG_ECOM_LOG_IMU_DATA) {
      u.imuData.status = 0x03FF;
    } else {
      u.imuShort.status = 0x07FF;
    }
    sbg_driver::observe_log(r, view_of(id, u));
    msg = deserialize(serialize(sbg_driver::to_diagnostic_array(r.snapshot(), stamp())));
    EXPECT_EQ(find_status(msg, "ins: imu")->level, DiagnosticStatus::OK);
    EXPECT_EQ(value_of(msg, "ins: imu", "imu.failed_checks"), std::nullopt);
  }
}

TEST(ObserveLog, SecondaryReceiverLogsAreNotMistakenForThePrimary)
{
  HealthReporter r;
  ASSERT_TRUE(r.configure(settings()));
  r.enter_active({});
  feed_gnss(r, SBG_ECOM_GNSS_POS_TYPE_RTK_INT, SBG_ECOM_LOG_GPS2_POS);
  feed_heading(r, 10.0F, 0.2F, SBG_ECOM_LOG_GPS2_HDT);
  const auto msg = sbg_driver::to_diagnostic_array(r.snapshot(), stamp());
  EXPECT_EQ(find_status(msg, "ins: gnss"), nullptr);
  EXPECT_EQ(find_status(msg, "ins: gnss heading"), nullptr);
  EXPECT_EQ(value_of(msg, "ins: stream", "logs.received"), "2");  // still counted as traffic
}

// ---- components → DiagnosticArray → CDR ---------------------------------------

TEST(DiagnosticArrayConversion, UsesTheGeneratedLevelConstantsAndStringValues)
{
  using sbg_driver::HealthComponent;
  using sbg_driver::HealthLevel;
  std::vector<HealthComponent> components;
  for (auto level : {HealthLevel::Ok, HealthLevel::Warn, HealthLevel::Error, HealthLevel::Stale}) {
    auto & c = components.emplace_back();
    c.name = "ins: c" + std::to_string(static_cast<int>(level));
    c.level = level;
  }
  components[0].message = "fine";
  components[0].hardware_id = "QUANTA 1";
  components[0].values = {
    {"temp.imu_c", 0.0},
    {"count", std::int64_t{3}},
    {"alert.active", false},
    {"serial", std::string{"00123"}},
    {"missing", sbg_driver::HealthNull{}}};

  const auto msg = sbg_driver::to_diagnostic_array(components, stamp(42));
  ASSERT_EQ(msg.status.size(), 4U);
  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_EQ(msg.status[0].level, DiagnosticStatus::OK);
  EXPECT_EQ(msg.status[1].level, DiagnosticStatus::WARN);
  EXPECT_EQ(msg.status[2].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(msg.status[3].level, DiagnosticStatus::STALE);
  EXPECT_EQ(msg.status[0].message, "fine");
  EXPECT_EQ(msg.status[0].hardware_id, "QUANTA 1");
  EXPECT_EQ(value_of(msg.status[0], "temp.imu_c"), "0");
  EXPECT_EQ(value_of(msg.status[0], "count"), "3");
  EXPECT_EQ(value_of(msg.status[0], "alert.active"), "false");
  EXPECT_EQ(value_of(msg.status[0], "serial"), "00123");
  EXPECT_EQ(value_of(msg.status[0], "missing"), "null");
}

// What a dashboard would be handed in each situation worth checking there.
// Deterministic (fixed clock, publisher ids and stamp), so the bytes can be
// pinned in test/fixtures/health_diagnostics.json.
std::map<std::string, DiagnosticArray> representative_reports()
{
  std::map<std::string, DiagnosticArray> out;
  HealthReporter::TimePoint now{std::chrono::hours{1}};
  const auto clock = [&now] { return now; };

  auto s = settings();
  s.required_gnss_fix = HealthSettings::GnssFix::RtkFixed;
  s.temp_imu_c.warn_above = 60.0;
  s.temp_imu_c.error_above = 85.0;

  HealthReporter first{clock, "5b0c9d3e-7a41-4c1e-9f6b-2d8e4a1c0f77"};
  EXPECT_TRUE(first.configure(s));
  first.enter_standby();
  out["standby_before_activation"] = sbg_driver::to_diagnostic_array(first.snapshot(), stamp());

  first.enter_active({"QUANTA-USG", "000123", 0x05080100});
  feed_status(first, k_all_ok & static_cast<std::uint16_t>(~SBG_ECOM_GENERAL_CPU_OK));
  now += milliseconds{1000};
  // CPU recovered (an explicit clear), main power failed (an active ERROR).
  feed_status(first, k_all_ok & static_cast<std::uint16_t>(~SBG_ECOM_GENERAL_MAIN_POWER_OK));
  feed_imu_short(first, 256 * 41 + 128);  // 41.5 °C
  feed_ekf(first, k_ekf_nav_position);
  feed_gnss(first, SBG_ECOM_GNSS_POS_TYPE_RTK_INT);
  feed_heading(first, 271.25F, 0.18F);
  now += milliseconds{250};
  first.note_log();
  out["active"] = sbg_driver::to_diagnostic_array(first.snapshot(), stamp());

  // The link dies: 6.5 s without a single log.
  now += milliseconds{6500};
  out["silent"] = sbg_driver::to_diagnostic_array(first.snapshot(), stamp());

  first.begin_transition("deactivating");
  first.enter_standby();
  now += milliseconds{30000};
  out["standby"] = sbg_driver::to_diagnostic_array(first.snapshot(), stamp());

  // The process restarts: a new publisher id, sequence from 1, a new inventory.
  HealthReporter second{clock, "c41f7a02-93de-4b65-8a10-6e2f5d9b3c88"};
  EXPECT_TRUE(second.configure(s));
  second.enter_active({"QUANTA-USG", "000123", 0x05080100});
  feed_status(second, k_all_ok, /*uptime=*/12);
  feed_imu_data(second, 0.0F);  // a valid zero
  feed_ekf(second, SBG_ECOM_SOL_MODE_AHRS | SBG_ECOM_SOL_ATTITUDE_VALID);
  feed_gnss(second, SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT);  // below the required rtk_fixed
  out["restart"] = sbg_driver::to_diagnostic_array(second.snapshot(), stamp());

  now += milliseconds{1000};
  feed_status(second, k_all_ok, /*uptime=*/13);
  feed_imu_data(second, std::numeric_limits<float>::quiet_NaN());  // reading lost
  feed_ekf(second, k_ekf_nav_position);
  feed_gnss(second, SBG_ECOM_GNSS_POS_TYPE_RTK_INT);
  out["temperature_unavailable"] = sbg_driver::to_diagnostic_array(second.snapshot(), stamp());

  now += milliseconds{1000};
  feed_status(second, k_all_ok, /*uptime=*/14);
  feed_imu_data(second, 63.25F);  // above the configured warn_above
  feed_ekf(second, k_ekf_nav_position);
  feed_gnss(second, SBG_ECOM_GNSS_POS_TYPE_RTK_INT);
  out["temperature_limit"] = sbg_driver::to_diagnostic_array(second.snapshot(), stamp());
  now += milliseconds{1000};
  feed_imu_data(second, 95.0F);
  out["temperature_error"] = sbg_driver::to_diagnostic_array(second.snapshot(), stamp());
  now += milliseconds{1000};
  feed_imu_data(second, std::numeric_limits<float>::infinity());
  out["temperature_error_unavailable"] =
    sbg_driver::to_diagnostic_array(second.snapshot(), stamp());
  return out;
}

TEST(DiagnosticArraySerialization, RoundTripsThroughRealCdr)
{
  for (const auto & [name, msg] : representative_reports()) {
    const auto bytes = serialize(msg);
    ASSERT_GT(bytes.size(), 4U) << name;
    // CDR encapsulation header: little-endian, no options.
    EXPECT_EQ(bytes[0], 0x00) << name;
    EXPECT_EQ(bytes[1], 0x01) << name;
    const auto back = deserialize(bytes);
    EXPECT_EQ(back, msg) << name;
    ASSERT_EQ(back.status.size(), msg.status.size());
    for (std::size_t i = 0; i < msg.status.size(); ++i) {
      EXPECT_EQ(back.status[i].level, msg.status[i].level) << name << " / " << msg.status[i].name;
    }
  }
  const auto reports = representative_reports();
  const auto & silent = reports.at("silent");
  EXPECT_EQ(find_status(silent, "ins: alert/MAIN_POWER")->level, DiagnosticStatus::STALE);
  EXPECT_EQ(value_of(silent, "ins: alert/MAIN_POWER", "health.last_level"), "2");
  EXPECT_EQ(find_status(silent, "ins: stream")->level, DiagnosticStatus::ERROR);
}

// The committed fixture is what the dashboard-side check decodes; this keeps
// it in step with what the code produces today. Compared after decoding, so
// it does not depend on the serializer's padding bytes. Regenerate with
//   SBG_HEALTH_FIXTURE_UPDATE=1 build/sbg_driver/test_health_publisher
TEST(DiagnosticArraySerialization, MatchesTheCommittedDashboardFixture)
{
  const auto reports = representative_reports();
  const std::string path = SBG_HEALTH_FIXTURE_PATH;
  if (std::getenv("SBG_HEALTH_FIXTURE_UPDATE") != nullptr) {
    std::ofstream file{path};
    file << "{\n";
    for (auto it = reports.begin(); it != reports.end(); ++it) {
      file << "  \"" << it->first << "\": \"" << base64(serialize(it->second)) << "\""
           << (std::next(it) == reports.end() ? "\n" : ",\n");
    }
    file << "}\n";
    SUCCEED() << "wrote " << path;
    return;
  }

  std::ifstream file{path};
  ASSERT_TRUE(file.good()) << "missing " << path << " — run with SBG_HEALTH_FIXTURE_UPDATE=1";
  std::map<std::string, DiagnosticArray> committed;
  const std::regex entry{R"re(^\s*"([a-z_]+)":\s*"([A-Za-z0-9+/=]+)",?\s*$)re"};
  for (std::string line; std::getline(file, line);) {
    if (std::smatch match; std::regex_match(line, match, entry)) {
      committed[match[1]] = deserialize(unbase64(match[2].str()));
    }
  }
  ASSERT_EQ(committed.size(), reports.size());
  for (const auto & [name, msg] : reports) {
    ASSERT_TRUE(committed.contains(name)) << name;
    EXPECT_EQ(committed.at(name), msg)
      << name
      << ": the health report changed — regenerate the fixture (SBG_HEALTH_FIXTURE_UPDATE=1) "
      << "and re-run tools/check_dashboard_health.sh";
  }
}

// ---- the heartbeat thread on a lifecycle node ---------------------------------

class HealthPublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    // Left unconfigured on purpose: health must not depend on the lifecycle
    // state the way the driver's LifecyclePublishers do.
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "health_publisher_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    exec_.add_node(node_->get_node_base_interface());
    sub_ = node_->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(50).reliable(),
      [this](DiagnosticArray::ConstSharedPtr msg) { received_.push_back(*msg); });
  }

  void TearDown() override
  {
    publisher_.reset();
    sub_.reset();
    exec_.remove_node(node_->get_node_base_interface());
    node_.reset();
  }

  void start(
    double interval_s, double stale_after_s, sbg_driver::HealthPublisher::Counters counters = {})
  {
    ASSERT_TRUE(reporter_.configure(settings(interval_s, stale_after_s)));
    publisher_ =
      std::make_unique<sbg_driver::HealthPublisher>(*node_, reporter_, std::move(counters));
  }

  bool spin_until(const std::function<bool()> & done, milliseconds timeout = milliseconds{3000})
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some(milliseconds{20});
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(milliseconds{5});
    }
    return done();
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_;
  HealthReporter reporter_;
  std::unique_ptr<sbg_driver::HealthPublisher> publisher_;
  std::vector<DiagnosticArray> received_;
};

TEST_F(HealthPublisherTest, HeartbeatsOnDiagnosticsReliableVolatileWithSystemStamps)
{
  reporter_.enter_standby();
  start(0.1, 5.0);
  ASSERT_TRUE(spin_until([this] { return received_.size() >= 4; }));

  const auto info = node_->get_publishers_info_by_topic("/diagnostics");
  ASSERT_EQ(info.size(), 1U);
  EXPECT_EQ(info[0].topic_type(), "diagnostic_msgs/msg/DiagnosticArray");
  EXPECT_EQ(info[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(info[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);

  const auto wall_now = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
  std::int64_t previous = 0;
  for (const auto & msg : received_) {
    // Every publication is the complete inventory, in standby too.
    for (const char * name :
         {"ins: service", "ins: stream", "ins: device", "ins: sensor alerts", "ins: temperature",
          "ins: ekf"}) {
      EXPECT_NE(find_status(msg, name), nullptr) << name;
    }
    EXPECT_EQ(value_of(msg, "ins: service", "state"), "standby");
    EXPECT_EQ(value_of(msg, "ins: service", "health.publish_interval_s"), "0.1");
    const auto sequence = std::stoll(*value_of(msg, "ins: service", "health.sequence"));
    EXPECT_GT(sequence, previous);
    previous = sequence;
    const auto age = wall_now - rclcpp::Time{msg.header.stamp, RCL_SYSTEM_TIME};
    EXPECT_LT(std::abs(age.seconds()), 10.0);
  }
}

TEST_F(HealthPublisherTest, SamplesTheStreamCountersAtEachPublication)
{
  reporter_.enter_active({});
  // The publisher's shutdown heartbeat can run in TearDown, after this test's
  // locals have gone away. Its counter source must own the sampled state.
  const auto drops = std::make_shared<std::atomic<std::uint64_t>>(7);
  start(0.1, 5.0, [drops] {
    return HealthReporter::StreamCounters{.composition_drops = drops->load()};
  });
  ASSERT_TRUE(spin_until([this] { return !received_.empty(); }));
  EXPECT_EQ(value_of(received_.back(), "ins: stream", "composition_drops"), "7");
  *drops = 9;
  const auto updated = [this] {
    return value_of(received_.back(), "ins: stream", "composition_drops") == "9";
  };
  ASSERT_TRUE(spin_until(updated));
}

TEST_F(HealthPublisherTest, PublishSoonDoesNotWaitForTheNextHeartbeat)
{
  reporter_.enter_standby();
  start(30.0, 120.0);
  ASSERT_TRUE(spin_until([this] { return received_.size() == 1; }));

  reporter_.enter_standby("activation failed: device open failed");
  publisher_->publish_soon();
  ASSERT_TRUE(spin_until([this] { return received_.size() == 2; }, milliseconds{2000}));
  const auto * service = find_status(received_.back(), "ins: service");
  ASSERT_NE(service, nullptr);
  EXPECT_EQ(service->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(service->message, "activation failed: device open failed");
}

TEST_F(HealthPublisherTest, StopsPromptlyAndLeavesTheFinalStateOnTheWire)
{
  reporter_.enter_active({});
  start(30.0, 120.0);
  ASSERT_TRUE(spin_until([this] { return received_.size() == 1; }));

  reporter_.enter_shutdown();
  const auto began = std::chrono::steady_clock::now();
  publisher_.reset();  // must not sit out the 30 s interval
  EXPECT_LT(std::chrono::steady_clock::now() - began, milliseconds{1000});

  ASSERT_TRUE(spin_until([this] { return received_.size() == 2; }));
  EXPECT_EQ(value_of(received_.back(), "ins: service", "state"), "shutdown");
  EXPECT_EQ(value_of(received_.back(), "ins: stream", "health.availability"), "paused");
  // Nothing further once the publisher is gone.
  EXPECT_FALSE(spin_until([this] { return received_.size() > 2; }, milliseconds{300}));
}

}  // namespace
