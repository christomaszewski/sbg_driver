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

#include "sbg_driver/health.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"

// HealthReporter against an injected monotonic clock: what each component
// says for valid, missing, invalid, unsupported and zero readings, how
// observations age through a silent stream and recover, and how faults are
// raised, kept and cleared.

namespace
{

using sbg_driver::HealthComponent;
using sbg_driver::HealthLevel;
using sbg_driver::HealthReporter;
using sbg_driver::HealthSettings;
using std::chrono::milliseconds;

constexpr std::uint16_t k_all_ok = 0x007F;
constexpr std::uint16_t k_main_power = 1U << 0;
constexpr std::uint16_t k_temperature = 1U << 4;
constexpr std::uint16_t k_cpu = 1U << 6;

// EkfNav.status: solution mode in the low nibble, validity bits above.
constexpr std::uint32_t k_ekf_nav_position = 4U | (0xFU << 4);

class HealthTest : public ::testing::Test
{
protected:
  static HealthSettings settings()
  {
    HealthSettings s;
    s.vehicle_id = "uav6";
    s.instance = "ins";
    s.transport = "udp";
    return s;  // 1 s heartbeat, 5 s validity, 2 s stream timeout
  }

  void SetUp() override { make(settings()); }

  void make(const HealthSettings & s, std::string publisher_id = {})
  {
    reporter_ = std::make_unique<HealthReporter>([this] { return now_; }, std::move(publisher_id));
    ASSERT_TRUE(reporter_->configure(s));
  }

  void advance(milliseconds by) { now_ += by; }

  void activate(HealthReporter::DeviceIdentity identity = {"QUANTA-USG", "000123", 0x05080100})
  {
    reporter_->enter_active(std::move(identity));
  }

  void status(std::uint16_t general = k_all_ok, std::uint32_t uptime_s = 100, std::uint8_t cpu = 12)
  {
    reporter_->note_log();
    reporter_->observe_status({.general = general, .uptime_s = uptime_s, .cpu_usage_pct = cpu});
  }

  void imu(double celsius)
  {
    reporter_->note_log();
    reporter_->observe_imu_status(0x03FF);
    reporter_->observe_imu_temperature(celsius);
  }

  void ekf(std::uint32_t word = k_ekf_nav_position)
  {
    reporter_->note_log();
    reporter_->observe_ekf(word);
  }

  // One healthy sample of every expected source.
  void feed_all(double celsius = 41.5)
  {
    status();
    imu(celsius);
    ekf();
  }

  void snap(HealthReporter::StreamCounters counters = {}) { last_ = reporter_->snapshot(counters); }

  [[nodiscard]] const HealthComponent * find(std::string_view component) const
  {
    const std::string name = "ins: " + std::string{component};
    for (const auto & c : last_) {
      if (c.name == name) {
        return &c;
      }
    }
    return nullptr;
  }

  [[nodiscard]] const HealthComponent & get(std::string_view component) const
  {
    const auto * c = find(component);
    if (c == nullptr) {
      ADD_FAILURE() << "no component '" << component << "'";
      static const HealthComponent missing{};
      return missing;
    }
    return *c;
  }

  // The KeyValue string, or nullopt when the component does not carry the key.
  [[nodiscard]] std::optional<std::string> value(
    std::string_view component, std::string_view key) const
  {
    const auto * v = get(component).find(key);
    return v == nullptr ? std::nullopt : std::optional{sbg_driver::to_diagnostic_string(*v)};
  }

  HealthReporter::TimePoint now_{std::chrono::hours{1}};
  std::unique_ptr<HealthReporter> reporter_;
  std::vector<HealthComponent> last_;
};

// ---- identity, inventory, freshness metadata -------------------------------

TEST_F(HealthTest, EveryComponentCarriesFleetIdentityAndFreshness)
{
  activate();
  feed_all();
  snap();

  std::set<std::string> names;
  for (const auto & c : last_) {
    names.insert(c.name);
    EXPECT_TRUE(c.name.starts_with("ins: ")) << c.name;
    for (const char * key :
         {"health.vehicle_id", "health.instance", "health.service", "health.publisher_id",
          "health.sequence", "health.publish_interval_s", "health.sample_id", "health.sample_age_s",
          "health.stale_after_s", "health.availability"}) {
      EXPECT_NE(c.find(key), nullptr) << c.name << " lacks " << key;
    }
    EXPECT_EQ(std::get<std::string>(*c.find("health.vehicle_id")), "uav6");
    EXPECT_EQ(std::get<std::string>(*c.find("health.instance")), "ins");
    EXPECT_EQ(std::get<std::string>(*c.find("health.service")), "sbg");
    EXPECT_EQ(std::get<std::string>(*c.find("health.publisher_id")), reporter_->publisher_id());
    EXPECT_EQ(std::get<std::string>(*c.find("health.availability")), "current") << c.name;
    EXPECT_EQ(c.level, HealthLevel::Ok) << c.name << ": " << c.message;
    EXPECT_EQ(c.find("health.error"), nullptr) << c.name;
    EXPECT_EQ(c.find("health.last_level"), nullptr) << c.name;
  }
  // GNSS components are optional: absent until the device streams those logs.
  const std::set<std::string> expected_names{
    "ins: service",     "ins: stream", "ins: device", "ins: sensor alerts",
    "ins: temperature", "ins: ekf",    "ins: imu"};
  EXPECT_EQ(names, expected_names);
}

TEST_F(HealthTest, HardwareIdIsTheDeviceForDeviceComponentsAndEmptyForSoftware)
{
  activate();
  feed_all();
  snap();
  EXPECT_EQ(get("service").hardware_id, "");
  EXPECT_EQ(get("stream").hardware_id, "");
  for (const char * c : {"device", "sensor alerts", "temperature", "ekf"}) {
    EXPECT_EQ(get(c).hardware_id, "QUANTA-USG 000123") << c;
  }
  EXPECT_EQ(value("device", "device.serial_number"), "000123");
  EXPECT_EQ(value("device", "device.firmware"), "0x05080100");
}

TEST_F(HealthTest, UnidentifiedDeviceHasNoHardwareId)
{
  activate({});  // file replay, or an interface that does not answer commands
  feed_all();
  snap();
  EXPECT_EQ(get("device").hardware_id, "");
  EXPECT_EQ(get("temperature").hardware_id, "");
  EXPECT_EQ(get("device").find("device.serial_number"), nullptr);
}

TEST_F(HealthTest, SequenceAdvancesPerPublicationWhileCachedSamplesKeepTheirId)
{
  activate();
  feed_all();
  snap();
  const auto id = value("temperature", "health.sample_id");
  const auto device_id = value("device", "health.sample_id");
  EXPECT_EQ(value("temperature", "health.sequence"), "1");
  EXPECT_EQ(value("temperature", "health.sample_age_s"), "0");

  advance(milliseconds{1500});
  reporter_->note_log();  // the stream is alive, but no new IMU or STATUS log
  snap();
  EXPECT_EQ(value("temperature", "health.sequence"), "2");
  EXPECT_EQ(value("service", "health.sequence"), "2");
  // Republishing a cached observation does not make it new or fresh.
  EXPECT_EQ(value("temperature", "health.sample_id"), id);
  EXPECT_EQ(value("temperature", "health.sample_age_s"), "1.5");
  EXPECT_EQ(value("temperature", "temp.imu_c"), "41.5");
  EXPECT_EQ(value("device", "health.sample_id"), device_id);

  imu(42.0);
  snap();
  EXPECT_NE(value("temperature", "health.sample_id"), id);
  EXPECT_EQ(value("temperature", "health.sample_age_s"), "0");
  EXPECT_EQ(value("device", "health.sample_id"), device_id);  // independent sources
  EXPECT_EQ(value("device", "health.sample_age_s"), "1.5");
}

TEST_F(HealthTest, PublisherRestartIsANewIdentityWithItsOwnSequence)
{
  const std::string first = reporter_->publisher_id();
  static const std::regex uuid{
    "[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}"};
  EXPECT_TRUE(std::regex_match(first, uuid)) << first;
  snap();
  snap();
  EXPECT_EQ(value("service", "health.sequence"), "2");

  make(settings());  // the process restarted
  EXPECT_NE(reporter_->publisher_id(), first);
  snap();
  EXPECT_EQ(value("service", "health.sequence"), "1");
  EXPECT_EQ(value("service", "health.publisher_id"), reporter_->publisher_id());
}

TEST_F(HealthTest, ReconfigureKeepsThePublisherAndItsSequence)
{
  // cleanup → configure re-applies settings on the same process: a consumer
  // that saw sequence N must not be handed N-1 under the same publisher id.
  const std::string id = reporter_->publisher_id();
  snap();
  snap();
  auto s = settings();
  s.instance = "ins2";
  ASSERT_TRUE(reporter_->configure(s));
  last_ = reporter_->snapshot();
  ASSERT_FALSE(last_.empty());
  EXPECT_EQ(last_.front().name, "ins2: service");
  EXPECT_EQ(sbg_driver::to_diagnostic_string(*last_.front().find("health.sequence")), "3");
  EXPECT_EQ(reporter_->publisher_id(), id);
}

// ---- measurements: valid, zero, invalid, unsupported, missing ---------------

TEST_F(HealthTest, ZeroIsAMeasurementAndVendorSentinelsAreNot)
{
  activate();
  status(k_all_ok, /*uptime_s=*/0, /*cpu=*/0);
  imu(0.0);
  snap();
  EXPECT_EQ(value("temperature", "temp.imu_c"), "0");
  EXPECT_EQ(get("temperature").level, HealthLevel::Ok);
  EXPECT_EQ(get("temperature").find("health.metric.temp.imu_c.state"), nullptr);
  EXPECT_EQ(value("device", "cpu.usage_pct"), "0");    // a real 0 %
  EXPECT_EQ(get("device").find("uptime_s"), nullptr);  // 0 = not available
  status(k_all_ok, 7, 0xFF);
  snap();
  EXPECT_EQ(value("device", "uptime_s"), "7");
  EXPECT_EQ(get("device").find("cpu.usage_pct"), nullptr);  // 0xFF = not available
}

TEST_F(HealthTest, NegativeTemperatureIsPublishedAsMeasured)
{
  activate();
  imu(-12.3456);
  snap();
  EXPECT_EQ(value("temperature", "temp.imu_c"), "-12.346");
}

TEST_F(HealthTest, NonFiniteTemperatureKeepsLastValidObservationUntilRecovery)
{
  auto s = settings();
  s.temp_imu_c.error_above = 85;
  make(s);
  activate();
  feed_all(95.0);
  snap();
  const auto sample = value("temperature", "health.sample_id");
  for (double bad :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    advance(milliseconds{1000});
    imu(bad);
    snap();
    EXPECT_EQ(get("temperature").level, HealthLevel::Stale);
    EXPECT_EQ(value("temperature", "health.last_level"), "2");
    EXPECT_EQ(value("temperature", "temp.imu_c"), "95");
    EXPECT_EQ(value("temperature", "health.sample_id"), sample);
    EXPECT_NE(value("temperature", "health.sample_age_s"), "0");
    EXPECT_EQ(value("temperature", "health.metric.temp.imu_c.state"), "unavailable");
    EXPECT_EQ(value("temperature", "health.availability"), "stale");
    EXPECT_NE(get("temperature").find("health.error"), nullptr);
  }
  advance(milliseconds{6000});
  snap();
  EXPECT_EQ(value("temperature", "health.last_level"), "2");
  reporter_->enter_standby();
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Error);
  EXPECT_EQ(value("temperature", "health.availability"), "paused");
  activate();
  imu(0.0);
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Ok);
  EXPECT_EQ(value("temperature", "temp.imu_c"), "0");
  EXPECT_NE(value("temperature", "health.sample_id"), sample);
  EXPECT_EQ(value("temperature", "health.sample_age_s"), "0");
  EXPECT_EQ(get("temperature").find("health.error"), nullptr);
  EXPECT_EQ(get("temperature").find("health.metric.temp.imu_c.state"), nullptr);
}

TEST_F(HealthTest, InvalidFirstTemperatureHasNoInventedObservation)
{
  activate();
  imu(std::numeric_limits<double>::quiet_NaN());
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Warn);
  EXPECT_EQ(value("temperature", "temp.imu_c"), "null");
  EXPECT_EQ(value("temperature", "health.availability"), "unavailable");
  const auto sample = value("temperature", "health.sample_id");
  advance(milliseconds{1000});
  imu(std::numeric_limits<double>::infinity());
  snap();
  EXPECT_EQ(value("temperature", "health.sample_id"), sample);
}

TEST_F(HealthTest, ImuSelfTestFaultSurvivesSilenceAndClearsOnlyOnHealthyStatus)
{
  activate();
  feed_all();
  for (unsigned bit = 0; bit < 10; ++bit) {
    reporter_->observe_imu_status(static_cast<std::uint16_t>(0x03FFU & ~(1U << bit)));
    snap();
    EXPECT_EQ(get("imu").level, HealthLevel::Error);
    EXPECT_NE(get("imu").find("imu.failed_checks"), nullptr);
  }
  const auto sample = value("imu", "health.sample_id");
  advance(milliseconds{6000});
  snap();
  EXPECT_EQ(get("imu").level, HealthLevel::Stale);
  EXPECT_EQ(value("imu", "health.last_level"), "2");
  EXPECT_EQ(value("imu", "health.sample_id"), sample);
  reporter_->observe_imu_status(0x07FF);  // high gyro scale is not a fault
  reporter_->note_log();
  snap();
  EXPECT_EQ(get("imu").level, HealthLevel::Ok);
  EXPECT_EQ(get("imu").find("imu.failed_checks"), nullptr);
}

TEST_F(HealthTest, ElectricalMetricsAreDeclaredUnsupportedAndNeverInvented)
{
  activate();
  feed_all();
  snap();
  for (const char * metric : {"supply.voltage_v", "supply.current_a", "supply.power_w"}) {
    EXPECT_EQ(value("device", std::string{"health.metric."} + metric + ".state"), "unsupported");
    for (const auto & c : last_) {
      EXPECT_EQ(c.find(metric), nullptr) << c.name << " invented " << metric;
    }
  }
  EXPECT_EQ(get("device").level, HealthLevel::Ok);  // unsupported is informational
}

TEST_F(HealthTest, MissingObservationsAreNotHealthy)
{
  activate();
  reporter_->note_log();  // streaming, but none of the health-bearing logs
  snap();
  for (const char * c : {"device", "sensor alerts", "temperature", "ekf"}) {
    EXPECT_EQ(get(c).level, HealthLevel::Stale) << c;
    EXPECT_EQ(value(c, "health.availability"), "unavailable") << c;
    EXPECT_EQ(get(c).find("health.last_level"), nullptr) << c;  // nothing was ever known
    EXPECT_EQ(get(c).find("health.error"), nullptr) << c << ": still within the grace period";
  }
  EXPECT_EQ(get("temperature").find("temp.imu_c"), nullptr);
  EXPECT_EQ(get("sensor alerts").find("alerts.active"), nullptr);  // unknown, not zero

  advance(milliseconds{5100});
  reporter_->note_log();
  snap();
  for (const char * c : {"device", "temperature", "ekf"}) {
    ASSERT_NE(get(c).find("health.error"), nullptr) << c;
    EXPECT_NE(value(c, "health.error")->find("since activation"), std::string::npos) << c;
    EXPECT_EQ(get(c).level, HealthLevel::Stale) << c;
  }
  EXPECT_EQ(value("temperature", "temp.imu_c"), "null");
  EXPECT_EQ(value("temperature", "health.metric.temp.imu_c.state"), "unavailable");
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);  // the link itself is fine
}

TEST_F(HealthTest, TemperatureFailureDoesNotSuppressPowerOrDeviceData)
{
  activate();
  status(k_all_ok & static_cast<std::uint16_t>(~k_main_power));
  imu(std::numeric_limits<double>::quiet_NaN());
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Warn);
  // The supply verdict comes from the STATUS log and is untouched.
  EXPECT_EQ(get("alert/MAIN_POWER").level, HealthLevel::Error);
  EXPECT_EQ(value("alert/MAIN_POWER", "health.availability"), "current");
  EXPECT_EQ(value("device", "device.general_status"), "0x007e");
  EXPECT_EQ(value("device", "uptime_s"), "100");

  // And the other way round: no IMU log at all leaves the device data current.
  make(settings());
  activate();
  status();
  advance(milliseconds{6000});
  status();
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Stale);
  EXPECT_EQ(get("device").level, HealthLevel::Ok);
  EXPECT_EQ(value("device", "health.availability"), "current");
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "0");
}

// ---- producer-side limits ---------------------------------------------------

TEST_F(HealthTest, NoTemperatureLimitsApplyUnlessConfigured)
{
  activate();
  for (double t : {-55.0, 0.0, 95.0, 140.0}) {
    imu(t);
    snap();
    EXPECT_EQ(get("temperature").level, HealthLevel::Ok) << t;
    EXPECT_EQ(get("temperature").message, "OK");
  }
}

TEST_F(HealthTest, ConfiguredLimitsTravelAsLevelAndMessage)
{
  auto s = settings();
  s.temp_imu_c = {
    .warn_above = 70.0, .warn_below = -20.0, .error_above = 85.0, .error_below = -40.0};
  make(s);
  activate();
  const auto level_at = [this](double t) {
    imu(t);
    snap();
    return get("temperature").level;
  };
  EXPECT_EQ(level_at(25.0), HealthLevel::Ok);
  EXPECT_EQ(level_at(70.0), HealthLevel::Ok);  // bounds are exclusive
  EXPECT_EQ(level_at(70.5), HealthLevel::Warn);
  EXPECT_NE(get("temperature").message.find("warn_above 70"), std::string::npos)
    << get("temperature").message;
  EXPECT_EQ(level_at(85.1), HealthLevel::Error);
  EXPECT_NE(get("temperature").message.find("error_above 85"), std::string::npos);
  EXPECT_EQ(level_at(-20.5), HealthLevel::Warn);
  EXPECT_EQ(level_at(-40.5), HealthLevel::Error);
  EXPECT_EQ(value("temperature", "temp.imu_c"), "-40.5");  // the reading is never clamped
  EXPECT_EQ(level_at(25.0), HealthLevel::Ok);
}

// ---- device status flags as alerts -----------------------------------------

TEST_F(HealthTest, FaultIsRaisedKeptAndExplicitlyCleared)
{
  activate();
  status();
  snap();
  EXPECT_EQ(find("alert/MAIN_POWER"), nullptr);  // never seen: not in the inventory
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "0");
  EXPECT_EQ(get("sensor alerts").level, HealthLevel::Ok);

  status(k_all_ok & static_cast<std::uint16_t>(~(k_main_power | k_cpu)));
  snap();
  const auto & power = get("alert/MAIN_POWER");
  EXPECT_EQ(power.level, HealthLevel::Error);
  EXPECT_EQ(power.message, "main power supply fault");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.code"), "MAIN_POWER");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.category"), "power");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.active"), "true");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.severity"), "ERROR");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.flag"), "SBG_ECOM_GENERAL_MAIN_POWER_OK");
  EXPECT_NE(
    value("alert/MAIN_POWER", "alert.detail")->find("main power supply"), std::string::npos);
  EXPECT_EQ(get("alert/CPU").level, HealthLevel::Warn);
  EXPECT_EQ(value("alert/CPU", "alert.severity"), "WARNING");
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "2");
  EXPECT_EQ(get("sensor alerts").level, HealthLevel::Error);  // the worst active one
  EXPECT_EQ(get("device").level, HealthLevel::Ok);            // faults live on the alerts

  status(k_all_ok & static_cast<std::uint16_t>(~k_cpu));  // main power recovered
  snap();
  const auto & cleared = get("alert/MAIN_POWER");  // retained, explicitly OK
  EXPECT_EQ(cleared.level, HealthLevel::Ok);
  EXPECT_EQ(cleared.message, "cleared");
  EXPECT_EQ(value("alert/MAIN_POWER", "alert.active"), "false");
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "1");
  EXPECT_EQ(get("sensor alerts").level, HealthLevel::Warn);

  status();
  snap();
  EXPECT_EQ(value("alert/CPU", "alert.active"), "false");
  EXPECT_EQ(get("sensor alerts").level, HealthLevel::Ok);
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "0");
  EXPECT_NE(find("alert/MAIN_POWER"), nullptr);  // known for the reporter's lifetime
}

TEST_F(HealthTest, SilenceCannotClearAFaultAndStaleErrorKeepsItsSeverity)
{
  activate();
  feed_all();
  status(k_all_ok & static_cast<std::uint16_t>(~k_temperature));
  snap();
  ASSERT_EQ(get("alert/TEMPERATURE").level, HealthLevel::Error);
  const auto sample = value("alert/TEMPERATURE", "health.sample_id");

  // The whole stream goes quiet: first the cached fault is "retrying"...
  advance(milliseconds{3000});
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Error);
  EXPECT_EQ(get("alert/TEMPERATURE").level, HealthLevel::Error);
  EXPECT_EQ(value("alert/TEMPERATURE", "health.availability"), "retrying");
  EXPECT_EQ(value("alert/TEMPERATURE", "alert.active"), "true");
  EXPECT_EQ(value("alert/TEMPERATURE", "health.sample_id"), sample);
  EXPECT_NE(value("alert/TEMPERATURE", "health.error")->find("stream silent"), std::string::npos);
  EXPECT_EQ(value("temperature", "health.availability"), "retrying");
  EXPECT_EQ(value("temperature", "temp.imu_c"), "41.5");  // last known, kept

  // ...then STALE, still carrying the ERROR it last had.
  advance(milliseconds{3000});
  snap();
  const auto & alert = get("alert/TEMPERATURE");
  EXPECT_EQ(alert.level, HealthLevel::Stale);
  EXPECT_EQ(value("alert/TEMPERATURE", "health.last_level"), "2");
  EXPECT_EQ(value("alert/TEMPERATURE", "health.availability"), "stale");
  EXPECT_EQ(value("alert/TEMPERATURE", "alert.active"), "true");
  EXPECT_EQ(value("alert/TEMPERATURE", "health.sample_age_s"), "6");
  EXPECT_EQ(value("sensor alerts", "health.last_level"), "2");
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "1");
  EXPECT_EQ(get("temperature").level, HealthLevel::Stale);
  EXPECT_EQ(value("temperature", "health.last_level"), "0");
  EXPECT_EQ(value("temperature", "temp.imu_c"), "41.5");
  EXPECT_EQ(value("ekf", "health.last_level"), "0");
  // The stream fault itself is current knowledge, not stale.
  EXPECT_EQ(get("stream").level, HealthLevel::Error);
  EXPECT_EQ(value("stream", "health.availability"), "current");

  // Recovery: a new log refreshes exactly the component it carries.
  status();
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  EXPECT_EQ(get("alert/TEMPERATURE").level, HealthLevel::Ok);
  EXPECT_EQ(value("alert/TEMPERATURE", "alert.active"), "false");
  EXPECT_EQ(value("alert/TEMPERATURE", "health.availability"), "current");
  EXPECT_NE(value("alert/TEMPERATURE", "health.sample_id"), sample);
  EXPECT_EQ(get("alert/TEMPERATURE").find("health.last_level"), nullptr);
  EXPECT_EQ(get("alert/TEMPERATURE").find("health.error"), nullptr);
  EXPECT_EQ(get("temperature").level, HealthLevel::Stale);  // no IMU log yet
}

TEST_F(HealthTest, OneLogGoingQuietStalesOnlyItsOwnComponents)
{
  activate();
  feed_all();
  for (int i = 0; i < 6; ++i) {
    advance(milliseconds{1000});
    imu(41.0);
    ekf();
  }
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  EXPECT_EQ(get("temperature").level, HealthLevel::Ok);
  EXPECT_EQ(get("ekf").level, HealthLevel::Ok);
  EXPECT_EQ(get("device").level, HealthLevel::Stale);
  EXPECT_EQ(value("device", "health.last_level"), "0");
  EXPECT_NE(value("device", "health.error")->find("SBG_ECOM_LOG_STATUS"), std::string::npos);
  EXPECT_EQ(value("device", "uptime_s"), "100");  // last known
}

TEST_F(HealthTest, IgnoredAlertsRaiseNothingButStayVisibleInTheRawWord)
{
  auto s = settings();
  s.ignored_alerts = {"DATALOGGER", "GPS_POWER"};
  make(s);
  activate();
  status(0x0001U | 0x0002U | 0x0008U | 0x0010U | 0x0040U);  // datalogger + gps power cleared
  snap();
  EXPECT_EQ(find("alert/DATALOGGER"), nullptr);
  EXPECT_EQ(find("alert/GPS_POWER"), nullptr);
  EXPECT_EQ(value("sensor alerts", "alerts.active"), "0");
  EXPECT_EQ(get("sensor alerts").level, HealthLevel::Ok);
  EXPECT_EQ(value("sensor alerts", "alerts.ignored"), "DATALOGGER,GPS_POWER");
  EXPECT_EQ(value("device", "device.general_status"), "0x005b");
}

// ---- stream -----------------------------------------------------------------

TEST_F(HealthTest, StreamWaitsThenFaultsWhenNothingArrivesAfterActivation)
{
  activate();
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Warn);  // not yet evidence of health
  EXPECT_EQ(value("stream", "logs.received"), "0");
  EXPECT_EQ(get("stream").find("frame_age_s"), nullptr);

  advance(milliseconds{2500});
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Error);
  EXPECT_NE(get("stream").message.find("since activation"), std::string::npos);

  reporter_->note_log();
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  EXPECT_EQ(get("stream").message, "receiving");
  EXPECT_EQ(value("stream", "logs.received"), "1");
  EXPECT_EQ(value("stream", "frame_age_s"), "0");
}

TEST_F(HealthTest, CompositionDropsWarnWhileTheyGrowAndDecayWhenTheyStop)
{
  activate();
  feed_all();
  snap({.composition_drops = 0});
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);

  advance(milliseconds{1000});
  feed_all();
  snap({.composition_drops = 3, .imu_orientation_misses = 2, .imu_short_in_use = true});
  EXPECT_EQ(get("stream").level, HealthLevel::Warn);
  EXPECT_EQ(value("stream", "composition_drops"), "3");
  EXPECT_EQ(value("stream", "imu_orientation_misses"), "2");
  EXPECT_EQ(value("stream", "imu_source"), "imu_short");

  advance(milliseconds{6000});  // no further drops for longer than the validity window
  feed_all();
  snap({.composition_drops = 3});
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  EXPECT_EQ(value("stream", "composition_drops"), "3");  // the total is still reported

  // A reactivation resets the owner's counter; that is not growth.
  activate();
  feed_all();
  snap({.composition_drops = 0});
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
}

TEST_F(HealthTest, DeviceCommandPausesTheStreamInsteadOfFaultingIt)
{
  activate();
  feed_all();
  {
    const sbg_driver::HealthStreamPause pause{*reporter_, "a settings save command"};
    advance(milliseconds{4000});  // the I/O thread is parked for the command
    snap();
    EXPECT_EQ(get("stream").level, HealthLevel::Ok);
    EXPECT_EQ(value("stream", "health.availability"), "paused");
    EXPECT_NE(get("stream").message.find("settings save"), std::string::npos);
    EXPECT_EQ(value("device", "health.availability"), "paused");
    EXPECT_EQ(get("device").level, HealthLevel::Ok);
  }
  // Silence is measured from the resume, not from the last pre-command log.
  advance(milliseconds{500});
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  advance(milliseconds{2000});
  snap();
  EXPECT_EQ(get("stream").level, HealthLevel::Error);
}

// ---- operational state --------------------------------------------------------

TEST_F(HealthTest, StandbyIsHealthyAndKeepsLastKnownObservationsMarkedPaused)
{
  activate();
  feed_all();
  status(k_all_ok & static_cast<std::uint16_t>(~k_main_power));
  snap();
  const auto sample = value("temperature", "health.sample_id");

  reporter_->begin_transition("deactivating");
  snap();
  EXPECT_EQ(value("service", "state"), "transitioning");
  EXPECT_EQ(get("service").level, HealthLevel::Ok);  // a state, not a fault
  EXPECT_EQ(get("service").message, "deactivating");

  reporter_->enter_standby();
  advance(milliseconds{60000});
  snap();
  EXPECT_EQ(value("service", "state"), "standby");
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);
  EXPECT_EQ(value("stream", "health.availability"), "paused");
  // Nothing is demanded of the device, so nothing is stale — but nothing is
  // refreshed either, and a last-known fault is not forgotten.
  EXPECT_EQ(get("temperature").level, HealthLevel::Ok);
  EXPECT_EQ(value("temperature", "health.availability"), "paused");
  EXPECT_EQ(value("temperature", "temp.imu_c"), "41.5");
  EXPECT_EQ(value("temperature", "health.sample_id"), sample);
  EXPECT_EQ(value("temperature", "health.sample_age_s"), "60");
  EXPECT_EQ(get("temperature").find("health.error"), nullptr);
  EXPECT_EQ(get("alert/MAIN_POWER").level, HealthLevel::Error);
  EXPECT_EQ(value("alert/MAIN_POWER", "health.availability"), "paused");
}

TEST_F(HealthTest, StandbyBeforeAnyObservationRaisesNoIssue)
{
  reporter_->enter_standby();
  snap();
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
  for (const char * c : {"device", "sensor alerts", "temperature", "ekf"}) {
    EXPECT_EQ(get(c).level, HealthLevel::Stale) << c;           // no data is not OK...
    EXPECT_EQ(value(c, "health.availability"), "paused") << c;  // ...but none is expected
    EXPECT_EQ(get(c).find("health.error"), nullptr) << c;
  }
  EXPECT_EQ(get("temperature").find("health.metric.temp.imu_c.state"), nullptr);
}

TEST_F(HealthTest, FailedActivationIsAnErrorUntilTheNextTransition)
{
  reporter_->begin_transition("activating");
  reporter_->enter_standby("activation failed: device open failed: transport failure");
  snap();
  EXPECT_EQ(value("service", "state"), "standby");
  EXPECT_EQ(get("service").level, HealthLevel::Error);
  EXPECT_NE(get("service").message.find("device open failed"), std::string::npos);

  reporter_->begin_transition("activating");
  snap();
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
  activate();
  snap();
  EXPECT_EQ(value("service", "state"), "active");
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
}

TEST_F(HealthTest, NewSessionDoesNotPassOffThePreviousSessionsDataAsCurrent)
{
  activate();
  feed_all();
  reporter_->enter_standby();
  advance(milliseconds{1000});
  activate();
  reporter_->note_log();
  snap();
  // One second old would be within validity, but the unit may have rebooted.
  EXPECT_EQ(get("temperature").level, HealthLevel::Stale);
  EXPECT_EQ(value("temperature", "health.availability"), "stale");
  EXPECT_EQ(value("temperature", "health.last_level"), "0");
  EXPECT_EQ(value("temperature", "temp.imu_c"), "41.5");
  imu(39.0);
  snap();
  EXPECT_EQ(get("temperature").level, HealthLevel::Ok);
  EXPECT_EQ(value("temperature", "temp.imu_c"), "39");
}

TEST_F(HealthTest, ReleasedDeviceShutdownAndLifecycleError)
{
  activate();
  feed_all();
  reporter_->set_device_released();  // settings save rebooted the unit
  advance(milliseconds{10000});
  snap();
  EXPECT_EQ(value("service", "state"), "active");
  EXPECT_EQ(get("service").level, HealthLevel::Warn);
  EXPECT_NE(get("service").message.find("deactivate then activate"), std::string::npos);
  EXPECT_EQ(get("stream").level, HealthLevel::Ok);  // silence is expected here
  EXPECT_EQ(value("stream", "health.availability"), "paused");
  EXPECT_EQ(value("device", "health.availability"), "paused");

  reporter_->enter_shutdown();
  snap();
  EXPECT_EQ(value("service", "state"), "shutdown");
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
  EXPECT_EQ(value("temperature", "health.availability"), "paused");

  reporter_->enter_unconfigured("lifecycle error — the node was reset to unconfigured");
  snap();
  EXPECT_EQ(value("service", "state"), "unconfigured");
  EXPECT_EQ(get("service").level, HealthLevel::Error);
}

TEST_F(HealthTest, CallbackExceptionsWarnWhileTheyKeepHappening)
{
  activate();
  feed_all();
  snap();
  EXPECT_EQ(value("service", "callback_exceptions"), "0");
  reporter_->note_callback_exception();
  snap();
  EXPECT_EQ(get("service").level, HealthLevel::Warn);
  EXPECT_EQ(value("service", "callback_exceptions"), "1");
  advance(milliseconds{6000});
  snap();
  EXPECT_EQ(get("service").level, HealthLevel::Ok);
  EXPECT_EQ(value("service", "callback_exceptions"), "1");
}

// ---- EKF and GNSS ------------------------------------------------------------

TEST_F(HealthTest, EkfSolutionModeMapsToLevelAndFlagsAreExposed)
{
  activate();
  const auto at = [this](std::uint32_t word) {
    ekf(word);
    snap();
    return get("ekf").level;
  };
  EXPECT_EQ(at(0), HealthLevel::Error);
  EXPECT_EQ(value("ekf", "ekf.solution_mode"), "UNINITIALIZED");
  EXPECT_EQ(at(1U | 0x30U), HealthLevel::Warn);
  EXPECT_EQ(at(2U | 0x30U), HealthLevel::Warn);
  EXPECT_EQ(value("ekf", "ekf.solution_mode"), "AHRS");
  EXPECT_EQ(at(3U | 0x70U), HealthLevel::Ok);
  EXPECT_EQ(at(4U | 0xF0U | (1U << 11) | (1U << 13)), HealthLevel::Ok);
  EXPECT_EQ(get("ekf").message, "NAV_POSITION");
  EXPECT_EQ(value("ekf", "ekf.solution_mode"), "NAV_POSITION");
  EXPECT_EQ(value("ekf", "ekf.status"), "0x000028f4");
  EXPECT_EQ(value("ekf", "ekf.attitude_valid"), "true");
  EXPECT_EQ(value("ekf", "ekf.heading_valid"), "true");
  EXPECT_EQ(value("ekf", "ekf.position_valid"), "true");
  EXPECT_EQ(value("ekf", "ekf.gps1_pos_used"), "true");
  EXPECT_EQ(value("ekf", "ekf.gps1_hdt_used"), "true");
  EXPECT_EQ(value("ekf", "ekf.gps1_vel_used"), "false");
}

TEST_F(HealthTest, NavigationModeCannotHideInvalidEstimates)
{
  activate();
  for (const auto flag : {0x10U, 0x20U, 0x40U, 0x80U}) {
    ekf(0xF4U & ~flag);
    snap();
    EXPECT_EQ(get("ekf").level, flag == 0x20U ? HealthLevel::Warn : HealthLevel::Error);
  }
  ekf(0xF4U & ~0xA0U);  // invalid heading must not hide invalid position
  snap();
  EXPECT_EQ(get("ekf").level, HealthLevel::Error);
  advance(milliseconds{6000});
  snap();
  EXPECT_EQ(get("ekf").level, HealthLevel::Stale);
  EXPECT_EQ(value("ekf", "health.last_level"), "2");
  ekf();
  snap();
  EXPECT_EQ(get("ekf").level, HealthLevel::Ok);
}

TEST_F(HealthTest, GnssReportsTheFixGradeAndWarnsBelowTheRequiredOne)
{
  const auto fix = [this](std::uint8_t type, std::uint8_t status = 0) {
    reporter_->note_log();
    reporter_->observe_gnss(
      {.solution_status = status,
       .fix_type = type,
       .sv_used = 19,
       .differential_age_cs = 150,
       .base_station_id = 42,
       .latitude_accuracy_m = 0.012,
       .longitude_accuracy_m = 0.011,
       .altitude_accuracy_m = 0.02});
    snap();
    return get("gnss").level;
  };
  activate();
  feed_all();
  snap();
  EXPECT_EQ(find("gnss"), nullptr);  // optional until GPS1_POS is streamed

  // Default expectation: any computed position.
  EXPECT_EQ(fix(7), HealthLevel::Ok);
  EXPECT_EQ(get("gnss").message, "RTK fixed");
  EXPECT_EQ(value("gnss", "gnss.fix_type"), "RTK_INT");
  EXPECT_EQ(value("gnss", "gnss.rtk"), "fixed");
  EXPECT_EQ(value("gnss", "gnss.sv_used"), "19");
  EXPECT_EQ(value("gnss", "gnss.differential_age_s"), "1.5");
  EXPECT_EQ(value("gnss", "gnss.base_station_id"), "42");
  EXPECT_EQ(value("gnss", "gnss.latitude_accuracy_m"), "0.012");
  EXPECT_EQ(get("gnss").hardware_id, "QUANTA-USG 000123");
  EXPECT_EQ(fix(6), HealthLevel::Ok);
  EXPECT_EQ(value("gnss", "gnss.rtk"), "float");
  EXPECT_EQ(fix(2), HealthLevel::Ok);
  EXPECT_EQ(get("gnss").message, "SINGLE fix");
  EXPECT_EQ(value("gnss", "gnss.rtk"), "none");
  EXPECT_EQ(fix(0), HealthLevel::Warn);
  EXPECT_EQ(fix(7, /*status=*/1), HealthLevel::Warn);
  EXPECT_NE(get("gnss").message.find("INSUFFICIENT_OBS"), std::string::npos);
  EXPECT_EQ(value("gnss", "gnss.rtk"), "none");  // a type without a computed solution

  auto s = settings();
  s.required_gnss_fix = HealthSettings::GnssFix::RtkFixed;
  make(s);
  activate();
  EXPECT_EQ(fix(7), HealthLevel::Ok);
  EXPECT_EQ(fix(6), HealthLevel::Warn);
  EXPECT_NE(get("gnss").message.find("below the required rtk_fixed"), std::string::npos);
  EXPECT_EQ(value("gnss", "gnss.required_fix"), "rtk_fixed");
  EXPECT_EQ(fix(2), HealthLevel::Warn);

  s.required_gnss_fix = HealthSettings::GnssFix::RtkFloat;
  make(s);
  activate();
  EXPECT_EQ(fix(6), HealthLevel::Ok);
  EXPECT_EQ(fix(4), HealthLevel::Warn);

  s.required_gnss_fix = HealthSettings::GnssFix::None;
  make(s);
  activate();
  EXPECT_EQ(fix(0), HealthLevel::Ok);
  EXPECT_EQ(value("gnss", "gnss.fix_type"), "NO_SOLUTION");
}

TEST_F(HealthTest, GnssVendorSentinelsAreOmitted)
{
  activate();
  reporter_->observe_gnss({.solution_status = 0, .fix_type = 2});
  snap();
  EXPECT_EQ(get("gnss").find("gnss.sv_used"), nullptr);
  EXPECT_EQ(get("gnss").find("gnss.differential_age_s"), nullptr);
  EXPECT_EQ(get("gnss").find("gnss.base_station_id"), nullptr);
  EXPECT_EQ(value("gnss", "gnss.latitude_accuracy_m"), "0");
}

TEST_F(HealthTest, GnssHeadingAppearsWithTheLogAndFollowsItsValidity)
{
  activate();
  feed_all();
  snap();
  EXPECT_EQ(find("gnss heading"), nullptr);  // single-antenna units never get one

  reporter_->observe_gnss_heading(
    {.solution_status = 0,
     .heading_valid = true,
     .heading_deg = 271.25,
     .accuracy_deg = 0.18,
     .baseline_valid = true,
     .baseline_m = 1.204,
     .sv_used = 14});
  snap();
  EXPECT_EQ(get("gnss heading").level, HealthLevel::Ok);
  EXPECT_EQ(value("gnss heading", "gnss.heading_deg"), "271.25");
  EXPECT_EQ(value("gnss heading", "gnss.heading_accuracy_deg"), "0.18");
  EXPECT_EQ(value("gnss heading", "gnss.baseline_m"), "1.204");
  EXPECT_EQ(value("gnss heading", "gnss.heading_valid"), "true");

  reporter_->observe_gnss_heading({.solution_status = 1, .heading_valid = false});
  snap();
  EXPECT_EQ(get("gnss heading").level, HealthLevel::Warn);
  EXPECT_NE(get("gnss heading").message.find("INSUFFICIENT_OBS"), std::string::npos);
  EXPECT_EQ(get("gnss heading").find("gnss.heading_deg"), nullptr);  // not a stale number
  EXPECT_EQ(value("gnss heading", "gnss.heading_valid"), "false");

  advance(milliseconds{6000});
  reporter_->note_log();
  snap();
  EXPECT_EQ(get("gnss heading").level, HealthLevel::Stale);
  EXPECT_EQ(value("gnss heading", "health.last_level"), "1");
}

// ---- settings validation and value rendering ---------------------------------

TEST(HealthSettingsTest, ValidatesIdentityTimingLimitsAndAlertCodes)
{
  const auto base = [] {
    HealthSettings s;
    s.instance = "ins";
    return s;
  };
  EXPECT_TRUE(sbg_driver::validate(base()));  // an empty vehicle_id is allowed

  const auto rejects = [&](auto && mutate) {
    auto s = base();
    mutate(s);
    return !sbg_driver::validate(s).has_value();
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_TRUE(rejects([](auto & s) { s.instance = ""; }));
  EXPECT_TRUE(rejects([](auto & s) { s.instance = "a/b"; }));
  EXPECT_TRUE(rejects([](auto & s) { s.instance = "front cam"; }));
  EXPECT_TRUE(rejects([](auto & s) { s.vehicle_id = "uav*"; }));
  EXPECT_TRUE(rejects([](auto & s) { s.vehicle_id = "uav#6"; }));
  for (double bad : {0.0, -1.0, nan, inf}) {
    EXPECT_TRUE(rejects([bad](auto & s) { s.publish_interval_s = bad; })) << bad;
    EXPECT_TRUE(rejects([bad](auto & s) { s.stale_after_s = bad; })) << bad;
    EXPECT_TRUE(rejects([bad](auto & s) { s.stream_timeout_s = bad; })) << bad;
  }
  EXPECT_TRUE(rejects([](auto & s) { s.publish_interval_s = 3.0; }));  // > stale_after / 2
  EXPECT_TRUE(rejects([inf](auto & s) { s.temp_imu_c.warn_above = inf; }));
  EXPECT_TRUE(rejects([nan](auto & s) { s.temp_imu_c.error_below = nan; }));
  EXPECT_TRUE(rejects([](auto & s) { s.temp_imu_c = {.warn_above = 80.0, .error_above = 70.0}; }));
  EXPECT_TRUE(
    rejects([](auto & s) { s.temp_imu_c = {.warn_below = -30.0, .error_below = -10.0}; }));
  EXPECT_TRUE(rejects([](auto & s) { s.temp_imu_c = {.warn_above = 10.0, .warn_below = 20.0}; }));
  EXPECT_TRUE(rejects([](auto & s) { s.ignored_alerts = {"NOT_A_FLAG"}; }));

  auto ok = base();
  ok.vehicle_id = "uav-6.a_b";
  ok.temp_imu_c = {
    .warn_above = 70.0, .warn_below = -20.0, .error_above = 85.0, .error_below = -40.0};
  ok.ignored_alerts = {"CPU"};
  EXPECT_TRUE(sbg_driver::validate(ok));

  // A rejected reconfiguration leaves the reporter as it was.
  HealthReporter reporter;
  ASSERT_TRUE(reporter.configure(ok));
  auto bad = ok;
  bad.instance = "";
  EXPECT_FALSE(reporter.configure(bad));
  EXPECT_EQ(reporter.snapshot().front().name, "ins: service");
}

TEST(HealthValueTest, RendersScalarsForKeyValueStrings)
{
  using sbg_driver::HealthValue;
  using sbg_driver::to_diagnostic_string;
  EXPECT_EQ(to_diagnostic_string(HealthValue{std::string{"00123"}}), "00123");
  EXPECT_EQ(to_diagnostic_string(HealthValue{0.0}), "0");
  EXPECT_EQ(to_diagnostic_string(HealthValue{-0.5}), "-0.5");
  EXPECT_EQ(to_diagnostic_string(HealthValue{43.125}), "43.125");
  EXPECT_EQ(to_diagnostic_string(HealthValue{std::int64_t{42}}), "42");
  EXPECT_EQ(to_diagnostic_string(HealthValue{true}), "true");
  EXPECT_EQ(to_diagnostic_string(HealthValue{false}), "false");
  EXPECT_EQ(to_diagnostic_string(HealthValue{sbg_driver::HealthNull{}}), "null");
  // A non-finite double can never reach the wire as a number.
  EXPECT_EQ(to_diagnostic_string(HealthValue{std::numeric_limits<double>::quiet_NaN()}), "null");
}

}  // namespace
