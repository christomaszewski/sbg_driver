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

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "gtest/gtest.h"
#include "sbg_driver/output_rate_limiter.hpp"
#include "sbg_driver/publishers.hpp"

namespace
{

using sbg_driver::OutputRateLimiter;

TEST(OutputRateLimiter, UnlimitedPreservesEveryCandidate)
{
  OutputRateLimiter limiter;
  for (const auto stamp : {100, 100, 99, 101}) {
    EXPECT_TRUE(limiter.allow(stamp));
  }
}

TEST(OutputRateLimiter, UsesSampleTimeAndDoesNotCatchUpAfterGaps)
{
  OutputRateLimiter limiter(1.0);
  EXPECT_TRUE(limiter.allow(0));
  EXPECT_FALSE(limiter.allow(5'000));
  EXPECT_TRUE(limiter.allow(1'000'000));
  EXPECT_TRUE(limiter.allow(30'000'000));
  EXPECT_FALSE(limiter.allow(30'005'000));
  EXPECT_FALSE(limiter.allow(29'000'000));
  EXPECT_TRUE(limiter.allow(31'000'000));
}

TEST(OutputRateLimiter, LongGapDoesNotPublishAgainAtAnImmediatelyFollowingOldDeadline)
{
  OutputRateLimiter limiter(1.0);
  ASSERT_TRUE(limiter.allow(0));
  ASSERT_TRUE(limiter.allow(30'995'000));
  EXPECT_FALSE(limiter.allow(31'000'000));
  EXPECT_FALSE(limiter.allow(31'500'000));
  EXPECT_TRUE(limiter.allow(31'995'000));
}

TEST(OutputRateLimiter, PhaseSchedulingPreservesNominalRateWithTimestampJitter)
{
  OutputRateLimiter limiter(200.0);
  for (std::int64_t i = 0; i <= 200; ++i) {
    const auto jitter = i == 0 ? 0 : (i % 2 == 0 ? 20 : -20);
    EXPECT_TRUE(limiter.allow(1'000'000 + i * 5'000 + jitter)) << i;
  }
}

TEST(OutputRateLimiter, NonDivisorRateDoesNotAccumulateDownsamplingBias)
{
  OutputRateLimiter limiter(60.0);
  std::size_t published = 0;
  for (std::int64_t stamp = 0; stamp <= 10'000'000; stamp += 5'000) {
    published += limiter.allow(stamp) ? 1u : 0u;
  }
  EXPECT_GE(published, 600u);
  EXPECT_LE(published, 602u);
}

TEST(OutputRateLimiter, ResetPublishesTheFirstSampleOfANewSession)
{
  OutputRateLimiter limiter(1.0);
  ASSERT_TRUE(limiter.allow(1'000'000));
  ASSERT_FALSE(limiter.allow(1'005'000));
  limiter.reset();
  EXPECT_TRUE(limiter.allow(1'010'000));
  limiter.reset();
  EXPECT_TRUE(limiter.allow(100));
}

TEST(OutputRateLimiter, RejectsNegativeAndNonFiniteRates)
{
  for (const auto rate :
       {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(OutputRateLimiter{rate}, std::invalid_argument);
  }
}

TEST(OutputRateLimiter, ExtremeFiniteRatesRemainBoundedAtClockResolution)
{
  OutputRateLimiter fastest(std::numeric_limits<double>::max());
  EXPECT_TRUE(fastest.allow(0));
  EXPECT_FALSE(fastest.allow(0));
  EXPECT_TRUE(fastest.allow(1));
  EXPECT_TRUE(fastest.allow(2));
  for (const auto rate :
       {std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min()}) {
    OutputRateLimiter slowest(rate);
    EXPECT_TRUE(slowest.allow(0));
    EXPECT_FALSE(slowest.allow(1'000'000));
    EXPECT_FALSE(slowest.allow(std::numeric_limits<std::int64_t>::max() / 2));
    EXPECT_TRUE(slowest.allow(std::numeric_limits<std::int64_t>::max()));
  }
}

TEST(OutputSampleTime, SharedTimelineKeepsInfrequentTopicsAcrossMultipleWraps)
{
  sbg_driver::SampleTimeUnwrapper clock;
  constexpr std::int64_t first = 1'000'000;
  clock.observe(static_cast<std::uint32_t>(first));
  EXPECT_EQ(clock.resolve(static_cast<std::uint32_t>(first)), first);
  // Other streams keep the clock advancing while this topic remains silent.
  for (std::int64_t elapsed = 1'000'000'000; elapsed <= 9'000'000'000; elapsed += 1'000'000'000) {
    const auto stamp = first + elapsed;
    clock.observe(static_cast<std::uint32_t>(stamp));
  }
  EXPECT_EQ(
    clock.resolve(static_cast<std::uint32_t>(first + 9'000'000'000)), first + 9'000'000'000);
}

TEST(OutputSampleTime, DelayedSamplesResolveBehindCurrentClockWithoutRewindingIt)
{
  sbg_driver::SampleTimeUnwrapper clock;
  clock.observe(0xFFFFF000u);
  clock.observe(5'000u);
  clock.observe(0xFFFFF800u);
  EXPECT_EQ(clock.resolve(5'000u), 4'294'967'296LL + 5'000);
  EXPECT_EQ(clock.resolve(0xFFFFF800u), 4'294'965'248LL);
  clock.reset();
  clock.observe(100u);
  EXPECT_EQ(clock.resolve(100u), 100);
}

class OutputRatesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "output_rates_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    exec_.add_node(node_->get_node_base_interface());
    subscribe("imu/data", imus_);
    subscribe("imu/temperature", temperatures_);
    subscribe("odom", odometry_);
    subscribe("sbg/ekf_status", ekf_statuses_);
    subscribe("raw/orientation", raw_orientations_);
    subscribe("gps/heading", headings_);
    subscribe("time_reference", utc_references_);
    subscribe("nmea", nmea_);
    subscribe("/tf", transforms_);
  }

  void TearDown() override
  {
    pubs_.reset();
    subscriptions_.clear();
    exec_.remove_node(node_->get_node_base_interface());
    node_.reset();
  }

  template <typename MessageT>
  void subscribe(const std::string & topic, std::vector<MessageT> & messages)
  {
    auto callback = [&messages](typename MessageT::ConstSharedPtr message) {
      messages.push_back(*message);
    };
    subscriptions_.push_back(node_->create_subscription<MessageT>(
      topic, rclcpp::SensorDataQoS().keep_last(1000), callback));
  }

  void start(sbg_driver::Publishers::Config cfg = {})
  {
    // Device samples are intentionally replayed faster than wall time.
    cfg.device_clock_real_time = false;
    pubs_ = std::make_unique<sbg_driver::Publishers>(*node_, cfg);
    pubs_->activate();
  }

  void spin()
  {
    pubs_->flush_pending();
    // A spin_some() call services each entity only once, including internal
    // readiness waitables. Exhaust the queue so batched samples are all seen.
    exec_.spin_all(std::chrono::milliseconds{100});
  }

  void feed_imu(std::uint32_t stamp, float angular_velocity = 1.0F)
  {
    SbgEComLogUnion log{};
    log.imuData.timeStamp = stamp;
    log.imuData.status = 0x03FF;
    log.imuData.accelerometers[2] = 9.81F;
    log.imuData.gyroscopes[0] = angular_velocity;
    log.imuData.temperature = 25.0F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, &log});
  }

  void feed_short(std::uint32_t stamp)
  {
    SbgEComLogUnion log{};
    log.imuShort.timeStamp = stamp;
    log.imuShort.status = 0x03FF;
    log.imuShort.deltaVelocity[2] = 2 * 1048576;
    log.imuShort.deltaAngle[0] = 67108864 / 2;
    log.imuShort.temperature = 25 * 256;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_SHORT, &log});
  }

  void feed_epoch(std::uint32_t stamp, float angular_velocity = 1.0F)
  {
    SbgEComLogUnion log{};
    log.ekfQuatData.timeStamp = stamp;
    log.ekfQuatData.status = 4 | (1 << 4) | (1 << 5);
    log.ekfQuatData.quaternion[0] = 1.0F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &log});
    feed_imu(stamp, angular_velocity);
    log = {};
    log.ekfVelBody.timeStamp = stamp;
    log.ekfVelBody.status = 4 | (1 << 6);
    log.ekfVelBody.velocity[0] = 2.0F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_VEL_BODY, &log});
    log = {};
    log.ekfNavData.timeStamp = stamp;
    log.ekfNavData.status = 4 | (1 << 7);
    log.ekfNavData.position[0] = 45.0;
    log.ekfNavData.position[1] = 10.0;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, &log});
  }

  void feed_heading(std::uint32_t stamp, bool valid = true)
  {
    SbgEComLogUnion log{};
    log.gpsHdtData.timeStamp = stamp;
    log.gpsHdtData.heading = 90.0F;
    log.gpsHdtData.headingAccuracy = valid ? 0.2F : 0.0F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, &log});
  }

  void feed_utc(std::uint32_t stamp, bool valid = true, std::uint8_t second = 0)
  {
    SbgEComLogUnion log{};
    log.utcData.timeStamp = stamp;
    log.utcData.status = valid ? 2u << 6 : 0;
    log.utcData.year = 2026;
    log.utcData.month = 10;
    log.utcData.day = 22;
    log.utcData.second = second;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, &log});
  }

  void feed_position(std::uint32_t stamp, bool valid = true)
  {
    SbgEComLogUnion log{};
    log.gpsPosData.timeStamp = stamp;
    log.gpsPosData.status =
      valid ? SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED | (SBG_ECOM_GNSS_POS_TYPE_SINGLE << 6) : 0;
    log.gpsPosData.latitude = 45.0;
    log.gpsPosData.longitude = 10.0;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, &log});
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unique_ptr<sbg_driver::Publishers> pubs_;
  std::vector<sensor_msgs::msg::Imu> imus_;
  std::vector<sensor_msgs::msg::Temperature> temperatures_;
  std::vector<nav_msgs::msg::Odometry> odometry_;
  std::vector<sbg_msgs::msg::EkfStatus> ekf_statuses_;
  std::vector<sensor_msgs::msg::Imu> raw_orientations_;
  std::vector<sensor_msgs::msg::Imu> headings_;
  std::vector<sensor_msgs::msg::TimeReference> utc_references_;
  std::vector<nmea_msgs::msg::Sentence> nmea_;
  std::vector<tf2_msgs::msg::TFMessage> transforms_;
};

TEST_F(OutputRatesTest, TwoHundredHertzMotionWithOneHertzTemperatureAndEkfStatus)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_data = 200.0;
  cfg.rates_hz.odom = 200.0;
  cfg.rates_hz.imu_temperature = 1.0;
  cfg.rates_hz.sbg_ekf_status = 1.0;
  start(cfg);
  for (std::uint32_t i = 0; i <= 200; ++i) {
    feed_epoch(i * 5'000);
    spin();
  }
  EXPECT_EQ(imus_.size(), 201u);
  EXPECT_EQ(odometry_.size(), 201u);
  EXPECT_EQ(temperatures_.size(), 2u);
  EXPECT_EQ(ekf_statuses_.size(), 2u);
  EXPECT_EQ(pubs_->diag_snapshot().composition_drops, 0u);
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 0u);
}

TEST_F(OutputRatesTest, ThrottledImuStillEnrichesEveryOdometryEpoch)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_data = 1.0;
  cfg.rates_hz.imu_temperature = 1.0;
  start(cfg);
  for (std::uint32_t i = 0; i <= 200; ++i) {
    feed_epoch(i * 5'000, static_cast<float>(i));
    spin();
  }
  ASSERT_EQ(odometry_.size(), 201u);
  EXPECT_EQ(imus_.size(), 2u);
  for (std::size_t i = 0; i < odometry_.size(); ++i) {
    EXPECT_DOUBLE_EQ(odometry_[i].twist.twist.angular.x, static_cast<double>(i));
  }
}

TEST_F(OutputRatesTest, ThrottledImuAndOdometryKeepRawOrientationAndTfIndependent)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_data = 1.0;
  cfg.rates_hz.odom = 1.0;
  cfg.rates_hz.tf = 200.0;
  cfg.publish_raw_orientation = true;
  cfg.broadcast_odom_to_base = true;
  start(cfg);
  for (std::uint32_t i = 0; i <= 200; ++i) {
    feed_epoch(i * 5'000, static_cast<float>(i));
    feed_heading(i * 5'000);
    spin();
  }
  EXPECT_EQ(imus_.size(), 2u);
  EXPECT_EQ(odometry_.size(), 2u);
  EXPECT_EQ(transforms_.size(), 201u);
  ASSERT_EQ(raw_orientations_.size(), 201u);
  for (std::size_t i = 0; i < raw_orientations_.size(); ++i) {
    EXPECT_DOUBLE_EQ(raw_orientations_[i].angular_velocity.x, static_cast<double>(i));
  }
  EXPECT_EQ(pubs_->diag_snapshot().composition_drops, 0u);
}

TEST_F(OutputRatesTest, TfCanBeThrottledWithoutReducingOdometry)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.tf = 1.0;
  cfg.broadcast_odom_to_base = true;
  start(cfg);
  for (std::uint32_t i = 0; i <= 200; ++i) {
    feed_epoch(i * 5'000);
    spin();
  }
  EXPECT_EQ(odometry_.size(), 201u);
  EXPECT_EQ(transforms_.size(), 2u);
}

TEST_F(OutputRatesTest, RateLimitsResetOnReactivationAndConfirmedDeviceRestart)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_temperature = 1.0;
  start(cfg);
  feed_imu(600'000'000);
  feed_imu(600'005'000);
  spin();
  ASSERT_EQ(temperatures_.size(), 1u);
  pubs_->deactivate();
  pubs_->activate();
  feed_imu(600'010'000);
  spin();
  ASSERT_EQ(temperatures_.size(), 2u);
  feed_imu(1'000);  // quarantined restart candidate
  feed_imu(2'000);  // advancing current-time stream confirms the restart
  spin();
  EXPECT_EQ(temperatures_.size(), 3u);
}

TEST_F(OutputRatesTest, TimestampRolloverDoesNotRestartTheRateWindow)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_temperature = 1.0;
  start(cfg);
  constexpr auto before_wrap = std::numeric_limits<std::uint32_t>::max() - 499'999u;
  feed_imu(before_wrap);
  feed_imu(0);
  spin();
  ASSERT_EQ(temperatures_.size(), 1u);
  feed_imu(500'000);
  spin();
  EXPECT_EQ(temperatures_.size(), 2u);
}

TEST_F(OutputRatesTest, AutoSourceSwitchKeepsIndependentOutputLimits)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.imu_data = 200.0;
  cfg.rates_hz.imu_temperature = 1.0;
  start(cfg);
  feed_imu(1'000'000);
  feed_short(1'005'000);
  feed_imu(1'010'000);  // ignored after switching to SHORT
  feed_short(1'010'000);
  spin();
  EXPECT_EQ(imus_.size(), 3u);
  EXPECT_EQ(temperatures_.size(), 1u);
  EXPECT_TRUE(pubs_->diag_snapshot().imu_short_in_use);
}

TEST_F(OutputRatesTest, InvalidOptionalMessagesDoNotConsumePublicationSlots)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.gnss_heading = 1.0;
  cfg.rates_hz.time_reference = 1.0;
  cfg.publish_nmea_gga = true;
  start(cfg);
  feed_utc(1'000, false);
  feed_heading(1'000, false);
  feed_position(1'000, false);
  spin();
  ASSERT_TRUE(headings_.empty());
  ASSERT_TRUE(utc_references_.empty());
  ASSERT_TRUE(nmea_.empty());
  feed_utc(2'000);
  feed_heading(2'000);
  feed_position(2'000);
  spin();
  EXPECT_EQ(headings_.size(), 1u);
  EXPECT_EQ(utc_references_.size(), 1u);
  EXPECT_EQ(nmea_.size(), 1u);
  feed_utc(3'000);
  feed_heading(3'000);
  feed_position(3'000);
  spin();
  EXPECT_EQ(headings_.size(), 1u);
  EXPECT_EQ(utc_references_.size(), 1u);
  EXPECT_EQ(nmea_.size(), 1u);
}

TEST_F(OutputRatesTest, ThrottledTimeReferenceStillRefreshesTheUtcAnchor)
{
  sbg_driver::Publishers::Config cfg;
  cfg.rates_hz.time_reference = 1.0;
  cfg.time_source = sbg_driver::TimeSource::DeviceUtc;
  start(cfg);
  feed_utc(1'000, true, 0);
  feed_utc(501'000, true, 2);
  feed_imu(501'000);
  spin();
  ASSERT_EQ(utc_references_.size(), 1u);
  ASSERT_EQ(imus_.size(), 1u);
  SbgEComLogUtc expected{};
  expected.year = 2026;
  expected.month = 10;
  expected.day = 22;
  expected.second = 2;
  EXPECT_EQ(rclcpp::Time{imus_[0].header.stamp}.nanoseconds(), sbg_driver::utc_epoch_ns(expected));
}

TEST_F(OutputRatesTest, NmeaRateIsConfigurableAndZeroDisablesItsDefaultThrottle)
{
  sbg_driver::Publishers::Config cfg;
  cfg.publish_nmea_gga = true;
  cfg.rates_hz.nmea = 0.0;
  start(cfg);
  for (std::uint32_t i = 0; i < 20; ++i) {
    feed_position(i * 5'000);
    spin();
  }
  EXPECT_EQ(nmea_.size(), 20u);
}

TEST_F(OutputRatesTest, PublishersRejectInvalidRateConfiguration)
{
  for (const auto rate :
       {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    sbg_driver::Publishers::Config cfg;
    cfg.rates_hz.imu_temperature = rate;
    EXPECT_THROW(sbg_driver::Publishers(*node_, cfg), std::invalid_argument);
  }
}

}  // namespace
