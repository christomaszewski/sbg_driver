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

#include "sbg_driver/publishers.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sbg/log_view.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <utility>
#include <vector>

// Drives Publishers::on_log() directly with synthetic LogViews and reads
// /imu/data back through an intra-process subscription on the same node, so
// delivery needs no DDS discovery: every publish is visible after one
// spin_some(). Exercises the IMU hold-and-pair logic and the imu.source
// selection, which have no other harness.

namespace
{

// EkfQuat.status with solution mode NAV_POSITION and attitude + heading
// valid — the ordinary healthy word.
constexpr std::uint32_t k_quat_valid = 4u | (1u << 4) | (1u << 5);

constexpr std::uint32_t k_gyro_high_scale_bit = 1u << 10;

class PublishersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // One rclcpp context for the whole binary; each test builds its own node.
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    // Every node is gone by now (TearDown resets them), so the context can go.
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "publishers_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    exec_.add_node(node_->get_node_base_interface());
    sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // Copy out: the intra-process buffer owns msg only for the callback.
        imu_msgs_.push_back(*msg);
      });
    raw_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "raw/orientation", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { raw_msgs_.push_back(*msg); });
  }

  void TearDown() override
  {
    pubs_.reset();
    sub_.reset();
    raw_sub_.reset();
    exec_.remove_node(node_->get_node_base_interface());
    node_.reset();
  }

  void make_publishers(std::uint32_t tolerance_us, sbg_driver::ImuSource source)
  {
    sbg_driver::Publishers::Config cfg{};
    cfg.imu_source = source;
    pubs_ = std::make_unique<sbg_driver::Publishers>(*node_, std::move(cfg));
    pubs_->set_epoch_tolerance_us(tolerance_us);
    pubs_->activate();
  }

  // Intra-process delivery is queued on publish and drained by the executor.
  void drain()
  {
    for (int i = 0; i < 5; ++i) {
      exec_.spin_some();
    }
  }

  void make_raw_publishers(std::uint32_t tolerance_us = 10'000, double utc_timeout_s = 5.0)
  {
    sbg_driver::Publishers::Config cfg{};
    cfg.publish_raw_orientation = true;
    cfg.utc_timeout_s = utc_timeout_s;
    cfg.raw_orientation_imu_tolerance_us = tolerance_us;
    cfg.time_source = sbg_driver::TimeSource::DeviceUtc;
    pubs_ = std::make_unique<sbg_driver::Publishers>(*node_, std::move(cfg));
    pubs_->activate();
  }

  void feed_heading(std::uint32_t ts, bool secondary = false, std::uint16_t status = 0)
  {
    SbgEComLogUnion u{};
    u.gpsHdtData.timeStamp = ts;
    u.gpsHdtData.heading = 90.0F;
    u.gpsHdtData.headingAccuracy = 1.0F;
    u.gpsHdtData.status = status;
    pubs_->on_log(
      sbg::LogView{
        SBG_ECOM_CLASS_LOG_ECOM_0, secondary ? SBG_ECOM_LOG_GPS2_HDT : SBG_ECOM_LOG_GPS1_HDT, &u});
  }

  void feed_imu_data(std::uint32_t ts, float ax = 0.0F)
  {
    SbgEComLogUnion u{};
    u.imuData.timeStamp = ts;
    u.imuData.status = 0x3FF;  // communication, BIT and sensor range all healthy
    u.imuData.accelerometers[0] = ax;
    u.imuData.accelerometers[2] = 9.81F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, &u});
  }

  void feed_imu_short(std::uint32_t ts, std::uint16_t status = 0x3FF)
  {
    SbgEComLogUnion u{};
    u.imuShort.timeStamp = ts;
    u.imuShort.status = status;
    u.imuShort.deltaVelocity[2] = 1048576 * 2;  // 2 m/s²
    u.imuShort.deltaAngle[0] = 67108864 / 2;    // 0.5 rad/s at standard scale
    u.imuShort.temperature = 256 * 25;          // 25 °C
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_SHORT, &u});
  }

  void feed_quat(std::uint32_t ts, std::uint32_t status = k_quat_valid)
  {
    SbgEComLogUnion u{};
    u.ekfQuatData.timeStamp = ts;
    u.ekfQuatData.status = status;
    u.ekfQuatData.quaternion[0] = 1.0F;
    u.ekfQuatData.eulerStdDev[0] = 0.01F;
    u.ekfQuatData.eulerStdDev[1] = 0.01F;
    u.ekfQuatData.eulerStdDev[2] = 0.02F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &u});
  }

  void feed_utc(std::uint32_t ts, bool valid = true)
  {
    SbgEComLogUnion u{};
    u.utcData.timeStamp = ts;
    u.utcData.status = valid ? 2u << 6 : 0;
    u.utcData.year = 2026;
    u.utcData.month = 9;
    u.utcData.day = 11;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, &u});
  }

  void feed_odom(
    std::uint32_t ts, std::uint32_t quat_status = k_quat_valid, float qw = 1.0F,
    double latitude = 45.0)
  {
    SbgEComLogUnion u{};
    u.ekfQuatData.timeStamp = ts;
    u.ekfQuatData.status = quat_status;
    u.ekfQuatData.quaternion[0] = qw;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, &u});
    u = {};
    u.ekfVelBody.timeStamp = ts;
    u.ekfVelBody.status = 4 | (1 << 6);
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_VEL_BODY, &u});
    u = {};
    u.ekfNavData.timeStamp = ts;
    u.ekfNavData.status = 4 | (1 << 7);
    u.ekfNavData.position[0] = latitude;
    u.ekfNavData.position[1] = 10.0;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, &u});
  }

  static bool has_orientation(const sensor_msgs::msg::Imu & m)
  {
    return m.orientation_covariance[0] >= 0.0;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub_;
  std::unique_ptr<sbg_driver::Publishers> pubs_;
  std::vector<sensor_msgs::msg::Imu> imu_msgs_;
  std::vector<sensor_msgs::msg::Imu> raw_msgs_;
};

TEST_F(PublishersTest, ImuBeforeSameEpochQuatIsHeldThenPairedOnQuatArrival)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_quat(500);  // an earlier epoch exists, so a newer quat may follow
  feed_imu_data(1000);
  drain();
  EXPECT_TRUE(imu_msgs_.empty()) << "sample must be held for its epoch's quat";

  feed_quat(1000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_TRUE(has_orientation(imu_msgs_[0]));
  EXPECT_DOUBLE_EQ(imu_msgs_[0].orientation.w, 1.0);
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 0u);
  EXPECT_EQ(pubs_->diag_snapshot().composition_drops, 0u);
}

TEST_F(PublishersTest, ImuWithNoQuatStreamPublishesImmediatelyWithoutOrientation)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_imu_data(1000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_FALSE(has_orientation(imu_msgs_[0]));
  EXPECT_NEAR(imu_msgs_[0].linear_acceleration.z, 9.81, 1e-5);
  // No quat stream: not a miss, nothing to tune.
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 0u);
}

TEST_F(PublishersTest, HeldImuIsRetiredByTheNextSampleAndCountedAsMiss)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_quat(500);
  feed_imu_data(1000);
  feed_imu_data(2000);  // retires 1000 (no quat came), holds 2000
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_FALSE(has_orientation(imu_msgs_[0]));
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 1u);
  // /odom composition is untouched by IMU misses.
  EXPECT_EQ(pubs_->diag_snapshot().composition_drops, 0u);

  feed_quat(2000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 2u);
  EXPECT_TRUE(has_orientation(imu_msgs_[1]));
}

TEST_F(PublishersTest, QuatMovingPastHeldSampleRetiresIt)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_quat(500);
  feed_imu_data(1000);
  feed_quat(1500);  // newer than the held sample, not its epoch
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_FALSE(has_orientation(imu_msgs_[0]));
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 1u);
}

TEST_F(PublishersTest, AsyncQuatWithinToleranceAttachesImmediately)
{
  make_publishers(10'000, sbg_driver::ImuSource::Auto);
  feed_quat(1000);
  feed_imu_data(4000);  // 3 ms after the quat: no hold, no miss
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_TRUE(has_orientation(imu_msgs_[0]));
  EXPECT_EQ(pubs_->diag_snapshot().imu_orientation_misses, 0u);
}

TEST_F(PublishersTest, QuatDisownedByEkfYieldsOrientationSentinel)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_quat(1000, 0u);  // UNINITIALIZED, nothing valid
  feed_imu_data(1000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_FALSE(has_orientation(imu_msgs_[0]));
}

TEST_F(PublishersTest, AutoPrefersImuShortOnceSeen)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_imu_data(1000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_FALSE(pubs_->diag_snapshot().imu_short_in_use);

  feed_imu_short(2000);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 2u);
  EXPECT_TRUE(pubs_->diag_snapshot().imu_short_in_use);
  EXPECT_NEAR(imu_msgs_[1].linear_acceleration.z, 2.0, 1e-6);
  EXPECT_NEAR(imu_msgs_[1].angular_velocity.x, 0.5, 1e-6);

  feed_imu_data(3000);  // ignored from now on
  drain();
  EXPECT_EQ(imu_msgs_.size(), 2u);
}

TEST_F(PublishersTest, ImuShortHighRangeGyroScaleIsApplied)
{
  make_publishers(0, sbg_driver::ImuSource::ImuShort);
  feed_imu_short(1000, 0x3FF | k_gyro_high_scale_bit);
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_NEAR(imu_msgs_[0].angular_velocity.x, (67108864.0 / 2.0) / 12304174.0, 1e-6);
}

TEST_F(PublishersTest, ImuDataSourceIgnoresImuShort)
{
  make_publishers(0, sbg_driver::ImuSource::ImuData);
  feed_imu_short(1000);
  drain();
  EXPECT_TRUE(imu_msgs_.empty());
  EXPECT_FALSE(pubs_->diag_snapshot().imu_short_in_use);
  feed_imu_data(2000);
  drain();
  EXPECT_EQ(imu_msgs_.size(), 1u);
}

TEST_F(PublishersTest, ActivateResetsSourceLatchAndHeldSample)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_imu_short(1000);
  feed_quat(1500);
  feed_imu_data(2000);  // ignored: short seen
  drain();
  ASSERT_EQ(imu_msgs_.size(), 1u);
  EXPECT_TRUE(pubs_->diag_snapshot().imu_short_in_use);

  pubs_->deactivate();
  pubs_->activate();  // new session: latch cleared, nothing held
  EXPECT_FALSE(pubs_->diag_snapshot().imu_short_in_use);
  feed_imu_data(3000);
  drain();
  EXPECT_EQ(imu_msgs_.size(), 2u);
}

TEST_F(PublishersTest, RawOrientationIsOptIn)
{
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_heading(1000);
  drain();
  EXPECT_TRUE(raw_msgs_.empty());
}

TEST_F(PublishersTest, DelayedHeadingUsesNearestHistoricalImuNotLatest)
{
  make_raw_publishers(100);
  feed_imu_data(1000, 1.0F);
  feed_imu_data(1100, 2.0F);
  feed_imu_data(2000, 3.0F);
  feed_heading(1080);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 1u);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration.x, 2.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], 0.0);
  EXPECT_TRUE(has_orientation(raw_msgs_[0]));
  EXPECT_EQ(raw_msgs_[0].header.frame_id, "imu_link");
}

TEST_F(PublishersTest, HeadingWithoutMatchingImuPublishesImmediatelyAndOnlyOnce)
{
  make_raw_publishers(0);
  feed_heading(1000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 1u);
  EXPECT_TRUE(has_orientation(raw_msgs_[0]));
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].angular_velocity_covariance[0], -1.0);
  feed_imu_data(1000);
  feed_heading(1000);        // repeated by a faster device output mode
  feed_heading(900);         // reordered older heading
  feed_heading(2000, true);  // secondary receiver must not interleave
  drain();
  EXPECT_EQ(raw_msgs_.size(), 1u);
}

TEST_F(PublishersTest, StaleImuDoesNotHideHeadingAndInvalidHeadingDoesNotHideImu)
{
  make_raw_publishers(100);
  feed_imu_data(1000);
  feed_heading(1100);  // inclusive tolerance
  feed_heading(1101);  // just outside
  feed_imu_data(2000, 4.0F);
  feed_heading(2000, false, 1);  // insufficient GNSS observations
  drain();
  ASSERT_EQ(raw_msgs_.size(), 3u);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[1].linear_acceleration_covariance[0], -1.0);
  EXPECT_TRUE(has_orientation(raw_msgs_[1]));
  EXPECT_FALSE(has_orientation(raw_msgs_[2]));
  EXPECT_DOUBLE_EQ(raw_msgs_[2].linear_acceleration.x, 4.0);
}

TEST_F(PublishersTest, RawHeadingAndImuMatchingSurviveTimestampRollover)
{
  make_raw_publishers(10);
  feed_heading(0xFFFFFFF0U);
  feed_imu_data(0xFFFFFFFEU, 7.0F);
  feed_heading(2U);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
  EXPECT_DOUBLE_EQ(raw_msgs_[1].linear_acceleration.x, 7.0);
}

TEST_F(PublishersTest, RawHeaderUsesGnssAcquisitionTimeUnderDeviceUtc)
{
  make_raw_publishers(100);
  SbgEComLogUnion u{};
  u.utcData.timeStamp = 0;
  u.utcData.status = 2u << 6;  // UTC initialized
  u.utcData.year = 2020;
  u.utcData.month = 1;
  u.utcData.day = 1;
  pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, &u});
  feed_imu_data(900);
  feed_heading(1000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 1u);
  EXPECT_EQ(raw_msgs_[0].header.stamp.sec, 1577836800);
  EXPECT_EQ(raw_msgs_[0].header.stamp.nanosec, 1'000'000u);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], 0.0);
}

TEST_F(PublishersTest, RawOrientationUsesSelectedImuShortAndClearsLegacyHistory)
{
  make_raw_publishers(0);
  feed_imu_data(1000);
  feed_imu_short(2000);
  feed_imu_data(2000, 99.0F);  // auto now ignores IMU_DATA
  feed_heading(1000);
  feed_heading(2000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[1].linear_acceleration.z, 2.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[1].angular_velocity.x, 0.5);
}

TEST_F(PublishersTest, RawHistoryIsBoundedAndResetOnReactivation)
{
  make_raw_publishers(0);
  for (std::uint32_t ts = 0; ts < 2049; ++ts) {
    feed_imu_data(ts);
  }
  feed_heading(0);  // evicted by the sample limit
  feed_heading(2048);
  pubs_->deactivate();
  feed_heading(2049);  // inactive: no publication
  pubs_->activate();
  feed_heading(2048);  // new session: no old history or deduplication latch
  drain();
  ASSERT_EQ(raw_msgs_.size(), 3u);
  EXPECT_DOUBLE_EQ(raw_msgs_[0].linear_acceleration_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[1].linear_acceleration_covariance[0], 0.0);
  EXPECT_DOUBLE_EQ(raw_msgs_[2].linear_acceleration_covariance[0], -1.0);
}

TEST_F(PublishersTest, RawHeadingRecoversFromRestartAndClearsOldImuHistory)
{
  make_raw_publishers();
  feed_imu_data(600'000'000, 42.0F);
  feed_heading(600'000'000);
  feed_heading(1000);  // first large backwards jump is quarantined
  feed_heading(2000);  // advancing new epoch confirms a restart
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
  EXPECT_EQ(raw_msgs_[0].linear_acceleration.x, 42.0);
  EXPECT_EQ(raw_msgs_[1].linear_acceleration_covariance[0], -1.0);
  feed_imu_data(3000, 7.0F);
  feed_heading(3000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 3u);
  EXPECT_EQ(raw_msgs_[2].linear_acceleration.x, 7.0);
}

TEST_F(PublishersTest, ReorderedHeadingsDoNotResetTheCurrentEpoch)
{
  make_raw_publishers();
  feed_heading(600'000'000);
  feed_heading(599'999'000);  // short reordering
  feed_heading(1000);         // one very old datagram is insufficient
  feed_heading(600'001'000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
}

TEST_F(PublishersTest, RolloverKeepsHeadingAndImuMatching)
{
  make_raw_publishers();
  feed_imu_data(0xFFFFFF00, 12.0F);
  feed_heading(0xFFFFFF00);
  feed_heading(0x100);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
  EXPECT_EQ(raw_msgs_[1].linear_acceleration.x, 12.0);
}

TEST_F(PublishersTest, InvalidUtcAfterRestartFallsBackUntilANewValidAnchor)
{
  make_raw_publishers();
  feed_utc(600'000'000);
  feed_utc(1000, false);
  const auto before = node_->now();
  feed_heading(2000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 1u);
  EXPECT_GE(rclcpp::Time{raw_msgs_[0].header.stamp}.nanoseconds(), before.nanoseconds());
  feed_utc(3000);
  feed_heading(4000);
  drain();
  ASSERT_EQ(raw_msgs_.size(), 2u);
  SbgEComLogUtc expected{};
  expected.year = 2026;
  expected.month = 9;
  expected.day = 11;
  EXPECT_EQ(
    rclcpp::Time{raw_msgs_[1].header.stamp}.nanoseconds(),
    sbg_driver::utc_epoch_ns(expected) + 1'000'000);
}

TEST_F(PublishersTest, ExpiredUtcAnchorCannotReviveAtTimestampRollover)
{
  make_raw_publishers(10'000, 0.01);
  feed_utc(1000);
  std::this_thread::sleep_for(std::chrono::milliseconds{25});
  const auto before = node_->now();
  feed_heading(1100);  // looks close to the old anchor, but it expired in steady time
  drain();
  ASSERT_EQ(raw_msgs_.size(), 1u);
  EXPECT_GE(rclcpp::Time{raw_msgs_[0].header.stamp}.nanoseconds(), before.nanoseconds());
}

TEST_F(PublishersTest, TfRequiresExplicitOptInAndAValidFinitePose)
{
  std::vector<tf2_msgs::msg::TFMessage> tfs;
  auto sub = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100),
    [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m) { tfs.push_back(*m); });
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_odom(1000);
  drain();
  EXPECT_TRUE(tfs.empty());
  pubs_.reset();
  sbg_driver::Publishers::Config cfg;
  cfg.broadcast_odom_to_base = true;
  pubs_ = std::make_unique<sbg_driver::Publishers>(*node_, cfg);
  pubs_->activate();
  feed_odom(2000, 4);             // attitude invalid
  feed_odom(3000, 4 | (1 << 4));  // yaw invalid
  feed_odom(4000, k_quat_valid, std::numeric_limits<float>::quiet_NaN());
  feed_odom(5000, k_quat_valid, 0.0F);  // degenerate quaternion
  feed_odom(6000, k_quat_valid, 1.0F, std::numeric_limits<double>::infinity());
  drain();
  EXPECT_TRUE(tfs.empty());
  feed_odom(7000);
  drain();
  ASSERT_EQ(tfs.size(), 1u);
  EXPECT_EQ(tfs[0].transforms[0].child_frame_id, "base_link");
}

TEST_F(PublishersTest, EventsRetainPhysicalInputChannel)
{
  std::vector<sbg_msgs::msg::Event> events;
  auto sub = node_->create_subscription<sbg_msgs::msg::Event>(
    "sbg/event", 10, [&](sbg_msgs::msg::Event::ConstSharedPtr m) { events.push_back(*m); });
  make_publishers(0, sbg_driver::ImuSource::Auto);
  for (std::uint16_t id = SBG_ECOM_LOG_EVENT_A; id <= SBG_ECOM_LOG_EVENT_E; ++id) {
    SbgEComLogUnion u{};
    u.eventMarker.timeStamp = 1234;
    u.eventMarker.status = 2;
    u.eventMarker.timeOffset0 = 7;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, id, &u});
    drain();
  }
  drain();
  ASSERT_EQ(events.size(), 5u);
  for (std::size_t i = 0; i < events.size(); ++i) {
    EXPECT_EQ(events[i].channel, i + 1);
    EXPECT_EQ(events[i].time_stamp_us, 1234u);
    EXPECT_EQ(events[i].time_offsets_us[0], 7);
  }
}

TEST_F(PublishersTest, DeviceRestartPreservesTheOdometryOrigin)
{
  std::vector<nav_msgs::msg::Odometry> odometry;
  auto sub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr m) { odometry.push_back(*m); });
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_odom(600'000'000);
  drain();
  feed_odom(1000, k_quat_valid, 1.0F, 45.001);  // quarantined reset candidate
  feed_odom(2000, k_quat_valid, 1.0F, 45.001);
  drain();
  ASSERT_EQ(odometry.size(), 2u);
  EXPECT_NEAR(odometry[0].pose.pose.position.x, 0.0, 1e-6);
  EXPECT_GT(odometry[1].pose.pose.position.x, 100.0);  // no new zero-origin jump
}

TEST_F(PublishersTest, InvalidFirstPositionCannotPoisonTheOdometryOrigin)
{
  std::vector<nav_msgs::msg::Odometry> odometry;
  auto sub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    [&](nav_msgs::msg::Odometry::ConstSharedPtr m) { odometry.push_back(*m); });
  make_publishers(0, sbg_driver::ImuSource::Auto);
  feed_odom(1000, k_quat_valid, 1.0F, std::numeric_limits<double>::quiet_NaN());
  feed_odom(2000);
  drain();
  ASSERT_EQ(odometry.size(), 1u);
  EXPECT_NEAR(odometry[0].pose.pose.position.x, 0.0, 1e-6);
}

}  // namespace
