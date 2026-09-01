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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sbg/log_view.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
  }

  void TearDown() override
  {
    pubs_.reset();
    sub_.reset();
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

  void feed_imu_data(std::uint32_t ts)
  {
    SbgEComLogUnion u{};
    u.imuData.timeStamp = ts;
    u.imuData.accelerometers[2] = 9.81F;
    pubs_->on_log(sbg::LogView{SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, &u});
  }

  void feed_imu_short(std::uint32_t ts, std::uint16_t status = 0)
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

  static bool has_orientation(const sensor_msgs::msg::Imu & m)
  {
    return m.orientation_covariance[0] >= 0.0;
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::unique_ptr<sbg_driver::Publishers> pubs_;
  std::vector<sensor_msgs::msg::Imu> imu_msgs_;
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
  feed_imu_short(1000, k_gyro_high_scale_bit);
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

}  // namespace
