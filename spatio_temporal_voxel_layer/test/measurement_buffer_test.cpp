#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "pcl_conversions/pcl_conversions.h"

// Test-only access to private members for heartbeat bookkeeping validation.
#define private public
#include "spatio_temporal_voxel_layer/bridge/measurement_buffer.hpp"
#undef private

namespace
{

geometry_msgs::msg::TransformStamped makeIdentityTransform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  return transform;
}

sensor_msgs::msg::PointCloud2 makeTestCloud(const std::string & frame_id, const rclcpp::Time & stamp)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.push_back(pcl::PointXYZ(1.0F, 2.0F, 0.5F));

  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ros_cloud.header.frame_id = frame_id;
  ros_cloud.header.stamp = stamp;
  return ros_cloud;
}

}  // namespace

class MeasurementBufferFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_);
  }

  void TearDown() override
  {
    tf_buffer_.reset();
  }

  buffer::MeasurementBufferBuilder makeBaseBuilder() const
  {
    buffer::MeasurementBufferBuilder builder;
    builder
      .setSourceName("depth_camera")
      .setTopicName("depth_camera/points")
      .setObservationKeepTime(10.0)
      .setExpectedUpdateRate(0.0)
      .setMinObstacleHeight(0.0)
      .setMaxObstacleHeight(3.0)
      .setObstacleRange(5.0)
      .setTfBuffer(tf_buffer_.get())
      .setGlobalFrame("map")
      .setSensorFrame("camera_link")
      .setTfTolerance(0.1)
      .setNearPlaneDist(0.1)
      .setFarPlaneDist(5.0)
      .setVerticalFov(1.0)
      .setVerticalFovPadding(0.0)
      .setHorizontalFov(0.0)
      .setDecayAcceleration(0.0)
      .setVoxelSize(0.05)
      .setFilter(buffer::Filters::NONE)
      .setVoxelMinPoints(0)
      .setEnabled(true)
      .setClearBufferAfterReading(false)
      .setModelType(ModelType::DEPTH_CAMERA)
      .setClock(clock_)
      .setLogger(rclcpp::get_logger("measurement_buffer_test"));
    return builder;
  }

  std::shared_ptr<rclcpp::Clock> clock_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

TEST_F(MeasurementBufferFixture, BuffersCloudWithMetadata)
{
  const auto stamp = clock_->now();
  auto transform = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(transform, "test_authority", true);

  auto config = makeBaseBuilder()
    .setObservationKeepTime(10.0)
    .setMarking(true)
    .setClearing(true)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  const auto & reading = readings.front();
  ASSERT_NE(reading._cloud, nullptr);
  ASSERT_EQ(reading._cloud->points.size(), 1U);
  EXPECT_FLOAT_EQ(reading._cloud->points.front().x, 1.0F);
  EXPECT_FLOAT_EQ(reading._cloud->points.front().y, 2.0F);
  EXPECT_FLOAT_EQ(reading._cloud->points.front().z, 0.5F);
  EXPECT_DOUBLE_EQ(reading._origin.x, 0.0);
  EXPECT_DOUBLE_EQ(reading._origin.y, 0.0);
  EXPECT_DOUBLE_EQ(reading._origin.z, 0.0);
  EXPECT_DOUBLE_EQ(reading._orientation.w, 1.0);
  EXPECT_TRUE(reading._marking);
  EXPECT_TRUE(reading._clearing);
}

TEST_F(MeasurementBufferFixture, RemovesStaleObservations)
{
  auto config = makeBaseBuilder()
    .setObservationKeepTime(0.01)
    .setMarking(true)
    .setClearing(false)
    .build();
  buffer::MeasurementBuffer buffer(config);

  auto stamp = clock_->now();
  auto transform = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(transform, "test_authority", true);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);
  ASSERT_EQ(readings.size(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  readings.clear();
  buffer.GetReadings(readings);
  EXPECT_TRUE(readings.empty());
}

TEST_F(MeasurementBufferFixture, HeightRelativeToBaseDropsCloudWhenBaseTfMissing)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  // Intentionally do NOT publish a transform for base_link.
  auto config = makeBaseBuilder()
    .setMarking(true)
    .setClearing(false)
    .setFilter(buffer::Filters::PASSTHROUGH)
    .setHeightRelativeToBase(true)
    .setRobotBaseFrame("base_link")
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  // Observation is rejected entirely (fail-safe: no marking).
  EXPECT_TRUE(readings.empty());
}

TEST_F(MeasurementBufferFixture, HeightRelativeToBaseRejectsWhenFilterNone)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  auto config = makeBaseBuilder()
    .setMarking(true)
    .setClearing(false)
    .setFilter(buffer::Filters::NONE)
    .setHeightRelativeToBase(true)
    .setRobotBaseFrame("base_link")
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  EXPECT_TRUE(readings.empty());
}

TEST_F(MeasurementBufferFixture, ObstacleHeightFilterDropsPointWhenEnabled)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  // Configure a height window that excludes the test point (z=0.5).
  auto config = makeBaseBuilder()
    .setMarking(true)
    .setClearing(false)
    .setFilter(buffer::Filters::PASSTHROUGH)
    .setMinObstacleHeight(1.0)
    .setMaxObstacleHeight(2.0)
    .setFilterObstacleHeight(true)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  ASSERT_NE(readings.front()._cloud, nullptr);
  EXPECT_TRUE(readings.front()._cloud->points.empty());
}

TEST_F(MeasurementBufferFixture, DisablingObstacleHeightFilterIgnoresMinMaxHeight)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  // Configure a height window that would normally reject the test point (z=0.5).
  auto config = makeBaseBuilder()
    .setMarking(true)
    .setClearing(false)
    .setFilter(buffer::Filters::PASSTHROUGH)
    .setMinObstacleHeight(1.0)
    .setMaxObstacleHeight(2.0)
    .setFilterObstacleHeight(false)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  ASSERT_NE(readings.front()._cloud, nullptr);
  ASSERT_EQ(readings.front()._cloud->points.size(), 1U);
  EXPECT_FLOAT_EQ(readings.front()._cloud->points.front().z, 0.5F);
}

TEST_F(MeasurementBufferFixture, DisablingObstacleHeightFilterAllowsHeightRelativeToBaseWithFilterNone)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  // With filter NONE, height_relative_to_base would normally reject when height filtering is enabled.
  // When obstacle-height filtering is disabled, this should buffer normally (and should NOT require a base tf).
  auto config = makeBaseBuilder()
    .setMarking(true)
    .setClearing(false)
    .setFilter(buffer::Filters::NONE)
    .setHeightRelativeToBase(true)
    .setRobotBaseFrame("base_link")
    .setFilterObstacleHeight(false)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);

  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  ASSERT_NE(readings.front()._cloud, nullptr);
  ASSERT_EQ(readings.front()._cloud->points.size(), 1U);
}

TEST_F(MeasurementBufferFixture, ClearingNearFarPlaneDistancesAreBuffered)
{
  const auto stamp = clock_->now();
  auto transform = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(transform, "test_authority", true);

  auto config = makeBaseBuilder()
    .setMarking(false)
    .setClearing(true)
    .setNearPlaneDist(0.5)
    .setFarPlaneDist(0.9)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);
  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  EXPECT_DOUBLE_EQ(readings.front()._near_plane_dist_in_m, 0.5);
  EXPECT_DOUBLE_EQ(readings.front()._far_plane_dist_in_m, 0.9);
}

TEST_F(MeasurementBufferFixture, ObstacleRangeDoesNotOverrideClearingPlaneDistances)
{
  const auto stamp = clock_->now();
  auto transform = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(transform, "test_authority", true);

  auto config = makeBaseBuilder()
    .setMarking(false)
    .setClearing(true)
    .setObstacleRange(4.2)
    .setNearPlaneDist(0.5)
    .setFarPlaneDist(0.9)
    .build();
  buffer::MeasurementBuffer buffer(config);

  const auto cloud = makeTestCloud("camera_link", stamp);
  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  std::vector<observation::MeasurementReading> readings;
  buffer.GetReadings(readings);

  ASSERT_EQ(readings.size(), 1U);
  EXPECT_DOUBLE_EQ(readings.front()._near_plane_dist_in_m, 0.5);
  EXPECT_DOUBLE_EQ(readings.front()._far_plane_dist_in_m, 0.9);
}

TEST_F(MeasurementBufferFixture, ClearingOnlyBufferUpdatesSuccessAndClearsLastErrorMessage)
{
  const auto stamp = clock_->now();
  auto camera_tf = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(camera_tf, "test_authority", true);

  auto config = makeBaseBuilder()
    .setMarking(false)
    .setClearing(true)
    .build();
  buffer::MeasurementBuffer buffer(config);

  // Seed an error state; clearing-only buffering should clear the last error message
  // and record a successful buffer time.
  {
    const auto now_ns = clock_->now().nanoseconds();
    const int64_t seeded_error_ns = (now_ns > 1000000) ? (now_ns - 1000000) : 0;
    buffer.last_error_time_ns_.store(seeded_error_ns, std::memory_order_relaxed);
    buffer.error_count_.store(1U, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(buffer.last_error_mutex_);
    buffer.last_error_message_ = "seeded error";
  }
  buffer.last_success_time_ns_.store(0, std::memory_order_relaxed);

  const auto before_success = buffer.GetLastSuccessfulBufferTime();
  EXPECT_EQ(before_success.nanoseconds(), 0);
  EXPECT_FALSE(buffer.GetLastErrorMessage().empty());

  const auto cloud = makeTestCloud("camera_link", stamp);
  buffer.Lock();
  buffer.BufferROSCloud(cloud);
  buffer.Unlock();

  const auto after_success = buffer.GetLastSuccessfulBufferTime();
  EXPECT_GT(after_success.nanoseconds(), 0);
  EXPECT_TRUE(buffer.GetLastErrorMessage().empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
