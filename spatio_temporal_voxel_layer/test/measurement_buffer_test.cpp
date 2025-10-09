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

#include "spatio_temporal_voxel_layer/bridge/measurement_buffer.hpp"

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

  std::shared_ptr<rclcpp::Clock> clock_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

TEST_F(MeasurementBufferFixture, BuffersCloudWithMetadata)
{
  buffer::MeasurementBuffer buffer(
    "depth_camera",
    "depth_camera/points",
    10.0,
    0.0,
    0.0,
    3.0,
    5.0,
    *tf_buffer_,
    "map",
    "camera_link",
    0.1,
    0.1,
    5.0,
    1.0,
    0.0,
    0.0,
  0.0,
    true,
    true,
    0.05,
    buffer::Filters::NONE,
    0,
    true,
    false,
    ModelType::DEPTH_CAMERA,
    clock_,
    rclcpp::get_logger("measurement_buffer_test"));

  const auto stamp = clock_->now();
  auto transform = makeIdentityTransform("map", "camera_link", stamp);
  tf_buffer_->setTransform(transform, "test_authority", true);

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
  buffer::MeasurementBuffer buffer(
    "depth_camera",
    "depth_camera/points",
    0.01,
    0.0,
    0.0,
    3.0,
    5.0,
    *tf_buffer_,
    "map",
    "camera_link",
    0.1,
    0.1,
    5.0,
    1.0,
    0.0,
    0.0,
  0.0,
    true,
    false,
    0.05,
    buffer::Filters::NONE,
    0,
    true,
    false,
    ModelType::DEPTH_CAMERA,
    clock_,
    rclcpp::get_logger("measurement_buffer_test"));

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
