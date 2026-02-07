#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"

// Test-only access to private members for CameraInfo/FOV initialization.
#define private public
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"
#include "spatio_temporal_voxel_layer/internal/observation_manager.hpp"
#include "spatio_temporal_voxel_layer/bridge/measurement_buffer.hpp"
#undef private

namespace
{

template <typename Predicate>
bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & exec,
  Predicate predicate,
  std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    exec.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

std::shared_ptr<buffer::MeasurementBuffer> makeDepthCameraBuffer(
  const std::string & source_name,
  tf2_ros::Buffer * tf_buffer,
  const rclcpp::Clock::SharedPtr & clock)
{
  buffer::MeasurementBufferBuilder builder;
  const auto config = builder
    .setSourceName(source_name)
    .setTopicName("depth/points")
    .setObservationKeepTime(10.0)
    .setExpectedUpdateRate(0.0)
    .setMinObstacleHeight(0.0)
    .setMaxObstacleHeight(3.0)
    .setObstacleRange(5.0)
    .setTfBuffer(tf_buffer)
    .setGlobalFrame("map")
    .setSensorFrame("camera_link")
    .setTfTolerance(0.1)
    .setNearPlaneDist(0.1)
    .setFarPlaneDist(5.0)
    .setVerticalFov(0.0)
    .setVerticalFovPadding(0.0)
    .setHorizontalFov(0.0)
    .setDecayAcceleration(0.0)
    .setVoxelSize(0.05)
    .setFilter(buffer::Filters::NONE)
    .setVoxelMinPoints(0)
    .setEnabled(true)
    .setClearBufferAfterReading(false)
    .setModelType(ModelType::DEPTH_CAMERA)
    .setClock(clock)
    .setLogger(rclcpp::get_logger("camera_info_fov_test"))
    .build();

  return std::make_shared<buffer::MeasurementBuffer>(config);
}

}  // namespace

TEST(CameraInfoFovInitTest, AppliesComputedPaddedFovAndUnsubscribesOnFirstValidMessage)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);

  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer layer;
  layer._observation_manager = std::make_unique<spatio_temporal_voxel_layer::internal::ObservationManager>();

  const std::string source_name = "rgbd_clear";
  auto buffer = makeDepthCameraBuffer(source_name, tf_buffer.get(), clock);
  layer._observation_manager->registerBuffer(buffer, false, true);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("camera_info_fov_test_node");

  // Use a regular rclcpp::Node for publishing. Lifecycle publishers must be activated
  // before publishing, which is unnecessary complexity for this unit test.
  auto pub_node = std::make_shared<rclcpp::Node>("camera_info_fov_test_pub_node");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node);

  const std::string camera_info_topic = "/camera/depth/camera_info";
  auto pub = pub_node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, rclcpp::SensorDataQoS());

  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer::ObservationSourceConfig cfg;
  cfg.name = source_name;
  cfg.fov_from_camera_info = true;
  cfg.camera_info_topic = camera_info_topic;
  cfg.camera_info_required = true;
  cfg.model_type = ModelType::DEPTH_CAMERA;
  cfg.horizontal_fov_padding_rad = 0.1;
  cfg.vertical_fov_padding_rad = 0.2;

  layer.initializeCameraInfoFovs(node, {cfg});

  const std::string dep_name = source_name + std::string("/camera_info");
  ASSERT_TRUE(layer.camera_info_dependencies_.count(dep_name) > 0U);
  EXPECT_TRUE(layer.camera_info_subscriptions_.count(dep_name) > 0U);

  // Send invalid CameraInfo first; dependency should remain uninitialized.
  sensor_msgs::msg::CameraInfo bad;
  bad.width = 0U;
  bad.height = 0U;
  bad.k[0] = 0.0;
  bad.k[4] = 0.0;
  pub->publish(bad);

  const bool saw_invalid = spinUntil(
    exec,
    [&]() {
      std::lock_guard<std::mutex> lock(layer.camera_info_mutex_);
      const auto & dep = layer.camera_info_dependencies_.at(dep_name);
      return (!dep.initialized) && (dep.error_msg.find("invalid CameraInfo") != std::string::npos);
    },
    std::chrono::milliseconds(500));
  EXPECT_TRUE(saw_invalid);

  // Now send a valid message; dependency should flip initialized and subscription should be dropped.
  sensor_msgs::msg::CameraInfo good;
  good.width = 640U;
  good.height = 480U;
  good.k[0] = 600.0;  // fx
  good.k[4] = 600.0;  // fy
  pub->publish(good);

  const bool initialized = spinUntil(
    exec,
    [&]() {
      std::lock_guard<std::mutex> lock(layer.camera_info_mutex_);
      return layer.camera_info_dependencies_.at(dep_name).initialized;
    },
    std::chrono::milliseconds(1000));
  ASSERT_TRUE(initialized);

  {
    std::lock_guard<std::mutex> lock(layer.camera_info_mutex_);
    EXPECT_TRUE(layer.camera_info_dependencies_.at(dep_name).error_msg.empty());
  }

  EXPECT_EQ(layer.camera_info_subscriptions_.count(dep_name), 0U);

  const double width = static_cast<double>(good.width);
  const double height = static_cast<double>(good.height);
  const double fx = good.k[0];
  const double fy = good.k[4];

  const double h_raw = 2.0 * std::atan2(width, 2.0 * fx);
  const double v_raw = 2.0 * std::atan2(height, 2.0 * fy);

  constexpr double kMinFovRad = 0.05;
  constexpr double kMaxFovRad = 3.13;

  const double h_expected = std::clamp(h_raw - cfg.horizontal_fov_padding_rad, kMinFovRad, kMaxFovRad);
  const double v_expected = std::clamp(v_raw - cfg.vertical_fov_padding_rad, kMinFovRad, kMaxFovRad);

  // Buffer fields are protected by the buffer's internal lock, but here we only read.
  EXPECT_NEAR(buffer->_horizontal_fov, h_expected, 1e-6);
  EXPECT_NEAR(buffer->_vertical_fov, v_expected, 1e-6);
}

TEST(CameraInfoFovInitTest, EmptyCameraInfoTopicSetsDependencyErrorAndDoesNotSubscribe)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(clock);

  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer layer;
  layer._observation_manager = std::make_unique<spatio_temporal_voxel_layer::internal::ObservationManager>();

  const std::string source_name = "rgbd_clear";
  auto buffer = makeDepthCameraBuffer(source_name, tf_buffer.get(), clock);
  layer._observation_manager->registerBuffer(buffer, false, true);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("camera_info_fov_test_node_empty_topic");

  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer::ObservationSourceConfig cfg;
  cfg.name = source_name;
  cfg.fov_from_camera_info = true;
  cfg.camera_info_topic = "";
  cfg.camera_info_required = true;
  cfg.model_type = ModelType::DEPTH_CAMERA;

  layer.initializeCameraInfoFovs(node, {cfg});

  const std::string dep_name = source_name + std::string("/camera_info");
  ASSERT_TRUE(layer.camera_info_dependencies_.count(dep_name) > 0U);
  EXPECT_EQ(layer.camera_info_subscriptions_.count(dep_name), 0U);

  std::lock_guard<std::mutex> lock(layer.camera_info_mutex_);
  const auto & dep = layer.camera_info_dependencies_.at(dep_name);
  EXPECT_FALSE(dep.initialized);
  EXPECT_NE(dep.error_msg.find("camera_info_topic is empty"), std::string::npos);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
