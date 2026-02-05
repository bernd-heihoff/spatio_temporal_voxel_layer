#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "spatio_temporal_voxel_layer/internal/heartbeat_evaluator.hpp"

using spatio_temporal_voxel_layer::internal::heartbeat::CostmapCycleState;
using spatio_temporal_voxel_layer::internal::heartbeat::HeartbeatConfig;
using spatio_temporal_voxel_layer::internal::heartbeat::SourceState;

namespace
{

rclcpp::Time t(int32_t sec, uint32_t nsec = 0U)
{
  return rclcpp::Time(sec, nsec, RCL_ROS_TIME);
}

HeartbeatConfig defaultConfig()
{
  HeartbeatConfig c;
  c.costmap_timeout_s = 5.0;
  c.default_sensor_timeout_s = 1.0;
  c.min_sensor_timeout_s = 0.2;
  c.expected_update_rate_multiplier = 2.0;
  return c;
}

CostmapCycleState healthyCostmap(const rclcpp::Time & now)
{
  CostmapCycleState s;
  s.layer_enabled = true;
  s.observation_manager_initialized = true;
  s.voxel_grid_initialized = true;
  s.has_null_measurement_buffer = false;

  s.update_bounds_last_success = now - rclcpp::Duration::from_seconds(0.1);
  s.update_bounds_last_error = t(0);
  s.update_bounds_last_error_msg.clear();

  s.update_costs_last_success = now - rclcpp::Duration::from_seconds(0.1);
  s.update_costs_last_error = t(0);
  s.update_costs_last_error_msg.clear();

  return s;
}

SourceState requiredSource(const std::string & name, const rclcpp::Time & last_success)
{
  SourceState s;
  s.source_name = name;
  s.required = true;
  s.enabled = true;
  s.expected_update_rate_s = 0.1;
  s.last_success = last_success;
  s.last_error = t(0);
  s.last_error_msg.clear();
  return s;
}

}  // namespace

TEST(HeartbeatEvaluatorTest, HealthyWhenFreshAndNoErrors)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  const auto costmap = healthyCostmap(now);
  const std::vector<SourceState> sources{requiredSource("camera", now - rclcpp::Duration::from_seconds(0.1))};

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, sources);

  EXPECT_TRUE(result.healthy);
  EXPECT_EQ(result.reason, "OK");
}

TEST(HeartbeatEvaluatorTest, LayerDisabledFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  auto costmap = healthyCostmap(now);
  costmap.layer_enabled = false;

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("layer disabled"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, NullMeasurementBufferFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  auto costmap = healthyCostmap(now);
  costmap.has_null_measurement_buffer = true;

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("null measurement buffer"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, UpdateBoundsNeverSucceededFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  auto costmap = healthyCostmap(now);
  costmap.update_bounds_last_success = t(0);

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("updateBounds never succeeded"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, UpdateCostsStaleFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  auto costmap = healthyCostmap(now);
  costmap.update_costs_last_success = now - rclcpp::Duration::from_seconds(10.0);

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("updateCosts stale"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, RequiredSourceStaleFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  const auto costmap = healthyCostmap(now);

  // expected rate is 0.1s; multiplier 2.0; min sensor timeout 0.2s => 0.2s timeout
  auto source = requiredSource("camera", now - rclcpp::Duration::from_seconds(1.0));

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {source});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("camera stale"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, RequiredSourceErrorAfterSuccessFailsClosed)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  const auto costmap = healthyCostmap(now);

  SourceState source;
  source.source_name = "camera";
  source.required = true;
  source.enabled = true;
  source.expected_update_rate_s = 0.0;
  source.last_success = now - rclcpp::Duration::from_seconds(1.0);
  source.last_error = now - rclcpp::Duration::from_seconds(0.1);
  source.last_error_msg = "boom";

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {source});

  EXPECT_FALSE(result.healthy);
  EXPECT_NE(result.reason.find("camera last error: boom"), std::string::npos);
}

TEST(HeartbeatEvaluatorTest, NonRequiredSourceDoesNotGateHeartbeat)
{
  const auto now = t(100);
  const auto config = defaultConfig();
  const auto costmap = healthyCostmap(now);

  SourceState source;
  source.source_name = "clearing_lidar";
  source.required = false;
  source.enabled = false;
  source.expected_update_rate_s = 0.0;
  source.last_success = t(0);
  source.last_error = now;
  source.last_error_msg = "ignored";

  const auto result = spatio_temporal_voxel_layer::internal::heartbeat::evaluateHeartbeat(
    now, config, costmap, {source});

  EXPECT_TRUE(result.healthy);
  EXPECT_EQ(result.reason, "OK");
}
