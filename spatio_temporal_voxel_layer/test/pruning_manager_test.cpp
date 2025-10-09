#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

#include "spatio_temporal_voxel_layer/internal/pruning_manager.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

using spatio_temporal_voxel_layer::internal::PruningConfig;
using spatio_temporal_voxel_layer::internal::PruningContext;
using spatio_temporal_voxel_layer::internal::PruningManager;

namespace
{

PruningConfig defaultConfig()
{
  PruningConfig config;
  config.enabled = true;
  config.mapping_mode = false;
  config.padding = 0.5;
  config.distance = 0.0;
  config.interval = rclcpp::Duration::from_seconds(0.0);
  config.z_min = -1.0;
  config.z_max = 1.0;
  config.voxel_size = 0.1;
  config.base_frame.clear();
  return config;
}

PruningContext makeContext(double origin_x, double origin_y)
{
  PruningContext ctx;
  ctx.origin_x = origin_x;
  ctx.origin_y = origin_y;
  ctx.size_x = 10.0;
  ctx.size_y = 5.0;
  ctx.global_frame = "map";
  return ctx;
}

}  // namespace

TEST(PruningManagerTest, DisabledPruningSkipsBoundingBox)
{
  PruningManager manager(nullptr, "map");
  auto config = defaultConfig();
  config.enabled = false;
  manager.setConfig(config);
  manager.resetState(rclcpp::Time(0, 0, RCL_ROS_TIME), 0.0, 0.0);

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  volume_grid::SpatioTemporalVoxelGrid::TimeSource time_source =
    [clock]() -> double {return clock->now().seconds();};
  volume_grid::SpatioTemporalVoxelGrid grid(
    time_source, 0.1f, 0.0, volume_grid::GlobalDecayModel::PERSISTENT, -1.0, false);

  const bool pruned = manager.pruneIfNeeded(
    makeContext(0.0, 0.0), rclcpp::Time(1, 0, RCL_ROS_TIME), grid, rclcpp::get_logger("test"));

  EXPECT_FALSE(pruned);
  EXPECT_FALSE(manager.lastBoundingBox().has_value());
}

TEST(PruningManagerTest, BoundingBoxComputedWhenConditionsMet)
{
  PruningManager manager(nullptr, "map");
  auto config = defaultConfig();
  config.distance = 0.1;
  manager.setConfig(config);
  manager.resetState(rclcpp::Time(0, 0, RCL_ROS_TIME), 0.0, 0.0);

  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  volume_grid::SpatioTemporalVoxelGrid::TimeSource time_source =
    [clock]() -> double {return clock->now().seconds();};
  volume_grid::SpatioTemporalVoxelGrid grid(
    time_source, 0.1f, 0.0, volume_grid::GlobalDecayModel::PERSISTENT, -1.0, false);

  const double origin_x = 2.0;
  const double origin_y = -1.0;
  const auto now = rclcpp::Time(2, 0, RCL_ROS_TIME);
  const bool pruned = manager.pruneIfNeeded(
    makeContext(origin_x, origin_y), now, grid, rclcpp::get_logger("test"));

  EXPECT_FALSE(pruned);  // grid is empty but pruning logic still evaluated
  ASSERT_TRUE(manager.lastBoundingBox().has_value());
  const auto & bbox = manager.lastBoundingBox().value();
  EXPECT_DOUBLE_EQ(bbox.min().x(), origin_x - config.padding);
  EXPECT_DOUBLE_EQ(bbox.max().x(), origin_x + 10.0 + config.padding);
  EXPECT_DOUBLE_EQ(bbox.min().y(), origin_y - config.padding);
  EXPECT_DOUBLE_EQ(bbox.max().y(), origin_y + 5.0 + config.padding);
  EXPECT_DOUBLE_EQ(bbox.min().z(), config.z_min);
  EXPECT_DOUBLE_EQ(bbox.max().z(), config.z_max);
  EXPECT_EQ(manager.lastPruneTime(), now);
  EXPECT_DOUBLE_EQ(manager.lastOriginX(), origin_x);
  EXPECT_DOUBLE_EQ(manager.lastOriginY(), origin_y);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
