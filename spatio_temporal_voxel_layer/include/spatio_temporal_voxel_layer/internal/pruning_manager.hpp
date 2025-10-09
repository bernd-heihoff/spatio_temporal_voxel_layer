/*
 * PruningManager provides encapsulated logic for deciding when and how to
 * prune the underlying voxel grid. Extracting this behavior improves
 * testability by allowing the pruning heuristics to be validated without the
 * full costmap plugin wiring.
 */

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__PRUNING_MANAGER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__PRUNING_MANAGER_HPP_

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "openvdb/math/BBox.h"

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"

namespace spatio_temporal_voxel_layer::internal
{

struct PruningConfig
{
  bool enabled{false};
  bool mapping_mode{false};
  double padding{0.0};
  double distance{0.0};
  rclcpp::Duration interval{rclcpp::Duration(0, 0)};
  double z_min{-1.0};
  double z_max{1.0};
  double voxel_size{0.05};
  std::string base_frame{"base_link"};
};

struct PruningContext
{
  double origin_x{0.0};
  double origin_y{0.0};
  double size_x{0.0};
  double size_y{0.0};
  std::string global_frame;
};

class PruningManager
{
public:
  PruningManager(tf2_ros::Buffer * tf_buffer, const std::string & global_frame);

  void setConfig(const PruningConfig & config);

  void resetState(const rclcpp::Time & now, double origin_x, double origin_y);

  bool pruneIfNeeded(
    const PruningContext & context,
    const rclcpp::Time & now,
    volume_grid::SpatioTemporalVoxelGrid & grid,
    rclcpp::Logger logger);

  const rclcpp::Time & lastPruneTime() const;
  double lastOriginX() const;
  double lastOriginY() const;

  void updateLastOrigin(double origin_x, double origin_y);
  void updateLastPruneTime(const rclcpp::Time & time);

  const PruningConfig & config() const;
  const std::optional<openvdb::BBoxd> & lastBoundingBox() const;

private:
  bool shouldPrune(const PruningContext & context, const rclcpp::Time & now) const;
  bool computeBoundingBox(
    const PruningContext & context,
    openvdb::BBoxd & bbox,
    rclcpp::Logger logger) const;

  tf2_ros::Buffer * tf_;
  std::string global_frame_;
  PruningConfig config_;
  rclcpp::Time last_prune_time_;
  double last_origin_x_{std::numeric_limits<double>::quiet_NaN()};
  double last_origin_y_{std::numeric_limits<double>::quiet_NaN()};
  std::optional<openvdb::BBoxd> last_bounding_box_;
};

}  // namespace spatio_temporal_voxel_layer::internal

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__PRUNING_MANAGER_HPP_
