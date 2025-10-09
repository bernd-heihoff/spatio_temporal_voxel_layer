#include "spatio_temporal_voxel_layer/internal/pruning_manager.hpp"

#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace spatio_temporal_voxel_layer::internal
{

PruningManager::PruningManager(tf2_ros::Buffer * tf_buffer, const std::string & global_frame)
: tf_(tf_buffer), global_frame_(global_frame)
{
}

void PruningManager::setConfig(const PruningConfig & config)
{
  config_ = config;
  if (config_.z_min > config_.z_max) {
    std::swap(config_.z_min, config_.z_max);
  }
  if (config_.z_min == config_.z_max) {
    config_.z_min -= config_.voxel_size;
    config_.z_max += config_.voxel_size;
  }
}

void PruningManager::resetState(const rclcpp::Time & now, double origin_x, double origin_y)
{
  last_prune_time_ = now;
  last_origin_x_ = origin_x;
  last_origin_y_ = origin_y;
}

bool PruningManager::pruneIfNeeded(
  const PruningContext & context,
  const rclcpp::Time & now,
  volume_grid::SpatioTemporalVoxelGrid & grid,
  rclcpp::Logger logger)
{
  last_bounding_box_.reset();

  if (!config_.enabled || config_.mapping_mode) {
    updateLastOrigin(context.origin_x, context.origin_y);
    return false;
  }

  if (!shouldPrune(context, now)) {
    return false;
  }

  openvdb::BBoxd bbox;
  if (!computeBoundingBox(context, bbox, logger)) {
    last_prune_time_ = now;
    updateLastOrigin(context.origin_x, context.origin_y);
    return false;
  }

  last_bounding_box_ = bbox;

  if (grid.ClipToBoundingBox(bbox)) {
    last_prune_time_ = now;
    updateLastOrigin(context.origin_x, context.origin_y);
    return true;
  }

  last_prune_time_ = now;
  updateLastOrigin(context.origin_x, context.origin_y);
  return false;
}

const rclcpp::Time & PruningManager::lastPruneTime() const
{
  return last_prune_time_;
}

double PruningManager::lastOriginX() const
{
  return last_origin_x_;
}

double PruningManager::lastOriginY() const
{
  return last_origin_y_;
}

void PruningManager::updateLastOrigin(double origin_x, double origin_y)
{
  last_origin_x_ = origin_x;
  last_origin_y_ = origin_y;
}

void PruningManager::updateLastPruneTime(const rclcpp::Time & time)
{
  last_prune_time_ = time;
}

const PruningConfig & PruningManager::config() const
{
  return config_;
}

const std::optional<openvdb::BBoxd> & PruningManager::lastBoundingBox() const
{
  return last_bounding_box_;
}

bool PruningManager::shouldPrune(const PruningContext & context, const rclcpp::Time & now) const
{
  if (config_.distance > 0.0) {
    if (std::isfinite(last_origin_x_) && std::isfinite(last_origin_y_)) {
      const double dx = context.origin_x - last_origin_x_;
      const double dy = context.origin_y - last_origin_y_;
      if (std::hypot(dx, dy) >= config_.distance) {
        return true;
      }
    } else {
      return true;
    }
  }

  if (now >= last_prune_time_ + config_.interval) {
    return true;
  }

  return false;
}

bool PruningManager::computeBoundingBox(
  const PruningContext & context,
  openvdb::BBoxd & bbox,
  rclcpp::Logger logger) const
{
  const double min_x = context.origin_x - config_.padding;
  const double max_x = context.origin_x + context.size_x + config_.padding;
  const double min_y = context.origin_y - config_.padding;
  const double max_y = context.origin_y + context.size_y + config_.padding;

  double min_z = config_.z_min;
  double max_z = config_.z_max;

  if (tf_ && !config_.base_frame.empty()) {
    try {
      const geometry_msgs::msg::TransformStamped base_in_global = tf_->lookupTransform(
        context.global_frame,
        config_.base_frame,
        tf2::TimePointZero);
      const double robot_z = static_cast<double>(base_in_global.transform.translation.z);
      min_z = robot_z + config_.z_min;
      max_z = robot_z + config_.z_max;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger, "Failed to center prune bounding box on robot: %s", ex.what());
    }
  }

  if (!(min_x < max_x && min_y < max_y && min_z < max_z)) {
    return false;
  }

  bbox = openvdb::BBoxd(
    openvdb::Vec3d(min_x, min_y, min_z),
    openvdb::Vec3d(max_x, max_y, max_z));
  return true;
}

}  // namespace spatio_temporal_voxel_layer::internal
