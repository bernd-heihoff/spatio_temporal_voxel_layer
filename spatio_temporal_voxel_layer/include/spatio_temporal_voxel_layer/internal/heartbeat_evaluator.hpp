#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__HEARTBEAT_EVALUATOR_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__HEARTBEAT_EVALUATOR_HPP_

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace spatio_temporal_voxel_layer
{
namespace internal
{
namespace heartbeat
{

struct HeartbeatConfig
{
  double costmap_timeout_s{0.0};
  double default_sensor_timeout_s{0.0};
  double min_sensor_timeout_s{0.0};
  double expected_update_rate_multiplier{1.0};
};

struct CostmapCycleState
{
  bool layer_enabled{true};
  bool observation_manager_initialized{true};
  bool voxel_grid_initialized{true};
  bool has_null_measurement_buffer{false};

  rclcpp::Time update_bounds_last_success;
  rclcpp::Time update_bounds_last_error;
  std::string update_bounds_last_error_msg;

  rclcpp::Time update_costs_last_success;
  rclcpp::Time update_costs_last_error;
  std::string update_costs_last_error_msg;
};

struct SourceState
{
  std::string source_name;
  bool required{true};
  bool enabled{true};
  double expected_update_rate_s{0.0};
  rclcpp::Time last_success;
  rclcpp::Time last_error;
  std::string last_error_msg;
};

struct HeartbeatResult
{
  bool healthy{false};
  std::string reason;
};

inline double computeSensorTimeoutSeconds(const HeartbeatConfig & config, double expected_rate_s)
{
  if (expected_rate_s > 0.0) {
    return std::max(
      config.min_sensor_timeout_s,
      expected_rate_s * config.expected_update_rate_multiplier);
  }

  return config.default_sensor_timeout_s;
}

inline void appendReason(std::ostringstream & reason, const std::string & msg)
{
  if (!reason.str().empty()) {
    reason << "; ";
  }
  reason << msg;
}

inline HeartbeatResult evaluateHeartbeat(
  const rclcpp::Time & now,
  const HeartbeatConfig & config,
  const CostmapCycleState & costmap,
  const std::vector<SourceState> & sources)
{
  bool healthy = true;
  std::ostringstream reason;

  const auto costmap_timeout = rclcpp::Duration::from_seconds(config.costmap_timeout_s);

  if (!costmap.layer_enabled) {
    healthy = false;
    appendReason(reason, "layer disabled");
  }

  if (!costmap.observation_manager_initialized) {
    healthy = false;
    appendReason(reason, "observation manager not initialized");
  }

  if (!costmap.voxel_grid_initialized) {
    healthy = false;
    appendReason(reason, "voxel grid not initialized");
  }

  if (costmap.update_bounds_last_error > costmap.update_bounds_last_success) {
    healthy = false;
    appendReason(reason, "updateBounds error: " + costmap.update_bounds_last_error_msg);
  }

  if (costmap.update_bounds_last_success.nanoseconds() == 0) {
    healthy = false;
    appendReason(reason, "updateBounds never succeeded");
  } else if ((now - costmap.update_bounds_last_success) > costmap_timeout) {
    healthy = false;
    appendReason(reason, "updateBounds stale");
  }

  if (costmap.update_costs_last_error > costmap.update_costs_last_success) {
    healthy = false;
    appendReason(reason, "updateCosts error: " + costmap.update_costs_last_error_msg);
  }

  if (costmap.update_costs_last_success.nanoseconds() == 0) {
    healthy = false;
    appendReason(reason, "updateCosts never succeeded");
  } else if ((now - costmap.update_costs_last_success) > costmap_timeout) {
    healthy = false;
    appendReason(reason, "updateCosts stale");
  }

  if (costmap.has_null_measurement_buffer) {
    healthy = false;
    appendReason(reason, "null measurement buffer");
  }

  for (const auto & source : sources) {
    if (!source.required) {
      continue;
    }

    if (!source.enabled) {
      healthy = false;
      appendReason(reason, source.source_name + " disabled");
      continue;
    }

    const auto timeout_s = computeSensorTimeoutSeconds(config, source.expected_update_rate_s);
    const auto timeout = rclcpp::Duration::from_seconds(timeout_s);

    if (source.last_error > source.last_success) {
      healthy = false;
      appendReason(reason, source.source_name + " last error: " + source.last_error_msg);
      continue;
    }

    if (source.last_success.nanoseconds() == 0) {
      healthy = false;
      appendReason(reason, source.source_name + " never buffered successfully");
      continue;
    }

    if ((now - source.last_success) > timeout) {
      healthy = false;
      std::ostringstream msg;
      msg << source.source_name << " stale (no successful buffer in " << timeout_s << "s)";
      appendReason(reason, msg.str());
      continue;
    }
  }

  HeartbeatResult result;
  result.healthy = healthy;
  result.reason = healthy ? std::string("OK") : reason.str();
  return result;
}

}  // namespace heartbeat
}  // namespace internal
}  // namespace spatio_temporal_voxel_layer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__HEARTBEAT_EVALUATOR_HPP_
