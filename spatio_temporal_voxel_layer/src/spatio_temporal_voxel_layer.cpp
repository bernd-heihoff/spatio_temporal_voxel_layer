/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
 *  Copyright (c) 2021, Samsung Research America
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *                         stevenmacenski@gmail.com
 *********************************************************************/

#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <sstream>

#include <pcl_conversions/pcl_conversions.h>

#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"
#include "spatio_temporal_voxel_layer/bridge/point_cloud_conversions.hpp"
#include "spatio_temporal_voxel_layer/internal/elevation_lethal.hpp"
#include "spatio_temporal_voxel_layer/internal/heartbeat_evaluator.hpp"
#include "openvdb/math/BBox.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/camera_info.hpp"

#include <Eigen/Geometry>
#include "visualization_msgs/msg/marker.hpp"

namespace spatio_temporal_voxel_layer
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using rcl_interfaces::msg::ParameterType;

/*****************************************************************************/
SpatioTemporalVoxelLayer::SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
SpatioTemporalVoxelLayer::~SpatioTemporalVoxelLayer(void)
/*****************************************************************************/
{
  _voxel_grid.reset();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::declareLayerParameters()
/*****************************************************************************/
{
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));
  declareParameter("publish_elevation_map", rclcpp::ParameterValue(false));
  declareParameter("voxel_size", rclcpp::ParameterValue(0.05));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("mark_threshold", rclcpp::ParameterValue(0));
  declareParameter("update_footprint_enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "track_unknown_space",
    rclcpp::ParameterValue(layered_costmap_->isTrackingUnknown()));
  declareParameter("decay_model", rclcpp::ParameterValue(0));
  declareParameter("voxel_decay", rclcpp::ParameterValue(-1.0));
  declareParameter("mapping_mode", rclcpp::ParameterValue(false));
  declareParameter("map_save_duration", rclcpp::ParameterValue(60.0));
  declareParameter(
    "max_elevation_above_robot_base",
    rclcpp::ParameterValue(-1.0));

  // Elevation-based lethal obstacle generation (disabled by default)
  // If enabled, marks a cell as lethal when max-min elevation within a local
  // square window exceeds the configured threshold.
  declareParameter("elevation_window_size", rclcpp::ParameterValue(0.0));
  declareParameter("elevation_lethal_threshold", rclcpp::ParameterValue(0.0));
  // Relative fraction (0..1) of window cells that must have finite elevation.
  declareParameter("elevation_window_min_samples", rclcpp::ParameterValue(0.0));

  declareParameter("prune_enabled", rclcpp::ParameterValue(false));
  declareParameter("prune_padding", rclcpp::ParameterValue(0.5));
  declareParameter("prune_distance", rclcpp::ParameterValue(0.0));
  declareParameter("prune_interval", rclcpp::ParameterValue(0.5));
  declareParameter("prune_z_min", rclcpp::ParameterValue(-1.0e6));
  declareParameter("prune_z_max", rclcpp::ParameterValue(1.0e6));
  declareParameter(
    "prune_robot_base_frame",
    rclcpp::ParameterValue(std::string("base_link")));

  // Heartbeat / health monitoring (disabled by default)
  declareParameter("publish_heartbeat", rclcpp::ParameterValue(false));
  declareParameter("heartbeat_topic", rclcpp::ParameterValue(std::string("heartbeat")));
  declareParameter("heartbeat_status_topic", rclcpp::ParameterValue(std::string("heartbeat_status")));
  declareParameter("heartbeat_period", rclcpp::ParameterValue(0.2));
  declareParameter("heartbeat_costmap_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("heartbeat_default_sensor_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("heartbeat_min_sensor_timeout", rclcpp::ParameterValue(0.2));
  declareParameter("heartbeat_expected_update_rate_multiplier", rclcpp::ParameterValue(2.5));

  // Debug visualization (disabled by default)
  declareParameter("publish_frustums", rclcpp::ParameterValue(false));
  declareParameter("frustum_topic", rclcpp::ParameterValue(std::string("frustums")));
  declareParameter("frustum_lifetime", rclcpp::ParameterValue(0.2));
  declareParameter("frustum_line_width", rclcpp::ParameterValue(0.03));
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::loadLayerParameters(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  bool & track_unknown_space,
  double & transform_tolerance,
  double & map_save_time)
/*****************************************************************************/
{
  track_unknown_space = layered_costmap_->isTrackingUnknown();
  transform_tolerance = 0.2;
  map_save_time = 60.0;

  node->get_parameter(name_ + ".observation_sources", _topics_string);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(name_ + ".enabled", _enabled);
  enabled_ = _enabled;
  node->get_parameter(name_ + ".publish_voxel_map", _publish_voxels);
  node->get_parameter(name_ + ".publish_elevation_map", _publish_elevation_map);
  node->get_parameter(name_ + ".voxel_size", _voxel_size);
  node->get_parameter(name_ + ".combination_method", _combination_method);
  node->get_parameter(name_ + ".mark_threshold", _mark_threshold);
  node->get_parameter(name_ + ".update_footprint_enabled", _update_footprint_enabled);
  node->get_parameter(name_ + ".track_unknown_space", track_unknown_space);
  int decay_model_int = 0;
  node->get_parameter(name_ + ".decay_model", decay_model_int);
  _decay_model = static_cast<volume_grid::GlobalDecayModel>(decay_model_int);
  node->get_parameter(name_ + ".voxel_decay", _voxel_decay);
  node->get_parameter(name_ + ".mapping_mode", _mapping_mode);
  node->get_parameter(name_ + ".map_save_duration", map_save_time);

  double max_elevation_above_robot_base = -1.0;
  node->get_parameter(
    name_ + ".max_elevation_above_robot_base",
    max_elevation_above_robot_base);
  if (max_elevation_above_robot_base > 0.0) {
    _max_elevation_above_robot_base = max_elevation_above_robot_base;
    _limit_elevation = true;
  } else {
    _max_elevation_above_robot_base = std::numeric_limits<double>::infinity();
    _limit_elevation = false;
  }

  node->get_parameter(name_ + ".elevation_window_size", _elevation_window_size_m);
  _elevation_window_size_m = std::max(0.0, _elevation_window_size_m);

  node->get_parameter(name_ + ".elevation_lethal_threshold", _elevation_lethal_threshold_m);
  _elevation_lethal_threshold_m = std::max(0.0, _elevation_lethal_threshold_m);

  node->get_parameter(name_ + ".elevation_window_min_samples", _elevation_window_min_samples);
  _elevation_window_min_samples = std::clamp(_elevation_window_min_samples, 0.0, 1.0);

  node->get_parameter(name_ + ".prune_enabled", _pruning_config.enabled);

  node->get_parameter(name_ + ".prune_padding", _pruning_config.padding);
  _pruning_config.padding = std::max(0.0, _pruning_config.padding);

  node->get_parameter(name_ + ".prune_distance", _pruning_config.distance);
  _pruning_config.distance = std::max(0.0, _pruning_config.distance);

  double prune_interval_seconds = 0.5;
  node->get_parameter(name_ + ".prune_interval", prune_interval_seconds);
  if (prune_interval_seconds < 0.0) {
    RCLCPP_WARN(
      logger_, "%s prune_interval must be non-negative, clamping to 0.0", getName().c_str());
    prune_interval_seconds = 0.0;
  }
  _pruning_config.interval = rclcpp::Duration::from_seconds(prune_interval_seconds);

  node->get_parameter(name_ + ".prune_z_min", _pruning_config.z_min);
  node->get_parameter(name_ + ".prune_z_max", _pruning_config.z_max);
  if (_pruning_config.z_min > _pruning_config.z_max) {
    RCLCPP_WARN(
      logger_, "%s prune_z_min > prune_z_max, swapping values.", getName().c_str());
    std::swap(_pruning_config.z_min, _pruning_config.z_max);
  }
  if (_pruning_config.z_min == _pruning_config.z_max) {
    _pruning_config.z_min -= _voxel_size;
    _pruning_config.z_max += _voxel_size;
  }

  node->get_parameter(name_ + ".prune_robot_base_frame", _pruning_config.base_frame);
  if (_pruning_config.base_frame.empty()) {
    RCLCPP_WARN(
      logger_, "%s prune_robot_base_frame is empty, defaulting to base_link.",
      getName().c_str());
    _pruning_config.base_frame = "base_link";
  }

  _pruning_config.voxel_size = _voxel_size;
  _pruning_config.mapping_mode = _mapping_mode;

  node->get_parameter(name_ + ".publish_heartbeat", publish_heartbeat_);
  node->get_parameter(name_ + ".heartbeat_topic", heartbeat_topic_);
  node->get_parameter(name_ + ".heartbeat_status_topic", heartbeat_status_topic_);
  node->get_parameter(name_ + ".heartbeat_period", heartbeat_period_s_);
  node->get_parameter(name_ + ".heartbeat_costmap_timeout", heartbeat_costmap_timeout_s_);
  node->get_parameter(name_ + ".heartbeat_default_sensor_timeout", heartbeat_default_sensor_timeout_s_);
  node->get_parameter(name_ + ".heartbeat_min_sensor_timeout", heartbeat_min_sensor_timeout_s_);
  node->get_parameter(
    name_ + ".heartbeat_expected_update_rate_multiplier",
    heartbeat_expected_update_rate_multiplier_);

  node->get_parameter(name_ + ".publish_frustums", publish_frustums_);
  node->get_parameter(name_ + ".frustum_topic", frustum_topic_);
  node->get_parameter(name_ + ".frustum_lifetime", frustum_lifetime_s_);
  node->get_parameter(name_ + ".frustum_line_width", frustum_line_width_);

  frustum_lifetime_s_ = std::max(0.0, frustum_lifetime_s_);
  frustum_line_width_ = std::max(0.001, frustum_line_width_);

  heartbeat_period_s_ = std::max(0.05, heartbeat_period_s_);
  heartbeat_costmap_timeout_s_ = std::max(heartbeat_period_s_, heartbeat_costmap_timeout_s_);
  heartbeat_default_sensor_timeout_s_ = std::max(heartbeat_period_s_, heartbeat_default_sensor_timeout_s_);
  heartbeat_min_sensor_timeout_s_ = std::max(0.0, heartbeat_min_sensor_timeout_s_);
  heartbeat_expected_update_rate_multiplier_ = std::max(1.0, heartbeat_expected_update_rate_multiplier_);
}

namespace
{

inline std_msgs::msg::ColorRGBA makeDebugColorFromName(const std::string & name)
{
  // Deterministic bright-ish color. (Not cryptographic; just for visualization.)
  uint32_t h = 2166136261u;
  for (const unsigned char c : name) {
    h ^= static_cast<uint32_t>(c);
    h *= 16777619u;
  }

  const float r = 0.35f + 0.65f * static_cast<float>((h >> 0) & 0xFF) / 255.0f;
  const float g = 0.35f + 0.65f * static_cast<float>((h >> 8) & 0xFF) / 255.0f;
  const float b = 0.35f + 0.65f * static_cast<float>((h >> 16) & 0xFF) / 255.0f;

  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = 1.0f;
  return c;
}

}  // namespace

/*****************************************************************************/
SpatioTemporalVoxelLayer::ObservationSourceConfig
SpatioTemporalVoxelLayer::loadObservationSourceConfig(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & source)
/*****************************************************************************/
{
  ObservationSourceConfig config;
  config.name = source;

  declareParameter(source + "." + "topic", rclcpp::ParameterValue(std::string("")));
  declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
  declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
  declareParameter(
    source + "." + "data_type",
    rclcpp::ParameterValue(std::string("PointCloud2")));
  declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(3.0));
  declareParameter(source + "." + "filter_obstacle_height", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "height_relative_to_base", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "required_for_heartbeat", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.5));

  declareParameter(source + "." + "min_z", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "max_z", rclcpp::ParameterValue(10.0));
  declareParameter(source + "." + "use_clearing_min_max_z", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "vertical_fov_angle", rclcpp::ParameterValue(0.7));
  declareParameter(source + "." + "vertical_fov_padding", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "horizontal_fov_angle", rclcpp::ParameterValue(1.04));

  // CameraInfo-derived FOV (evaluated once at startup)
  declareParameter(source + "." + "fov_from_camera_info", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "camera_info_topic", rclcpp::ParameterValue(std::string("")));
  declareParameter(source + "." + "camera_info_required", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "camera_info_timeout", rclcpp::ParameterValue(1.0));
  declareParameter(source + "." + "vertical_fov_padding_rad", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "horizontal_fov_padding_rad", rclcpp::ParameterValue(0.0));

  declareParameter(source + "." + "decay_acceleration", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "filter", rclcpp::ParameterValue(std::string("passthrough")));
  declareParameter(source + "." + "voxel_min_points", rclcpp::ParameterValue(0));
  declareParameter(source + "." + "clear_after_reading", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "enabled", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "model_type", rclcpp::ParameterValue(0));

  node->get_parameter(name_ + "." + source + "." + "topic", config.topic);
  node->get_parameter(name_ + "." + source + "." + "sensor_frame", config.sensor_frame);
  node->get_parameter(name_ + "." + source + "." + "observation_persistence", config.observation_keep_time);
  node->get_parameter(name_ + "." + source + "." + "expected_update_rate", config.expected_update_rate);
  node->get_parameter(name_ + "." + source + "." + "data_type", config.data_type);
  node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", config.min_obstacle_height);
  node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", config.max_obstacle_height);
  node->get_parameter(name_ + "." + source + "." + "filter_obstacle_height", config.filter_obstacle_height);
  node->get_parameter(name_ + "." + source + "." + "height_relative_to_base", config.height_relative_to_base);
  node->get_parameter(name_ + "." + source + "." + "required_for_heartbeat", config.required_for_heartbeat);
  node->get_parameter(name_ + "." + source + "." + "inf_is_valid", config.inf_is_valid);
  node->get_parameter(name_ + "." + source + "." + "marking", config.marking);
  node->get_parameter(name_ + "." + source + "." + "clearing", config.clearing);
  node->get_parameter(name_ + "." + source + "." + "obstacle_range", config.obstacle_range);

  node->get_parameter(name_ + "." + source + "." + "min_z", config.min_z);
  node->get_parameter(name_ + "." + source + "." + "max_z", config.max_z);
  node->get_parameter(name_ + "." + source + "." + "use_clearing_min_max_z", config.use_clearing_min_max_z);
  node->get_parameter(name_ + "." + source + "." + "vertical_fov_angle", config.vertical_fov);
  node->get_parameter(name_ + "." + source + "." + "vertical_fov_padding", config.vertical_fov_padding);
  node->get_parameter(name_ + "." + source + "." + "horizontal_fov_angle", config.horizontal_fov);

  node->get_parameter(name_ + "." + source + "." + "fov_from_camera_info", config.fov_from_camera_info);
  node->get_parameter(name_ + "." + source + "." + "camera_info_topic", config.camera_info_topic);
  node->get_parameter(name_ + "." + source + "." + "camera_info_required", config.camera_info_required);
  node->get_parameter(name_ + "." + source + "." + "camera_info_timeout", config.camera_info_timeout_s);
  node->get_parameter(name_ + "." + source + "." + "vertical_fov_padding_rad", config.vertical_fov_padding_rad);
  node->get_parameter(name_ + "." + source + "." + "horizontal_fov_padding_rad", config.horizontal_fov_padding_rad);

  node->get_parameter(name_ + "." + source + "." + "decay_acceleration", config.decay_acceleration);

  std::string filter_str;
  node->get_parameter(name_ + "." + source + "." + "filter", filter_str);
  if (filter_str == "passthrough") {
    RCLCPP_INFO(logger_, "Passthough filter activated.");
    config.filter = buffer::Filters::PASSTHROUGH;
  } else if (filter_str == "voxel") {
    RCLCPP_INFO(logger_, "Voxel filter activated.");
    config.filter = buffer::Filters::VOXEL;
  } else {
    RCLCPP_INFO(logger_, "No filters activated.");
    config.filter = buffer::Filters::NONE;
  }

  node->get_parameter(name_ + "." + source + "." + "voxel_min_points", config.voxel_min_points);
  node->get_parameter(name_ + "." + source + "." + "clear_after_reading", config.clear_after_reading);
  node->get_parameter(name_ + "." + source + "." + "enabled", config.enabled);

  int model_type_int = 0;
  node->get_parameter(name_ + "." + source + "." + "model_type", model_type_int);
  config.model_type = static_cast<ModelType>(model_type_int);

  if (config.fov_from_camera_info) {
    // Fail-closed for clearing: start with invalid frustum until CameraInfo-derived FOV is applied.
    // If camera_info_required is false, the configured angles remain usable as fallback.
    if (config.camera_info_required) {
      config.vertical_fov = 0.0;
      config.horizontal_fov = 0.0;
    }

    config.camera_info_timeout_s = std::max(0.0, config.camera_info_timeout_s);
    config.vertical_fov_padding_rad = std::max(0.0, config.vertical_fov_padding_rad);
    config.horizontal_fov_padding_rad = std::max(0.0, config.horizontal_fov_padding_rad);
  }

  if (!(config.data_type == "PointCloud2" || config.data_type == "LaserScan")) {
    throw std::runtime_error("Only topics that use pointclouds or laser scans are supported.");
  }

  return config;
}

/*****************************************************************************/
buffer::MeasurementBufferConfig SpatioTemporalVoxelLayer::createMeasurementBufferConfig(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const ObservationSourceConfig & config,
  double transform_tolerance) const
/*****************************************************************************/
{
  return buffer::MeasurementBufferBuilder{}
         .setSourceName(config.name)
         .setTopicName(config.topic)
         .setObservationKeepTime(config.observation_keep_time)
         .setExpectedUpdateRate(config.expected_update_rate)
         .setMinObstacleHeight(config.min_obstacle_height)
         .setMaxObstacleHeight(config.max_obstacle_height)
        .setFilterObstacleHeight(config.filter_obstacle_height)
      .setHeightRelativeToBase(config.height_relative_to_base)
      .setRobotBaseFrame(_pruning_config.base_frame)
         .setObstacleRange(config.obstacle_range)
         .setTfBuffer(tf_)
         .setGlobalFrame(_global_frame)
         .setSensorFrame(config.sensor_frame)
         .setTfTolerance(transform_tolerance)
         .setMinZ(config.min_z)
         .setMaxZ(config.max_z)
         .setUseClearingMinMaxZ(config.use_clearing_min_max_z)
         .setVerticalFov(config.vertical_fov)
         .setVerticalFovPadding(config.vertical_fov_padding)
         .setHorizontalFov(config.horizontal_fov)
         .setDecayAcceleration(config.decay_acceleration)
         .setMarking(config.marking)
         .setClearing(config.clearing)
         .setVoxelSize(_voxel_size)
         .setFilter(config.filter)
         .setVoxelMinPoints(config.voxel_min_points)
         .setEnabled(config.enabled)
         .setClearBufferAfterReading(config.clear_after_reading)
         .setModelType(config.model_type)
         .setClock(node->get_clock())
         .setLogger(node->get_logger())
         .build();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::configureObservationSource(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const ObservationSourceConfig & config,
  const rclcpp::SubscriptionOptions & sub_opt,
  double transform_tolerance)
/*****************************************************************************/
{
  auto buffer_config = createMeasurementBufferConfig(node, config, transform_tolerance);
  auto buffer = std::make_shared<buffer::MeasurementBuffer>(buffer_config);

  heartbeat_required_sources_[config.name] = config.required_for_heartbeat;

  if (_observation_manager) {
    _observation_manager->registerBuffer(buffer, config.marking, config.clearing);
  }

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = 50;

  internal::ObservationManager::SubscriberPtr subscriber;
  internal::ObservationManager::NotifierPtr notifier;

  if (config.data_type == "LaserScan") {
    auto laser_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,
        rclcpp_lifecycle::LifecycleNode>>(node, config.topic, custom_qos_profile, sub_opt);
    laser_sub->unsubscribe();

    auto laser_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      *laser_sub, *tf_, _global_frame, 50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      tf2::durationFromSec(transform_tolerance));

    if (config.inf_is_valid) {
      laser_filter->registerCallback(
        std::bind(&SpatioTemporalVoxelLayer::LaserScanValidInfCallback, this, _1, buffer));
    } else {
      laser_filter->registerCallback(
        std::bind(&SpatioTemporalVoxelLayer::LaserScanCallback, this, _1, buffer));
    }

    laser_filter->setTolerance(rclcpp::Duration::from_seconds(0.05));

    subscriber = laser_sub;
    notifier = laser_filter;
  } else if (config.data_type == "PointCloud2") {
    auto cloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
        rclcpp_lifecycle::LifecycleNode>>(node, config.topic, custom_qos_profile, sub_opt);
    cloud_sub->unsubscribe();

    auto cloud_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *cloud_sub, *tf_, _global_frame, 50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      tf2::durationFromSec(transform_tolerance));
    cloud_filter->registerCallback(
      std::bind(&SpatioTemporalVoxelLayer::PointCloud2Callback, this, _1, buffer));

    subscriber = cloud_sub;
    notifier = cloud_filter;
  }

  finalizeObservationSource(node, config, buffer, subscriber, notifier);
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::finalizeObservationSource(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const ObservationSourceConfig & config,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer,
  const internal::ObservationManager::SubscriberPtr & subscriber,
  const internal::ObservationManager::NotifierPtr & notifier)
/*****************************************************************************/
{
  if (_observation_manager) {
    if (subscriber) {
      _observation_manager->addSubscriber(subscriber);
    }
    if (notifier) {
      _observation_manager->addNotifier(notifier);
    }
  }

  auto toggle_srv_callback = std::bind(
    &SpatioTemporalVoxelLayer::BufferEnablerCallback, this,
    _1, _2, _3, buffer, subscriber);

  std::string toggle_topic = config.name + "/toggle_enabled";
  auto server = node->create_service<std_srvs::srv::SetBool>(
    toggle_topic, toggle_srv_callback, rmw_qos_profile_services_default, callback_group_);

  if (_observation_manager) {
    _observation_manager->addEnableService(server);
  }

  if (!config.sensor_frame.empty() && notifier) {
    std::vector<std::string> target_frames;
    target_frames.reserve(2);
    target_frames.push_back(_global_frame);
    target_frames.push_back(config.sensor_frame);
    notifier->setTargetFrames(target_frames);
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::onInitialize(void)
/*****************************************************************************/
{
  RCLCPP_INFO(
    logger_,
    "%s being initialized as SpatioTemporalVoxelLayer!", getName().c_str());

  // initialize parameters, grid, and sub/pubs
  _global_frame = std::string(layered_costmap_->getGlobalFrameID());
  RCLCPP_INFO(
    logger_, "%s's global frame is %s.",
    getName().c_str(), _global_frame.c_str());

  auto node = node_.lock();
  declareLayerParameters();

  bool track_unknown_space;
  double transform_tolerance;
  double map_save_time;
  loadLayerParameters(node, track_unknown_space, transform_tolerance, map_save_time);

  RCLCPP_INFO(
    logger_,
    "%s loaded parameters from parameter server.", getName().c_str());
  if (_mapping_mode) {
    _map_save_duration = std::make_unique<rclcpp::Duration>(
      map_save_time, 0.0);
  }
  _last_map_save_time = node->now();

  if (track_unknown_space) {
    default_value_ = nav2_costmap_2d::NO_INFORMATION;
  } else {
    default_value_ = nav2_costmap_2d::FREE_SPACE;
  }

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  auto pub_opt = rclcpp::PublisherOptions();
  pub_opt.callback_group = callback_group_;

  _voxel_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "voxel_grid", rclcpp::QoS(1), pub_opt);
  _elevation_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "elevation_map", rclcpp::QoS(1), pub_opt);

  if (publish_frustums_) {
    const std::string topic = frustum_topic_.empty() ? std::string("frustums") : frustum_topic_;
    frustum_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic, rclcpp::QoS(1), pub_opt);
  }

  if (publish_heartbeat_) {
    const auto resolve_topic = [this](const std::string & topic, const std::string & fallback) -> std::string {
        const std::string chosen = topic.empty() ? fallback : topic;
        if (!chosen.empty() && chosen.front() == '/') {
          return chosen;
        }
        return name_ + "/" + chosen;
      };

    heartbeat_pub_ = node->create_publisher<std_msgs::msg::Bool>(
      resolve_topic(heartbeat_topic_, "heartbeat"), rclcpp::QoS(1), pub_opt);
    heartbeat_status_pub_ = node->create_publisher<std_msgs::msg::String>(
      resolve_topic(heartbeat_status_topic_, "heartbeat_status"), rclcpp::QoS(1), pub_opt);

    heartbeat_timer_ = node->create_wall_timer(
      std::chrono::duration<double>(heartbeat_period_s_),
      std::bind(&SpatioTemporalVoxelLayer::heartbeatTimerCallback, this),
      callback_group_);
  }

  auto save_grid_callback = std::bind(
    &SpatioTemporalVoxelLayer::SaveGridCallback, this, _1, _2, _3);
  _grid_saver = node->create_service<spatio_temporal_voxel_layer::srv::SaveGrid>(
    "save_grid", save_grid_callback, rmw_qos_profile_services_default, callback_group_);

  auto grid_clock = node->get_clock();
  volume_grid::SpatioTemporalVoxelGrid::TimeSource time_source =
    [grid_clock]() -> double {
      return grid_clock ? grid_clock->now().seconds() : 0.0;
    };
  _voxel_grid = std::make_unique<volume_grid::SpatioTemporalVoxelGrid>(
    time_source, _voxel_size, static_cast<double>(default_value_), _decay_model,
    _voxel_decay, _publish_voxels);

  _pruning_manager = std::make_unique<internal::PruningManager>(tf_, _global_frame);
  _pruning_manager->setConfig(_pruning_config);

  const auto last_prune_time = node->now() - _pruning_config.interval;
  _pruning_manager->resetState(last_prune_time, getOriginX(), getOriginY());

  _observation_manager = std::make_unique<internal::ObservationManager>();

  matchSize();

  RCLCPP_INFO(logger_, "%s created underlying voxel grid.", getName().c_str());

  std::vector<ObservationSourceConfig> source_configs;
  std::stringstream ss(_topics_string);
  std::string source_name;
  while (ss >> source_name) {
    source_configs.emplace_back(loadObservationSourceConfig(node, source_name));
  }

  for (const auto & config : source_configs) {
    configureObservationSource(node, config, sub_opt, transform_tolerance);
  }

  initializeCameraInfoFovs(node, source_configs);

  current_ = true;
  was_reset_ = false;

  RCLCPP_INFO(logger_, "%s initialization complete!", getName().c_str());
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::initializeCameraInfoFovs(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::vector<ObservationSourceConfig> & source_configs)
/*****************************************************************************/
{
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_info_dependencies_.clear();
    camera_info_subscriptions_.clear();
  }

  if (!node || !_observation_manager) {
    return;
  }

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  for (const auto & config : source_configs) {
    if (!config.fov_from_camera_info) {
      continue;
    }

    const std::string dep_name = config.name + std::string("/camera_info");

    CameraInfoDependencyState dep;
    dep.required = config.camera_info_required;
    dep.initialized = false;
    dep.error_msg = "waiting for CameraInfo";

    if (config.model_type != ModelType::DEPTH_CAMERA) {
      // Only depth-camera frustums use CameraInfo-derived FOV.
      dep.initialized = true;
      dep.error_msg.clear();
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_dependencies_[dep_name] = dep;
      continue;
    }

    if (config.camera_info_topic.empty()) {
      dep.error_msg = "camera_info_topic is empty";
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_dependencies_[dep_name] = dep;
      continue;
    }

    // Copy just what we need into the callback.
    const std::string source_name = config.name;
    const std::string camera_info_topic = config.camera_info_topic;
    const double h_pad_rad = std::max(0.0, config.horizontal_fov_padding_rad);
    const double v_pad_rad = std::max(0.0, config.vertical_fov_padding_rad);

    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_dependencies_[dep_name] = dep;
    }

    auto sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      [this, dep_name, source_name, h_pad_rad, v_pad_rad](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
      {
        if (!msg) {
          return;
        }

        const double width = static_cast<double>(msg->width);
        const double height = static_cast<double>(msg->height);
        const double fx = static_cast<double>(msg->k[0]);
        const double fy = static_cast<double>(msg->k[4]);

        if (!(width > 0.0 && height > 0.0 && fx > 0.0 && fy > 0.0)) {
          std::lock_guard<std::mutex> lock(camera_info_mutex_);
          auto & dep = camera_info_dependencies_[dep_name];
          dep.initialized = false;
          dep.error_msg = "invalid CameraInfo intrinsics (need width/height/fx/fy > 0)";
          return;
        }

        const double h_fov_raw = 2.0 * std::atan2(width, 2.0 * fx);
        const double v_fov_raw = 2.0 * std::atan2(height, 2.0 * fy);

        constexpr double kMinFovRad = 0.05;
        constexpr double kMaxFovRad = 3.13;

        const double h_fov = std::clamp(h_fov_raw - h_pad_rad, kMinFovRad, kMaxFovRad);
        const double v_fov = std::clamp(v_fov_raw - v_pad_rad, kMinFovRad, kMaxFovRad);

        if (_observation_manager) {
          auto buf = _observation_manager->bufferBySource(source_name);
          if (buf) {
            buf->Lock();
            buf->SetHorizontalFovAngle(h_fov);
            buf->SetVerticalFovAngle(v_fov);
            buf->Unlock();

            {
              std::lock_guard<std::mutex> lock(camera_info_mutex_);
              auto & dep = camera_info_dependencies_[dep_name];
              dep.initialized = true;
              dep.error_msg.clear();
              camera_info_subscriptions_.erase(dep_name);
            }

            RCLCPP_INFO(
              logger_,
              "%s %s: computed FOV from CameraInfo (h=%.3frad, v=%.3frad)",
              getName().c_str(), source_name.c_str(), h_fov, v_fov);
          } else {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            auto & dep = camera_info_dependencies_[dep_name];
            dep.initialized = false;
            dep.error_msg = "failed to find measurement buffer for source";
          }
        }
      },
      sub_opt);

    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_subscriptions_[dep_name] = sub;
    }

    RCLCPP_INFO(
      logger_,
      "%s %s: waiting indefinitely for CameraInfo on %s",
      getName().c_str(), config.name.c_str(), camera_info_topic.c_str());
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // laser scan where infinity is invalid callback function
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message->header;
  try {
    _laser_projector.transformLaserScanToPointCloud(
      message->header.frame_id, *message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "TF returned a transform exception to frame %s: %s",
      _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(*message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::LaserScanValidInfCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // Filter infinity to max_range
  float epsilon = 0.0001;
  sensor_msgs::msg::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    _laser_projector.transformLaserScanToPointCloud(
      message.header.frame_id, message, cloud, *tf_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "TF returned a transform exception to frame %s: %s",
      _global_frame.c_str(), ex.what());
    _laser_projector.projectLaser(message, cloud);
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(cloud);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::PointCloud2Callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
  const std::shared_ptr<buffer::MeasurementBuffer> & buffer)
/*****************************************************************************/
{
  if (!buffer->IsEnabled()) {
    return;
  }
  // buffer the point cloud
  buffer->Lock();
  buffer->BufferROSCloud(*message);
  buffer->Unlock();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::BufferEnablerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response,
  const std::shared_ptr<buffer::MeasurementBuffer> buffer,
  const std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>> &subcriber
  )
/*****************************************************************************/
{
  buffer->Lock();
  if (buffer->IsEnabled() != request->data) {
    buffer->SetEnabled(request->data);
    if (request->data) {
      if (subcriber) {
        subcriber->subscribe();
      }
      buffer->ResetLastUpdatedTime();
      response->message = "Enabling sensor";
    } else if (subcriber) {
      subcriber->unsubscribe();
      ResetGrid();
      response->message = "Disabling sensor";
    }
  } else {
    response->message = "Sensor already in the required state doing nothing";
  }
  buffer->Unlock();
  response->success = true;
}


/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetMarkingObservations(
  std::vector<observation::MeasurementReading> & marking_observations) const
/*****************************************************************************/
{
  if (!_observation_manager) {
    return false;
  }

  return _observation_manager->collectMarkingObservations(marking_observations);
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::GetClearingObservations(
  std::vector<observation::MeasurementReading> & clearing_observations) const
/*****************************************************************************/
{
  if (!_observation_manager) {
    return false;
  }

  return _observation_manager->collectClearingObservations(clearing_observations);
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ObservationsResetAfterReading() const
/*****************************************************************************/
{
  if (_observation_manager) {
    _observation_manager->resetBuffersAfterReading();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
/*****************************************************************************/
{
  // updates layer costmap to include footprint for clearing in voxel grid
  if (!_update_footprint_enabled) {
    return false;
  }
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw,
    getFootprint(), _transformed_footprint);
  for (unsigned int i = 0; i < _transformed_footprint.size(); i++) {
    touch(
      _transformed_footprint[i].x, _transformed_footprint[i].y,
      min_x, min_y, max_x, max_y);
  }

  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::activate(void)
/*****************************************************************************/
{
  // subscribe and place info in buffers from sensor sources
  RCLCPP_INFO(logger_, "%s was activated.", getName().c_str());

  if (_observation_manager) {
    _observation_manager->activateSubscribers();
    _observation_manager->resetLastUpdatedTime();
  }

  // Add callback for dynamic parametrs
  auto node = node_.lock();
  dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SpatioTemporalVoxelLayer::dynamicParametersCallback, this, _1));
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::deactivate(void)
/*****************************************************************************/
{
  // unsubscribe from all sensor sources
  RCLCPP_INFO(logger_, "%s was deactivated.", getName().c_str());

  if (_observation_manager) {
    _observation_manager->deactivateSubscribers();
  }
  dyn_params_handler.reset();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::reset(void)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  // reset layer
  Costmap2D::resetMaps();
  this->ResetGrid();

  current_ = false;
  was_reset_ = true;

  if (_observation_manager) {
    _observation_manager->resetLastUpdatedTime();
  }
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::AddStaticObservations(
  const observation::MeasurementReading & obs)
/*****************************************************************************/
{
  // observations to always be added to the map each update cycle not marked
  RCLCPP_INFO(
    logger_,
    "%s: Adding static observation to map.", getName().c_str());

  if (!_observation_manager) {
    RCLCPP_WARN(logger_, "Observation manager unavailable, cannot add static observation.");
    return false;
  }

  _observation_manager->addStaticObservation(obs);
  return true;
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::RemoveStaticObservations(void)
/*****************************************************************************/
{
  // kill all static observations added to each update cycle
  RCLCPP_INFO(
    logger_,
    "%s: Removing static observations to map.", getName().c_str());

  if (!_observation_manager) {
    RCLCPP_WARN(logger_, "Observation manager unavailable, cannot clear static observations.");
    return false;
  }

  _observation_manager->clearStaticObservations();
  return true;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::ResetGrid(void)
/*****************************************************************************/
{
  if (!_voxel_grid->ResetGrid()) {
    RCLCPP_WARN(logger_, "Did not clear level set in %s!", getName().c_str());
  }
  std::fill(_elevation_layer.begin(), _elevation_layer.end(), _no_elevation_data);
  std::fill(_elevation_layer_m.begin(), _elevation_layer_m.end(), _no_elevation_data_m);
  _active_elevation_indices.clear();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::matchSize(void)
/*****************************************************************************/
{
  // match the master costmap size, volume_grid maintains full w/ expiration.
  CostmapLayer::matchSize();
  const size_t cell_count = static_cast<size_t>(getSizeInCellsX()) * static_cast<size_t>(getSizeInCellsY());
  _elevation_layer.assign(cell_count, _no_elevation_data);
  _elevation_layer_m.assign(cell_count, _no_elevation_data_m);
  _active_elevation_indices.clear();
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
/*****************************************************************************/
{
  auto node = node_.lock();
  const auto now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);

  try {
    // update costs in master_grid with costmap_
    if (!_enabled) {
      return;
    }

    // if not current due to reset, set current now after clearing
    if (!current_ && was_reset_) {
      was_reset_ = false;
      current_ = true;
    }

    if (_update_footprint_enabled) {
      setConvexPolygonCost(_transformed_footprint, nav2_costmap_2d::FREE_SPACE);
    }

    switch (_combination_method) {
      case 0:
        updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        break;
      case 1:
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        break;
      default:
        break;
    }

    if (node) {
      last_update_costs_success_ns_.store(now.nanoseconds(), std::memory_order_relaxed);
    }
  } catch (const std::exception & ex) {
    if (node) {
      last_update_costs_error_ns_.store(now.nanoseconds(), std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(heartbeat_error_mutex_);
      last_update_costs_error_msg_ = ex.what();
    }
    RCLCPP_ERROR(logger_, "%s updateCosts exception: %s", getName().c_str(), ex.what());
    current_ = false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::UpdateROSCostmap(
  double * min_x, double * min_y, double * max_x, double * max_y,
  volume_grid::OccupanyCellSet & cleared_cells)
/*****************************************************************************/
{
  // grabs map of occupied cells from grid and adds to costmap_
  Costmap2D::resetMaps();

  const bool lethal_from_elevation_enabled =
    (_elevation_window_size_m > 0.0) && (_elevation_lethal_threshold_m > 0.0);
  const double window_size_m = lethal_from_elevation_enabled ? _elevation_window_size_m : 0.0;
  const double window_half_size_m = lethal_from_elevation_enabled ? (window_size_m * 0.5) : 0.0;
  const int window_half_extent_cells = lethal_from_elevation_enabled ?
    static_cast<int>(std::max(
      1.0,
      std::ceil(window_half_size_m / static_cast<double>(getResolution())))) : 0;

  double base_z = 0.0;
  bool limit_elevation = _limit_elevation && std::isfinite(_max_elevation_above_robot_base);
  double elevation_ceiling = std::numeric_limits<double>::quiet_NaN();
  if (limit_elevation) {
    if (!getRobotBaseHeight(base_z)) {
      auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(
          logger_,
          "%s disabling elevation limit; failed to obtain base height.",
          getName().c_str());
      } else {
        RCLCPP_WARN(
          logger_,
          "Disabling elevation limit; failed to obtain base height.");
      }
      limit_elevation = false;
    } else {
      elevation_ceiling = base_z + _max_elevation_above_robot_base;
    }
  }

  if (_elevation_layer.size() !=
    static_cast<size_t>(getSizeInCellsX()) * static_cast<size_t>(getSizeInCellsY()))
  {
    matchSize();
  }

  for (const size_t index : _active_elevation_indices) {
    if (index < _elevation_layer.size()) {
      _elevation_layer[index] = _no_elevation_data;
      _elevation_layer_m[index] = _no_elevation_data_m;
    }
  }

  std::vector<size_t> new_active_indices;
  auto * column_map = _voxel_grid->GetColumnElevationMap();
  auto * touched_columns = _voxel_grid->GetTouchedColumns();

  int min_mx = static_cast<int>(getSizeInCellsX());
  int min_my = static_cast<int>(getSizeInCellsY());
  int max_mx = -1;
  int max_my = -1;

  if (touched_columns) {
    new_active_indices.reserve(touched_columns->size());
    for (const auto & cell : *touched_columns) {
      uint map_x, map_y;
      if (!worldToMap(cell.x, cell.y, map_x, map_y)) {
        continue;
      }

      min_mx = std::min(min_mx, static_cast<int>(map_x));
      min_my = std::min(min_my, static_cast<int>(map_y));
      max_mx = std::max(max_mx, static_cast<int>(map_x));
      max_my = std::max(max_my, static_cast<int>(map_y));

      const size_t index = getIndex(map_x, map_y);
      if (index >= _elevation_layer.size()) {
        continue;
      }

      _elevation_layer[index] = _no_elevation_data;
      _elevation_layer_m[index] = _no_elevation_data_m;

      const auto column_it = column_map->find(cell);
      if (column_it != column_map->end()) {
        const auto & column = column_it->second;
        const bool passes_threshold = !(_mark_threshold > 0 &&
          static_cast<int>(column.point_count) < _mark_threshold);

        if (!column.empty() && passes_threshold) {
          if (limit_elevation) {
            int32_t limited_index = volume_grid::ColumnElevation::NO_DATA;
            double limited_height = std::numeric_limits<double>::quiet_NaN();
            if (!std::isnan(elevation_ceiling) &&
              column.highestBelow(elevation_ceiling, limited_index, limited_height))
            {
              _elevation_layer[index] = limited_index;
              _elevation_layer_m[index] = static_cast<float>(limited_height);
              new_active_indices.push_back(index);
            }
          } else if (!std::isnan(column.elevation_m)) {
            _elevation_layer[index] = column.elevation_index;
            _elevation_layer_m[index] = static_cast<float>(column.elevation_m);
            new_active_indices.push_back(index);
          }
        }
      }

      touch(cell.x, cell.y, min_x, min_y, max_x, max_y);
      if (lethal_from_elevation_enabled) {
        touch(cell.x - window_half_size_m, cell.y - window_half_size_m, min_x, min_y, max_x, max_y);
        touch(cell.x + window_half_size_m, cell.y + window_half_size_m, min_x, min_y, max_x, max_y);
      }
    }
  }

  _active_elevation_indices = std::move(new_active_indices);

  volume_grid::OccupanyCellSet::iterator cell;
  for (cell = cleared_cells.begin(); cell != cleared_cells.end(); ++cell)
  {
    uint map_x, map_y;
    if (!worldToMap(cell->x, cell->y, map_x, map_y)) {
      continue;
    }

    min_mx = std::min(min_mx, static_cast<int>(map_x));
    min_my = std::min(min_my, static_cast<int>(map_y));
    max_mx = std::max(max_mx, static_cast<int>(map_x));
    max_my = std::max(max_my, static_cast<int>(map_y));

    const size_t index = getIndex(map_x, map_y);
    if (index < _elevation_layer.size()) {
      if (!column_map || column_map->find(*cell) == column_map->end()) {
        _elevation_layer[index] = _no_elevation_data;
        _elevation_layer_m[index] = _no_elevation_data_m;
      }
    }
    touch(cell->x, cell->y, min_x, min_y, max_x, max_y);
    if (lethal_from_elevation_enabled) {
      touch(cell->x - window_half_size_m, cell->y - window_half_size_m, min_x, min_y, max_x, max_y);
      touch(cell->x + window_half_size_m, cell->y + window_half_size_m, min_x, min_y, max_x, max_y);
    }
  }

  if (lethal_from_elevation_enabled && max_mx >= 0 && max_my >= 0) {
    const int size_x = static_cast<int>(getSizeInCellsX());
    const int size_y = static_cast<int>(getSizeInCellsY());

    const int eval_start_x = std::max(0, std::min(size_x - 1, min_mx - window_half_extent_cells));
    const int eval_start_y = std::max(0, std::min(size_y - 1, min_my - window_half_extent_cells));
    const int eval_end_x = std::max(0, std::min(size_x - 1, max_mx + window_half_extent_cells));
    const int eval_end_y = std::max(0, std::min(size_y - 1, max_my + window_half_extent_cells));

    const auto lethal = internal::computeElevationLethalMask(
      _elevation_layer_m,
      size_x,
      size_y,
      eval_start_x,
      eval_start_y,
      eval_end_x,
      eval_end_y,
      window_half_extent_cells,
      _elevation_lethal_threshold_m,
      _elevation_window_min_samples);

    for (int ey = 0; ey < lethal.height; ++ey) {
      const int map_y = lethal.start_y + ey;
      for (int ex = 0; ex < lethal.width; ++ex) {
        const int map_x = lethal.start_x + ex;
        const size_t i = static_cast<size_t>(ey) * static_cast<size_t>(lethal.width) +
          static_cast<size_t>(ex);
        if (i >= lethal.lethal.size()) {
          continue;
        }
        if (lethal.lethal[i] != 0U) {
          setCost(static_cast<uint>(map_x), static_cast<uint>(map_y), nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
/*****************************************************************************/
{
  auto node = node_.lock();

  try {
    // grabs new max bounds for the costmap
    if (!_enabled) {
      return;
    }

    // Required because UpdateROSCostmap will also lock if AFTER we lock here voxel_grid_lock,
    // and if clearArea is called in between, we will have a deadlock
    boost::unique_lock<mutex_t> cm_lock(*getMutex());

    boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

    if (!node) {
      RCLCPP_WARN(logger_, "%s could not lock lifecycle node for pruning", getName().c_str());
      return;
    }

    // Steve's Note June 22, 2018
    // I dislike this necessity, I can't remove the master grid's knowledge about
    // STVL on the fly so I have play games with the API even though this isn't
    // really a rolling plugin implementation. It works, but isn't ideal.
    if (layered_costmap_->isRolling()) {
      updateOrigin(
        robot_x - getSizeInMetersX() / 2,
        robot_y - getSizeInMetersY() / 2);
    }

    if (_pruning_manager && _voxel_grid) {
      _pruning_config.mapping_mode = _mapping_mode;
      _pruning_manager->setConfig(_pruning_config);

      internal::PruningContext context;
      context.origin_x = getOriginX();
      context.origin_y = getOriginY();
      context.size_x = getSizeInMetersX();
      context.size_y = getSizeInMetersY();
      context.global_frame = _global_frame;

      _pruning_manager->pruneIfNeeded(context, node->now(), *_voxel_grid, logger_);
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    std::vector<observation::MeasurementReading> marking_observations,
      clearing_observations;
    current = GetMarkingObservations(marking_observations) && current;
    current = GetClearingObservations(clearing_observations) && current;
    ObservationsResetAfterReading();
    current_ = current;

    if (publish_frustums_ && frustum_pub_ && node) {
      visualization_msgs::msg::MarkerArray msg;
      msg.markers.reserve(clearing_observations.size());

      int32_t marker_id = 0;
      for (const auto & obs : clearing_observations) {
        if (!obs._clearing) {
          continue;
        }

        if (obs._model_type != ModelType::DEPTH_CAMERA) {
          continue;
        }

        const double vFOV = obs._vertical_fov_in_rad;
        const double hFOV = obs._horizontal_fov_in_rad;
        const double min_d = obs._min_z_in_m;
        const double max_d = obs._max_z_in_m;

        if (vFOV <= 0.0 || hFOV <= 0.0 || max_d <= 0.0 || max_d <= min_d) {
          continue;
        }

        visualization_msgs::msg::Marker m;
        m.header.frame_id = _global_frame;
        m.header.stamp = node->now();
        m.ns = obs._source_name.empty() ? std::string("stvl_frustum") : (std::string("stvl_frustum/") + obs._source_name);
        m.id = marker_id++;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = obs._origin.x;
        m.pose.position.y = obs._origin.y;
        m.pose.position.z = obs._origin.z;
        m.pose.orientation.x = obs._orientation.x;
        m.pose.orientation.y = obs._orientation.y;
        m.pose.orientation.z = obs._orientation.z;
        m.pose.orientation.w = obs._orientation.w;
        m.scale.x = static_cast<float>(frustum_line_width_);
        m.color = makeDebugColorFromName(m.ns);
        m.lifetime = rclcpp::Duration::from_seconds(frustum_lifetime_s_);

        const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
        const Eigen::Affine3d rx1 = Eigen::Affine3d(
          Eigen::AngleAxisd(vFOV / 2.0, Eigen::Vector3d::UnitX()));
        const Eigen::Affine3d rx2 = Eigen::Affine3d(
          Eigen::AngleAxisd(-vFOV / 2.0, Eigen::Vector3d::UnitX()));
        const Eigen::Affine3d ry1 = Eigen::Affine3d(
          Eigen::AngleAxisd(hFOV / 2.0, Eigen::Vector3d::UnitY()));
        const Eigen::Affine3d ry2 = Eigen::Affine3d(
          Eigen::AngleAxisd(-hFOV / 2.0, Eigen::Vector3d::UnitY()));

        std::array<Eigen::Vector3d, 4> rays{
          rx1 * ry1 * Z,
          rx2 * ry1 * Z,
          rx2 * ry2 * Z,
          rx1 * ry2 * Z
        };

        std::array<Eigen::Vector3d, 8> corners;
        for (size_t i = 0; i < rays.size(); ++i) {
          corners[2 * i + 0] = rays[i] * min_d;
          corners[2 * i + 1] = rays[i] * max_d;
        }

        const auto add_edge = [&m, &corners](int a, int b) {
            geometry_msgs::msg::Point p;
            p.x = corners[static_cast<size_t>(a)].x();
            p.y = corners[static_cast<size_t>(a)].y();
            p.z = corners[static_cast<size_t>(a)].z();
            m.points.push_back(p);
            p.x = corners[static_cast<size_t>(b)].x();
            p.y = corners[static_cast<size_t>(b)].y();
            p.z = corners[static_cast<size_t>(b)].z();
            m.points.push_back(p);
          };

        // Near plane (min_d): indices 0,2,4,6
        add_edge(0, 2);
        add_edge(2, 4);
        add_edge(4, 6);
        add_edge(6, 0);

        // Far plane (max_d): indices 1,3,5,7
        add_edge(1, 3);
        add_edge(3, 5);
        add_edge(5, 7);
        add_edge(7, 1);

        // Connect near->far
        add_edge(0, 1);
        add_edge(2, 3);
        add_edge(4, 5);
        add_edge(6, 7);

        msg.markers.push_back(std::move(m));
      }

      frustum_pub_->publish(msg);
    }

    volume_grid::OccupanyCellSet cleared_cells;

    // navigation mode: clear observations, mapping mode: save maps and publish
    bool should_save = false;
    if (_map_save_duration) {
      should_save = node->now() - _last_map_save_time > *_map_save_duration;
    }
    if (!_mapping_mode) {
      _voxel_grid->ClearFrustums(clearing_observations, cleared_cells);
    } else if (should_save) {
      _last_map_save_time = node->now();
      time_t rawtime;
      struct tm * timeinfo;
      char time_buffer[100];
      time(&rawtime);
      timeinfo = localtime(&rawtime);  //NOLINT
      strftime(time_buffer, 100, "%F-%r", timeinfo);

      auto request =
        std::make_shared<spatio_temporal_voxel_layer::srv::SaveGrid::Request>();
      auto response =
        std::make_shared<spatio_temporal_voxel_layer::srv::SaveGrid::Response>();
      request->file_name = time_buffer;
      SaveGridCallback(nullptr, request, response);
    }

    // mark observations
    _voxel_grid->Mark(marking_observations);

    // update the ROS Layered Costmap
    UpdateROSCostmap(min_x, min_y, max_x, max_y, cleared_cells);

    // publish point cloud in navigation mode
    if (_publish_voxels && !_mapping_mode) {
      stvl::core::PointCloud occupancy_cloud;
      _voxel_grid->GetOccupancyPointCloud(occupancy_cloud);
      const auto pc2_msg = stvl::bridge::toPointCloud2(occupancy_cloud, _global_frame, node->now());
      _voxel_pub->publish(pc2_msg);
    }

    if (_publish_elevation_map && !_mapping_mode && _elevation_pub) {
      double elevation_ceiling = std::numeric_limits<double>::quiet_NaN();
      bool limit_elevation = _limit_elevation &&
        std::isfinite(_max_elevation_above_robot_base);
      if (limit_elevation) {
        double elevation_base_z = 0.0;
        if (getRobotBaseHeight(elevation_base_z)) {
          elevation_ceiling = elevation_base_z + _max_elevation_above_robot_base;
        } else {
          limit_elevation = false;
        }
      }

      stvl::core::PointCloud elevation_cloud;
      _voxel_grid->GetElevationPointCloud(elevation_cloud, limit_elevation, elevation_ceiling);
      auto elevation_msg = stvl::bridge::toPointCloud2(elevation_cloud, _global_frame, node->now());
      _elevation_pub->publish(elevation_msg);
    }

    // update footprint
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    last_update_bounds_success_ns_.store(node->now().nanoseconds(), std::memory_order_relaxed);
  } catch (const std::exception & ex) {
    if (node) {
      last_update_bounds_error_ns_.store(node->now().nanoseconds(), std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(heartbeat_error_mutex_);
      last_update_bounds_error_msg_ = ex.what();
    }
    RCLCPP_ERROR(logger_, "%s updateBounds exception: %s", getName().c_str(), ex.what());
    current_ = false;
  }
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::heartbeatTimerCallback()
/*****************************************************************************/
{
  if (!publish_heartbeat_ || !heartbeat_pub_ || !heartbeat_status_pub_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  const auto now = node->now();
  const auto clock_type = node->get_clock()->get_clock_type();

  internal::heartbeat::HeartbeatConfig hb_config;
  hb_config.costmap_timeout_s = heartbeat_costmap_timeout_s_;
  hb_config.default_sensor_timeout_s = heartbeat_default_sensor_timeout_s_;
  hb_config.min_sensor_timeout_s = heartbeat_min_sensor_timeout_s_;
  hb_config.expected_update_rate_multiplier = heartbeat_expected_update_rate_multiplier_;

  internal::heartbeat::CostmapCycleState costmap;
  costmap.layer_enabled = _enabled;
  costmap.observation_manager_initialized = static_cast<bool>(_observation_manager);
  costmap.voxel_grid_initialized = static_cast<bool>(_voxel_grid);

  costmap.update_bounds_last_success = rclcpp::Time(
    last_update_bounds_success_ns_.load(std::memory_order_relaxed), clock_type);
  costmap.update_bounds_last_error = rclcpp::Time(
    last_update_bounds_error_ns_.load(std::memory_order_relaxed), clock_type);
  costmap.update_costs_last_success = rclcpp::Time(
    last_update_costs_success_ns_.load(std::memory_order_relaxed), clock_type);
  costmap.update_costs_last_error = rclcpp::Time(
    last_update_costs_error_ns_.load(std::memory_order_relaxed), clock_type);
  {
    std::lock_guard<std::mutex> lock(heartbeat_error_mutex_);
    costmap.update_bounds_last_error_msg = last_update_bounds_error_msg_;
    costmap.update_costs_last_error_msg = last_update_costs_error_msg_;
  }

  std::vector<internal::heartbeat::SourceState> sources;
  if (_observation_manager) {
    _observation_manager->forEachBuffer(
      [this, &sources, &costmap](const internal::ObservationManager::BufferPtr & buffer) {
        if (!buffer) {
          costmap.has_null_measurement_buffer = true;
          return;
        }

        const auto source = buffer->GetSourceName();
        const auto required_it = heartbeat_required_sources_.find(source);
        const bool required = required_it == heartbeat_required_sources_.end() ? true : required_it->second;
        if (!required) {
          return;
        }

        internal::heartbeat::SourceState state;
        state.source_name = source;
        state.required = true;
        state.enabled = buffer->IsEnabled();
        state.expected_update_rate_s = buffer->GetExpectedUpdateRateSeconds();
        state.last_success = buffer->GetLastSuccessfulBufferTime();
        state.last_error = buffer->GetLastErrorTime();
        state.last_error_msg = buffer->GetLastErrorMessage();
        sources.push_back(std::move(state));
      });
  }

  // Add synthetic required sources for one-time startup dependencies (e.g. CameraInfo-derived FOV).
  // These are considered healthy once initialized and are kept fresh by latching last_success=now.
  std::vector<std::pair<std::string, CameraInfoDependencyState>> camera_info_deps_snapshot;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    camera_info_deps_snapshot.reserve(camera_info_dependencies_.size());
    for (const auto & kv : camera_info_dependencies_) {
      camera_info_deps_snapshot.emplace_back(kv.first, kv.second);
    }
  }

  for (const auto & kv : camera_info_deps_snapshot) {
    const auto & name = kv.first;
    const auto & dep = kv.second;
    if (!dep.required) {
      continue;
    }

    internal::heartbeat::SourceState s;
    s.source_name = name;
    s.required = true;
    s.enabled = true;
    s.expected_update_rate_s = 0.0;

    if (dep.initialized) {
      s.last_success = now;
      s.last_error = rclcpp::Time(0, 0, clock_type);
      s.last_error_msg.clear();
    } else {
      s.last_success = rclcpp::Time(0, 0, clock_type);
      s.last_error = now;
      s.last_error_msg = dep.error_msg.empty() ? std::string("not initialized") : dep.error_msg;
    }

    sources.push_back(std::move(s));
  }

  const auto result = internal::heartbeat::evaluateHeartbeat(now, hb_config, costmap, sources);

  std_msgs::msg::Bool hb;
  hb.data = result.healthy;
  heartbeat_pub_->publish(hb);

  std_msgs::msg::String status;
  status.data = result.reason;
  heartbeat_status_pub_->publish(status);
}

/*****************************************************************************/
bool SpatioTemporalVoxelLayer::getRobotBaseHeight(double & base_z)
{
  if (_pruning_config.base_frame.empty() || !tf_) {
    return false;
  }

  try {
    const geometry_msgs::msg::TransformStamped base_in_global = tf_->lookupTransform(
      _global_frame, _pruning_config.base_frame, tf2::TimePointZero);
    base_z = static_cast<double>(base_in_global.transform.translation.z);
    return true;
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "%s failed to lookup robot base height: %s",
        getName().c_str(), ex.what());
    } else {
      RCLCPP_WARN(
        logger_, "%s failed to lookup robot base height: %s",
        getName().c_str(), ex.what());
    }
  }

  return false;
}

void SpatioTemporalVoxelLayer::SaveGridCallback(
  const std::shared_ptr<rmw_request_id_t>/*header*/,
  const std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Request> req,
  std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Response> resp)
/*****************************************************************************/
{
  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  double map_size_bytes;

  if (_voxel_grid->SaveGrid(req->file_name, map_size_bytes) ) {
    RCLCPP_INFO(
      logger_,
      "SpatioTemporalVoxelLayer: Saved %s grid! Has memory footprint of %f bytes.",
      req->file_name.c_str(), map_size_bytes);
    resp->map_size_bytes = map_size_bytes;
    resp->status = true;
    return;
  }

  RCLCPP_WARN(logger_, "SpatioTemporalVoxelLayer: Failed to save grid.");
  resp->status = false;
}

rcl_interfaces::msg::SetParametersResult
SpatioTemporalVoxelLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    std::stringstream ss(_topics_string);
    std::string source;
    while (ss >> source) {
      if (type == ParameterType::PARAMETER_DOUBLE) {
        auto apply_to_buffer = [&](auto && setter)
        {
          if (!_observation_manager) {
            return;
          }
          auto target_buffer = _observation_manager->bufferBySource(source);
          if (!target_buffer) {
            return;
          }
          target_buffer->Lock();
          setter(*target_buffer);
          target_buffer->Unlock();
        };

        if (name == name_ + "." + source + "." + "min_obstacle_height") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetMinObstacleHeight(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "max_obstacle_height") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetMaxObstacleHeight(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "min_z") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetMinZ(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "max_z") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetMaxZ(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "vertical_fov_angle") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetVerticalFovAngle(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "vertical_fov_padding") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetVerticalFovPadding(parameter.as_double());
          });
        } else if (name == name_ + "." + source + "." + "horizontal_fov_angle") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetHorizontalFovAngle(parameter.as_double());
          });
        }
      }

      if (type == ParameterType::PARAMETER_BOOL) {
        auto apply_to_buffer = [&](auto && setter)
        {
          if (!_observation_manager) {
            return;
          }
          auto target_buffer = _observation_manager->bufferBySource(source);
          if (!target_buffer) {
            return;
          }
          target_buffer->Lock();
          setter(*target_buffer);
          target_buffer->Unlock();
        };

        if (name == name_ + "." + source + "." + "filter_obstacle_height") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetFilterObstacleHeight(parameter.as_bool());
          });
        } else if (name == name_ + "." + source + "." + "use_clearing_min_max_z") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetUseClearingMinMaxZ(parameter.as_bool());
          });
        } else if (name == name_ + "." + source + "." + "height_relative_to_base") {
          apply_to_buffer([&](buffer::MeasurementBuffer & buf) {
            buf.SetHeightRelativeToBase(parameter.as_bool());
          });
        }
      }

      if (type == ParameterType::PARAMETER_DOUBLE) {
        bool pruning_updated = false;

        if (name == name_ + "." + "prune_padding") {
          _pruning_config.padding = std::max(0.0, parameter.as_double());
          pruning_updated = true;
        } else if (name == name_ + "." + "prune_distance") {
          _pruning_config.distance = std::max(0.0, parameter.as_double());
          if (_pruning_manager) {
            _pruning_manager->updateLastOrigin(getOriginX(), getOriginY());
          }
          pruning_updated = true;
        } else if (name == name_ + "." + "prune_interval") {
          double prune_interval_seconds = parameter.as_double();
          if (prune_interval_seconds < 0.0) {
            prune_interval_seconds = 0.0;
          }
          _pruning_config.interval = rclcpp::Duration::from_seconds(prune_interval_seconds);
          if (_pruning_manager) {
            auto node = node_.lock();
            if (node) {
              _pruning_manager->updateLastPruneTime(node->now() - _pruning_config.interval);
            }
          }
          pruning_updated = true;
        } else if (name == name_ + "." + "prune_z_min") {
          _pruning_config.z_min = parameter.as_double();
          if (_pruning_config.z_min > _pruning_config.z_max) {
            RCLCPP_WARN(
              logger_, "%s prune_z_min > prune_z_max, swapping values.", getName().c_str());
            std::swap(_pruning_config.z_min, _pruning_config.z_max);
          }
          if (_pruning_config.z_min == _pruning_config.z_max) {
            _pruning_config.z_min -= _voxel_size;
            _pruning_config.z_max += _voxel_size;
          }
          pruning_updated = true;
        } else if (name == name_ + "." + "prune_z_max") {
          _pruning_config.z_max = parameter.as_double();
          if (_pruning_config.z_min > _pruning_config.z_max) {
            RCLCPP_WARN(
              logger_, "%s prune_z_min > prune_z_max, swapping values.", getName().c_str());
            std::swap(_pruning_config.z_min, _pruning_config.z_max);
          }
          if (_pruning_config.z_min == _pruning_config.z_max) {
            _pruning_config.z_min -= _voxel_size;
            _pruning_config.z_max += _voxel_size;
          }
          pruning_updated = true;
        } else if (name == name_ + "." + "max_elevation_above_robot_base") {
          const double value = parameter.as_double();
          if (value > 0.0) {
            _max_elevation_above_robot_base = value;
            _limit_elevation = true;
          } else {
            _max_elevation_above_robot_base = std::numeric_limits<double>::infinity();
            _limit_elevation = false;
          }
        }

        if (pruning_updated && _pruning_manager) {
          _pruning_manager->setConfig(_pruning_config);
        }
      }
    }

    if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + "." + "enabled") {
        bool enable = parameter.as_bool();
        if (enabled_ != enable) {
          if (_observation_manager) {
            if (enable) {
              _observation_manager->activateSubscribers();
              _observation_manager->resetLastUpdatedTime();
            } else {
              _observation_manager->deactivateSubscribers();
            }
          }
        }
        enabled_ = enable;
      } else if (name == name_ + "." + "prune_enabled") {
        _pruning_config.enabled = parameter.as_bool();
        if (_pruning_manager) {
          _pruning_manager->setConfig(_pruning_config);
          auto node = node_.lock();
          if (node) {
            _pruning_manager->updateLastPruneTime(node->now() - _pruning_config.interval);
          }
        }
      } else if (name == name_ + "." + "publish_voxel_map") {
        _publish_voxels = parameter.as_bool();
      } else if (name == name_ + "." + "publish_elevation_map") {
        _publish_elevation_map = parameter.as_bool();
      }
    }

    if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + "." + "mark_threshold") {
        _mark_threshold = parameter.as_int();
      }
    }

    if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + "." + "prune_robot_base_frame") {
        const auto value = parameter.as_string();
        if (value.empty()) {
          auto node = node_.lock();
          if (node) {
            RCLCPP_WARN(
              logger_, "%s prune_robot_base_frame cannot be empty, keeping previous value %s.",
              getName().c_str(), _pruning_config.base_frame.c_str());
          }
        } else {
          _pruning_config.base_frame = value;
          if (_pruning_manager) {
            _pruning_manager->setConfig(_pruning_config);
          }

          if (_observation_manager) {
            _observation_manager->forEachBuffer([&](internal::ObservationManager::BufferPtr & buf) {
              if (buf) {
                buf->SetRobotBaseFrame(_pruning_config.base_frame);
              }
            });
          }
        }
      }
    }
  }

  result.successful = true;
  return result;
}

/*****************************************************************************/
void SpatioTemporalVoxelLayer::clearArea(
  int start_x, int start_y, int end_x, int end_y, bool invert_area)
/*****************************************************************************/
{
  // convert map coords to world coords
  volume_grid::occupany_cell start_world(0, 0);
  volume_grid::occupany_cell end_world(0, 0);
  mapToWorld(start_x, start_y, start_world.x, start_world.y);
  mapToWorld(end_x, end_y, end_world.x, end_world.y);

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);
  _voxel_grid->ResetGridArea(start_world, end_world, invert_area);
  CostmapLayer::clearArea(start_x, start_y, end_x, end_y, invert_area);
}

}  // namespace spatio_temporal_voxel_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer,
  nav2_costmap_2d::Layer)
