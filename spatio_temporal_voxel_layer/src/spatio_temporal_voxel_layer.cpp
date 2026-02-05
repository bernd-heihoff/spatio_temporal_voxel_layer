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
#include "openvdb/math/BBox.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
}

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
  declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
  declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
  declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.5));

  declareParameter(source + "." + "min_z", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "max_z", rclcpp::ParameterValue(10.0));
  declareParameter(source + "." + "vertical_fov_angle", rclcpp::ParameterValue(0.7));
  declareParameter(source + "." + "vertical_fov_padding", rclcpp::ParameterValue(0.0));
  declareParameter(source + "." + "horizontal_fov_angle", rclcpp::ParameterValue(1.04));
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
  node->get_parameter(name_ + "." + source + "." + "inf_is_valid", config.inf_is_valid);
  node->get_parameter(name_ + "." + source + "." + "marking", config.marking);
  node->get_parameter(name_ + "." + source + "." + "clearing", config.clearing);
  node->get_parameter(name_ + "." + source + "." + "obstacle_range", config.obstacle_range);

  node->get_parameter(name_ + "." + source + "." + "min_z", config.min_z);
  node->get_parameter(name_ + "." + source + "." + "max_z", config.max_z);
  node->get_parameter(name_ + "." + source + "." + "vertical_fov_angle", config.vertical_fov);
  node->get_parameter(name_ + "." + source + "." + "vertical_fov_padding", config.vertical_fov_padding);
  node->get_parameter(name_ + "." + source + "." + "horizontal_fov_angle", config.horizontal_fov);
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
         .setObstacleRange(config.obstacle_range)
         .setTfBuffer(tf_)
         .setGlobalFrame(_global_frame)
         .setSensorFrame(config.sensor_frame)
         .setTfTolerance(transform_tolerance)
         .setMinZ(config.min_z)
         .setMaxZ(config.max_z)
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

  current_ = true;
  was_reset_ = false;

  RCLCPP_INFO(logger_, "%s initialization complete!", getName().c_str());
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
  // grabs new max bounds for the costmap
  if (!_enabled) {
    return;
  }

  // Required because UpdateROSCostmap will also lock if AFTER we lock here voxel_grid_lock,
  // and if clearArea is called in between, we will have a deadlock
  boost::unique_lock<mutex_t> cm_lock(*getMutex());

  boost::recursive_mutex::scoped_lock lock(_voxel_grid_lock);

  auto node = node_.lock();
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
