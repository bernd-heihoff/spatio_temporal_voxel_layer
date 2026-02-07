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
 * Purpose: Replace the ROS voxel grid / obstacle layers using VoxelGrid
 *          with OpenVDB's more efficient and capacble voxel library with
 *          ray tracing and knn.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_

// STL
#include <time.h>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <limits>
// voxel grid
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_grid.hpp"
#include "spatio_temporal_voxel_layer/internal/observation_manager.hpp"
#include "spatio_temporal_voxel_layer/internal/pruning_manager.hpp"
#include "spatio_temporal_voxel_layer/bridge/measurement_buffer.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
// costmap
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
// openVDB
#include "openvdb/openvdb.h"
// msgs
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "spatio_temporal_voxel_layer/srv/save_grid.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
// projector
#include "laser_geometry/laser_geometry.hpp"
// tf
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/buffer_core.h"

namespace spatio_temporal_voxel_layer
{

// Core ROS voxel layer class
class SpatioTemporalVoxelLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SpatioTemporalVoxelLayer(void);
  virtual ~SpatioTemporalVoxelLayer(void);

  // Core Functions
  virtual void onInitialize(void);
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  // Functions to interact with other layers
  virtual void matchSize(void);

  // Functions for layer high level operations
  virtual void reset(void);
  virtual void activate(void);
  virtual void deactivate(void);
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert_area=false) override;

  virtual bool isClearable() {return true;}

  const std::vector<int32_t> & getElevationLayer() const {return _elevation_layer;}
  int32_t getNoElevationDataValue() const {return _no_elevation_data;}
  const std::vector<float> & getElevationLayerMeters() const {return _elevation_layer_m;}
  float getNoElevationMetersValue() const {return _no_elevation_data_m;}

  // Functions for sensor feeds
  bool GetMarkingObservations(std::vector<observation::MeasurementReading> & marking_observations)
  const;
  bool GetClearingObservations(std::vector<observation::MeasurementReading> & marking_observations)
  const;
  void ObservationsResetAfterReading() const;

  // Functions to interact with maps
  void UpdateROSCostmap(
    double * min_x, double * min_y, double * max_x, double * max_y,
    volume_grid::OccupanyCellSet & cleared_cells);
  bool updateFootprint(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);
  void ResetGrid(void);

  // Saving grids callback for openVDB
  void SaveGridCallback(
    const std::shared_ptr<rmw_request_id_t>/*header*/,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Request> req,
    std::shared_ptr<spatio_temporal_voxel_layer::srv::SaveGrid::Response> resp);

private:
  // Sensor callbacks
  void LaserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);
  void LaserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr raw_message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);
  void PointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer);

  void heartbeatTimerCallback();

  bool getRobotBaseHeight(double & base_z);

  // Functions for adding static obstacle zones
  bool AddStaticObservations(const observation::MeasurementReading & obs);
  bool RemoveStaticObservations(void);

  // Enable/Disable callback
  void BufferEnablerCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response,
    const std::shared_ptr<buffer::MeasurementBuffer> buffer,
    const std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>
      & subcriber
    );

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  struct ObservationSourceConfig
  {
    std::string name;
    std::string topic;
    std::string sensor_frame;
    std::string data_type;
    bool inf_is_valid{false};
    bool marking{true};
    bool clearing{false};
    bool clear_after_reading{false};
    bool enabled{true};
    double observation_keep_time{0.0};
    double expected_update_rate{0.0};
    double min_obstacle_height{0.0};
    double max_obstacle_height{0.0};
    bool filter_obstacle_height{true};
    bool height_relative_to_base{false};
    bool required_for_heartbeat{true};
    double obstacle_range{0.0};
    double min_z{0.0};
    double max_z{0.0};
    bool use_clearing_min_max_z{true};
    double vertical_fov{0.0};
    double vertical_fov_padding{0.0};
    double horizontal_fov{0.0};
    double decay_acceleration{0.0};
    int voxel_min_points{0};
    buffer::Filters filter{buffer::Filters::NONE};
    ModelType model_type{ModelType::DEPTH_CAMERA};
  };

  void declareLayerParameters();
  void loadLayerParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    bool & track_unknown_space,
    double & transform_tolerance,
    double & map_save_time);
  ObservationSourceConfig loadObservationSourceConfig(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & source);
  buffer::MeasurementBufferConfig createMeasurementBufferConfig(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const ObservationSourceConfig & config,
    double transform_tolerance) const;
  void configureObservationSource(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const ObservationSourceConfig & config,
    const rclcpp::SubscriptionOptions & sub_opt,
    double transform_tolerance);
  void finalizeObservationSource(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const ObservationSourceConfig & config,
    const std::shared_ptr<buffer::MeasurementBuffer> & buffer,
    const internal::ObservationManager::SubscriberPtr & subscriber,
    const internal::ObservationManager::NotifierPtr & notifier);

  laser_geometry::LaserProjection _laser_projector;
  std::unique_ptr<internal::ObservationManager> _observation_manager;

  bool _publish_voxels, _publish_elevation_map, _mapping_mode, was_reset_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _voxel_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _elevation_pub;
  rclcpp::Service<spatio_temporal_voxel_layer::srv::SaveGrid>::SharedPtr _grid_saver;
  std::unique_ptr<rclcpp::Duration> _map_save_duration;
  rclcpp::Time _last_map_save_time;
  std::string _global_frame;
  double _voxel_size, _voxel_decay;
  int _combination_method, _mark_threshold;
  volume_grid::GlobalDecayModel _decay_model;
  bool _update_footprint_enabled, _enabled;
  internal::PruningConfig _pruning_config;
  std::unique_ptr<internal::PruningManager> _pruning_manager;
  double _max_elevation_above_robot_base{std::numeric_limits<double>::infinity()};
  bool _limit_elevation{false};
  std::vector<geometry_msgs::msg::Point> _transformed_footprint;
  std::unique_ptr<volume_grid::SpatioTemporalVoxelGrid> _voxel_grid;
  boost::recursive_mutex _voxel_grid_lock;

  std::vector<int32_t> _elevation_layer;
  int32_t _no_elevation_data{std::numeric_limits<int32_t>::min()};
  std::vector<float> _elevation_layer_m;
  float _no_elevation_data_m{std::numeric_limits<float>::quiet_NaN()};
  std::vector<size_t> _active_elevation_indices;

  double _elevation_window_size_m{0.0};
  double _elevation_lethal_threshold_m{0.0};
  double _elevation_window_min_samples{0.0};

  std::string _topics_string;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;

  // Heartbeat / health monitoring
  bool publish_heartbeat_{false};
  std::string heartbeat_topic_{"heartbeat"};
  std::string heartbeat_status_topic_{"heartbeat_status"};
  double heartbeat_period_s_{0.2};
  double heartbeat_costmap_timeout_s_{1.0};
  double heartbeat_default_sensor_timeout_s_{1.0};
  double heartbeat_min_sensor_timeout_s_{0.2};
  double heartbeat_expected_update_rate_multiplier_{2.5};
  std::unordered_map<std::string, bool> heartbeat_required_sources_{};

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_status_pub_;

  std::atomic<int64_t> last_update_bounds_success_ns_{0};
  std::atomic<int64_t> last_update_bounds_error_ns_{0};
  std::atomic<int64_t> last_update_costs_success_ns_{0};
  std::atomic<int64_t> last_update_costs_error_ns_{0};
  mutable std::mutex heartbeat_error_mutex_;
  std::string last_update_bounds_error_msg_;
  std::string last_update_costs_error_msg_;

};

}  // namespace spatio_temporal_voxel_layer
#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_LAYER_HPP_
