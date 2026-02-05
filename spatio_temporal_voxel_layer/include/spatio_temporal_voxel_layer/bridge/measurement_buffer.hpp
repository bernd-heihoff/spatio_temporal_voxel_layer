/********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
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
 * Purpose: buffer incoming measurements for grid
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__MEASUREMENT_BUFFER_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__MEASUREMENT_BUFFER_HPP_

// STL
#include <atomic>
#include <vector>
#include <list>
#include <string>
#include <chrono>
#include <memory>
#include <mutex>
// measurement structs
#include "spatio_temporal_voxel_layer/measurement_reading.h"
// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
// TF
#include "tf2_ros/buffer.h"
#include "message_filters/subscriber.h"
// msgs
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// Mutex
#include "boost/thread.hpp"

namespace buffer
{

enum class Filters
{
  NONE = 0,
  VOXEL = 1,
  PASSTHROUGH = 2
};

// conveniences for line lengths
typedef std::list<observation::MeasurementReading>::iterator readings_iter;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud_ptr;

struct MeasurementBufferConfig
{
  std::string source_name;
  std::string topic_name;
  double observation_keep_time{0.0};
  double expected_update_rate{0.0};
  double min_obstacle_height{0.0};
  double max_obstacle_height{0.0};
  bool height_relative_to_base{false};
  std::string robot_base_frame;
  double obstacle_range{0.0};
  tf2_ros::Buffer * tf_buffer{nullptr};
  std::string global_frame;
  std::string sensor_frame;
  double tf_tolerance{0.0};
  double min_z{0.0};
  double max_z{0.0};
  double vertical_fov{0.0};
  double vertical_fov_padding{0.0};
  double horizontal_fov{0.0};
  double decay_acceleration{0.0};
  bool marking{true};
  bool clearing{false};
  double voxel_size{0.0};
  Filters filter{Filters::NONE};
  int voxel_min_points{0};
  bool enabled{true};
  bool clear_buffer_after_reading{false};
  ModelType model_type{ModelType::DEPTH_CAMERA};
  rclcpp::Clock::SharedPtr clock{nullptr};
  rclcpp::Logger logger{rclcpp::get_logger("measurement_buffer")};
};

class MeasurementBufferBuilder
{
public:
  MeasurementBufferBuilder & setSourceName(const std::string & value)
  {
    config_.source_name = value;
    return *this;
  }

  MeasurementBufferBuilder & setTopicName(const std::string & value)
  {
    config_.topic_name = value;
    return *this;
  }

  MeasurementBufferBuilder & setObservationKeepTime(double value)
  {
    config_.observation_keep_time = value;
    return *this;
  }

  MeasurementBufferBuilder & setExpectedUpdateRate(double value)
  {
    config_.expected_update_rate = value;
    return *this;
  }

  MeasurementBufferBuilder & setMinObstacleHeight(double value)
  {
    config_.min_obstacle_height = value;
    return *this;
  }

  MeasurementBufferBuilder & setMaxObstacleHeight(double value)
  {
    config_.max_obstacle_height = value;
    return *this;
  }

  MeasurementBufferBuilder & setHeightRelativeToBase(bool value)
  {
    config_.height_relative_to_base = value;
    return *this;
  }

  MeasurementBufferBuilder & setRobotBaseFrame(const std::string & value)
  {
    config_.robot_base_frame = value;
    return *this;
  }

  MeasurementBufferBuilder & setObstacleRange(double value)
  {
    config_.obstacle_range = value;
    return *this;
  }

  MeasurementBufferBuilder & setTfBuffer(tf2_ros::Buffer * value)
  {
    config_.tf_buffer = value;
    return *this;
  }

  MeasurementBufferBuilder & setGlobalFrame(const std::string & value)
  {
    config_.global_frame = value;
    return *this;
  }

  MeasurementBufferBuilder & setSensorFrame(const std::string & value)
  {
    config_.sensor_frame = value;
    return *this;
  }

  MeasurementBufferBuilder & setTfTolerance(double value)
  {
    config_.tf_tolerance = value;
    return *this;
  }

  MeasurementBufferBuilder & setMinZ(double value)
  {
    config_.min_z = value;
    return *this;
  }

  MeasurementBufferBuilder & setMaxZ(double value)
  {
    config_.max_z = value;
    return *this;
  }

  MeasurementBufferBuilder & setVerticalFov(double value)
  {
    config_.vertical_fov = value;
    return *this;
  }

  MeasurementBufferBuilder & setVerticalFovPadding(double value)
  {
    config_.vertical_fov_padding = value;
    return *this;
  }

  MeasurementBufferBuilder & setHorizontalFov(double value)
  {
    config_.horizontal_fov = value;
    return *this;
  }

  MeasurementBufferBuilder & setDecayAcceleration(double value)
  {
    config_.decay_acceleration = value;
    return *this;
  }

  MeasurementBufferBuilder & setMarking(bool value)
  {
    config_.marking = value;
    return *this;
  }

  MeasurementBufferBuilder & setClearing(bool value)
  {
    config_.clearing = value;
    return *this;
  }

  MeasurementBufferBuilder & setVoxelSize(double value)
  {
    config_.voxel_size = value;
    return *this;
  }

  MeasurementBufferBuilder & setFilter(Filters value)
  {
    config_.filter = value;
    return *this;
  }

  MeasurementBufferBuilder & setVoxelMinPoints(int value)
  {
    config_.voxel_min_points = value;
    return *this;
  }

  MeasurementBufferBuilder & setEnabled(bool value)
  {
    config_.enabled = value;
    return *this;
  }

  MeasurementBufferBuilder & setClearBufferAfterReading(bool value)
  {
    config_.clear_buffer_after_reading = value;
    return *this;
  }

  MeasurementBufferBuilder & setModelType(ModelType value)
  {
    config_.model_type = value;
    return *this;
  }

  MeasurementBufferBuilder & setClock(const rclcpp::Clock::SharedPtr & clock)
  {
    config_.clock = clock;
    return *this;
  }

  MeasurementBufferBuilder & setLogger(const rclcpp::Logger & logger)
  {
    config_.logger = logger;
    return *this;
  }

  MeasurementBufferConfig build() const
  {
    return config_;
  }

private:
  MeasurementBufferConfig config_;
};

// Measurement buffer
class MeasurementBuffer
{
public:
  explicit MeasurementBuffer(const MeasurementBufferConfig & config);

  ~MeasurementBuffer(void);

  // Buffers for different types of pointclouds
  void BufferROSCloud(const sensor_msgs::msg::PointCloud2 & cloud);

  // Get measurements from the buffer
  void GetReadings(std::vector<observation::MeasurementReading> & observations);

  // enabler setter getter
  bool IsEnabled(void) const;
  void SetEnabled(const bool & enabled);

  // Source name getter
  std::string GetSourceName(void) const;

  // Topic name getter
  std::string GetTopicName(void) const;

  // Heartbeat / health bookkeeping (wall time from the configured clock)
  rclcpp::Time GetLastReceivedTime(void) const;
  rclcpp::Time GetLastSuccessfulBufferTime(void) const;
  rclcpp::Time GetLastErrorTime(void) const;
  uint64_t GetErrorCount(void) const;
  std::string GetLastErrorMessage(void) const;
  double GetExpectedUpdateRateSeconds(void) const;

  // params setters
  void SetMinObstacleHeight(const double & min_obstacle_height);
  void SetMaxObstacleHeight(const double & max_obstacle_height);
  void SetHeightRelativeToBase(const bool & enabled);
  void SetRobotBaseFrame(const std::string & frame);
  void SetMinZ(const double & min_z);
  void SetMaxZ(const double & max_z);
  void SetVerticalFovPadding(const double & vertical_fov_padding);
  void SetHorizontalFovAngle(const double & horizontal_fov_angle);
  void SetVerticalFovAngle(const double & vertical_fov_angle);

  // State knoweldge if sensors are operating as expected
  bool UpdatedAtExpectedRate(void) const;
  void ResetLastUpdatedTime(void);
  void ResetAllMeasurements(void);
  bool ClearAfterReading(void);

  // Public mutex locks
  void Lock(void);
  void Unlock(void);

private:
  // Removing old observations from buffer
  void RemoveStaleObservations(void);

  observation::MeasurementReading & CreateObservationSlot(double stamp_in_seconds);
  geometry_msgs::msg::PoseStamped MakeLocalSensorPose(
    const std::string & origin_frame,
    const rclcpp::Time & stamp) const;
  geometry_msgs::msg::PoseStamped TransformPoseToGlobal(
    const geometry_msgs::msg::PoseStamped & local_pose) const;
  point_cloud_ptr TransformCloudToGlobal(const sensor_msgs::msg::PointCloud2 & cloud) const;
  void ApplyFilter(
    sensor_msgs::msg::PointCloud2 & cloud,
    const builtin_interfaces::msg::Time & stamp) const;
  void PopulateObservationMetadata(
    observation::MeasurementReading & observation,
    const geometry_msgs::msg::PoseStamped & global_pose,
    double stamp_in_seconds) const;
  void AssignPointCloud(
    observation::MeasurementReading & observation,
    const sensor_msgs::msg::PointCloud2 & cloud) const;

  tf2_ros::Buffer & _buffer;
  const rclcpp::Duration _observation_keep_time, _expected_update_rate;
  rclcpp::Time _last_updated;
  boost::recursive_mutex _lock;
  std::string _global_frame, _sensor_frame, _source_name, _topic_name;
  std::list<observation::MeasurementReading> _observation_list;
  double _min_obstacle_height, _max_obstacle_height, _obstacle_range, _tf_tolerance;
  bool _height_relative_to_base{false};
  std::string _robot_base_frame;
  double _min_z, _max_z, _vertical_fov, _vertical_fov_padding, _horizontal_fov;
  double _decay_acceleration, _voxel_size;
  bool _marking, _clearing;
  Filters _filter;
  int _voxel_min_points;
  bool _clear_buffer_after_reading, _enabled;
  ModelType _model_type;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  std::atomic<int64_t> last_received_time_ns_{0};
  std::atomic<int64_t> last_success_time_ns_{0};
  std::atomic<int64_t> last_error_time_ns_{0};
  std::atomic<uint64_t> error_count_{0};
  mutable std::mutex last_error_mutex_;
  std::string last_error_message_;
};

}  // namespace buffer

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__MEASUREMENT_BUFFER_HPP_
