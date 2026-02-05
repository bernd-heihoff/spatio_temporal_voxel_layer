/*********************************************************************
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
 *********************************************************************/

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <stdexcept>
#include "spatio_temporal_voxel_layer/bridge/measurement_buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace buffer
{

namespace
{

tf2_ros::Buffer & requireTfBuffer(tf2_ros::Buffer * buffer)
{
  if (!buffer) {
    throw std::invalid_argument("MeasurementBuffer requires non-null tf buffer");
  }
  return *buffer;
}

}

using namespace std::chrono_literals;

/*****************************************************************************/
MeasurementBuffer::MeasurementBuffer(const MeasurementBufferConfig & config)
: _buffer(requireTfBuffer(config.tf_buffer)),
  _observation_keep_time(rclcpp::Duration::from_seconds(config.observation_keep_time)),
  _expected_update_rate(rclcpp::Duration::from_seconds(config.expected_update_rate)),
  _last_updated(rclcpp::Time(0, 0, RCL_STEADY_TIME)),
  _global_frame(config.global_frame),
  _sensor_frame(config.sensor_frame),
  _source_name(config.source_name),
  _topic_name(config.topic_name),
  _min_obstacle_height(config.min_obstacle_height),
  _max_obstacle_height(config.max_obstacle_height),
  _obstacle_range(config.obstacle_range),
  _tf_tolerance(config.tf_tolerance),
  _height_relative_to_base(config.height_relative_to_base),
  _robot_base_frame(config.robot_base_frame),
  _min_z(config.min_z),
  _max_z(config.max_z),
  _vertical_fov(config.vertical_fov),
  _vertical_fov_padding(config.vertical_fov_padding),
  _horizontal_fov(config.horizontal_fov),
  _decay_acceleration(config.decay_acceleration),
  _voxel_size(config.voxel_size),
  _marking(config.marking),
  _clearing(config.clearing),
  _filter(config.filter),
  _voxel_min_points(config.voxel_min_points),
  _clear_buffer_after_reading(config.clear_buffer_after_reading),
  _enabled(config.enabled),
  _model_type(config.model_type),
  clock_(config.clock),
  logger_(config.logger)
/*****************************************************************************/
{
  if (!clock_) {
    throw std::invalid_argument("MeasurementBuffer requires non-null clock");
  }
  _last_updated = clock_->now();
}

/*****************************************************************************/
MeasurementBuffer::~MeasurementBuffer(void)
/*****************************************************************************/
{
}

/*****************************************************************************/
void MeasurementBuffer::BufferROSCloud(
  const sensor_msgs::msg::PointCloud2 & cloud)
/*****************************************************************************/
{
  const std::string origin_frame =
    _sensor_frame.empty() ? cloud.header.frame_id : _sensor_frame;

  const double stamp_in_seconds = rclcpp::Time(cloud.header.stamp).seconds();
  observation::MeasurementReading & observation =
    CreateObservationSlot(stamp_in_seconds);

  try {
    const auto local_pose = MakeLocalSensorPose(origin_frame, cloud.header.stamp);
    const auto global_pose = TransformPoseToGlobal(local_pose);
    PopulateObservationMetadata(observation, global_pose, stamp_in_seconds);

    if (_clearing && !_marking) {
      _last_updated = clock_->now();
      RemoveStaleObservations();
      return;
    }

    auto global_cloud = TransformCloudToGlobal(cloud);
    ApplyFilter(*global_cloud, cloud.header.stamp);
    AssignPointCloud(observation, *global_cloud);
  } catch (const tf2::TransformException & ex) {
    _observation_list.pop_front();
    RCLCPP_ERROR(
      logger_,
      "TF Exception for sensor frame: %s, cloud frame: %s, %s",
      _sensor_frame.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  } catch (const std::exception & ex) {
    _observation_list.pop_front();
    RCLCPP_ERROR(
      logger_,
      "Failed to buffer cloud for %s (%s): %s",
      _source_name.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  _last_updated = clock_->now();
  RemoveStaleObservations();
}

/*****************************************************************************/
void MeasurementBuffer::GetReadings(
  std::vector<observation::MeasurementReading> & observations)
/*****************************************************************************/
{
  RemoveStaleObservations();

  for (readings_iter it = _observation_list.begin();
    it != _observation_list.end(); ++it)
  {
    observations.push_back(*it);
  }
}

/*****************************************************************************/
void MeasurementBuffer::RemoveStaleObservations(void)
/*****************************************************************************/
{
  if (_observation_list.empty()) {
    return;
  }

  readings_iter it = _observation_list.begin();
  if (_observation_keep_time == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0))) {
    _observation_list.erase(++it, _observation_list.end());
    return;
  }

  for (it = _observation_list.begin(); it != _observation_list.end(); ++it) {
    const double time_diff = clock_->now().seconds() - it->_stamp_in_seconds;
    if (time_diff > _observation_keep_time.seconds()) {
      _observation_list.erase(it, _observation_list.end());
      return;
    }
  }
}

/*****************************************************************************/
void MeasurementBuffer::ResetAllMeasurements(void)
/*****************************************************************************/
{
  _observation_list.clear();
}

/*****************************************************************************/
bool MeasurementBuffer::ClearAfterReading(void)
/*****************************************************************************/
{
  return _clear_buffer_after_reading;
}

/*****************************************************************************/
bool MeasurementBuffer::UpdatedAtExpectedRate(void) const
/*****************************************************************************/
{
  if (_expected_update_rate == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0))) {
    return true;
  }

  const rclcpp::Duration update_time = clock_->now() - _last_updated;
  bool current = update_time.seconds() <= _expected_update_rate.seconds();
  if (!current) {
    RCLCPP_WARN(
      logger_,
      "%s buffer updated in %.2fs, it should be updated every %.2fs.",
      _topic_name.c_str(), update_time.seconds(),
      _expected_update_rate.seconds());
  }
  return current;
}

/*****************************************************************************/
bool MeasurementBuffer::IsEnabled(void) const
/*****************************************************************************/
{
  return _enabled;
}

/*****************************************************************************/
void MeasurementBuffer::SetEnabled(const bool & enabled)
/*****************************************************************************/
{
  _enabled = enabled;
}

/*****************************************************************************/
std::string MeasurementBuffer::GetSourceName(void) const
/*****************************************************************************/
{
  return _source_name;
}

/*****************************************************************************/
void MeasurementBuffer::SetMinObstacleHeight(const double & min_obstacle_height)
/*****************************************************************************/
{
  _min_obstacle_height = min_obstacle_height;
}

/*****************************************************************************/
void MeasurementBuffer::SetMaxObstacleHeight(const double & max_obstacle_height)
/*****************************************************************************/
{
  _max_obstacle_height = max_obstacle_height;
}

/*****************************************************************************/
void MeasurementBuffer::SetHeightRelativeToBase(const bool & enabled)
/*****************************************************************************/
{
  _height_relative_to_base = enabled;
}

/*****************************************************************************/
void MeasurementBuffer::SetRobotBaseFrame(const std::string & frame)
/*****************************************************************************/
{
  _robot_base_frame = frame;
}

/*****************************************************************************/
void MeasurementBuffer::SetMinZ(const double & min_z)
/*****************************************************************************/
{
  _min_z = min_z;
}

/*****************************************************************************/
void MeasurementBuffer::SetMaxZ(const double & max_z)
/*****************************************************************************/
{
  _max_z = max_z;
}

/*****************************************************************************/
void MeasurementBuffer::SetVerticalFovAngle(const double & vertical_fov_angle)
/*****************************************************************************/
{
  _vertical_fov = vertical_fov_angle;
}

/*****************************************************************************/
void MeasurementBuffer::SetVerticalFovPadding(const double & vertical_fov_padding)
/*****************************************************************************/
{
  _vertical_fov_padding = vertical_fov_padding;
}

/*****************************************************************************/
void MeasurementBuffer::SetHorizontalFovAngle(const double & horizontal_fov_angle)
/*****************************************************************************/
{
  _horizontal_fov = horizontal_fov_angle;
}

/*****************************************************************************/
void MeasurementBuffer::ResetLastUpdatedTime(void)
/*****************************************************************************/
{
  _last_updated = clock_->now();
}

/*****************************************************************************/
void MeasurementBuffer::Lock(void)
/*****************************************************************************/
{
  _lock.lock();
}

/*****************************************************************************/
void MeasurementBuffer::Unlock(void)
/*****************************************************************************/
{
  _lock.unlock();
}

/*****************************************************************************/
observation::MeasurementReading & MeasurementBuffer::CreateObservationSlot(double stamp_in_seconds)
/*****************************************************************************/
{
  _observation_list.push_front(observation::MeasurementReading());
  _observation_list.front()._stamp_in_seconds = stamp_in_seconds;
  return _observation_list.front();
}

/*****************************************************************************/
geometry_msgs::msg::PoseStamped MeasurementBuffer::MakeLocalSensorPose(
  const std::string & origin_frame,
  const rclcpp::Time & stamp) const
/*****************************************************************************/
{
  geometry_msgs::msg::PoseStamped local_pose;
  local_pose.pose.position.x = 0.0;
  local_pose.pose.position.y = 0.0;
  local_pose.pose.position.z = 0.0;
  local_pose.pose.orientation.x = 0.0;
  local_pose.pose.orientation.y = 0.0;
  local_pose.pose.orientation.z = 0.0;
  local_pose.pose.orientation.w = 1.0;
  local_pose.header.frame_id = origin_frame;
  local_pose.header.stamp = stamp;
  return local_pose;
}

/*****************************************************************************/
geometry_msgs::msg::PoseStamped MeasurementBuffer::TransformPoseToGlobal(
  const geometry_msgs::msg::PoseStamped & local_pose) const
/*****************************************************************************/
{
  geometry_msgs::msg::PoseStamped global_pose;
  _buffer.canTransform(
    _global_frame, local_pose.header.frame_id,
    tf2_ros::fromMsg(local_pose.header.stamp), tf2::durationFromSec(0.5));
  _buffer.transform(local_pose, global_pose, _global_frame);
  return global_pose;
}

/*****************************************************************************/
point_cloud_ptr MeasurementBuffer::TransformCloudToGlobal(
  const sensor_msgs::msg::PointCloud2 & cloud) const
/*****************************************************************************/
{
  point_cloud_ptr global_cloud(new sensor_msgs::msg::PointCloud2());
  geometry_msgs::msg::TransformStamped tf_stamped = _buffer.lookupTransform(
    _global_frame, cloud.header.frame_id,
    tf2_ros::fromMsg(cloud.header.stamp));
  tf2::doTransform(cloud, *global_cloud, tf_stamped);
  return global_cloud;
}

/*****************************************************************************/
void MeasurementBuffer::ApplyFilter(
  sensor_msgs::msg::PointCloud2 & cloud,
  const builtin_interfaces::msg::Time & stamp) const
/*****************************************************************************/
{
  if (_filter == Filters::NONE) {
    if (_height_relative_to_base) {
      throw std::runtime_error(
        _source_name +
        " height_relative_to_base is true but filter is NONE; rejecting observation (enable 'passthrough' or 'voxel').");
    }
    return;
  }

  double height_offset_z = 0.0;
  if (_height_relative_to_base) {
    if (_robot_base_frame.empty()) {
      throw std::runtime_error(
        _source_name +
        " height_relative_to_base is true but robot_base_frame is empty; rejecting observation.");
    } else {
      try {
        const geometry_msgs::msg::TransformStamped base_in_global = _buffer.lookupTransform(
          _global_frame, _robot_base_frame, tf2_ros::fromMsg(stamp));
        height_offset_z = static_cast<double>(base_in_global.transform.translation.z);
      } catch (const tf2::TransformException & ex) {
        throw std::runtime_error(
          _source_name +
          " failed to lookup base frame '" + _robot_base_frame +
          "' in '" + _global_frame +
          "' for height-relative filtering: " + ex.what() + "; rejecting observation.");
      }
    }
  }

  double min_h = _min_obstacle_height + height_offset_z;
  double max_h = _max_obstacle_height + height_offset_z;
  if (min_h > max_h) {
    std::swap(min_h, max_h);
  }

  pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(cloud, *cloud_pcl);

  if (_filter == Filters::VOXEL) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_pcl);
    sor.setFilterFieldName("z");
    sor.setFilterLimits(min_h, max_h);
    sor.setDownsampleAllData(false);
    float v_s = static_cast<float>(_voxel_size);
    sor.setLeafSize(v_s, v_s, v_s);
    sor.setMinimumPointsNumberPerVoxel(static_cast<unsigned int>(_voxel_min_points));
    sor.filter(*cloud_filtered);
    pcl_conversions::fromPCL(*cloud_filtered, cloud);
  } else if (_filter == Filters::PASSTHROUGH) {
    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
    pass_through_filter.setInputCloud(cloud_pcl);
    pass_through_filter.setKeepOrganized(false);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(min_h, max_h);
    pass_through_filter.filter(*cloud_filtered);
    pcl_conversions::fromPCL(*cloud_filtered, cloud);
  }
}

/*****************************************************************************/
void MeasurementBuffer::PopulateObservationMetadata(
  observation::MeasurementReading & observation,
  const geometry_msgs::msg::PoseStamped & global_pose,
  double stamp_in_seconds) const
/*****************************************************************************/
{
  observation._origin.x = global_pose.pose.position.x;
  observation._origin.y = global_pose.pose.position.y;
  observation._origin.z = global_pose.pose.position.z;

  const auto & orientation = global_pose.pose.orientation;
  observation._orientation =
    stvl::core::Quaternion{orientation.x, orientation.y, orientation.z, orientation.w};
  observation._obstacle_range_in_m = _obstacle_range;
  observation._min_z_in_m = _min_z;
  observation._max_z_in_m = _max_z;
  observation._vertical_fov_in_rad = _vertical_fov;
  observation._vertical_fov_padding_in_m = _vertical_fov_padding;
  observation._horizontal_fov_in_rad = _horizontal_fov;
  observation._decay_acceleration = _decay_acceleration;
  observation._clearing = _clearing;
  observation._marking = _marking;
  observation._model_type = _model_type;
  observation._stamp_in_seconds = stamp_in_seconds;
}

/*****************************************************************************/
void MeasurementBuffer::AssignPointCloud(
  observation::MeasurementReading & observation,
  const sensor_msgs::msg::PointCloud2 & cloud) const
/*****************************************************************************/
{
  auto pcl_cloud = std::make_shared<stvl::core::PointCloud>();
  pcl::fromROSMsg(cloud, *pcl_cloud);
  observation._cloud = std::move(pcl_cloud);
}

}  // namespace buffer
