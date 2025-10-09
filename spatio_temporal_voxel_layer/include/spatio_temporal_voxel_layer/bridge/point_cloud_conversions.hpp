#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__POINT_CLOUD_CONVERSIONS_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__POINT_CLOUD_CONVERSIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "spatio_temporal_voxel_layer/core/types.hpp"

namespace stvl::bridge
{

inline sensor_msgs::msg::PointCloud2 toPointCloud2(
  const stvl::core::PointCloud & cloud,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return msg;
}

}  // namespace stvl::bridge

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__BRIDGE__POINT_CLOUD_CONVERSIONS_HPP_
