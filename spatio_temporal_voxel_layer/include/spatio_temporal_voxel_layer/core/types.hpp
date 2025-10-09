#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__CORE__TYPES_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__CORE__TYPES_HPP_

#include <memory>

#include "Eigen/Geometry"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace stvl::core
{

using Scalar = double;

struct Point
{
  Scalar x{0.0};
  Scalar y{0.0};
  Scalar z{0.0};
};

struct Quaternion
{
  Scalar x{0.0};
  Scalar y{0.0};
  Scalar z{0.0};
  Scalar w{1.0};
};

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

inline Eigen::Vector3d toEigen(const Point & p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Quaterniond toEigen(const Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

inline Point fromEigen(const Eigen::Vector3d & v)
{
  return Point{v.x(), v.y(), v.z()};
}

inline Quaternion fromEigen(const Eigen::Quaterniond & q)
{
  return Quaternion{q.x(), q.y(), q.z(), q.w()};
}

}  // namespace stvl::core

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__CORE__TYPES_HPP_
