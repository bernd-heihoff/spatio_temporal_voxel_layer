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
 * Purpose: Implement OpenVDB's voxel library with ray tracing for our
 *          internal voxel grid layer.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_

// STL
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>
#include <memory>
#include <string>
#include <limits>
#include <cstdint>
#include <functional>
// PCL
#include "pcl/common/transforms.h"
#include "pcl/PCLPointCloud2.h"
// OpenVDB
#include "openvdb/openvdb.h"
#include "openvdb/tools/GridTransformer.h"
#include "openvdb/math/BBox.h"
#include "openvdb/tools/RayIntersector.h"

#include "spatio_temporal_voxel_layer/core/types.hpp"
#include "spatio_temporal_voxel_layer/measurement_reading.h"

// measurement struct and frustum models
#include "spatio_temporal_voxel_layer/frustum_models/depth_camera_frustum.hpp"
#include "spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp"
// Mutex and locks
#include "boost/thread.hpp"
#include "boost/thread/recursive_mutex.hpp"

namespace volume_grid
{

enum GlobalDecayModel
{
  LINEAR = 0,
  EXPONENTIAL = 1,
  PERSISTENT = 2
};

// Structure for an occupied cell for map
struct occupany_cell
{
  occupany_cell(const double & _x, const double & _y)
  : x(_x), y(_y)
  {
  }

  bool operator==(const occupany_cell & other) const
  {
    return x == other.x && y == other.y;
  }

  double x, y;
};

struct ColumnElevation
{
  static constexpr int32_t NO_DATA = std::numeric_limits<int32_t>::min();

  void reset()
  {
    elevation_index = NO_DATA;
    point_count = 0U;
    elevation_m = std::numeric_limits<double>::quiet_NaN();
    samples.clear();
  }

  bool empty() const
  {
    return elevation_index == NO_DATA;
  }

  void updateWithMeasurement(int32_t new_index, double world_z)
  {
    samples.push_back({new_index, world_z});
    if (empty() || new_index > elevation_index ||
      (new_index == elevation_index && world_z > elevation_m))
    {
      elevation_index = new_index;
      elevation_m = world_z;
    }
    ++point_count;
  }

  bool highestBelow(double limit, int32_t & out_index, double & out_world_z) const
  {
    double best_world_z = std::numeric_limits<double>::lowest();
    int32_t best_index = NO_DATA;
    bool found = false;

    for (const auto & sample : samples) {
      if (sample.world_z <= limit && sample.world_z > best_world_z) {
        best_world_z = sample.world_z;
        best_index = sample.index;
        found = true;
      }
    }

    if (found) {
      out_index = best_index;
      out_world_z = best_world_z;
      return true;
    }

    return false;
  }

  int32_t elevation_index{NO_DATA};
  uint32_t point_count{0U};
  double elevation_m{std::numeric_limits<double>::quiet_NaN()};
  struct Sample
  {
    int32_t index;
    double world_z;
  };
  std::vector<Sample> samples;
};

struct OccupanyCellHash
{
  std::size_t operator()(const occupany_cell & cell) const noexcept
  {
    return (std::hash<double>()(cell.x) ^ (std::hash<double>()(cell.y) << 1)) >> 1;
  }
};

using ColumnElevationMap = std::unordered_map<occupany_cell, ColumnElevation, OccupanyCellHash>;
using OccupanyCellSet = std::unordered_set<occupany_cell, OccupanyCellHash>;

// Structure for wrapping frustum model and necessary metadata
struct frustum_model
{
  frustum_model(geometry::Frustum * _frustum, const double & _factor)
  : frustum(_frustum), accel_factor(_factor)
  {
  }
  ~frustum_model()
  {
    if (frustum) {
      delete frustum;
    }
  }
  geometry::Frustum * frustum;
  const double accel_factor;
};

// Core voxel grid structure and interface
class SpatioTemporalVoxelGrid
{
public:
  // conveniences for line lengths
  typedef openvdb::math::Ray<openvdb::Real> GridRay;
  typedef openvdb::math::Ray<openvdb::Real>::Vec3T Vec3Type;

  using TimeSource = std::function<double()>;

  SpatioTemporalVoxelGrid(
    TimeSource time_source,
    const float & voxel_size, const double & background_value,
    const int & decay_model, const double & voxel_decay,
    const bool & pub_voxels);
  ~SpatioTemporalVoxelGrid(void);

  // Core making and clearing functions
  void Mark(const std::vector<observation::MeasurementReading> & marking_observations);
  void operator()(const observation::MeasurementReading & obs) const;
  void ClearFrustums(
    const std::vector<observation::MeasurementReading> & clearing_observations,
    OccupanyCellSet & cleared_cells);

  // Get the pointcloud of the underlying occupancy grid
  void GetOccupancyPointCloud(stvl::core::PointCloud & cloud);
  void GetElevationPointCloud(
    stvl::core::PointCloud & cloud, bool limit, double max_world_z);
  ColumnElevationMap * GetColumnElevationMap();
  OccupanyCellSet * GetTouchedColumns();

  // Clear the grid
  bool ResetGrid(void);
  void ResetGridArea(const occupany_cell & start, const occupany_cell & end, bool invert_area=false);
  bool ClipToBoundingBox(const openvdb::BBoxd & bbox);

  // Save the file to file with size information
  bool SaveGrid(const std::string & file_name, double & map_size_bytes);

protected:
  // Initialize grid metadata and library
  void InitializeGrid(void);

  // grid accessor methods
  bool MarkGridPoint(const openvdb::Coord & pt, const double & value) const;
  bool ClearGridPoint(const openvdb::Coord & pt) const;

  // Check occupancy status of the grid
  bool IsGridEmpty(void) const;

  // Get time information for clearing
  double GetTemporalClearingDuration(const double & time_delta);
  double GetFrustumAcceleration(
    const double & time_delta, const double & acceleration_factor);
  void TemporalClearAndGenerateCostmap(
    std::vector<frustum_model> & frustums,
    OccupanyCellSet & cleared_cells);

  // Populate the costmap ROS api and pointcloud with a marked point
  void PopulateCostmapAndPointcloud(const openvdb::Coord & pt);

  // Utilities for tranformation
  openvdb::Vec3d WorldToIndex(const openvdb::Vec3d & coord) const;
  openvdb::Vec3d IndexToWorld(const openvdb::Coord & coord) const;

  TimeSource time_source_;

  mutable openvdb::DoubleGrid::Ptr _grid;
  int _decay_model;
  double _background_value, _voxel_size, _voxel_decay;
  bool _pub_voxels;
  std::unique_ptr<stvl::core::PointCloud> grid_points_;
  ColumnElevationMap _column_elevations;
  OccupanyCellSet _touched_columns;
  boost::mutex _grid_lock;
};

}  // namespace volume_grid

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__SPATIO_TEMPORAL_VOXEL_GRID_HPP_
