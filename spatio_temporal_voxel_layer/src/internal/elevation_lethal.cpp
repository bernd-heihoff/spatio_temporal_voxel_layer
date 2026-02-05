#include "spatio_temporal_voxel_layer/internal/elevation_lethal.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "spatio_temporal_voxel_layer/internal/sliding_window_2d.hpp"

namespace spatio_temporal_voxel_layer::internal
{

namespace
{
inline size_t idx2d(int x, int y, int w)
{
  return static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
}

}  // namespace

LethalMaskGrid computeElevationLethalMask(
  const std::vector<float> & elevation,
  int size_x,
  int size_y,
  int eval_start_x,
  int eval_start_y,
  int eval_end_x,
  int eval_end_y,
  int window_half_extent_cells,
  double threshold_m,
  double min_samples_fraction)
{
  LethalMaskGrid out;

  if (size_x <= 0 || size_y <= 0) {
    return out;
  }

  if (threshold_m <= 0.0) {
    return out;
  }

  min_samples_fraction = std::clamp(min_samples_fraction, 0.0, 1.0);

  eval_start_x = std::max(0, std::min(size_x - 1, eval_start_x));
  eval_end_x = std::max(0, std::min(size_x - 1, eval_end_x));
  eval_start_y = std::max(0, std::min(size_y - 1, eval_start_y));
  eval_end_y = std::max(0, std::min(size_y - 1, eval_end_y));

  if (eval_start_x > eval_end_x || eval_start_y > eval_end_y) {
    return out;
  }

  const int k = std::max(0, window_half_extent_cells);

  const auto stats = computeWindowStats(
    elevation,
    size_x,
    size_y,
    eval_start_x,
    eval_start_y,
    eval_end_x,
    eval_end_y,
    k);

  out.start_x = stats.start_x;
  out.start_y = stats.start_y;
  out.width = stats.width;
  out.height = stats.height;
  out.lethal.assign(static_cast<size_t>(out.width) * static_cast<size_t>(out.height), 0U);

  for (int y = 0; y < out.height; ++y) {
    const int map_y = out.start_y + y;

    for (int x = 0; x < out.width; ++x) {
      const int map_x = out.start_x + x;

      const size_t map_index = static_cast<size_t>(map_y) * static_cast<size_t>(size_x) +
        static_cast<size_t>(map_x);
      if (map_index >= elevation.size()) {
        continue;
      }

      const float center_h = elevation[map_index];
      if (!std::isfinite(center_h)) {
        continue;
      }

      const int wx0 = std::max(0, map_x - k);
      const int wx1 = std::min(size_x - 1, map_x + k);
      const int wy0 = std::max(0, map_y - k);
      const int wy1 = std::min(size_y - 1, map_y + k);
      const int window_cells = (wx1 - wx0 + 1) * (wy1 - wy0 + 1);

      const int required_samples = (min_samples_fraction <= 0.0) ? 1 :
        std::max(1, static_cast<int>(
          std::ceil(min_samples_fraction * static_cast<double>(window_cells))));

      const size_t i = idx2d(x, y, out.width);
      if (static_cast<int>(stats.count_vals[i]) < required_samples) {
        continue;
      }

      const float min_h = stats.min_vals[i];
      const float max_h = stats.max_vals[i];
      if (!std::isfinite(min_h) || !std::isfinite(max_h)) {
        continue;
      }

      if (static_cast<double>(max_h) - static_cast<double>(min_h) > threshold_m) {
        out.lethal[i] = 1U;
      }
    }
  }

  return out;
}

}  // namespace spatio_temporal_voxel_layer::internal
