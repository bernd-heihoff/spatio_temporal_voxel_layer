#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__SLIDING_WINDOW_2D_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__SLIDING_WINDOW_2D_HPP_

#include <cstdint>
#include <vector>

namespace spatio_temporal_voxel_layer::internal
{

struct WindowStatsGrid
{
  int start_x{0};
  int start_y{0};
  int width{0};
  int height{0};

  std::vector<float> min_vals;
  std::vector<float> max_vals;
  std::vector<uint32_t> count_vals;
};

// Computes, for each cell in the evaluation bbox, the min/max elevation and number of finite
// samples inside a square window with half-extent (in cells). NaN values are ignored.
//
// - `elevation` is a row-major array of size (size_x * size_y).
// - Eval bbox is inclusive: [eval_start_x, eval_end_x] x [eval_start_y, eval_end_y].
// - Window is clipped to map bounds.
WindowStatsGrid computeWindowStats(
  const std::vector<float> & elevation,
  int size_x,
  int size_y,
  int eval_start_x,
  int eval_start_y,
  int eval_end_x,
  int eval_end_y,
  int window_half_extent_cells);

}  // namespace spatio_temporal_voxel_layer::internal

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__SLIDING_WINDOW_2D_HPP_
