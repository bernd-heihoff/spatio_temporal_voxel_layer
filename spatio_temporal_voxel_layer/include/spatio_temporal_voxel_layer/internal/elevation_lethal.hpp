#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__ELEVATION_LETHAL_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__ELEVATION_LETHAL_HPP_

#include <cstdint>
#include <vector>

namespace spatio_temporal_voxel_layer::internal
{

struct LethalMaskGrid
{
  int start_x{0};
  int start_y{0};
  int width{0};
  int height{0};

  // Row-major mask (width * height). 1 = lethal, 0 = not lethal.
  std::vector<uint8_t> lethal;
};

// Computes a lethal mask for an elevation grid.
//
// A cell is marked lethal when:
// - the number of finite samples in the clipped square window is LESS than
//   `min_samples_fraction` (0..1) of the window cells (conservative: treat
//   insufficient data as lethal)
//   OR
// - the center cell elevation is finite AND the number of finite samples in
//   the clipped square window meets `min_samples_fraction` AND
//   (max_elevation - min_elevation) within the window exceeds `threshold_m`
//
// Inputs:
// - `elevation` is row-major array of size (size_x * size_y).
// - Eval bbox is inclusive: [eval_start_x, eval_end_x] x [eval_start_y, eval_end_y].
// - Window half extent is in cells.
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
  double min_samples_fraction);

}  // namespace spatio_temporal_voxel_layer::internal

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__INTERNAL__ELEVATION_LETHAL_HPP_
