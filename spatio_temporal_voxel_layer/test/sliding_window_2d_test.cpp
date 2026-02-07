#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <random>

#include "spatio_temporal_voxel_layer/internal/sliding_window_2d.hpp"
#include "spatio_temporal_voxel_layer/internal/elevation_lethal.hpp"

namespace
{

inline size_t idx2d(int x, int y, int w)
{
  return static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
}

}  // namespace

TEST(SlidingWindow2D, ComputesMinMaxAndCountsWithNaNs)
{
  // 5x5 grid, with NaNs sprinkled.
  const int size_x = 5;
  const int size_y = 5;

  std::vector<float> grid(static_cast<size_t>(size_x * size_y),
    std::numeric_limits<float>::quiet_NaN());

  // Put a small 3x3 patch of finite values centered at (2,2)
  // Values chosen so min=1, max=9 in that patch.
  float v = 1.0f;
  for (int y = 1; y <= 3; ++y) {
    for (int x = 1; x <= 3; ++x) {
      grid[idx2d(x, y, size_x)] = v;
      v += 1.0f;
    }
  }

  const auto stats = spatio_temporal_voxel_layer::internal::computeWindowStats(
    grid,
    size_x,
    size_y,
    /*eval_start_x=*/2,
    /*eval_start_y=*/2,
    /*eval_end_x=*/2,
    /*eval_end_y=*/2,
    /*half_extent=*/1);

  ASSERT_EQ(stats.width, 1);
  ASSERT_EQ(stats.height, 1);
  ASSERT_EQ(stats.start_x, 2);
  ASSERT_EQ(stats.start_y, 2);

  EXPECT_FLOAT_EQ(stats.min_vals[0], 1.0f);
  EXPECT_FLOAT_EQ(stats.max_vals[0], 9.0f);
  EXPECT_EQ(stats.count_vals[0], 9U);
}

TEST(SlidingWindow2D, ClipsWindowAtEdges)
{
  // 3x3 grid, finite only on the first row.
  const int size_x = 3;
  const int size_y = 3;

  std::vector<float> grid(static_cast<size_t>(size_x * size_y),
    std::numeric_limits<float>::quiet_NaN());

  grid[idx2d(0, 0, size_x)] = 10.0f;
  grid[idx2d(1, 0, size_x)] = 20.0f;
  grid[idx2d(2, 0, size_x)] = 30.0f;

  // Evaluate at corner (0,0) with half_extent=1. Window clips to map bounds.
  const auto stats = spatio_temporal_voxel_layer::internal::computeWindowStats(
    grid,
    size_x,
    size_y,
    /*eval_start_x=*/0,
    /*eval_start_y=*/0,
    /*eval_end_x=*/0,
    /*eval_end_y=*/0,
    /*half_extent=*/1);

  ASSERT_EQ(stats.width, 1);
  ASSERT_EQ(stats.height, 1);

  // Window covers x=0..1, y=0..1. Only two finite samples.
  EXPECT_FLOAT_EQ(stats.min_vals[0], 10.0f);
  EXPECT_FLOAT_EQ(stats.max_vals[0], 20.0f);
  EXPECT_EQ(stats.count_vals[0], 2U);
}

namespace
{
std::vector<uint8_t> bruteForceLethal(
  const std::vector<float> & elevation,
  int size_x,
  int size_y,
  int eval_start_x,
  int eval_start_y,
  int eval_end_x,
  int eval_end_y,
  int half_extent,
  double threshold_m,
  double min_samples_fraction)
{
  eval_start_x = std::max(0, std::min(size_x - 1, eval_start_x));
  eval_end_x = std::max(0, std::min(size_x - 1, eval_end_x));
  eval_start_y = std::max(0, std::min(size_y - 1, eval_start_y));
  eval_end_y = std::max(0, std::min(size_y - 1, eval_end_y));

  const int w = eval_end_x - eval_start_x + 1;
  const int h = eval_end_y - eval_start_y + 1;
  std::vector<uint8_t> out(static_cast<size_t>(w) * static_cast<size_t>(h), 0U);

  min_samples_fraction = std::clamp(min_samples_fraction, 0.0, 1.0);
  half_extent = std::max(0, half_extent);

  for (int y = 0; y < h; ++y) {
    const int my = eval_start_y + y;
    for (int x = 0; x < w; ++x) {
      const int mx = eval_start_x + x;
      const int wx0 = std::max(0, mx - half_extent);
      const int wx1 = std::min(size_x - 1, mx + half_extent);
      const int wy0 = std::max(0, my - half_extent);
      const int wy1 = std::min(size_y - 1, my + half_extent);
      const int window_cells = (wx1 - wx0 + 1) * (wy1 - wy0 + 1);

      const int required = (min_samples_fraction <= 0.0) ? 1 :
        std::max(1, static_cast<int>(
          std::ceil(min_samples_fraction * static_cast<double>(window_cells))));

      int count = 0;
      double minv = std::numeric_limits<double>::infinity();
      double maxv = -std::numeric_limits<double>::infinity();

      for (int yy = wy0; yy <= wy1; ++yy) {
        for (int xx = wx0; xx <= wx1; ++xx) {
          const float v = elevation[idx2d(xx, yy, size_x)];
          if (!std::isfinite(v)) {
            continue;
          }
          ++count;
          minv = std::min(minv, static_cast<double>(v));
          maxv = std::max(maxv, static_cast<double>(v));
        }
      }

      if (count < required) {
        out[idx2d(x, y, w)] = 1U;
        continue;
      }

      const float center = elevation[idx2d(mx, my, size_x)];
      if (!std::isfinite(center)) {
        continue;
      }

      if ((maxv - minv) > threshold_m) {
        out[idx2d(x, y, w)] = 1U;
      }
    }
  }

  return out;
}

}  // namespace

TEST(ElevationLethal, DeterministicStepTriggers)
{
  const int size_x = 5;
  const int size_y = 5;
  std::vector<float> grid(static_cast<size_t>(size_x * size_y), 0.0f);

  // Make a step in the middle column.
  for (int y = 0; y < size_y; ++y) {
    grid[idx2d(2, y, size_x)] = 1.0f;
  }

  const int k = 1;
  const double threshold = 0.5;
  const double min_frac = 1.0;

  const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
    grid, size_x, size_y,
    0, 0, size_x - 1, size_y - 1,
    k, threshold, min_frac);

  // Cells adjacent to the step should see min=0, max=1 in their 3x3 windows.
  // We just sanity-check a couple.
  ASSERT_EQ(lethal.width, size_x);
  ASSERT_EQ(lethal.height, size_y);
  EXPECT_EQ(lethal.lethal[idx2d(1, 2, lethal.width)], 1U);
  EXPECT_EQ(lethal.lethal[idx2d(3, 2, lethal.width)], 1U);
}

TEST(ElevationLethal, MatchesBruteForceRandom)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> val_dist(-2.0f, 2.0f);
  std::uniform_real_distribution<float> nan_dist(0.0f, 1.0f);

  for (int iter = 0; iter < 50; ++iter) {
    const int size_x = 3 + (iter % 6);  // 3..8
    const int size_y = 3 + ((iter * 3) % 6);
    std::vector<float> grid(static_cast<size_t>(size_x * size_y), 0.0f);
    for (auto & v : grid) {
      v = (nan_dist(rng) < 0.2f) ? std::numeric_limits<float>::quiet_NaN() : val_dist(rng);
    }

    const int k = (iter % 3);  // 0..2
    const double threshold = 0.7;
    const double min_frac = (iter % 2 == 0) ? 0.0 : 0.6;

    const int ex0 = 0;
    const int ey0 = 0;
    const int ex1 = size_x - 1;
    const int ey1 = size_y - 1;

    const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
      grid, size_x, size_y, ex0, ey0, ex1, ey1, k, threshold, min_frac);

    const auto brute = bruteForceLethal(
      grid, size_x, size_y, ex0, ey0, ex1, ey1, k, threshold, min_frac);

    ASSERT_EQ(lethal.width, size_x);
    ASSERT_EQ(lethal.height, size_y);
    ASSERT_EQ(lethal.lethal.size(), brute.size());
    EXPECT_EQ(lethal.lethal, brute);
  }
}

TEST(ElevationLethal, MinSamplesFractionBoundariesEdgeClipped)
{
  // Evaluate at the top-left corner with k=1, so the window is clipped to 2x2 => 4 cells.
  // Provide only 3 finite samples in that clipped window.
  const int size_x = 3;
  const int size_y = 3;
  const float NaN = std::numeric_limits<float>::quiet_NaN();

  std::vector<float> grid{
    0.0f, 1.0f, NaN,
    0.0f, NaN,  NaN,
    NaN,  NaN,  NaN
  };

  const int k = 1;
  const double threshold = 0.5;

  // min_samples_fraction=1.0 requires all 4 clipped cells to be finite -> should NOT trigger.
  {
    const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
      grid, size_x, size_y,
      /*eval_start_x=*/0, /*eval_start_y=*/0,
      /*eval_end_x=*/0, /*eval_end_y=*/0,
      k, threshold, /*min_samples_fraction=*/1.0);
    ASSERT_EQ(lethal.width, 1);
    ASSERT_EQ(lethal.height, 1);
    EXPECT_EQ(lethal.lethal[0], 1U);
  }

  // Just below 1.0 still rounds up via ceil() for 4 cells: ceil(0.999 * 4) = 4.
  {
    const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
      grid, size_x, size_y,
      0, 0, 0, 0,
      k, threshold, /*min_samples_fraction=*/0.999);
    EXPECT_EQ(lethal.lethal[0], 1U);
  }

  // 0.75 requires ceil(0.75 * 4) = 3 samples -> should trigger (range is 1.0).
  {
    const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
      grid, size_x, size_y,
      0, 0, 0, 0,
      k, threshold, /*min_samples_fraction=*/0.75);
    EXPECT_EQ(lethal.lethal[0], 1U);
  }

  // 0.0 means "at least 1 sample" -> should also trigger.
  {
    const auto lethal = spatio_temporal_voxel_layer::internal::computeElevationLethalMask(
      grid, size_x, size_y,
      0, 0, 0, 0,
      k, threshold, /*min_samples_fraction=*/0.0);
    EXPECT_EQ(lethal.lethal[0], 1U);
  }
}
