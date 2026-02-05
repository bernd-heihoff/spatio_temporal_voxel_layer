#include "spatio_temporal_voxel_layer/internal/sliding_window_2d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace spatio_temporal_voxel_layer::internal
{

namespace
{
struct DequeItem
{
  int idx;
  float value;
};

template<typename ValueAt>
void sliding_min_1d(ValueAt value_at, int n, int k, float neutral, float * out)
{
  std::deque<DequeItem> dq;
  dq.clear();

  for (int j = -k; j <= (n - 1 + k); ++j) {
    const float v = (j < 0 || j >= n) ? neutral : value_at(j);
    while (!dq.empty() && v <= dq.back().value) {
      dq.pop_back();
    }
    dq.push_back(DequeItem{j, v});

    const int window_start = j - 2 * k;
    while (!dq.empty() && dq.front().idx < window_start) {
      dq.pop_front();
    }

    const int out_idx = j - k;
    if (out_idx >= 0 && out_idx < n) {
      out[out_idx] = dq.front().value;
    }
  }
}

template<typename ValueAt>
void sliding_max_1d(ValueAt value_at, int n, int k, float neutral, float * out)
{
  std::deque<DequeItem> dq;
  dq.clear();

  for (int j = -k; j <= (n - 1 + k); ++j) {
    const float v = (j < 0 || j >= n) ? neutral : value_at(j);
    while (!dq.empty() && v >= dq.back().value) {
      dq.pop_back();
    }
    dq.push_back(DequeItem{j, v});

    const int window_start = j - 2 * k;
    while (!dq.empty() && dq.front().idx < window_start) {
      dq.pop_front();
    }

    const int out_idx = j - k;
    if (out_idx >= 0 && out_idx < n) {
      out[out_idx] = dq.front().value;
    }
  }
}

inline size_t idx2d(int x, int y, int w)
{
  return static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
}

}  // namespace

WindowStatsGrid computeWindowStats(
  const std::vector<float> & elevation,
  int size_x,
  int size_y,
  int eval_start_x,
  int eval_start_y,
  int eval_end_x,
  int eval_end_y,
  int window_half_extent_cells)
{
  WindowStatsGrid result;

  if (size_x <= 0 || size_y <= 0) {
    return result;
  }

  eval_start_x = std::max(0, std::min(size_x - 1, eval_start_x));
  eval_end_x = std::max(0, std::min(size_x - 1, eval_end_x));
  eval_start_y = std::max(0, std::min(size_y - 1, eval_start_y));
  eval_end_y = std::max(0, std::min(size_y - 1, eval_end_y));

  if (eval_start_x > eval_end_x || eval_start_y > eval_end_y) {
    return result;
  }

  const int k = std::max(0, window_half_extent_cells);

  // Support bbox includes full window for every eval cell (except at map edges where it clips).
  const int support_start_x = std::max(0, std::min(size_x - 1, eval_start_x - k));
  const int support_end_x = std::max(0, std::min(size_x - 1, eval_end_x + k));
  const int support_start_y = std::max(0, std::min(size_y - 1, eval_start_y - k));
  const int support_end_y = std::max(0, std::min(size_y - 1, eval_end_y + k));

  const int support_w = support_end_x - support_start_x + 1;
  const int support_h = support_end_y - support_start_y + 1;

  const float pos_inf = std::numeric_limits<float>::infinity();
  const float neg_inf = -std::numeric_limits<float>::infinity();

  std::vector<float> src_val(static_cast<size_t>(support_w) * static_cast<size_t>(support_h),
    std::numeric_limits<float>::quiet_NaN());
  std::vector<uint8_t> src_valid(static_cast<size_t>(support_w) * static_cast<size_t>(support_h), 0U);

  for (int sy = 0; sy < support_h; ++sy) {
    const int my = support_start_y + sy;
    for (int sx = 0; sx < support_w; ++sx) {
      const int mx = support_start_x + sx;
      const size_t map_index = static_cast<size_t>(my) * static_cast<size_t>(size_x) +
        static_cast<size_t>(mx);
      if (map_index >= elevation.size()) {
        continue;
      }

      const float h = elevation[map_index];
      const size_t local = idx2d(sx, sy, support_w);
      src_val[local] = h;
      src_valid[local] = std::isfinite(h) ? 1U : 0U;
    }
  }

  // Integral image of validity mask for O(1) window sample counts.
  std::vector<uint32_t> prefix(
    static_cast<size_t>(support_w + 1) * static_cast<size_t>(support_h + 1), 0U);
  for (int y = 0; y < support_h; ++y) {
    uint32_t row_sum = 0U;
    for (int x = 0; x < support_w; ++x) {
      row_sum += static_cast<uint32_t>(src_valid[idx2d(x, y, support_w)]);
      const size_t p = idx2d(x + 1, y + 1, support_w + 1);
      prefix[p] = prefix[idx2d(x + 1, y, support_w + 1)] + row_sum;
    }
  }

  auto window_sample_count_support = [&](int x0, int y0, int x1, int y1) -> uint32_t {
    // x0..x1, y0..y1 inclusive in support-local coords
    const int xa = std::max(0, std::min(support_w - 1, x0));
    const int xb = std::max(0, std::min(support_w - 1, x1));
    const int ya = std::max(0, std::min(support_h - 1, y0));
    const int yb = std::max(0, std::min(support_h - 1, y1));
    if (xa > xb || ya > yb) {
      return 0U;
    }

    const size_t stride = static_cast<size_t>(support_w + 1);
    const size_t A = static_cast<size_t>(ya) * stride + static_cast<size_t>(xa);
    const size_t B = static_cast<size_t>(ya) * stride + static_cast<size_t>(xb + 1);
    const size_t C = static_cast<size_t>(yb + 1) * stride + static_cast<size_t>(xa);
    const size_t D = static_cast<size_t>(yb + 1) * stride + static_cast<size_t>(xb + 1);
    return prefix[D] - prefix[B] - prefix[C] + prefix[A];
  };

  // Horizontal pass.
  std::vector<float> row_min(static_cast<size_t>(support_w) * static_cast<size_t>(support_h), pos_inf);
  std::vector<float> row_max(static_cast<size_t>(support_w) * static_cast<size_t>(support_h), neg_inf);

  for (int y = 0; y < support_h; ++y) {
    const size_t base = static_cast<size_t>(y) * static_cast<size_t>(support_w);

    auto min_at = [&](int x) -> float {
      const size_t i = base + static_cast<size_t>(x);
      return src_valid[i] ? src_val[i] : pos_inf;
    };
    auto max_at = [&](int x) -> float {
      const size_t i = base + static_cast<size_t>(x);
      return src_valid[i] ? src_val[i] : neg_inf;
    };

    sliding_min_1d(min_at, support_w, k, pos_inf, &row_min[base]);
    sliding_max_1d(max_at, support_w, k, neg_inf, &row_max[base]);
  }

  // Vertical pass.
  std::vector<float> support_min(
    static_cast<size_t>(support_w) * static_cast<size_t>(support_h), pos_inf);
  std::vector<float> support_max(
    static_cast<size_t>(support_w) * static_cast<size_t>(support_h), neg_inf);

  std::vector<float> col_out(static_cast<size_t>(support_h));

  for (int x = 0; x < support_w; ++x) {
    sliding_min_1d(
      [&](int y) {
        return row_min[idx2d(x, y, support_w)];
      },
      support_h, k, pos_inf, col_out.data());
    for (int y = 0; y < support_h; ++y) {
      support_min[idx2d(x, y, support_w)] = col_out[static_cast<size_t>(y)];
    }

    sliding_max_1d(
      [&](int y) {
        return row_max[idx2d(x, y, support_w)];
      },
      support_h, k, neg_inf, col_out.data());
    for (int y = 0; y < support_h; ++y) {
      support_max[idx2d(x, y, support_w)] = col_out[static_cast<size_t>(y)];
    }
  }

  // Emit results for eval bbox.
  result.start_x = eval_start_x;
  result.start_y = eval_start_y;
  result.width = eval_end_x - eval_start_x + 1;
  result.height = eval_end_y - eval_start_y + 1;

  result.min_vals.assign(static_cast<size_t>(result.width) * static_cast<size_t>(result.height), pos_inf);
  result.max_vals.assign(static_cast<size_t>(result.width) * static_cast<size_t>(result.height), neg_inf);
  result.count_vals.assign(static_cast<size_t>(result.width) * static_cast<size_t>(result.height), 0U);

  for (int ey = 0; ey < result.height; ++ey) {
    const int map_y = eval_start_y + ey;
    const int sy = map_y - support_start_y;

    for (int ex = 0; ex < result.width; ++ex) {
      const int map_x = eval_start_x + ex;
      const int sx = map_x - support_start_x;

      const size_t out_i = idx2d(ex, ey, result.width);
      result.min_vals[out_i] = support_min[idx2d(sx, sy, support_w)];
      result.max_vals[out_i] = support_max[idx2d(sx, sy, support_w)];

      const int wx0_map = std::max(0, map_x - k);
      const int wx1_map = std::min(size_x - 1, map_x + k);
      const int wy0_map = std::max(0, map_y - k);
      const int wy1_map = std::min(size_y - 1, map_y + k);

      const int wx0 = wx0_map - support_start_x;
      const int wx1 = wx1_map - support_start_x;
      const int wy0 = wy0_map - support_start_y;
      const int wy1 = wy1_map - support_start_y;

      result.count_vals[out_i] = window_sample_count_support(wx0, wy0, wx1, wy1);
    }
  }

  return result;
}

}  // namespace spatio_temporal_voxel_layer::internal
