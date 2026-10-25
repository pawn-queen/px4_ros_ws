#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <vector>

namespace powerplant {
struct DepthPixel {
  int x = 0, y = 0;
  float depth = 0;
};
struct DepthSamplingConfig {
  bool adaptive = true;
  int min_stride = 1, max_stride = 16, max_rays = 12000;
  double min_range = .25, max_range = 18, ray_spacing = .2,
         edge_threshold = .25;
};
struct DepthSamplingResult {
  std::vector<DepthPixel> pixels;
  size_t valid_pixels = 0, candidate_rays = 0;
  double valid_fraction = 0;
  bool budget_limited = false;
};
// Organized-depth sampling. Every input pixel is inspected for range/edges;
// only selected measured pixels are projected/raycast. Leaves contain their
// actual nearest-depth pixel, never a synthesized tile-centre measurement.
inline DepthSamplingResult sample_depth(const cv::Mat &depth, double fx,
                                        double fy,
                                        const DepthSamplingConfig &config) {
  if (depth.type() != CV_32FC1 || depth.empty() || !std::isfinite(fx) ||
      !std::isfinite(fy) || fx <= 0 || fy <= 0 || config.min_stride < 1 ||
      config.max_stride < config.min_stride || config.max_rays < 1 ||
      !std::isfinite(config.ray_spacing) || config.ray_spacing <= 0 ||
      !std::isfinite(config.edge_threshold) || config.edge_threshold < 0 ||
      !std::isfinite(config.min_range) || !std::isfinite(config.max_range) ||
      config.min_range <= 0 || config.max_range <= config.min_range)
    throw std::invalid_argument("invalid adaptive depth input/configuration");
  DepthSamplingResult result;
  auto valid = [&](float z) {
    return std::isfinite(z) && z >= config.min_range && z <= config.max_range;
  };
  for (int y = 0; y < depth.rows; ++y)
    for (int x = 0; x < depth.cols; ++x)
      result.valid_pixels += valid(depth.at<float>(y, x));
  result.valid_fraction =
      double(result.valid_pixels) / (double(depth.rows) * depth.cols);

  std::vector<std::vector<DepthPixel>> tiles;
  for (int top = 0; top < depth.rows; top += config.max_stride)
    for (int left = 0; left < depth.cols; left += config.max_stride) {
      std::vector<DepthPixel> leaves;
      std::function<void(int, int, int, int)> split = [&](int x0, int y0,
                                                          int x1, int y1) {
        float near = INFINITY, far = -INFINITY;
        DepthPixel nearest;
        size_t count = 0;
        for (int y = y0; y < y1; ++y)
          for (int x = x0; x < x1; ++x) {
            const float z = depth.at<float>(y, x);
            if (!valid(z))
              continue;
            ++count;
            if (z < near) {
              near = z;
              nearest = {x, y, z};
            }
            far = std::max(far, z);
          }
        if (!count)
          return;
        const int width = x1 - x0, height = y1 - y0;
        const bool can_x = width > config.min_stride;
        const bool can_y = height > config.min_stride;
        const bool edge = far - near > config.edge_threshold ||
                          count != size_t(width) * height;
        const bool too_wide =
            far * std::max(width / fx, height / fy) > config.ray_spacing;
        if ((can_x || can_y) && (edge || too_wide)) {
          const int middle_x = can_x ? x0 + width / 2 : x1;
          const int middle_y = can_y ? y0 + height / 2 : y1;
          split(x0, y0, middle_x, middle_y);
          if (middle_x < x1)
            split(middle_x, y0, x1, middle_y);
          if (middle_y < y1)
            split(x0, middle_y, middle_x, y1);
          if (middle_x < x1 && middle_y < y1)
            split(middle_x, middle_y, x1, y1);
        } else
          leaves.push_back(nearest);
      };
      if (config.adaptive)
        split(left, top, std::min(left + config.max_stride, depth.cols),
              std::min(top + config.max_stride, depth.rows));
      else {
        const float z = depth.at<float>(top, left);
        if (valid(z))
          leaves.push_back({left, top, z});
      }
      if (!leaves.empty()) {
        // Preserve the closest observed obstacle from every coarse tile before
        // spending extra rays on fine detail. Equal depths use stable pixels.
        std::sort(leaves.begin(), leaves.end(),
                  [](const auto &a, const auto &b) {
                    if (a.depth != b.depth)
                      return a.depth < b.depth;
                    return std::pair(a.y, a.x) < std::pair(b.y, b.x);
                  });
        result.candidate_rays += leaves.size();
        tiles.push_back(std::move(leaves));
      }
    }
  result.budget_limited = result.candidate_rays > size_t(config.max_rays);
  result.pixels.reserve(
      std::min(result.candidate_rays, size_t(config.max_rays)));
  // Round-robin coarse tiles avoids consuming the entire budget in the first
  // image corner. If even one ray/tile exceeds budget, subsample tiles evenly.
  for (size_t round = 0; result.pixels.size() < size_t(config.max_rays);
       ++round) {
    std::vector<size_t> available;
    for (size_t index = 0; index < tiles.size(); ++index)
      if (round < tiles[index].size())
        available.push_back(index);
    if (available.empty())
      break;
    const size_t count = std::min(available.size(), size_t(config.max_rays) -
                                                        result.pixels.size());
    for (size_t i = 0; i < count; ++i) {
      const size_t bucket = count == 1
                                ? available.size() / 2
                                : i * (available.size() - 1) / (count - 1);
      result.pixels.push_back(tiles[available[bucket]][round]);
    }
  }
  return result;
}
} // namespace powerplant
