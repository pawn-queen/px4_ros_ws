#include "px4_powerplant_mission_cpp/core.hpp"
#include "px4_powerplant_mission_cpp/depth_sampling.hpp"
#include "px4_powerplant_mission_cpp/raycast.hpp"
#include <gtest/gtest.h>
#include <random>
#include <set>

namespace powerplant {
namespace {
using OrderedKeys = std::set<Key>;
OrderedKeys traced(const Vec3 &start, const Vec3 &end, double resolution) {
  OrderedKeys result;
  walk_ray_positive(start, end, resolution,
                    [&](const Key &cell, long double enter, long double exit) {
                      EXPECT_GT(exit, enter);
                      EXPECT_GE(enter, 0);
                      EXPECT_LE(exit, 1);
                      EXPECT_TRUE(result.insert(cell).second);
                    });
  return result;
}
// Independent segment/AABB clipping oracle. Parallel face rays use the
// canonical half-open side; isolated nonparallel contacts have zero length.
OrderedKeys clipped(const Vec3 &start, const Vec3 &end, double resolution) {
  OrderedKeys result;
  if ((start - end).squaredNorm() == 0)
    return result;
  Key low = key(start.cwiseMin(end), resolution);
  Key high = key(start.cwiseMax(end), resolution);
  for (int x = low[0]; x <= high[0]; ++x)
    for (int y = low[1]; y <= high[1]; ++y)
      for (int z = low[2]; z <= high[2]; ++z) {
        const Key candidate{x, y, z};
        long double enter = 0, exit = 1;
        bool possible = true;
        for (int axis = 0; axis < 3; ++axis) {
          const long double a = start[axis];
          const long double d = static_cast<long double>(end[axis]) - a;
          const long double first =
              candidate[axis] * static_cast<long double>(resolution);
          const long double last = first + resolution;
          if (!d) {
            possible = possible && a >= first && a < last;
            continue;
          }
          long double t0 = (first - a) / d, t1 = (last - a) / d;
          if (t1 < t0)
            std::swap(t0, t1);
          enter = std::max(enter, t0);
          exit = std::min(exit, t1);
        }
        if (possible && exit > enter)
          result.insert(candidate);
      }
  return result;
}
} // namespace

TEST(ExactRay, IndependentClippingOracleForBoundariesAndRandomSegments) {
  const std::vector<std::pair<Vec3, Vec3>> boundaries = {
      {{0, 0, 0}, {-3, -3, -3}},
      {{-3, -3, -3}, {0, 0, 0}},
      {{.5, 1, 0}, {3, 1, 0}},
      {{3, 1, 0}, {.5, 1, 0}},
      {{.5, .5, .5}, {2.5, 2.5, 2.5}},
      {{.1, .1, .1}, {4.2, 3.8, .1}},
      {{.5, .5, .5}, {.5, .5, .5}},
      {{.5, .5, .5}, {.50000001, .49999999, .5}}};
  for (const auto &[start, end] : boundaries)
    EXPECT_EQ(traced(start, end, .5), clipped(start, end, .5));
  std::mt19937 generator(45123);
  std::uniform_real_distribution<double> coordinate(-3, 3);
  for (int i = 0; i < 200; ++i) {
    Vec3 start(coordinate(generator), coordinate(generator),
               coordinate(generator));
    Vec3 end(coordinate(generator), coordinate(generator),
             coordinate(generator));
    EXPECT_EQ(traced(start, end, .5), clipped(start, end, .5)) << i;
  }
}
TEST(ExactRay, IsolatedCornerTouchesDoNotBecomeFreeEvidence) {
  auto cells = traced(Vec3(.5, .5, .5), Vec3(2.5, 2.5, 2.5), 1);
  EXPECT_EQ(cells, (OrderedKeys{{0, 0, 0}, {1, 1, 1}, {2, 2, 2}}));
  EXPECT_FALSE(cells.count({1, 0, 0}));
  // This short interval is skipped by the old half-voxel sampling.
  EXPECT_TRUE(traced(Vec3(.1, .1, .1), Vec3(4.2, 3.8, .1), 1).count({1, 0, 0}));
}
TEST(ExactRay, EndpointHitsDominateFreeAndRepeatedRaysUpdateOnce) {
  VoxelGrid grid;
  grid.resolution = 1;
  grid.insert_rays(Path(3, Vec3(.5, .5, .5)),
                   {Vec3(2.5, .5, .5), Vec3(4.5, .5, .5), Vec3(4.5, .5, .5)});
  EXPECT_DOUBLE_EQ(grid.cells.at({2, 0, 0}), grid.hit);
  EXPECT_DOUBLE_EQ(grid.cells.at({3, 0, 0}), grid.miss);
  EXPECT_DOUBLE_EQ(grid.cells.at({4, 0, 0}), grid.hit);
  EXPECT_EQ(grid.revision, 1);
  VoxelGrid boundary;
  boundary.resolution = 1;
  boundary.insert_rays({Vec3(0, .5, .5)}, {Vec3(-2, .5, .5)});
  EXPECT_FALSE(boundary.cells.count({0, 0, 0}));
  EXPECT_DOUBLE_EQ(boundary.cells.at({-1, 0, 0}), boundary.miss);
  EXPECT_DOUBLE_EQ(boundary.cells.at({-2, 0, 0}), boundary.hit);
}
TEST(ExactRay, RejectsInvalidResolutionAndExcessiveTraversalAtomically) {
  VoxelGrid grid;
  grid.resolution = 0;
  EXPECT_THROW(grid.insert_rays({Vec3::Zero()}, {Vec3::Ones()}),
               std::invalid_argument);
  grid.resolution = 1;
  EXPECT_THROW(grid.insert_rays({Vec3::Zero()}, {Vec3(2000000, 0, 0)}),
               std::length_error);
  EXPECT_TRUE(grid.cells.empty());
}
TEST(AdaptiveDepth, DetectsUnsampledThinObstacleAtItsActualPixel) {
  cv::Mat depth(16, 16, CV_32FC1, cv::Scalar(8));
  depth.at<float>(7, 5) = 1;
  DepthSamplingConfig config;
  config.max_stride = 16;
  config.max_rays = 1000;
  auto samples = sample_depth(depth, 100, 100, config);
  auto found = std::find_if(
      samples.pixels.begin(), samples.pixels.end(), [](const auto &pixel) {
        return pixel.x == 5 && pixel.y == 7 && pixel.depth == 1;
      });
  ASSERT_NE(found, samples.pixels.end());
  EXPECT_DOUBLE_EQ(samples.valid_fraction, 1);
  for (const auto &pixel : samples.pixels)
    EXPECT_EQ(pixel.depth, depth.at<float>(pixel.y, pixel.x));
}
TEST(AdaptiveDepth, FarFieldSpacingAddsRaysAndInvalidPixelsRemainUnknown) {
  DepthSamplingConfig config;
  config.max_stride = 16;
  config.max_rays = 10000;
  cv::Mat near(32, 32, CV_32FC1, cv::Scalar(1));
  cv::Mat far(32, 32, CV_32FC1, cv::Scalar(12));
  auto a = sample_depth(near, 100, 100, config);
  auto b = sample_depth(far, 100, 100, config);
  EXPECT_GT(b.pixels.size(), a.pixels.size());
  far.at<float>(1, 2) = NAN;
  far.at<float>(3, 4) = 0;
  b = sample_depth(far, 100, 100, config);
  EXPECT_EQ(b.valid_pixels, 32u * 32 - 2);
  for (const auto &pixel : b.pixels)
    EXPECT_TRUE(std::isfinite(pixel.depth) && pixel.depth > 0);
}
TEST(AdaptiveDepth, BudgetIsBoundedAndDistributedAcrossImage) {
  cv::Mat depth(64, 64, CV_32FC1, cv::Scalar(12));
  DepthSamplingConfig config;
  config.max_rays = 4;
  config.max_stride = 8;
  auto samples = sample_depth(depth, 100, 100, config);
  ASSERT_EQ(samples.pixels.size(), 4u);
  EXPECT_TRUE(samples.budget_limited);
  auto [low, high] = std::minmax_element(
      samples.pixels.begin(), samples.pixels.end(),
      [](const auto &a, const auto &b) { return a.y < b.y; });
  EXPECT_LT(low->y, 16);
  EXPECT_GE(high->y, 48);
}
TEST(AdaptiveDepth, DisabledModeRetainsMeasuredLegacyGridAndValidatesLimits) {
  cv::Mat depth(17, 17, CV_32FC1, cv::Scalar(2));
  DepthSamplingConfig config;
  config.adaptive = false;
  config.max_stride = 8;
  auto samples = sample_depth(depth, 100, 100, config);
  EXPECT_EQ(samples.pixels.size(), 9u);
  for (const auto &pixel : samples.pixels) {
    EXPECT_EQ(pixel.x % 8, 0);
    EXPECT_EQ(pixel.y % 8, 0);
  }
  config.max_rays = 0;
  EXPECT_THROW(sample_depth(depth, 100, 100, config), std::invalid_argument);
}
} // namespace powerplant
