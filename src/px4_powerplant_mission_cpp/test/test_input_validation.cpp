#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace powerplant {
TEST(InputTime, RejectsMissingStaleFutureAndMismatchedDomain) {
  EXPECT_FALSE(stamp_reason(0, 1000000000, .5, .1).empty());
  EXPECT_FALSE(stamp_reason(1000000000, 0, .5, .1).empty());
  EXPECT_FALSE(stamp_reason(1000000000, 1600000000, .5, .1).empty());
  EXPECT_FALSE(stamp_reason(1200000000, 1000000000, .5, .1).empty());
  EXPECT_FALSE(stamp_reason(1700000000000000000, 1000000000, .5, .1).empty());
  EXPECT_TRUE(stamp_reason(1000000000, 1400000000, .5, .1).empty());
  EXPECT_TRUE(stamp_reason(1050000000, 1000000000, .5, .1).empty());
}
TEST(InputTime, InvalidLimitsCannotDisableSafety) {
  EXPECT_FALSE(stamp_reason(1, 1, -1, .1).empty());
  EXPECT_FALSE(stamp_reason(1, 1, .5, -.1).empty());
  EXPECT_FALSE(stamp_reason(1, 1, std::numeric_limits<double>::quiet_NaN(), .1).empty());
}
std::array<double, 36> covariance() {
  std::array<double, 36> result{};
  for (int axis = 0; axis < 6; ++axis)
    result[axis * 7] = .1;
  return result;
}
TEST(InputCovariance, ChecksPositionAndEveryOrientationAxis) {
  auto value = covariance();
  EXPECT_TRUE(pose_covariance_reason(value, 1, .5).empty());
  for (int axis = 0; axis < 6; ++axis) {
    auto bad = value;
    bad[axis * 7] = axis < 3 ? 2.0 : 1.0;
    EXPECT_FALSE(pose_covariance_reason(bad, 1, .5).empty()) << axis;
    bad[axis * 7] = 0;
    EXPECT_FALSE(pose_covariance_reason(bad, 1, .5).empty()) << axis;
  }
}
TEST(InputCovariance, RejectsNonFiniteAsymmetricAndImpossibleCrossCovariance) {
  auto value = covariance();
  value[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(pose_covariance_reason(value, 1, .5).empty());
  value = covariance();
  value[1] = .01;
  EXPECT_FALSE(pose_covariance_reason(value, 1, .5).empty());
  value[6] = .01;
  EXPECT_TRUE(pose_covariance_reason(value, 1, .5).empty());
  value[1] = value[6] = .2;
  EXPECT_FALSE(pose_covariance_reason(value, 1, .5).empty());
}
} // namespace powerplant
