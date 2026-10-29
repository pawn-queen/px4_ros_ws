#include "px4_powerplant_mission_cpp/inspection.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

using namespace powerplant;
namespace {
InspectionCamera camera(int width = 80, int height = 60) {
  InspectionCamera c;
  c.width = width;
  c.height = height;
  c.fx = width;
  c.fy = width;
  c.cx = .5 * (width - 1);
  c.cy = .5 * (height - 1);
  c.calibrated = true;
  return c;
}
RegistrationInput registration_input() {
  RegistrationInput in;
  in.epoch = 17;
  in.rgb_stamp_ns = in.depth_stamp_ns = 1000000000;
  in.rgb_pose.stamp = in.rgb_stamp_ns;
  in.depth_pose.stamp = in.depth_stamp_ns;
  in.depth_camera = camera();
  in.rgb_camera = camera(160, 120);
  // A different field of view: a simple 2x resolution scale is incorrect.
  in.rgb_camera.fx = in.rgb_camera.fy = 240;
  in.bbox = {100, 40, 145, 85};
  in.class_name = "gas_cylinders";
  in.confidence = .9;
  in.depth = cv::Mat(60, 80, CV_32FC1, cv::Scalar(NAN));
  for (int y = 20; y <= 35; ++y)
    for (int x = 20; x <= 32; ++x)
      in.depth.at<float>(y, x) = 3;
  // RGB u=100..145 corresponds to depth u approximately46.3..61.3,
  // not to the 50..72.5 interval given by a resolution-only scale.
  for (int y = 22; y <= 37; ++y)
    for (int x = 47; x <= 60; ++x)
      in.depth.at<float>(y, x) = 2;
  return in;
}
RegisteredTargetObservation plane(int sample, double offset_y = 0) {
  RegisteredTargetObservation obs;
  obs.epoch = 17;
  obs.rgb_stamp_ns = obs.depth_stamp_ns = 1000000000LL + sample * 100000000LL;
  obs.class_name = "gas_cylinders";
  obs.confidence = .9;
  obs.camera_origin = Vec3(0, 0, 1.2);
  obs.center = Vec3(3.05, offset_y, 1.2);
  obs.valid_depth_fraction = .9;
  for (int y = 0; y < 17; ++y)
    for (int z = 0; z < 17; ++z)
      obs.points.emplace_back(3.05, -.39 + y * .05 + offset_y, .81 + z * .05);
  return obs;
}
InspectionSession confirmed_session() {
  InspectionSession session;
  session.reset(17, 99);
  EXPECT_TRUE(session.observe(plane(1)));
  EXPECT_TRUE(session.observe(plane(2)));
  EXPECT_TRUE(session.observe(plane(3)));
  EXPECT_TRUE(session.confirmed());
  return session;
}
std::vector<uint64_t> stable_ids(const InspectionSession &session) {
  std::vector<uint64_t> ids;
  for (const auto &patch : session.patches())
    if (patch.stable)
      ids.push_back(patch.id);
  return ids;
}
PhotoQualityInput quality_input() {
  PhotoQualityInput in;
  in.rgb = cv::Mat(160, 160, CV_8UC3);
  for (int y = 0; y < in.rgb.rows; ++y)
    for (int x = 0; x < in.rgb.cols; ++x) {
      const uint8_t value = ((x / 4 + y / 4) % 2) ? 70 : 180;
      in.rgb.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
    }
  in.bbox = {10, 10, 150, 150};
  in.target_associated = in.settled = true;
  in.valid_depth_fraction = .9;
  in.speed_mps = .05;
  in.yaw_rate_radps = .01;
  return in;
}
} // namespace

TEST(TaskRoiInspection, RejectsExteriorSurfaceEvenWhenCentroidClaimsInside) {
  InspectionConfig config;
  config.task_roi = TaskRoi{Vec3(3, -.2, .8), Vec3(3.1, .2, 1.7)};
  InspectionSession session(config);
  session.reset(17, 99);
  auto observation = plane(1, 5);
  observation.center = Vec3(3.05, 0, 1.2);
  EXPECT_FALSE(session.observation_in_task_roi(observation));
  EXPECT_FALSE(session.observe(observation));
  EXPECT_FALSE(session.confirmed());
  EXPECT_EQ(session.latest_sample_ns(), 0);
  EXPECT_TRUE(session.patches().empty());
}

TEST(TaskRoiInspection, SurfaceIntersectionAdmitsTargetWithExteriorCentroid) {
  InspectionConfig config;
  config.task_roi = TaskRoi{Vec3(3, .02, .8), Vec3(3.1, .20, 1.7)};
  InspectionSession session(config);
  session.reset(17, 99);
  for (int sample : {1, 2, 3}) {
    const auto observation = plane(sample, .4);
    ASSERT_FALSE(config.task_roi->contains(observation.center));
    ASSERT_TRUE(session.observation_in_task_roi(observation));
    ASSERT_TRUE(session.observe(observation));
  }
  EXPECT_TRUE(session.confirmed());
  EXPECT_GT(session.coverage_report().modeled_area_m2, 0);
  EXPECT_LT(session.coverage_report().modeled_area_m2,
            confirmed_session().coverage_report().modeled_area_m2);
}

TEST(TaskRoiInspection, ClipsBoundaryPatchAreaAndKeepsTaskDenominator) {
  InspectionConfig config;
  // A 5 cm wide strip cuts the nominal 10 cm surfel squares in half.
  config.task_roi = TaskRoi{Vec3(3, .02, .81), Vec3(3.1, .07, 1.61)};
  InspectionSession session(config);
  session.reset(17, 99);
  for (int sample : {1, 2, 3}) ASSERT_TRUE(session.observe(plane(sample)));
  ASSERT_TRUE(session.confirmed());
  const auto ids = stable_ids(session);
  ASSERT_FALSE(ids.empty());
  for (const auto &patch : session.patches())
    EXPECT_LE(patch.area_m2, .005 + 1e-10);
  session.admit_required(ids);
  const auto required = session.coverage_report().required_area_m2;
  EXPECT_GT(required, 0);
  const auto token = session.photo_token(plane(3).rgb_stamp_ns, ids);
  const auto committed = session.commit_photo(token, true, true);
  ASSERT_TRUE(committed.accepted);
  EXPECT_NEAR(committed.new_area_m2, required, 1e-10);
  EXPECT_TRUE(session.coverage_report().complete);
  EXPECT_FALSE(session.observe(plane(4, 5)));
  session.classify_views({});
  EXPECT_DOUBLE_EQ(session.coverage_report().required_area_m2, required);
  EXPECT_DOUBLE_EQ(session.coverage_report().covered_area_m2, required);
}

TEST(TaskRoiInspection, RejectsInvalidTaskGeometryAtSessionConstruction) {
  InspectionConfig config;
  config.task_roi = TaskRoi{Vec3::Zero(), Vec3(0, 1, 1)};
  EXPECT_THROW(InspectionSession session(config), std::invalid_argument);
}

TEST(P2Inspection, RegistersDifferentCameraFieldsOfViewUsingIntrinsics) {
  auto in = registration_input();
  std::string reason;
  auto registered = register_target_observation(in, {}, &reason);
  ASSERT_TRUE(registered) << reason;
  EXPECT_EQ(registered->rgb_stamp_ns, in.rgb_stamp_ns);
  EXPECT_NEAR(registered->center.x(), 2, 1e-9);
  EXPECT_LT(registered->center.y(), 0);
  EXPECT_FALSE(registered->points.empty());
  for (const auto &p : registered->points) {
    const double u = in.rgb_camera.cx - in.rgb_camera.fx * p.y() / p.x();
    EXPECT_GE(u, in.bbox[0]);
    EXPECT_LT(u, in.bbox[2]);
  }
}

TEST(P2Inspection, RejectsMismatchedExposurePosesCalibrationAndAssociation) {
  auto in = registration_input();
  std::string reason;
  in.rgb_pose.stamp -= 1;
  EXPECT_FALSE(register_target_observation(in, {}, &reason));
  EXPECT_EQ(reason, "UNPAIRED_EXPOSURES_OR_POSES");
  in.rgb_pose.stamp = in.rgb_stamp_ns;
  in.rgb_camera.calibrated = false;
  EXPECT_FALSE(register_target_observation(in));
  in.rgb_camera.calibrated = true;
  in.predicted_center = Vec3(-100, 0, 0);
  EXPECT_FALSE(register_target_observation(in, {}, &reason));
  EXPECT_EQ(reason, "NO_ASSOCIATED_DEPTH_COMPONENT");
  in.predicted_center.reset();
  in.depth_stamp_ns -= 100000000;
  in.depth_pose.stamp = in.depth_stamp_ns;
  EXPECT_FALSE(register_target_observation(in));
}

TEST(P2Inspection, RegistrationUsesExposurePoseRatherThanLatestPose) {
  auto in = registration_input();
  const auto original = register_target_observation(in);
  ASSERT_TRUE(original);
  in.depth_pose.position = in.rgb_pose.position = Vec3(5, -2, 1);
  in.depth_pose.orientation = in.rgb_pose.orientation =
      Quat(Eigen::AngleAxisd(.7, Vec3::UnitZ()));
  const auto moved = register_target_observation(in);
  ASSERT_TRUE(moved);
  EXPECT_LT((moved->center - (in.rgb_pose.position +
                              in.rgb_pose.orientation * original->center))
                .norm(),
            1e-8);
}

TEST(P2Inspection, DepthComponentsExcludeBackgroundUsingLockedAssociation) {
  auto in = registration_input();
  in.depth.setTo(cv::Scalar(4));
  for (int y = 22; y <= 37; ++y)
    for (int x = 47; x <= 60; ++x)
      in.depth.at<float>(y, x) = 2;
  in.bbox = {60, 20, 150, 105};
  in.predicted_center = Vec3(2, -.35, 0);
  const auto observed = register_target_observation(in);
  ASSERT_TRUE(observed);
  EXPECT_NEAR(observed->center.x(), 2, 1e-9);
  for (const auto &point : observed->points)
    EXPECT_NEAR(point.x(), 2, 1e-9);
}

TEST(P2Inspection, RequiresIndependentConfirmationAndKeepsLockedIdentity) {
  InspectionSession session;
  session.reset(17, 99);
  EXPECT_TRUE(session.observe(plane(1)));
  EXPECT_FALSE(session.confirmed());
  EXPECT_FALSE(session.observe(plane(1)));
  EXPECT_TRUE(session.observe(plane(2)));
  EXPECT_FALSE(session.confirmed());
  auto different = plane(3, 2);
  EXPECT_FALSE(session.observe(different));
  EXPECT_TRUE(session.observe(plane(3)));
  EXPECT_TRUE(session.confirmed());
  auto epoch = plane(4);
  epoch.epoch = 18;
  EXPECT_FALSE(session.observe(epoch));
  EXPECT_GT(session.coverage_report().modeled_patches, 0u);
}

TEST(P2Inspection, ExpiredUnconfirmedBackgroundCanRecoverWithoutOldSurfelEvidence) {
  InspectionSession session;
  session.reset(17, 99);
  ASSERT_TRUE(session.observe(plane(1, 2)));
  ASSERT_TRUE(session.observe(plane(2, 2)));
  ASSERT_FALSE(session.confirmed());
  ASSERT_GT(session.coverage_report().modeled_patches, 0u);
  EXPECT_FALSE(session.observe(plane(3))); // Still inside the hypothesis window.

  // More than two seconds after the first mistaken observation, a real target
  // beyond the association radius must get three independent confirmations.
  ASSERT_TRUE(session.observe(plane(22)));
  EXPECT_FALSE(session.confirmed());
  EXPECT_EQ(session.target_id(), 99u);
  EXPECT_EQ(session.epoch(), 17);
  EXPECT_EQ(session.coverage_report().modeled_patches, 0u);
  for (const auto &patch : session.patches())
    EXPECT_LT(std::abs(patch.position.y()), .5); // Expired background is gone.
  ASSERT_TRUE(session.observe(plane(23)));
  EXPECT_FALSE(session.confirmed());
  ASSERT_TRUE(session.observe(plane(24)));
  EXPECT_TRUE(session.confirmed());
  EXPECT_NEAR(session.center().y(), 0, 1e-9);

  // Expiry must never replace an already confirmed object's identity.
  EXPECT_FALSE(session.observe(plane(50, 2)));
  EXPECT_TRUE(session.confirmed());
  EXPECT_NEAR(session.center().y(), 0, 1e-9);
}

TEST(P2Inspection,
     CoverageRequiresQualifiedSavedActualExposureAndRejectsReplay) {
  auto session = confirmed_session();
  const auto ids = session.observed_patch_ids(plane(3));
  ASSERT_FALSE(ids.empty());
  session.admit_required(ids);
  const auto token = session.photo_token(plane(3).rgb_stamp_ns, ids);
  EXPECT_FALSE(session.commit_photo(token, false, true).accepted);
  EXPECT_FALSE(session.commit_photo(token, true, false).accepted);
  EXPECT_DOUBLE_EQ(session.coverage_report().covered_area_m2, 0);
  const auto committed = session.commit_photo(token, true, true);
  EXPECT_TRUE(committed.accepted);
  EXPECT_GT(committed.new_area_m2, 0);
  EXPECT_TRUE(session.coverage_report().complete);
  EXPECT_FALSE(session.commit_photo(token, true, true).accepted);
  const auto invented = session.photo_token(plane(5).rgb_stamp_ns, ids);
  EXPECT_FALSE(session.commit_photo(invented, true, true).accepted);
}

TEST(P2Inspection, CoverageCommitBindsEveryPatchToItsActualExposureEvidence) {
  auto session = confirmed_session();
  const auto ids = session.observed_patch_ids(plane(3));
  ASSERT_FALSE(ids.empty());
  session.admit_required(ids);
  const auto legitimate = session.photo_token(plane(3).rgb_stamp_ns, ids);
  auto forged = legitimate;
  forged.sample_ns = plane(9).rgb_stamp_ns; // No such observed exposure.
  EXPECT_FALSE(session.commit_photo(forged, true, true).accepted);
  auto duplicate = legitimate;
  duplicate.patches.emplace_back(legitimate.patches.front().first,
                                 legitimate.patches.front().second + 1);
  EXPECT_FALSE(session.commit_photo(duplicate, true, true).accepted);
  EXPECT_DOUBLE_EQ(session.coverage_report().covered_area_m2, 0);

  ASSERT_TRUE(session.observe(plane(4, .3)));
  ASSERT_TRUE(session.observe(plane(5, .3)));
  const auto expanded_ids = session.observed_patch_ids(plane(5, .3));
  const std::set<uint64_t> old_ids(ids.begin(), ids.end());
  ASSERT_TRUE(std::any_of(expanded_ids.begin(), expanded_ids.end(),
                          [&](uint64_t id) { return !old_ids.count(id); }));
  const auto expanded = session.photo_token(plane(5).rgb_stamp_ns, expanded_ids);
  forged = expanded;
  forged.sample_ns = plane(3).rgb_stamp_ns; // Real exposure, wrong patch bundle.
  EXPECT_FALSE(session.commit_photo(forged, true, true).accepted);
  EXPECT_DOUBLE_EQ(session.coverage_report().covered_area_m2, 0);
  EXPECT_TRUE(session.commit_photo(expanded, true, true).accepted);
}

TEST(P2Inspection, ViewRankingUsesExecutableGoReturnTimesAndNeverIgnoresRejection) {
  const auto session = confirmed_session();
  PlanningMap map;
  map.resolution = .5;
  map.bounds = Bounds{Key{-2, -8, -2}, Key{14, 8, 8}};
  for (int x = -2; x <= 14; ++x)
    for (int y = -8; y <= 8; ++y)
      for (int z = -2; z <= 8; ++z)
        map.free.insert({x, y, z});
  ExplorationConfig config;
  config.clearance_radius_m = 0;
  config.max_search_radius_m = 20;
  const Vec3 current(0, 0, 1.2);
  const auto image_geometry = camera(160, 120);
  const auto baseline = session.candidate_views(map, current, current, config,
                                               image_geometry);
  ASSERT_GT(baseline.size(), 1u);
  const auto path_length = [](const Path &path) {
    double length = 0;
    for (size_t i = 1; i < path.size(); ++i)
      length += (path[i] - path[i - 1]).norm();
    return length;
  };
  const double first_distance = path_length(baseline.front().path) +
                                path_length(baseline.front().return_path);
  const auto preferred = std::find_if(
      baseline.begin() + 1, baseline.end(), [&](const InspectionView &view) {
        return view.new_area_m2 > 0 &&
               path_length(view.path) + path_length(view.return_path) >
                   first_distance + 1e-6;
      });
  ASSERT_NE(preferred, baseline.end());
  const int expected_id = preferred->view_id;
  const Vec3 expected_position = preferred->position;
  bool checked_go = false, checked_return = false;
  config.path_time_s = [&](const Path &path, double, double,
                            bool returning) -> std::optional<double> {
    if (returning) {
      checked_return = true;
      return 0;
    }
    checked_go = true;
    return (path.back() - expected_position).norm() < 1e-9 ? 0 : 10000;
  };
  const auto ranked = session.candidate_views(map, current, current, config,
                                             image_geometry);
  ASSERT_FALSE(ranked.empty());
  EXPECT_TRUE(checked_go);
  EXPECT_TRUE(checked_return);
  EXPECT_EQ(ranked.front().view_id, expected_id);
  config.path_time_s = [](const Path &, double, double,
                          bool) -> std::optional<double> { return {}; };
  EXPECT_TRUE(session.candidate_views(map, current, current, config,
                                      image_geometry).empty());
  config.path_time_s = [](const Path &, double, double,
                          bool) -> std::optional<double> { return NAN; };
  EXPECT_TRUE(session.candidate_views(map, current, current, config,
                                      image_geometry).empty());
  config.path_time_s = [](const Path &, double, double,
                          bool) -> std::optional<double> { return -1; };
  EXPECT_TRUE(session.candidate_views(map, current, current, config,
                                      image_geometry).empty());
}

TEST(P2Inspection,
     NewStableGeometryGrowsDenominatorAndOcclusionNeverShrinksIt) {
  auto session = confirmed_session();
  auto initial = stable_ids(session);
  session.admit_required(initial);
  ASSERT_TRUE(
      session
          .commit_photo(session.photo_token(plane(3).rgb_stamp_ns, initial),
                        true, true)
          .accepted);
  const double before = session.coverage_report().required_area_m2;
  EXPECT_TRUE(session.observe(plane(4, .3)));
  EXPECT_TRUE(session.observe(plane(5, .3)));
  session.admit_required(stable_ids(session));
  auto report = session.coverage_report();
  EXPECT_GT(report.required_area_m2, before);
  EXPECT_LT(report.fraction, 1);
  session.classify_views({});
  auto blocked = session.coverage_report();
  EXPECT_DOUBLE_EQ(blocked.required_area_m2, report.required_area_m2);
  EXPECT_GT(blocked.unreachable_area_m2, 0);
  EXPECT_FALSE(blocked.complete);
  EXPECT_TRUE(blocked.unknown_extent);
}

TEST(P2Inspection, ObsoletePhotoTargetEpochOrGeometryCannotCommit) {
  auto session = confirmed_session();
  const auto ids = stable_ids(session);
  session.admit_required(ids);
  auto token = session.photo_token(plane(3).rgb_stamp_ns, ids);
  auto wrong = token;
  wrong.target_id += 1;
  EXPECT_FALSE(session.commit_photo(wrong, true, true).accepted);
  wrong = token;
  wrong.epoch += 1;
  EXPECT_FALSE(session.commit_photo(wrong, true, true).accepted);
  wrong = token;
  ASSERT_FALSE(wrong.patches.empty());
  wrong.patches[0].second += 1;
  EXPECT_FALSE(session.commit_photo(wrong, true, true).accepted);
  EXPECT_DOUBLE_EQ(session.coverage_report().covered_area_m2, 0);
  session.reset(18, 100);
  EXPECT_FALSE(session.commit_photo(token, true, true).accepted);
  EXPECT_FALSE(session.coverage_report().complete);
}

TEST(P2Inspection, RetryBudgetIsBoundedAndConfigurationRejectsWeakening) {
  InspectionSession session;
  EXPECT_TRUE(session.note_attempt(7));
  EXPECT_TRUE(session.note_attempt(7));
  EXPECT_TRUE(session.note_attempt(7));
  EXPECT_FALSE(session.view_available(7));
  EXPECT_FALSE(session.note_attempt(7));
  auto config = InspectionConfig{};
  config.patch_support_frames = 1;
  EXPECT_THROW(InspectionSession{config}, std::invalid_argument);
  config = InspectionConfig{};
  config.coverage_fraction = 0;
  EXPECT_THROW(InspectionSession{config}, std::invalid_argument);
}

TEST(P2Inspection,
     ModelRevisionChangesForStableGeometryButNotRepeatedEvidence) {
  InspectionSession session;
  session.reset(17, 99);
  const auto empty_revision = session.model_revision();
  ASSERT_TRUE(session.observe(plane(1)));
  EXPECT_EQ(session.model_revision(), empty_revision);
  ASSERT_TRUE(session.observe(plane(2)));
  const auto stable_revision = session.model_revision();
  EXPECT_GT(stable_revision, empty_revision);
  ASSERT_TRUE(session.observe(plane(3)));
  EXPECT_EQ(session.model_revision(), stable_revision);
  const auto ids = stable_ids(session);
  EXPECT_GT(session.uncovered_area(ids), 0);
  ASSERT_TRUE(session
                  .commit_photo(session.photo_token(plane(3).rgb_stamp_ns, ids),
                                true, true)
                  .accepted);
  EXPECT_DOUBLE_EQ(session.uncovered_area(ids), 0);
  ASSERT_TRUE(session.observe(plane(4, .3)));
  ASSERT_TRUE(session.observe(plane(5, .3)));
  EXPECT_GT(session.model_revision(), stable_revision);
  const auto expanded_revision = session.model_revision();
  session.reset(18, 100);
  EXPECT_GT(session.model_revision(), expanded_revision);
  EXPECT_FALSE(session.coverage_report().complete);
}

TEST(P2Inspection,
     PhotoQualityRejectsBlurExposureClippingMotionAndMissingTarget) {
  auto in = quality_input();
  EXPECT_TRUE(assess_photo_quality(in).accepted);
  auto blurred = in;
  cv::GaussianBlur(in.rgb, blurred.rgb, cv::Size(31, 31), 8);
  EXPECT_FALSE(assess_photo_quality(blurred).accepted);
  in = quality_input();
  in.rgb.setTo(cv::Scalar(0, 0, 0));
  auto black = assess_photo_quality(in);
  EXPECT_NE(
      std::find(black.reasons.begin(), black.reasons.end(), "UNDEREXPOSED"),
      black.reasons.end());
  in.rgb.setTo(cv::Scalar(255, 255, 255));
  auto white = assess_photo_quality(in);
  EXPECT_NE(
      std::find(white.reasons.begin(), white.reasons.end(), "OVEREXPOSED"),
      white.reasons.end());
  in = quality_input();
  in.bbox[0] = 0;
  EXPECT_FALSE(assess_photo_quality(in).accepted);
  in = quality_input();
  in.speed_mps = 2;
  EXPECT_FALSE(assess_photo_quality(in).accepted);
  in = quality_input();
  in.target_associated = false;
  EXPECT_FALSE(assess_photo_quality(in).accepted);
}
