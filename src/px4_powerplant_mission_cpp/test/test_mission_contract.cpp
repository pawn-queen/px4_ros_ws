#include "px4_powerplant_mission_cpp/mission_contract.hpp"
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <gtest/gtest.h>

namespace powerplant {
namespace {
TEST(MissionContract, localization_status_fields_are_complete) {
  auto payload = parse(R"({
    "contract_version": 1,
    "status_stamp_ns": 1,
    "status_sequence": 2,
    "localization_epoch": 3,
    "navigation_sample_ns": 4,
    "uwb_input_healthy": true,
    "has_independent_position": true,
    "position_variance_enu_m2": [0.1, 0.2, 0.3]
  })");
  for (const auto *key : mission_contract::kLocalizationStatusFields)
    EXPECT_TRUE(payload.isMember(key)) << key;
  EXPECT_EQ(payload[mission_contract::kContractVersion].asInt(),
            mission_contract::kVersion);
}

TEST(MissionContract, yolo_detection_fields_are_complete) {
  auto payload = parse(R"({
    "contract_version": 1,
    "stamp": {"sec": 1, "nanosec": 2},
    "frame_id": "camera_optical",
    "image_width": 640,
    "image_height": 360,
    "detections": [{"class_name": "target", "confidence": 0.9,
                    "bbox_xyxy": [1, 2, 3, 4]}]
  })");
  for (const auto *key : mission_contract::kYoloDetectionFields)
    EXPECT_TRUE(payload.isMember(key)) << key;
  const auto &item = payload[mission_contract::kDetections][0];
  for (const auto *key : mission_contract::kYoloDetectionItemFields)
    EXPECT_TRUE(item.isMember(key)) << key;
  EXPECT_EQ(payload[mission_contract::kContractVersion].asInt(),
            mission_contract::kVersion);
}
} // namespace
} // namespace powerplant
