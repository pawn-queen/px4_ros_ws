#pragma once
#include <array>

namespace powerplant::mission_contract {
// Keep this header, the Python mission_contract.py and
// config/mission_contract.json in sync.
inline constexpr int kVersion = 1;
inline constexpr const char *kContractVersion = "contract_version";

inline constexpr const char *kStatusStampNs = "status_stamp_ns";
inline constexpr const char *kStatusSequence = "status_sequence";
inline constexpr const char *kLocalizationEpoch = "localization_epoch";
inline constexpr const char *kNavigationSampleNs = "navigation_sample_ns";
inline constexpr const char *kUwbInputHealthy = "uwb_input_healthy";
inline constexpr const char *kHasIndependentPosition = "has_independent_position";
inline constexpr const char *kPositionVarianceEnuM2 = "position_variance_enu_m2";

inline constexpr std::array<const char *, 7> kLocalizationStatusFields{
    kStatusStampNs, kStatusSequence, kLocalizationEpoch,
    kNavigationSampleNs, kUwbInputHealthy, kHasIndependentPosition,
    kPositionVarianceEnuM2};

inline constexpr const char *kStamp = "stamp";
inline constexpr const char *kSec = "sec";
inline constexpr const char *kNanosec = "nanosec";
inline constexpr const char *kFrameId = "frame_id";
inline constexpr const char *kImageWidth = "image_width";
inline constexpr const char *kImageHeight = "image_height";
inline constexpr const char *kDetections = "detections";
inline constexpr const char *kConfidence = "confidence";
inline constexpr const char *kBboxXyxy = "bbox_xyxy";
inline constexpr const char *kClassName = "class_name";

inline constexpr std::array<const char *, 5> kYoloDetectionFields{
    kStamp, kFrameId, kImageWidth, kImageHeight, kDetections};

inline constexpr std::array<const char *, 3> kYoloDetectionItemFields{
    kClassName, kConfidence, kBboxXyxy};
} // namespace powerplant::mission_contract
