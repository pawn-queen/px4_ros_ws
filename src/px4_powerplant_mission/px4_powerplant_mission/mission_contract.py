"""Shared Python side of the Python/C++ JSON mission contract.

The C++ mirror lives in ``mission_contract.hpp`` and the checked-in schema in
``config/mission_contract.json``. Keep all three in sync.
"""
from __future__ import annotations

from typing import Any, Mapping


CONTRACT_VERSION = 1
LOCALIZATION_STATUS_VERSION = 1
YOLO_DETECTION_VERSION = 1

LOCALIZATION_STATUS_FIELDS = (
    "status_stamp_ns",
    "status_sequence",
    "localization_epoch",
    "navigation_sample_ns",
    "uwb_input_healthy",
    "has_independent_position",
    "position_variance_enu_m2",
)
YOLO_DETECTION_FIELDS = (
    "stamp",
    "frame_id",
    "image_width",
    "image_height",
    "detections",
)
YOLO_DETECTION_ITEM_FIELDS = ("class_name", "confidence", "bbox_xyxy")


def _require_mapping(payload: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(payload, Mapping):
        raise ValueError(f"{name} must be a JSON object")
    return payload


def _check_version(payload: Mapping[str, Any], expected: int, name: str) -> None:
    version = payload.get("contract_version")
    if version is None:
        return
    if not isinstance(version, int) or isinstance(version, bool) or version != expected:
        raise ValueError(f"{name} contract_version must be {expected}, got {version!r}")


def validate_localization_status(payload: Any) -> None:
    """Validate the required localization-status fields.

    ``contract_version`` is optional for old logs and required for newly
    published status messages.
    """
    payload = _require_mapping(payload, "localization status")
    missing = [key for key in LOCALIZATION_STATUS_FIELDS if key not in payload]
    if missing:
        raise ValueError("localization status missing fields: " + ", ".join(missing))
    _check_version(payload, LOCALIZATION_STATUS_VERSION, "localization status")
    if not isinstance(payload["uwb_input_healthy"], bool):
        raise ValueError("localization status uwb_input_healthy must be bool")
    if not isinstance(payload["has_independent_position"], bool):
        raise ValueError("localization status has_independent_position must be bool")
    if not isinstance(payload["position_variance_enu_m2"], (list, tuple)) or len(
        payload["position_variance_enu_m2"]
    ) != 3:
        raise ValueError("localization status position_variance_enu_m2 must have 3 values")


def validate_yolo_detection(payload: Any) -> None:
    payload = _require_mapping(payload, "YOLO detection")
    missing = [key for key in YOLO_DETECTION_FIELDS if key not in payload]
    if missing:
        raise ValueError("YOLO detection missing fields: " + ", ".join(missing))
    _check_version(payload, YOLO_DETECTION_VERSION, "YOLO detection")
    stamp = payload["stamp"]
    if not isinstance(stamp, Mapping) or not {"sec", "nanosec"} <= set(stamp):
        raise ValueError("YOLO detection stamp must contain sec and nanosec")
    detections = payload["detections"]
    if not isinstance(detections, list):
        raise ValueError("YOLO detection detections must be an array")
    for index, detection in enumerate(detections):
        detection = _require_mapping(detection, f"YOLO detection[{index}]")
        missing = [key for key in YOLO_DETECTION_ITEM_FIELDS if key not in detection]
        if missing:
            raise ValueError(
                f"YOLO detection[{index}] missing fields: " + ", ".join(missing)
            )
        bbox = detection["bbox_xyxy"]
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise ValueError(f"YOLO detection[{index}].bbox_xyxy must have 4 values")
