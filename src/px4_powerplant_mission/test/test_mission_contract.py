import json
from pathlib import Path

import pytest

from px4_powerplant_mission.mission_contract import (
    LOCALIZATION_STATUS_FIELDS,
    LOCALIZATION_STATUS_VERSION,
    YOLO_DETECTION_FIELDS,
    YOLO_DETECTION_ITEM_FIELDS,
    YOLO_DETECTION_VERSION,
    validate_localization_status,
    validate_yolo_detection,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/mission_contract.json"


def schema():
    return json.loads(CONFIG.read_text())["contracts"]


def status(**changes):
    payload = {
        "contract_version": LOCALIZATION_STATUS_VERSION,
        "status_stamp_ns": 1,
        "status_sequence": 2,
        "localization_epoch": 3,
        "navigation_sample_ns": 4,
        "uwb_input_healthy": True,
        "has_independent_position": True,
        "position_variance_enu_m2": [0.1, 0.2, 0.3],
    }
    payload.update(changes)
    return payload


def detection(**changes):
    payload = {
        "contract_version": YOLO_DETECTION_VERSION,
        "stamp": {"sec": 1, "nanosec": 2},
        "frame_id": "camera_optical",
        "image_width": 640,
        "image_height": 360,
        "detections": [
            {"class_name": "target", "confidence": 0.9, "bbox_xyxy": [1, 2, 3, 4]}
        ],
    }
    payload.update(changes)
    return payload


def test_schema_and_python_field_lists_match():
    data = schema()
    assert tuple(data["localization_status"]["required"]) == LOCALIZATION_STATUS_FIELDS
    assert tuple(data["yolo_detection"]["required"]) == YOLO_DETECTION_FIELDS
    assert tuple(data["yolo_detection"]["item_required"]) == YOLO_DETECTION_ITEM_FIELDS
    assert data["localization_status"]["version"] == LOCALIZATION_STATUS_VERSION
    assert data["yolo_detection"]["version"] == YOLO_DETECTION_VERSION


def test_valid_status_and_detection_payloads():
    validate_localization_status(status())
    validate_yolo_detection(detection())


def test_old_status_without_version_stays_valid():
    old = status()
    old.pop("contract_version")
    validate_localization_status(old)


def test_missing_status_field_is_rejected():
    payload = status()
    payload.pop("localization_epoch")
    with pytest.raises(ValueError, match="localization_epoch"):
        validate_localization_status(payload)


def test_bad_status_version_is_rejected():
    with pytest.raises(ValueError, match="contract_version"):
        validate_localization_status(status(contract_version=99))


def test_bad_detection_item_is_rejected():
    payload = detection()
    payload["detections"][0].pop("bbox_xyxy")
    with pytest.raises(ValueError, match="bbox_xyxy"):
        validate_yolo_detection(payload)
