"""YOLO object detection node using packaged model assets."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from px4_powerplant_mission.common.qos import reliable_qos_profile, sensor_qos_profile


class YoloDetectorNode(Node):
    """Run YOLO inference and publish JSON detections plus an annotated image."""

    package_name = "px4_powerplant_mission"

    def __init__(self) -> None:
        super().__init__("powerplant_yolo_detector")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.model = None
        self.model_names: dict[int, str] = {}
        self._load_model()

        self.detection_pub = self.create_publisher(
            String,
            self.get_parameter("detections_topic").value,
            reliable_qos_profile(),
        )
        self.annotated_pub = self.create_publisher(
            Image,
            self.get_parameter("annotated_image_topic").value,
            sensor_qos_profile(),
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._image_callback,
            sensor_qos_profile(),
        )
        self.get_logger().info("YOLO detector node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera")
        self.declare_parameter("detections_topic", "/powerplant/perception/yolo_detections")
        self.declare_parameter("annotated_image_topic", "/powerplant/perception/yolo_annotated")
        self.declare_parameter("model_path", "models/yolo/best.pt")
        self.declare_parameter("fallback_model_path", "models/yolo/yolov8n.pt")
        self.declare_parameter("ultralytics_repo", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("device", "cpu")

    def _load_model(self) -> None:
        repo_value = str(self.get_parameter("ultralytics_repo").value)
        repo = self._resolve_path(repo_value) if repo_value else None
        if repo and repo.exists():
            sys.path.insert(0, str(repo))

        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "ultralytics is not importable in "
                f"{sys.executable}. Install torch/ultralytics in this Python "
                "environment, pass yolo_python:=/path/to/python, or set "
                f"ultralytics_repo to a local checkout: {exc}"
            )
            return

        model_path = self._resolve_path(str(self.get_parameter("model_path").value))
        if not model_path.exists():
            model_path = self._resolve_path(str(self.get_parameter("fallback_model_path").value))
        if not model_path.exists():
            self.get_logger().error(f"YOLO model not found: {model_path}")
            return

        try:
            self.model = YOLO(str(model_path))
            names = getattr(self.model, "names", {})
            self.model_names = {int(k): str(v) for k, v in dict(names).items()}
            self.get_logger().info(f"loaded YOLO model: {model_path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to load YOLO model {model_path}: {exc}")

    def _resolve_path(self, value: str) -> Path:
        path = Path(value).expanduser()
        if path.is_absolute():
            return path

        roots = self._asset_search_roots()
        for base in roots:
            candidate = base / path
            if candidate.exists():
                return candidate
        return roots[0] / path

    def _asset_search_roots(self) -> list[Path]:
        roots: list[Path] = []
        try:
            roots.append(Path(get_package_share_directory(self.package_name)))
        except PackageNotFoundError:
            pass
        roots.append(Path(__file__).resolve().parents[2])
        roots.append(Path.cwd())
        return roots

    def _image_callback(self, msg: Image) -> None:
        if self.model is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        try:
            results = self.model.predict(
                source=image,
                conf=float(self.get_parameter("confidence_threshold").value),
                iou=float(self.get_parameter("iou_threshold").value),
                imgsz=int(self.get_parameter("image_size").value),
                device=str(self.get_parameter("device").value),
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return

        detections = self._result_to_json(results[0], msg)
        self.detection_pub.publish(String(data=json.dumps(detections, ensure_ascii=False)))

        if bool(self.get_parameter("publish_annotated").value):
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(np.asarray(annotated), encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

    def _result_to_json(self, result: Any, msg: Image) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": msg.header.frame_id,
            "image_width": int(msg.width),
            "image_height": int(msg.height),
            "detections": [],
        }
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return payload

        xyxy = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else []
        conf = boxes.conf.cpu().numpy() if boxes.conf is not None else []
        cls = boxes.cls.cpu().numpy() if boxes.cls is not None else []
        for index, box in enumerate(xyxy):
            class_id = int(cls[index]) if index < len(cls) else -1
            payload["detections"].append(
                {
                    "class_id": class_id,
                    "class_name": self.model_names.get(class_id, str(class_id)),
                    "confidence": round(float(conf[index]), 4) if index < len(conf) else None,
                    "bbox_xyxy": [round(float(v), 2) for v in box.tolist()],
                }
            )
        return payload


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
