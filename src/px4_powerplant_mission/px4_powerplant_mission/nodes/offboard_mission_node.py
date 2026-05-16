"""PX4 offboard mission controller with YOLO search and inspection workflow."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from px4_powerplant_mission.common.frames import (
    enu_to_ned,
    quaternion_xyzw_to_yaw,
    rotate_vector_xyzw,
    wrap_pi,
    yaw_enu_to_ned,
)
from px4_powerplant_mission.common.qos import (
    px4_qos_profile,
    reliable_qos_profile,
    sensor_qos_profile,
)
from px4_powerplant_mission.path_planning.grid_astar import GridInfo, plan_astar


PHASE_IDLE = "IDLE"
PHASE_START = "START"
PHASE_TAKING_OFF = "TAKING_OFF"
PHASE_GLOBAL_SEARCH = "GLOBAL_SEARCH"
PHASE_TARGETING_CYCLE = "TARGETING_CYCLE"
PHASE_RETURN_HOME = "RETURN_HOME"
PHASE_LANDING = "LANDING"
PHASE_COMPLETE = "COMPLETE"


@dataclass
class TargetObservation:
    """A YOLO detection projected into the ENU map frame."""

    class_name: str
    confidence: float
    bbox_xyxy: list[float]
    position_enu: np.ndarray
    stamp_ns: int


class OffboardMissionNode(Node):
    """Run the inspection mission state machine and publish PX4 setpoints."""

    def __init__(self) -> None:
        super().__init__("powerplant_offboard_mission")
        self._declare_parameters()
        self.bridge = CvBridge()

        self.current_position_enu: np.ndarray | None = None
        self.current_orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.current_yaw_enu = 0.0
        self.home_position_enu: np.ndarray | None = None
        self.vehicle_status = VehicleStatus()
        self.offboard_counter = 0
        self.active = bool(self.get_parameter("auto_start").value)
        self.phase = PHASE_START if self.active else PHASE_IDLE
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_command_ack: VehicleCommandAck | None = None

        self.base_waypoints = self._parse_waypoints(self.get_parameter("mission_waypoints_enu").value)
        self.active_waypoints = list(self.base_waypoints)
        self.goal_waypoint_indices: set[int] = set()
        self.waypoint_index = 0

        self.grid: np.ndarray | None = None
        self.grid_info: GridInfo | None = None
        self.latest_depth_m: np.ndarray | None = None
        self.latest_depth_stamp_ns = 0
        self.depth_camera_info: CameraInfo | None = None
        self.latest_rgb_msg: Image | None = None

        self.target_observation: TargetObservation | None = None
        self.inspection_captured_indices: set[int] = set()
        self.inspection_photo_count = 0
        self.mission_started_ns = 0
        self.search_started_ns = 0
        self.search_loop_count = 0
        self.landing_started_ns = 0
        self.landing_command_sent = False
        self.return_reason = ""

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            self.get_parameter("offboard_control_mode_topic").value,
            px4_qos_profile(),
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            self.get_parameter("trajectory_setpoint_topic").value,
            px4_qos_profile(),
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.get_parameter("vehicle_command_topic").value,
            px4_qos_profile(),
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("mission_status_topic").value,
            reliable_qos_profile(),
        )
        self.inspection_event_pub = self.create_publisher(
            String,
            self.get_parameter("inspection_event_topic").value,
            reliable_qos_profile(),
        )
        self.target_position_pub = self.create_publisher(
            PointStamped,
            self.get_parameter("target_position_topic").value,
            reliable_qos_profile(),
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("localized_odom_topic").value,
            self._odom_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            VehicleStatus,
            self.get_parameter("vehicle_status_topic").value,
            self._vehicle_status_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            VehicleCommandAck,
            self.get_parameter("vehicle_command_ack_topic").value,
            self._vehicle_command_ack_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("occupancy_grid_topic").value,
            self._occupancy_grid_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            String,
            self.get_parameter("yolo_detections_topic").value,
            self._yolo_detections_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_image_topic").value,
            self._depth_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_camera_info_topic").value,
            self._depth_camera_info_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_image_topic").value,
            self._rgb_callback,
            sensor_qos_profile(),
        )

        self.create_service(Trigger, "start_powerplant_mission", self._start_mission)
        self.create_service(Trigger, "land_powerplant_mission", self._land_mission)
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info("offboard mission node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("occupancy_grid_topic", "/powerplant/map/occupancy_grid")
        self.declare_parameter("yolo_detections_topic", "/powerplant/perception/yolo_detections")
        self.declare_parameter("target_position_topic", "/powerplant/perception/target_position")
        self.declare_parameter("inspection_event_topic", "/powerplant/control/inspection_events")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        self.declare_parameter("depth_camera_info_topic", "/depth_camera/camera_info")
        self.declare_parameter("rgb_image_topic", "/camera")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
        self.declare_parameter("vehicle_command_ack_topic", "/fmu/out/vehicle_command_ack")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("mission_status_topic", "/powerplant/control/mission_status")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("command_retry_interval_s", 1.0)
        self.declare_parameter("takeoff_height_m", 5.0)
        self.declare_parameter("return_home_height_m", 5.0)
        self.declare_parameter("target_acceptance_m", 0.45)
        self.declare_parameter("yaw_acceptance_rad", 0.25)
        self.declare_parameter("cruise_yaw_enu_rad", 0.0)
        self.declare_parameter("use_occupancy_grid_planner", True)
        self.declare_parameter("planner_inflation_radius_m", 0.6)
        self.declare_parameter("path_waypoint_stride", 4)
        self.declare_parameter("target_class_name", "gas_cylinder")
        self.declare_parameter("min_target_confidence", 0.35)
        self.declare_parameter("target_memory_s", 8.0)
        self.declare_parameter("loop_search_waypoints", True)
        self.declare_parameter("search_timeout_s", 300.0)
        self.declare_parameter("target_standoff_m", 2.2)
        self.declare_parameter("inspection_altitude_m", 5.0)
        self.declare_parameter("inspection_orbit_points", 4)
        self.declare_parameter("inspection_required_photos", 4)
        self.declare_parameter("inspection_photo_dir", "/home/pawn/px4_ros_ws/inspection_photos")
        self.declare_parameter("require_image_for_photo", False)
        self.declare_parameter("camera_fx", 430.0)
        self.declare_parameter("camera_fy", 430.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        self.declare_parameter("camera_translation_flu", [0.12, -0.03, 0.24])
        self.declare_parameter("bbox_depth_sample_fraction", 0.45)
        self.declare_parameter("min_detection_depth_m", 0.25)
        self.declare_parameter("max_detection_depth_m", 18.0)
        self.declare_parameter("max_depth_age_s", 1.0)
        self.declare_parameter("land_complete_height_m", 0.25)
        self.declare_parameter("land_complete_timeout_s", 20.0)
        self.declare_parameter(
            "mission_waypoints_enu",
            [
                0.0, 0.0, 5.0,
                24.0, 0.0, 5.0,
                24.0, 24.0, 5.0,
                0.0, 24.0, 5.0,
                0.0, 0.0, 5.0,
            ],
        )

    def _parse_waypoints(self, values: list[float]) -> list[np.ndarray]:
        if len(values) % 3 != 0:
            self.get_logger().warn("mission_waypoints_enu must be x,y,z triples; using hold point")
            return [np.array([0.0, 0.0, float(self.get_parameter("takeoff_height_m").value)], dtype=float)]
        return [
            np.asarray(values[index:index + 3], dtype=float)
            for index in range(0, len(values), 3)
        ]

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_position_enu = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        self.current_orientation_xyzw = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        self.current_yaw_enu = quaternion_xyzw_to_yaw(self.current_orientation_xyzw)

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _vehicle_command_ack_callback(self, msg: VehicleCommandAck) -> None:
        self.last_command_ack = msg
        if msg.result not in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):
            self.get_logger().warn(
                "PX4 command %s was not accepted: %s"
                % (self._command_name(msg.command), self._command_result_name(msg.result)),
                throttle_duration_sec=1.0,
            )

    def _occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        data[data < 0] = 0
        self.grid = data.astype(np.int8)
        self.grid_info = GridInfo(
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
        )

    def _depth_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"depth image conversion failed: {exc}", throttle_duration_sec=5.0)
            return
        depth = np.asarray(image, dtype=np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if msg.encoding in ("16UC1", "mono16"):
            depth = depth * 0.001
        self.latest_depth_m = depth
        self.latest_depth_stamp_ns = self.get_clock().now().nanoseconds

    def _depth_camera_info_callback(self, msg: CameraInfo) -> None:
        self.depth_camera_info = msg

    def _rgb_callback(self, msg: Image) -> None:
        self.latest_rgb_msg = msg

    def _yolo_detections_callback(self, msg: String) -> None:
        if self.current_position_enu is None or self.latest_depth_m is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"invalid YOLO detection JSON: {exc}", throttle_duration_sec=5.0)
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return
        best_observation: TargetObservation | None = None
        for detection in sorted(detections, key=lambda item: float(item.get("confidence") or 0.0), reverse=True):
            observation = self._observation_from_detection(detection, payload)
            if observation is None:
                continue
            best_observation = observation
            break

        if best_observation is None:
            return
        self.target_observation = best_observation
        self._publish_target_position(best_observation)

    def _observation_from_detection(
        self,
        detection: dict[str, Any],
        payload: dict[str, Any],
    ) -> TargetObservation | None:
        class_name = str(detection.get("class_name", ""))
        target_class = str(self.get_parameter("target_class_name").value)
        if target_class and class_name != target_class:
            return None
        confidence = float(detection.get("confidence") or 0.0)
        if confidence < float(self.get_parameter("min_target_confidence").value):
            return None

        bbox = detection.get("bbox_xyxy")
        if not isinstance(bbox, list) or len(bbox) != 4:
            return None
        image_width = int(payload.get("image_width") or 0)
        image_height = int(payload.get("image_height") or 0)
        position = self._bbox_to_map_position([float(v) for v in bbox], image_width, image_height)
        if position is None:
            return None
        return TargetObservation(
            class_name=class_name,
            confidence=confidence,
            bbox_xyxy=[float(v) for v in bbox],
            position_enu=position,
            stamp_ns=self.get_clock().now().nanoseconds,
        )

    def _bbox_to_map_position(
        self,
        bbox_xyxy: list[float],
        image_width: int,
        image_height: int,
    ) -> np.ndarray | None:
        if self.current_position_enu is None or self.latest_depth_m is None:
            return None
        if self._age_s(self.latest_depth_stamp_ns) > float(self.get_parameter("max_depth_age_s").value):
            return None

        depth = self.latest_depth_m
        height, width = depth.shape[:2]
        if image_width <= 0 or image_height <= 0 or width <= 0 or height <= 0:
            return None

        scale_x = width / float(image_width)
        scale_y = height / float(image_height)
        x1, y1, x2, y2 = bbox_xyxy
        cx = 0.5 * (x1 + x2) * scale_x
        cy = 0.5 * (y1 + y2) * scale_y
        box_w = max(2.0, (x2 - x1) * scale_x)
        box_h = max(2.0, (y2 - y1) * scale_y)
        fraction = float(self.get_parameter("bbox_depth_sample_fraction").value)
        half_w = max(1, int(0.5 * box_w * fraction))
        half_h = max(1, int(0.5 * box_h * fraction))
        u0 = max(0, int(round(cx)) - half_w)
        u1 = min(width, int(round(cx)) + half_w + 1)
        v0 = max(0, int(round(cy)) - half_h)
        v1 = min(height, int(round(cy)) + half_h + 1)
        if u1 <= u0 or v1 <= v0:
            return None

        crop = depth[v0:v1, u0:u1]
        min_depth = float(self.get_parameter("min_detection_depth_m").value)
        max_depth = float(self.get_parameter("max_detection_depth_m").value)
        valid = crop[np.isfinite(crop) & (crop >= min_depth) & (crop <= max_depth)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))

        fx, fy, cam_cx, cam_cy = self._camera_intrinsics()
        optical_x = (cx - cam_cx) * z / fx
        optical_y = (cy - cam_cy) * z / fy
        body_flu = np.array([z, -optical_x, -optical_y], dtype=float)
        body_flu += np.asarray(self.get_parameter("camera_translation_flu").value, dtype=float)
        return self.current_position_enu + rotate_vector_xyzw(self.current_orientation_xyzw, body_flu)

    def _camera_intrinsics(self) -> tuple[float, float, float, float]:
        if self.depth_camera_info is not None and self.depth_camera_info.k[0] > 0.0:
            return (
                float(self.depth_camera_info.k[0]),
                float(self.depth_camera_info.k[4]),
                float(self.depth_camera_info.k[2]),
                float(self.depth_camera_info.k[5]),
            )
        return (
            float(self.get_parameter("camera_fx").value),
            float(self.get_parameter("camera_fy").value),
            float(self.get_parameter("camera_cx").value),
            float(self.get_parameter("camera_cy").value),
        )

    def _start_mission(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.active = True
        self.phase = PHASE_START
        self.offboard_counter = 0
        self.waypoint_index = 0
        self.active_waypoints = []
        self.goal_waypoint_indices.clear()
        self.target_observation = None
        self.inspection_captured_indices.clear()
        self.inspection_photo_count = 0
        self.landing_command_sent = False
        self.return_reason = ""
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.search_loop_count = 0
        self.mission_started_ns = self.get_clock().now().nanoseconds
        response.success = True
        response.message = "mission requested: START"
        return response

    def _land_mission(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._enter_landing("manual land service")
        response.success = True
        response.message = "land command sent"
        return response

    def _timer_callback(self) -> None:
        self._publish_offboard_heartbeat()
        if self.current_position_enu is None:
            self._publish_status()
            return

        if self.phase == PHASE_LANDING:
            self._update_landing()
            self._publish_status()
            return

        if not self.active:
            self._publish_status()
            return

        if self.phase == PHASE_START:
            self._initialize_mission()

        self._maybe_engage_offboard()
        target, yaw = self._select_target_and_yaw()
        self._publish_position_setpoint(target, yaw)
        self._update_phase(target, yaw)
        self._publish_status(target)

    def _initialize_mission(self) -> None:
        if self.current_position_enu is None:
            return
        self.home_position_enu = self.current_position_enu.copy()
        self.phase = PHASE_TAKING_OFF
        self.search_started_ns = 0
        self.search_loop_count = 0
        self.active_waypoints = []
        self.goal_waypoint_indices.clear()
        self.waypoint_index = 0
        self._publish_event("mission_initialized", {"home_enu": self._round_list(self.home_position_enu)})

    def _maybe_engage_offboard(self) -> None:
        if self.offboard_counter >= 10:
            retry_interval_ns = int(float(self.get_parameter("command_retry_interval_s").value) * 1e9)
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and now_ns - self.last_offboard_request_ns >= retry_interval_ns
            ):
                self.last_offboard_request_ns = now_ns
                self.get_logger().info("requesting PX4 Offboard mode", throttle_duration_sec=2.0)
                self._engage_offboard()
            if (
                bool(self.get_parameter("arm_on_start").value)
                and self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
                and now_ns - self.last_arm_request_ns >= retry_interval_ns
            ):
                self.last_arm_request_ns = now_ns
                self.get_logger().info("requesting PX4 arm", throttle_duration_sec=2.0)
                self._arm()
        elif self.offboard_counter == 0:
            self.get_logger().info("warming up Offboard setpoints", throttle_duration_sec=2.0)
        self.offboard_counter = min(self.offboard_counter + 1, 1000)

    def _select_target_and_yaw(self) -> tuple[np.ndarray, float]:
        if self.current_position_enu is None:
            return np.zeros(3, dtype=float), 0.0

        if self.phase == PHASE_TAKING_OFF:
            home = self.home_position_enu if self.home_position_enu is not None else self.current_position_enu
            target = home.copy()
            target[2] = float(self.get_parameter("takeoff_height_m").value)
            return target, float(self.get_parameter("cruise_yaw_enu_rad").value)

        if self.phase in (PHASE_GLOBAL_SEARCH, PHASE_RETURN_HOME, PHASE_TARGETING_CYCLE):
            target = self._current_nav_target()
            yaw = float(self.get_parameter("cruise_yaw_enu_rad").value)
            if self.phase == PHASE_TARGETING_CYCLE and self.target_observation is not None:
                yaw = self._yaw_to_face(self.target_observation.position_enu)
            elif self.phase == PHASE_RETURN_HOME:
                yaw = self._yaw_to_face(target)
            return target, yaw

        return self.current_position_enu.copy(), self.current_yaw_enu

    def _update_phase(self, target: np.ndarray, yaw: float) -> None:
        if self.current_position_enu is None:
            return

        if self.phase == PHASE_TAKING_OFF:
            if self._has_reached(target, tolerance=0.35):
                self._enter_global_search()
            return

        if self.phase == PHASE_GLOBAL_SEARCH:
            if self._target_is_recent():
                self._enter_targeting_cycle()
                return
            search_timeout_s = float(self.get_parameter("search_timeout_s").value)
            if (
                search_timeout_s > 0.0
                and self.search_started_ns
                and self._age_s(self.search_started_ns) > search_timeout_s
            ):
                self._enter_return_home("search timeout")
                return
            if self._advance_path_if_reached(target):
                self._handle_search_path_complete()
            return

        if self.phase == PHASE_TARGETING_CYCLE:
            self._update_targeting_cycle(target, yaw)
            return

        if self.phase == PHASE_RETURN_HOME and self._advance_path_if_reached(target):
            self._enter_landing(self.return_reason or "return home complete")

    def _enter_global_search(self) -> None:
        self.phase = PHASE_GLOBAL_SEARCH
        self.search_started_ns = self.get_clock().now().nanoseconds
        self.search_loop_count = 0
        self._set_navigation_goals(self.base_waypoints)
        self._publish_event(
            "phase_changed",
            {"phase": self.phase, "search_waypoints": len(self.active_waypoints)},
        )

    def _handle_search_path_complete(self) -> None:
        if bool(self.get_parameter("loop_search_waypoints").value) and self.base_waypoints:
            self.search_loop_count += 1
            self._set_navigation_goals(self.base_waypoints)
            self._publish_event(
                "search_loop_completed",
                {
                    "search_loop_count": self.search_loop_count,
                    "search_age_s": round(self._age_s(self.search_started_ns), 2),
                    "next_waypoints": len(self.active_waypoints),
                },
            )
            return
        self._enter_return_home("target not found")

    def _enter_targeting_cycle(self) -> None:
        if self.target_observation is None:
            return
        self.phase = PHASE_TARGETING_CYCLE
        self.inspection_captured_indices.clear()
        self.inspection_photo_count = 0
        goals = self._inspection_goals(self.target_observation.position_enu)
        self._set_navigation_goals(goals)
        self._publish_event(
            "target_acquired",
            {
                "target_class": self.target_observation.class_name,
                "target_enu": self._round_list(self.target_observation.position_enu),
                "inspection_waypoints": len(self.goal_waypoint_indices),
            },
        )

    def _update_targeting_cycle(self, target: np.ndarray, yaw: float) -> None:
        if self.target_observation is None:
            self._enter_return_home("target lost before targeting")
            return
        if not self._has_reached(target):
            return
        if self.waypoint_index not in self.goal_waypoint_indices:
            self._advance_path_if_reached(target)
            return
        yaw_error = abs(wrap_pi(yaw - self.current_yaw_enu))
        if yaw_error > float(self.get_parameter("yaw_acceptance_rad").value):
            return
        if self.waypoint_index not in self.inspection_captured_indices:
            if not self._capture_inspection_photo(self.waypoint_index):
                return
            self.inspection_captured_indices.add(self.waypoint_index)
            self.inspection_photo_count += 1

        required = int(self.get_parameter("inspection_required_photos").value)
        if self.inspection_photo_count >= required or self.waypoint_index >= len(self.active_waypoints) - 1:
            self._enter_return_home("inspection complete")
            return
        self._advance_path_if_reached(target)

    def _enter_return_home(self, reason: str) -> None:
        self.phase = PHASE_RETURN_HOME
        self.return_reason = reason
        if self.home_position_enu is None and self.current_position_enu is not None:
            self.home_position_enu = self.current_position_enu.copy()
        home = self.home_position_enu if self.home_position_enu is not None else np.zeros(3, dtype=float)
        goal = home.copy()
        goal[2] = float(self.get_parameter("return_home_height_m").value)
        self._set_navigation_goals([goal])
        self._publish_event("return_home", {"reason": reason, "home_enu": self._round_list(goal)})

    def _enter_landing(self, reason: str) -> None:
        self.phase = PHASE_LANDING
        self.active = False
        self.return_reason = reason
        self.landing_started_ns = self.get_clock().now().nanoseconds
        if not self.landing_command_sent:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_command_sent = True
        self._publish_event("landing", {"reason": reason})

    def _update_landing(self) -> None:
        if self.current_position_enu is None:
            return
        landed_by_height = self.current_position_enu[2] <= float(self.get_parameter("land_complete_height_m").value)
        timed_out = self._age_s(self.landing_started_ns) > float(self.get_parameter("land_complete_timeout_s").value)
        if landed_by_height or timed_out:
            self.phase = PHASE_COMPLETE
            self.landing_command_sent = False
            self._publish_event(
                "mission_complete",
                {
                    "landed_by_height": bool(landed_by_height),
                    "landing_timeout": bool(timed_out and not landed_by_height),
                },
            )

    def _set_navigation_goals(self, goals: list[np.ndarray]) -> None:
        self.active_waypoints, self.goal_waypoint_indices = self._plan_waypoint_sequence(goals)
        if not self.active_waypoints and self.current_position_enu is not None:
            self.active_waypoints = [self.current_position_enu.copy()]
            self.goal_waypoint_indices = {0}
        self.waypoint_index = 0

    def _plan_waypoint_sequence(self, goals: list[np.ndarray]) -> tuple[list[np.ndarray], set[int]]:
        if self.current_position_enu is None:
            return list(goals), set(range(len(goals)))
        if (
            not bool(self.get_parameter("use_occupancy_grid_planner").value)
            or self.grid is None
            or self.grid_info is None
        ):
            return list(goals), set(range(len(goals)))

        planned: list[np.ndarray] = []
        goal_indices: set[int] = set()
        start = self.current_position_enu
        stride = max(1, int(self.get_parameter("path_waypoint_stride").value))
        for goal in goals:
            path_xy = plan_astar(
                self.grid,
                self.grid_info,
                start[:2],
                goal[:2],
                inflation_radius_m=float(self.get_parameter("planner_inflation_radius_m").value),
            )
            if path_xy:
                for x, y in path_xy[::stride]:
                    self._append_waypoint(planned, np.array([x, y, goal[2]], dtype=float))
            self._append_waypoint(planned, goal)
            goal_indices.add(len(planned) - 1)
            start = goal
        return planned, goal_indices

    def _append_waypoint(self, waypoints: list[np.ndarray], waypoint: np.ndarray) -> None:
        if waypoints and np.linalg.norm(waypoints[-1] - waypoint) < 0.05:
            waypoints[-1] = waypoint
            return
        waypoints.append(waypoint)

    def _inspection_goals(self, target_enu: np.ndarray) -> list[np.ndarray]:
        if self.current_position_enu is None:
            return []
        radius = float(self.get_parameter("target_standoff_m").value)
        altitude = float(self.get_parameter("inspection_altitude_m").value)
        count = max(1, int(self.get_parameter("inspection_orbit_points").value))
        from_target = self.current_position_enu[:2] - target_enu[:2]
        norm = float(np.linalg.norm(from_target))
        if norm < 1e-6:
            from_target = np.array([math.cos(self.current_yaw_enu), math.sin(self.current_yaw_enu)], dtype=float)
        else:
            from_target = from_target / norm

        base_angle = math.atan2(from_target[1], from_target[0])
        goals = []
        for index in range(count):
            angle = base_angle + index * 2.0 * math.pi / count
            offset = np.array([math.cos(angle), math.sin(angle)], dtype=float) * radius
            goals.append(np.array([target_enu[0] + offset[0], target_enu[1] + offset[1], altitude], dtype=float))
        return goals

    def _current_nav_target(self) -> np.ndarray:
        if self.current_position_enu is None:
            return np.zeros(3, dtype=float)
        if not self.active_waypoints:
            return self.current_position_enu.copy()
        self.waypoint_index = min(self.waypoint_index, len(self.active_waypoints) - 1)
        return self.active_waypoints[self.waypoint_index]

    def _advance_path_if_reached(self, target: np.ndarray) -> bool:
        if not self._has_reached(target):
            return False
        if self.waypoint_index < len(self.active_waypoints) - 1:
            self.waypoint_index += 1
            return False
        return True

    def _has_reached(self, target: np.ndarray, tolerance: float | None = None) -> bool:
        if self.current_position_enu is None:
            return False
        threshold = tolerance if tolerance is not None else float(self.get_parameter("target_acceptance_m").value)
        return float(np.linalg.norm(self.current_position_enu - target)) <= threshold

    def _target_is_recent(self) -> bool:
        if self.target_observation is None:
            return False
        return self._age_s(self.target_observation.stamp_ns) <= float(self.get_parameter("target_memory_s").value)

    def _yaw_to_face(self, target_enu: np.ndarray) -> float:
        if self.current_position_enu is None:
            return self.current_yaw_enu
        delta = target_enu[:2] - self.current_position_enu[:2]
        if float(np.linalg.norm(delta)) < 1e-6:
            return self.current_yaw_enu
        return math.atan2(float(delta[1]), float(delta[0]))

    def _capture_inspection_photo(self, waypoint_index: int) -> bool:
        event: dict[str, Any] = {
            "waypoint_index": waypoint_index,
            "photo_index": self.inspection_photo_count + 1,
        }
        if self.target_observation is not None:
            event["target_enu"] = self._round_list(self.target_observation.position_enu)
            event["target_class"] = self.target_observation.class_name
        if self.current_position_enu is not None:
            event["vehicle_enu"] = self._round_list(self.current_position_enu)

        if self.latest_rgb_msg is None:
            event["saved"] = False
            event["reason"] = "no_rgb_image"
            self._publish_event("inspection_photo", event)
            return not bool(self.get_parameter("require_image_for_photo").value)

        try:
            import cv2  # type: ignore

            image = self.bridge.imgmsg_to_cv2(self.latest_rgb_msg, desired_encoding="bgr8")
            directory = Path(str(self.get_parameter("inspection_photo_dir").value)).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            filename = directory / f"inspection_{self.inspection_photo_count + 1:02d}_{self._timestamp_us()}.jpg"
            ok = bool(cv2.imwrite(str(filename), image))
            event["saved"] = ok
            event["path"] = str(filename)
            self._publish_event("inspection_photo", event)
            return ok
        except Exception as exc:  # noqa: BLE001
            event["saved"] = False
            event["reason"] = str(exc)
            self._publish_event("inspection_photo", event)
            return not bool(self.get_parameter("require_image_for_photo").value)

    def _publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self.offboard_pub.publish(msg)

    def _publish_position_setpoint(self, position_enu: np.ndarray, yaw_enu: float) -> None:
        ned = enu_to_ned(position_enu)
        msg = TrajectorySetpoint()
        msg.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw_enu_to_ned(yaw_enu))
        msg.yawspeed = math.nan
        msg.timestamp = self._timestamp_us()
        self.setpoint_pub.publish(msg)

    def _publish_target_position(self, observation: TargetObservation) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(observation.position_enu[0])
        msg.point.y = float(observation.position_enu[1])
        msg.point.z = float(observation.position_enu[2])
        self.target_position_pub.publish(msg)
        self._publish_event(
            "target_position",
            {
                "target_class": observation.class_name,
                "confidence": round(observation.confidence, 4),
                "target_enu": self._round_list(observation.position_enu),
            },
        )

    def _engage_offboard(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _arm(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.command_pub.publish(msg)

    def _publish_status(self, target: np.ndarray | None = None) -> None:
        payload: dict[str, Any] = {
            "phase": self.phase,
            "active": self.active,
            "waypoint_index": self.waypoint_index,
            "waypoint_count": len(self.active_waypoints),
            "nav_state": int(self.vehicle_status.nav_state),
            "arming_state": int(self.vehicle_status.arming_state),
            "pre_flight_checks_pass": bool(self.vehicle_status.pre_flight_checks_pass),
            "accepts_offboard_setpoints": bool(self.vehicle_status.accepts_offboard_setpoints),
            "photos": self.inspection_photo_count,
        }
        if self.phase == PHASE_GLOBAL_SEARCH:
            payload["search_loop_count"] = self.search_loop_count
            payload["search_age_s"] = round(self._age_s(self.search_started_ns), 2)
        if self.active_waypoints:
            payload["current_waypoint_enu"] = self._round_list(
                self.active_waypoints[min(self.waypoint_index, len(self.active_waypoints) - 1)]
            )
        if self.last_command_ack is not None:
            payload["last_command_ack"] = {
                "command": self._command_name(self.last_command_ack.command),
                "result": self._command_result_name(self.last_command_ack.result),
            }
        if target is not None:
            payload["setpoint_enu"] = self._round_list(target)
        if self.current_position_enu is not None:
            payload["vehicle_enu"] = self._round_list(self.current_position_enu)
        if self.target_observation is not None:
            payload["target"] = {
                "class_name": self.target_observation.class_name,
                "confidence": round(self.target_observation.confidence, 4),
                "position_enu": self._round_list(self.target_observation.position_enu),
                "age_s": round(self._age_s(self.target_observation.stamp_ns), 2),
            }
        if self.return_reason:
            payload["return_reason"] = self.return_reason
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_event(self, event: str, fields: dict[str, Any]) -> None:
        payload = {"event": event, "phase": self.phase, **fields}
        self.inspection_event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _age_s(self, stamp_ns: int) -> float:
        if stamp_ns <= 0:
            return math.inf
        return (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9

    def _round_list(self, values: np.ndarray, digits: int = 2) -> list[float]:
        return [round(float(v), digits) for v in values.tolist()]

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _command_name(self, command: int) -> str:
        names = {
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE: "DO_SET_MODE",
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM: "ARM_DISARM",
            VehicleCommand.VEHICLE_CMD_NAV_LAND: "NAV_LAND",
        }
        return names.get(int(command), str(int(command)))

    def _command_result_name(self, result: int) -> str:
        names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: "ACCEPTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "DENIED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "UNSUPPORTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "FAILED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: "IN_PROGRESS",
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "CANCELLED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY: "COMMAND_LONG_ONLY",
            VehicleCommandAck.VEHICLE_CMD_RESULT_COMMAND_INT_ONLY: "COMMAND_INT_ONLY",
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME: "UNSUPPORTED_MAV_FRAME",
        }
        return names.get(int(result), str(int(result)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OffboardMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
