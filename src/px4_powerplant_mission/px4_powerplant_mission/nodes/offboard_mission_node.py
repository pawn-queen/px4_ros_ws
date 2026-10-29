"""PX4 offboard mission controller with mapped frontier exploration and inspection."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from px4_msgs.msg import (
    BatteryStatus,
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from px4_powerplant_mission.common.frames import (
    enu_to_ned,
    quaternion_xyzw_to_roll_pitch,
    quaternion_xyzw_to_yaw,
    rotate_vector_xyzw,
    wrap_pi,
    yaw_enu_to_ned,
)
from px4_powerplant_mission.common.initial_scan import YawScanSequence, has_observed_cells
from px4_powerplant_mission.common.qos import (
    px4_qos_profile,
    reliable_qos_profile,
    sensor_qos_profile,
)
from px4_powerplant_mission.path_planning.grid_astar import GridInfo, world_to_grid
from px4_powerplant_mission.path_planning.frontier_3d import (
    ExplorationConfig,
    path_is_known_free,
    plan_frontier,
    plan_known_free_path,
)


PHASE_IDLE = "IDLE"
PHASE_START = "START"
PHASE_SAFE_TAKEOFF = "SAFE_TAKEOFF"
PHASE_HOVER_SCAN = "HOVER_SCAN"
PHASE_EXPLORE = "EXPLORE"
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


@dataclass(frozen=True)
class DistanceVarianceEstimate:
    """Conservative spatial estimate from current covariance and travel distance."""

    origin_xyz: tuple[float, float, float]
    resolution_m: float
    base_variance_m2: float
    growth_m2_per_m: float

    def get(self, key: tuple[int, int, int], default: float | None = None) -> float:
        del default
        centre = tuple((key[axis] + 0.5) * self.resolution_m for axis in range(3))
        distance = math.dist(self.origin_xyz, centre)
        return self.base_variance_m2 + self.growth_m2_per_m * distance


def _plan_safe_goal_sequence(
    free_keys: frozenset[tuple[int, int, int]],
    occupied_keys: frozenset[tuple[int, int, int]],
    resolution_m: float,
    bounds_keys: tuple[tuple[int, int, int], tuple[int, int, int]],
    start_xyz: np.ndarray,
    goals: list[np.ndarray],
    goal_ids: list[int],
    config: ExplorationConfig,
) -> tuple[list[np.ndarray], set[int], dict[int, int]]:
    """Compute inspection paths without touching the ROS node or timer thread."""
    planned: list[np.ndarray] = []
    goal_indices: set[int] = set()
    waypoint_ids: dict[int, int] = {}
    start = start_xyz.copy()
    for goal_index, goal in enumerate(goals):
        path = plan_known_free_path(
            free_keys, occupied_keys, resolution_m, bounds_keys, start, goal, config
        )
        if not path:
            continue
        for point in path[1:]:
            if planned and np.linalg.norm(planned[-1] - point) < 0.05:
                planned[-1] = point.copy()
            else:
                planned.append(point.copy())
        if not planned:
            planned.append(goal.copy())
        marker = len(planned) - 1
        goal_indices.add(marker)
        waypoint_ids[marker] = goal_ids[goal_index] if goal_index < len(goal_ids) else goal_index
        start = goal.copy()
    return planned, goal_indices, waypoint_ids


class OffboardMissionNode(Node):
    """Run the inspection mission state machine and publish PX4 setpoints."""

    def __init__(self) -> None:
        super().__init__("powerplant_offboard_mission")
        self._declare_parameters()
        self.bridge = CvBridge()

        self.current_position_enu: np.ndarray | None = None
        self.current_orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.current_yaw_enu = 0.0
        self.current_speed_mps = math.inf
        self.current_yaw_rate_radps = math.inf
        self.home_position_enu: np.ndarray | None = None
        self.vehicle_status = VehicleStatus()
        self.failsafe_flags = FailsafeFlags()
        self.offboard_counter = 0
        self.active = bool(self.get_parameter("auto_start").value)
        self.phase = PHASE_START if self.active else PHASE_IDLE
        self.last_odom_stamp_ns = 0
        self.odom_position_variance_m2 = math.inf
        self.odom_height_variance_m2 = math.inf
        self.odom_tilt_variance_rad2 = math.inf
        self.command_wait_reason = ""
        self.commanded_setpoint_enu: np.ndarray | None = None
        self.commanded_yaw_enu = 0.0
        self.last_setpoint_update_ns = 0
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_command_ack: VehicleCommandAck | None = None

        self.active_waypoints: list[np.ndarray] = []
        self.goal_waypoint_indices: set[int] = set()
        self.goal_waypoint_ids: dict[int, int] = {}
        self.navigation_goals: list[np.ndarray] = []
        self.navigation_goal_ids: list[int] = []
        self.waypoint_index = 0

        self.pending_voxel_cloud: PointCloud2 | None = None
        self.pending_voxel_status: dict[str, Any] | None = None
        self.free_voxel_keys: set[tuple[int, int, int]] = set()
        self.occupied_voxel_keys: set[tuple[int, int, int]] = set()
        self.voxel_resolution_m = 0.0
        self.map_revision = -1
        self.map_observation_revision = 0
        self.visited_viewpoint_keys: set[tuple[int, int, int]] = set()
        self.map_snapshot_stamp_ns = 0
        self.map_snapshot_received_ns = 0
        self.map_snapshot_complete = False
        self.explore_plan_revision = -1
        self.explore_plan_stamp_ns = 0
        self.explore_wait_reason = ""
        self.explore_goal_enu: np.ndarray | None = None
        self.explore_goal_yaw_enu = 0.0
        self.explore_score = 0.0
        self.explore_viewpoint_entered_ns = 0
        self.explore_viewpoint_depth_baseline = 0
        self.return_goal_enu: np.ndarray | None = None
        self.return_plan_revision = -1
        self.return_plan_stamp_ns = 0
        self.battery_remaining_fraction = math.nan
        self.battery_time_remaining_s = math.nan
        self.battery_connected = False
        self.battery_stamp_ns = 0
        self.planning_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="mission_3d_plan")
        self.explore_future: Future | None = None
        self.explore_job_revision = -1
        self.return_future: Future | None = None
        self.return_job_revision = -1
        self.target_future: Future | None = None
        self.target_job_revision = -1
        self.target_plan_revision = -1

        self.grid: np.ndarray | None = None
        self.grid_info: GridInfo | None = None
        self.grid_stamp_ns = 0
        self.grid_known_cells = 0
        self.map_fusion_depth_count = 0
        self.map_fusion_lidar_count = 0
        self.map_fusion_depth_sample_stamp_ns = 0
        self.map_fusion_lidar_sample_stamp_ns = 0
        self.map_fusion_depth_received_ns = 0
        self.map_fusion_lidar_received_ns = 0
        self.map_fusion_status_received = False
        self.initial_scan_depth_baseline: int | None = None
        self.initial_scan_lidar_baseline: int | None = None
        self.initial_scan_step_depth_baseline: int | None = None
        self.initial_scan_step_lidar_baseline: int | None = None
        self.last_replan_ns = 0
        self.avoidance_startup_gate_open = not bool(
            self.get_parameter("avoidance_startup_gate_enabled").value
        )
        self.safety_stop_active = False
        self.safety_stop_reason = ""
        self.latest_depth_m: np.ndarray | None = None
        self.latest_depth_stamp_ns = 0
        self.last_valid_lidar_stamp_ns = 0
        self.depth_camera_info: CameraInfo | None = None
        self.latest_rgb_msg: Image | None = None

        self.target_observation: TargetObservation | None = None
        self.inspection_captured_indices: set[int] = set()
        self.inspection_photo_count = 0
        self.mission_started_ns = 0
        self.search_started_ns = 0
        self.landing_started_ns = 0
        self.landing_command_sent = False
        self.return_reason = ""
        self.initial_scan_anchor_enu: np.ndarray | None = None
        self.initial_scan_sequence: YawScanSequence | None = None
        self.initial_scan_entered_ns = 0
        self.initial_scan_hover_since_ns = 0
        self.initial_scan_rotating = False
        self.initial_scan_wait_reason = ""

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
        self.planned_path_pub = self.create_publisher(
            NavPath,
            self.get_parameter("planned_path_topic").value,
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
            FailsafeFlags,
            self.get_parameter("failsafe_flags_topic").value,
            self._failsafe_flags_callback,
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
            self.get_parameter("map_fusion_status_topic").value,
            self._map_fusion_status_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("observed_voxels_topic").value,
            self._observed_voxels_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            BatteryStatus,
            self.get_parameter("battery_status_topic").value,
            self._battery_status_callback,
            px4_qos_profile(),
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
            LaserScan,
            self.get_parameter("laser_scan_topic").value,
            self._laser_scan_callback,
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

    def destroy_node(self) -> bool:
        self.planning_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("occupancy_grid_topic", "/powerplant/map/occupancy_grid")
        self.declare_parameter("map_fusion_status_topic", "/powerplant/map/fusion_status")
        self.declare_parameter("observed_voxels_topic", "/powerplant/map/observed_voxels")
        self.declare_parameter("planned_path_topic", "/powerplant/control/planned_path")
        self.declare_parameter("yolo_detections_topic", "/powerplant/perception/yolo_detections")
        self.declare_parameter("target_position_topic", "/powerplant/perception/target_position")
        self.declare_parameter("inspection_event_topic", "/powerplant/control/inspection_events")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        self.declare_parameter("laser_scan_topic", "/mapping_lidar")
        self.declare_parameter("depth_camera_info_topic", "/depth_camera/camera_info")
        self.declare_parameter("rgb_image_topic", "/camera")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
        self.declare_parameter("battery_status_topic", "/fmu/out/battery_status")
        self.declare_parameter("failsafe_flags_topic", "/fmu/out/failsafe_flags")
        self.declare_parameter("vehicle_command_ack_topic", "/fmu/out/vehicle_command_ack")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("mission_status_topic", "/powerplant/control/mission_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("command_retry_interval_s", 1.0)
        self.declare_parameter("offboard_warmup_setpoints", 20)
        self.declare_parameter("require_localization_for_start", True)
        self.declare_parameter("max_localization_age_s", 0.5)
        self.declare_parameter("max_start_position_variance_m2", 4.0)
        self.declare_parameter("max_start_height_variance_m2", 4.0)
        self.declare_parameter("require_preflight_checks", True)
        self.declare_parameter("require_accepts_offboard_setpoints", False)
        self.declare_parameter("takeoff_height_m", 5.0)
        self.declare_parameter("initial_scan_hover_s", 2.0)
        self.declare_parameter("initial_scan_yaw_steps", 12)
        self.declare_parameter("initial_scan_yaw_rate_radps", 0.2)
        self.declare_parameter("initial_scan_yaw_dwell_s", 0.3)
        self.declare_parameter("initial_scan_yaw_tolerance_rad", 0.12)
        self.declare_parameter("initial_scan_max_tilt_rad", 0.35)
        self.declare_parameter("initial_scan_max_tilt_variance_rad2", 0.5)
        self.declare_parameter("initial_scan_max_speed_mps", 0.35)
        self.declare_parameter("initial_scan_max_hover_yaw_rate_radps", 0.12)
        self.declare_parameter("initial_scan_max_position_error_m", 0.5)
        self.declare_parameter("initial_scan_max_horizontal_setpoint_speed_mps", 0.3)
        self.declare_parameter("initial_scan_max_sensor_age_s", 2.0)
        self.declare_parameter("initial_scan_min_valid_lidar_ranges", 10)
        self.declare_parameter("initial_scan_min_known_grid_cells", 10)
        self.declare_parameter("initial_scan_timeout_s", 90.0)
        self.declare_parameter("return_home_height_m", 5.0)
        self.declare_parameter("target_acceptance_m", 0.45)
        self.declare_parameter("yaw_acceptance_rad", 0.25)
        self.declare_parameter("cruise_yaw_enu_rad", 0.0)
        self.declare_parameter("max_horizontal_setpoint_speed_mps", 2.0)
        self.declare_parameter("max_vertical_setpoint_speed_mps", 0.8)
        self.declare_parameter("max_yaw_rate_radps", 0.6)
        self.declare_parameter("avoidance_startup_gate_enabled", True)
        self.declare_parameter("avoidance_startup_min_height_m", 2.0)
        self.declare_parameter("avoidance_startup_max_tilt_rad", 0.35)
        self.declare_parameter("replan_interval_s", 2.0)
        self.declare_parameter("safety_stop_enabled", True)
        self.declare_parameter("safety_stop_distance_m", 1.2)
        self.declare_parameter("safety_stop_width_m", 0.5)
        self.declare_parameter("safety_stop_treat_unknown_as_blocked", False)
        self.declare_parameter("target_class_name", "gas_cylinders")
        self.declare_parameter("min_target_confidence", 0.35)
        self.declare_parameter("target_memory_s", 8.0)
        self.declare_parameter("lock_target_after_acquisition", True)
        self.declare_parameter("explore_max_duration_s", 300.0)
        self.declare_parameter("explore_min_replan_interval_s", 1.0)
        self.declare_parameter("explore_max_map_age_s", 3.0)
        self.declare_parameter("explore_min_x_m", -24.0)
        self.declare_parameter("explore_max_x_m", 34.0)
        self.declare_parameter("explore_min_y_m", -9.0)
        self.declare_parameter("explore_max_y_m", 49.0)
        self.declare_parameter("explore_min_z_m", 3.5)
        self.declare_parameter("explore_max_z_m", 7.0)
        self.declare_parameter("explore_clearance_radius_m", 0.35)
        self.declare_parameter("explore_max_search_radius_m", 10.0)
        self.declare_parameter("explore_sensor_range_m", 8.0)
        self.declare_parameter("explore_local_path_length_m", 2.0)
        self.declare_parameter("explore_viewpoint_dwell_s", 0.4)
        self.declare_parameter("explore_cruise_speed_mps", 1.0)
        self.declare_parameter("explore_return_speed_mps", 1.0)
        self.declare_parameter("explore_return_reserve_s", 30.0)
        self.declare_parameter("explore_max_position_variance_m2", 1.0)
        self.declare_parameter("explore_localization_variance_growth_m2_per_m", 0.01)
        self.declare_parameter("explore_min_battery_fraction", 0.20)
        self.declare_parameter("explore_battery_max_age_s", 3.0)
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
        previous_yaw = self.current_yaw_enu
        previous_odom_ns = self.last_odom_stamp_ns
        self.current_yaw_enu = quaternion_xyzw_to_yaw(self.current_orientation_xyzw)
        velocity = msg.twist.twist.linear
        self.current_speed_mps = math.sqrt(
            velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
        )
        now_ns = self.get_clock().now().nanoseconds
        dt_s = (now_ns - previous_odom_ns) * 1e-9
        self.current_yaw_rate_radps = (
            abs(wrap_pi(self.current_yaw_enu - previous_yaw)) / dt_s
            if 0.0 < dt_s < 1.0
            else math.inf
        )
        self.last_odom_stamp_ns = now_ns
        self.odom_position_variance_m2 = max(
            self._positive_or_inf(msg.pose.covariance[0]),
            self._positive_or_inf(msg.pose.covariance[7]),
        )
        self.odom_height_variance_m2 = self._positive_or_inf(msg.pose.covariance[14])
        self.odom_tilt_variance_rad2 = max(
            self._positive_or_inf(msg.pose.covariance[21]),
            self._positive_or_inf(msg.pose.covariance[28]),
        )

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _failsafe_flags_callback(self, msg: FailsafeFlags) -> None:
        self.failsafe_flags = msg

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
        self.grid = data.astype(np.int8)
        self.grid_known_cells = int(np.count_nonzero(self.grid >= 0))
        self.grid_info = GridInfo(
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
        )
        self.grid_stamp_ns = self.get_clock().now().nanoseconds

    def _map_fusion_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            depth_count = int(payload["depth_fused_count"])
            lidar_count = int(payload["lidar_fused_count"])
            depth_stamp_ns = int(payload.get("depth_last_sample_stamp_ns") or 0)
            lidar_stamp_ns = int(payload.get("lidar_last_sample_stamp_ns") or 0)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.get_logger().warn("invalid mapper fusion status", throttle_duration_sec=5.0)
            return
        if depth_count < 0 or lidar_count < 0:
            return
        restarted = (
            self.map_fusion_status_received
            and (
                depth_count < self.map_fusion_depth_count
                or lidar_count < self.map_fusion_lidar_count
            )
        )
        receipt_ns = self.get_clock().now().nanoseconds
        if depth_count > self.map_fusion_depth_count or (restarted and depth_count > 0):
            self.map_fusion_depth_received_ns = receipt_ns
        if lidar_count > self.map_fusion_lidar_count or (restarted and lidar_count > 0):
            self.map_fusion_lidar_received_ns = receipt_ns
        self.map_fusion_depth_count = depth_count
        self.map_fusion_lidar_count = lidar_count
        self.map_fusion_depth_sample_stamp_ns = depth_stamp_ns
        self.map_fusion_lidar_sample_stamp_ns = lidar_stamp_ns
        self.map_fusion_status_received = True
        if self.phase == PHASE_HOVER_SCAN:
            if restarted or self.initial_scan_depth_baseline is None:
                self.initial_scan_depth_baseline = depth_count
            if restarted or self.initial_scan_lidar_baseline is None:
                self.initial_scan_lidar_baseline = lidar_count
            if restarted:
                self.initial_scan_step_depth_baseline = depth_count
                self.initial_scan_step_lidar_baseline = lidar_count
        if restarted:
            self.map_snapshot_complete = False
            self.map_revision = -1
            self.free_voxel_keys.clear()
            self.occupied_voxel_keys.clear()
        self.pending_voxel_status = payload
        if (not payload.get("observed_voxels_complete", False)
                or payload.get("map_stamp_ns") != self.map_snapshot_stamp_ns):
            self.map_snapshot_complete = False
        self._try_commit_voxel_snapshot()

    def _observed_voxels_callback(self, msg: PointCloud2) -> None:
        self.pending_voxel_cloud = msg
        self._try_commit_voxel_snapshot()

    def _try_commit_voxel_snapshot(self) -> None:
        cloud = self.pending_voxel_cloud
        status = self.pending_voxel_status
        if cloud is None or status is None:
            return
        stamp_ns = int(cloud.header.stamp.sec) * 1_000_000_000 + int(cloud.header.stamp.nanosec)
        try:
            if stamp_ns != int(status["map_stamp_ns"]):
                return
            if cloud.header.frame_id != str(self.get_parameter("map_frame").value):
                return
            if not bool(status["observed_voxels_complete"]) or not bool(status["mapping_enabled"]):
                self.map_snapshot_complete = False
                return
            resolution = float(status["voxel_resolution_m"])
            revision = int(status["map_revision"])
            expected = int(status["observed_voxel_count"])
            if not math.isfinite(resolution) or resolution <= 0 or revision < self.map_revision:
                return
            if expected < 0 or expected != int(status["observed_voxels_point_count"]):
                return
            fields = {field.name: field for field in cloud.fields}
            for name in ("x", "y", "z", "state"):
                field = fields.get(name)
                if field is None or field.datatype != PointField.FLOAT32 or field.count != 1:
                    return
            rows = point_cloud2.read_points_numpy(
                cloud, field_names=("x", "y", "z", "state"), skip_nans=False
            )
            if rows.shape != (expected, 4) or not np.all(np.isfinite(rows)):
                return
            states = rows[:, 3]
            if not np.all(np.isin(states, (-1.0, 0.0, 1.0))):
                return
            keys = np.floor(rows[:, :3] / resolution).astype(np.int64)
            observed = set(map(tuple, keys))
            if len(observed) != expected:
                return
            free = set(map(tuple, keys[states == 0.0]))
            occupied = set(map(tuple, keys[states == 1.0]))
            if len(free) != int(status["free_voxel_count"]):
                return
            if len(occupied) != int(status["occupied_voxel_count"]):
                return
            if expected - len(free) - len(occupied) != int(status["uncertain_voxel_count"]):
                return
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.get_logger().warn(f"invalid observed voxel snapshot: {exc}", throttle_duration_sec=5.0)
            return
        if free != self.free_voxel_keys or occupied != self.occupied_voxel_keys:
            self.map_observation_revision += 1
        self.free_voxel_keys = free
        self.occupied_voxel_keys = occupied
        self.voxel_resolution_m = resolution
        self.map_revision = revision
        self.map_snapshot_stamp_ns = stamp_ns
        self.map_snapshot_received_ns = self.get_clock().now().nanoseconds
        self.map_snapshot_complete = True
        self.pending_voxel_cloud = None
        self.pending_voxel_status = None

    def _battery_status_callback(self, msg: BatteryStatus) -> None:
        self.battery_remaining_fraction = float(msg.remaining)
        self.battery_time_remaining_s = float(msg.time_remaining_s)
        self.battery_connected = bool(msg.connected)
        self.battery_stamp_ns = self.get_clock().now().nanoseconds

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

    def _laser_scan_callback(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=float)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        minimum = max(1, int(self.get_parameter("initial_scan_min_valid_lidar_ranges").value))
        if int(np.count_nonzero(valid)) >= minimum:
            self.last_valid_lidar_stamp_ns = self.get_clock().now().nanoseconds

    def _depth_camera_info_callback(self, msg: CameraInfo) -> None:
        self.depth_camera_info = msg

    def _rgb_callback(self, msg: Image) -> None:
        self.latest_rgb_msg = msg

    def _yolo_detections_callback(self, msg: String) -> None:
        if self.current_position_enu is None or self.latest_depth_m is None:
            return
        if (
            self.phase == PHASE_TARGETING_CYCLE
            and bool(self.get_parameter("lock_target_after_acquisition").value)
            and self._target_is_recent()
        ):
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
        self.goal_waypoint_ids.clear()
        self.target_observation = None
        self.inspection_captured_indices.clear()
        self.inspection_photo_count = 0
        self.landing_command_sent = False
        self.return_reason = ""
        self.command_wait_reason = ""
        self._reset_commanded_setpoint()
        self.last_offboard_request_ns = 0
        self.last_arm_request_ns = 0
        self.last_replan_ns = 0
        self.avoidance_startup_gate_open = not bool(
            self.get_parameter("avoidance_startup_gate_enabled").value
        )
        self.safety_stop_active = False
        self.safety_stop_reason = ""
        self._reset_initial_scan()
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
            if not self._ready_to_initialize_mission():
                self._publish_status()
                return
            self._initialize_mission()

        if self.phase == PHASE_EXPLORE:
            self._prepare_exploration()
        elif self.phase == PHASE_RETURN_HOME:
            self._prepare_return_home()
        if self.phase == PHASE_LANDING or not self.active:
            self._publish_status()
            return
        self._maybe_replan_path()
        nav_target, nav_yaw = self._select_target_and_yaw()
        safety_stop, safety_reason = self._safety_stop_required(nav_target)
        if safety_stop:
            self._set_safety_stop(True, safety_reason)
            hold_target = self.current_position_enu.copy()
            self._reset_commanded_setpoint()
            setpoint = hold_target
            self._publish_position_setpoint(setpoint, self.current_yaw_enu)
            self._maybe_engage_offboard()
            self._publish_status(nav_target, setpoint)
            return
        self._set_safety_stop(False, "")

        setpoint, setpoint_yaw = self._slew_setpoint(nav_target, nav_yaw)
        self._publish_position_setpoint(setpoint, setpoint_yaw)
        self._maybe_engage_offboard()
        self._update_phase(nav_target, nav_yaw)
        self._publish_status(nav_target, setpoint)

    def _initialize_mission(self) -> None:
        if self.current_position_enu is None:
            return
        self.home_position_enu = self.current_position_enu.copy()
        self._reset_commanded_setpoint()
        self.phase = PHASE_SAFE_TAKEOFF
        self.mission_started_ns = self.get_clock().now().nanoseconds
        self.search_started_ns = 0
        self.explore_plan_revision = -1
        self.explore_plan_stamp_ns = 0
        self.return_plan_revision = -1
        self.return_plan_stamp_ns = 0
        if self.explore_future is not None:
            self.explore_future.cancel()
        if self.return_future is not None:
            self.return_future.cancel()
        if self.target_future is not None:
            self.target_future.cancel()
        self.explore_future = None
        self.return_future = None
        self.target_future = None
        self.visited_viewpoint_keys.clear()
        self.active_waypoints = []
        self.goal_waypoint_indices.clear()
        self.goal_waypoint_ids.clear()
        self.waypoint_index = 0
        self._publish_event("mission_initialized", {"home_enu": self._round_list(self.home_position_enu)})

    def _maybe_engage_offboard(self) -> None:
        self.offboard_counter = min(self.offboard_counter + 1, 1000)
        warmup_count = int(self.get_parameter("offboard_warmup_setpoints").value)
        if self.offboard_counter < warmup_count:
            self.command_wait_reason = f"warming up Offboard setpoints {self.offboard_counter}/{warmup_count}"
            self.get_logger().info(self.command_wait_reason, throttle_duration_sec=2.0)
            return

        if not self._localization_is_ready():
            self.command_wait_reason = "waiting for recent independent localization"
            self.get_logger().warn(self.command_wait_reason, throttle_duration_sec=5.0)
            return

        retry_interval_ns = int(float(self.get_parameter("command_retry_interval_s").value) * 1e9)
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and now_ns - self.last_offboard_request_ns >= retry_interval_ns
        ):
            self.last_offboard_request_ns = now_ns
            self.get_logger().info("requesting PX4 Offboard mode", throttle_duration_sec=2.0)
            self._engage_offboard()
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.command_wait_reason = "waiting for PX4 to enter Offboard mode"
            return

        if (
            bool(self.get_parameter("require_preflight_checks").value)
            and not bool(self.vehicle_status.pre_flight_checks_pass)
        ):
            self.command_wait_reason = self._preflight_wait_reason()
            self.get_logger().warn(self.command_wait_reason, throttle_duration_sec=5.0)
            return

        if (
            bool(self.get_parameter("require_accepts_offboard_setpoints").value)
            and not bool(self.vehicle_status.accepts_offboard_setpoints)
        ):
            self.command_wait_reason = "waiting for PX4 to accept Offboard setpoints"
            self.get_logger().warn(self.command_wait_reason, throttle_duration_sec=5.0)
            return

        if (
            bool(self.get_parameter("arm_on_start").value)
            and self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
            and now_ns - self.last_arm_request_ns >= retry_interval_ns
        ):
            self.last_arm_request_ns = now_ns
            self.get_logger().info("requesting PX4 arm", throttle_duration_sec=2.0)
            self._arm()
            return

        self.command_wait_reason = ""

    def _ready_to_initialize_mission(self) -> bool:
        if self.current_position_enu is None:
            self.command_wait_reason = "waiting for localization odometry"
            return False
        if not self._localization_is_ready():
            self.command_wait_reason = "waiting for recent independent localization"
            return False
        if not self._tilt_estimate_is_ready():
            self.command_wait_reason = "waiting for reliable roll/pitch estimate"
            return False
        self.command_wait_reason = ""
        return True

    def _localization_is_ready(self) -> bool:
        if not bool(self.get_parameter("require_localization_for_start").value):
            return self.current_position_enu is not None
        if self.current_position_enu is None:
            return False
        if self._age_s(self.last_odom_stamp_ns) > float(self.get_parameter("max_localization_age_s").value):
            return False
        max_xy = float(self.get_parameter("max_start_position_variance_m2").value)
        max_z = float(self.get_parameter("max_start_height_variance_m2").value)
        return self.odom_position_variance_m2 <= max_xy and self.odom_height_variance_m2 <= max_z

    def _tilt_estimate_is_ready(self) -> bool:
        return self.odom_tilt_variance_rad2 <= float(
            self.get_parameter("initial_scan_max_tilt_variance_rad2").value
        )

    def _exploration_config(self) -> ExplorationConfig:
        return ExplorationConfig(
            clearance_radius_m=float(self.get_parameter("explore_clearance_radius_m").value),
            max_search_radius_m=float(self.get_parameter("explore_max_search_radius_m").value),
            sensor_range_m=float(self.get_parameter("explore_sensor_range_m").value),
            max_local_path_m=float(self.get_parameter("explore_local_path_length_m").value),
            cruise_speed_mps=float(self.get_parameter("explore_cruise_speed_mps").value),
            return_speed_mps=float(self.get_parameter("explore_return_speed_mps").value),
            reserve_s=float(self.get_parameter("explore_return_reserve_s").value),
            max_position_variance_m2=float(
                self.get_parameter("explore_max_position_variance_m2").value
            ),
        )

    def _mission_bounds_keys(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        resolution = self.voxel_resolution_m
        low = tuple(int(math.ceil(float(self.get_parameter(name).value) / resolution - 0.5))
                    for name in ("explore_min_x_m", "explore_min_y_m", "explore_min_z_m"))
        high = tuple(int(math.floor(float(self.get_parameter(name).value) / resolution - 0.5))
                     for name in ("explore_max_x_m", "explore_max_y_m", "explore_max_z_m"))
        return low, high

    def _map_is_fresh(self) -> bool:
        max_age_s = float(self.get_parameter("explore_max_map_age_s").value)
        age_s = self._age_s(self.map_snapshot_received_ns)
        depth_age_s = self._age_s(self.map_fusion_depth_received_ns)
        lidar_age_s = self._age_s(self.map_fusion_lidar_received_ns)
        return (
            self.map_snapshot_complete
            and self.voxel_resolution_m > 0.0
            and bool(self.free_voxel_keys)
            and 0.0 <= age_s <= max_age_s
            and 0.0 <= depth_age_s <= max_age_s
            and 0.0 <= lidar_age_s <= max_age_s
        )

    def _remaining_flight_s(self) -> float:
        limit_s = float(self.get_parameter("explore_max_duration_s").value)
        remaining = max(0.0, limit_s - self._age_s(self.mission_started_ns))
        battery_age = self._age_s(self.battery_stamp_ns)
        if (self.battery_connected
                and 0.0 <= battery_age <= float(self.get_parameter("explore_battery_max_age_s").value)):
            if math.isfinite(self.battery_time_remaining_s) and self.battery_time_remaining_s > 0:
                remaining = min(remaining, self.battery_time_remaining_s)
        return remaining

    def _battery_is_low(self) -> bool:
        battery_age = self._age_s(self.battery_stamp_ns)
        return (
            self.battery_connected
            and 0.0 <= battery_age <= float(self.get_parameter("explore_battery_max_age_s").value)
            and math.isfinite(self.battery_remaining_fraction)
            and self.battery_remaining_fraction >= 0.0
            and self.battery_remaining_fraction
            <= float(self.get_parameter("explore_min_battery_fraction").value)
        )

    def _home_air_point(self) -> np.ndarray | None:
        if self.home_position_enu is None:
            return None
        goal = self.home_position_enu.copy()
        goal[2] = float(self.get_parameter("return_home_height_m").value)
        return goal

    def _safe_route(self, start: np.ndarray, goal: np.ndarray) -> list[np.ndarray]:
        return plan_known_free_path(
            self.free_voxel_keys,
            self.occupied_voxel_keys,
            self.voxel_resolution_m,
            self._mission_bounds_keys(),
            start,
            goal,
            config=self._exploration_config(),
        )

    @staticmethod
    def _route_length_m(path: list[np.ndarray]) -> float:
        return sum(float(np.linalg.norm(end - start)) for start, end in zip(path, path[1:]))

    def _remaining_local_path_is_safe(self) -> bool:
        if not self._map_is_fresh() or self.current_position_enu is None:
            return False
        path = [self.current_position_enu, *self.active_waypoints[self.waypoint_index:]]
        return path_is_known_free(
            self.free_voxel_keys,
            self.occupied_voxel_keys,
            self.voxel_resolution_m,
            self._mission_bounds_keys(),
            path,
            config=self._exploration_config(),
        )

    def _set_local_route(self, path: list[np.ndarray], max_distance_m: float) -> None:
        """Publish only the next short, already verified segment of a 3D route."""
        if self.current_position_enu is None or not path:
            self.active_waypoints = []
            return
        start = self.current_position_enu
        points: list[np.ndarray] = []
        travelled = 0.0
        previous = start
        for point in path[1:]:
            length = float(np.linalg.norm(point - previous))
            if length < 1e-6:
                continue
            if travelled + length > max_distance_m:
                fraction = max(0.0, (max_distance_m - travelled) / length)
                if fraction > 1e-6:
                    points.append(previous + (point - previous) * fraction)
                break
            points.append(point.copy())
            travelled += length
            previous = point
        self._reset_commanded_setpoint()
        self.active_waypoints = points
        self.waypoint_index = 0
        self.explore_viewpoint_entered_ns = 0
        self.goal_waypoint_indices.clear()
        self.goal_waypoint_ids.clear()
        self._publish_planned_path()

    def _prepare_exploration(self) -> None:
        if self.current_position_enu is None:
            return
        if self._target_is_recent():
            self._enter_targeting_cycle()
            return
        if not self._map_is_fresh():
            self.active_waypoints = []
            self.explore_wait_reason = "waiting for complete, fresh 3D voxel snapshot"
            return
        if not self._localization_is_ready() or (
            self.odom_position_variance_m2
            > float(self.get_parameter("explore_max_position_variance_m2").value)
        ):
            self.active_waypoints = []
            self.explore_wait_reason = "waiting for reliable exploration localization"
            return
        home = self._home_air_point()
        if home is None:
            self.active_waypoints = []
            self.explore_wait_reason = "home position unavailable"
            return
        config = self._exploration_config()
        remaining_s = self._remaining_flight_s()
        if self._battery_is_low() or remaining_s <= config.reserve_s:
            self._enter_return_home("return reserve reached")
            return
        path_blocked = bool(self.active_waypoints and not self._remaining_local_path_is_safe())
        if path_blocked:
            self.active_waypoints = []
            self.explore_wait_reason = "local route blocked; replanning"
            self._publish_event("explore_path_blocked", {"map_revision": self.map_revision})
        if self.active_waypoints and self.explore_viewpoint_entered_ns and not path_blocked:
            return

        if self.explore_future is not None:
            if not self.explore_future.done():
                if not self.active_waypoints:
                    self.explore_wait_reason = "planning next safe frontier viewpoint"
                return
            try:
                plan = self.explore_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"3D frontier planning failed: {exc}", throttle_duration_sec=5.0)
                plan = None
            self.explore_future = None
            if self.explore_job_revision != self.map_observation_revision:
                self.explore_wait_reason = "new voxel observations arrived during planning"
                self.explore_plan_stamp_ns = 0
                return
            self.explore_plan_revision = self.map_observation_revision
            if plan is None:
                self.active_waypoints = []
                self.explore_wait_reason = "no reachable frontier viewpoint with return reserve"
                self._enter_return_home("exploration finished or no safe frontier")
                return
            if (remaining_s <= plan.return_time_s + config.reserve_s
                    or not path_is_known_free(
                        self.free_voxel_keys, self.occupied_voxel_keys,
                        self.voxel_resolution_m, self._mission_bounds_keys(),
                        [self.current_position_enu, *plan.local_path_xyz[1:]], config=config,
                    )):
                self.active_waypoints = []
                self.explore_wait_reason = "frontier route or return reserve changed; replanning"
                self.explore_plan_stamp_ns = 0
                return
            self.explore_goal_enu = plan.goal_xyz.copy()
            self.explore_goal_yaw_enu = float(plan.goal_yaw_rad)
            self.explore_score = float(plan.score)
            self._set_local_route(plan.local_path_xyz, config.max_local_path_m)
            self.explore_wait_reason = ""
            self._publish_event("explore_viewpoint_selected", {
                "goal_enu": self._round_list(plan.goal_xyz),
                "frontier_key": list(plan.frontier_key),
                "visible_unknown_voxels": plan.visible_unknown_count,
                "surface_gain": round(plan.surface_gain, 2),
                "outbound_time_s": round(plan.outbound_time_s, 2),
                "return_time_s": round(plan.return_time_s, 2),
                "clearance_m": round(plan.clearance_m, 2),
                "localization_variance_m2": round(plan.localization_variance_m2, 3),
                "return_margin_s": round(plan.reserve_margin_s, 2),
                "score": round(plan.score, 3),
                "local_path_points": len(self.active_waypoints),
            })
            return

        now_ns = self.get_clock().now().nanoseconds
        new_observation = self.map_observation_revision != self.explore_plan_revision
        interval_ns = int(float(self.get_parameter("explore_min_replan_interval_s").value) * 1e9)
        interval_elapsed = now_ns - self.explore_plan_stamp_ns >= interval_ns
        if self.active_waypoints and (not new_observation or not interval_elapsed):
            return
        if not interval_elapsed and self.explore_plan_stamp_ns and not path_blocked:
            return
        self.explore_job_revision = self.map_observation_revision
        self.explore_plan_stamp_ns = now_ns
        self.explore_wait_reason = "planning next safe frontier viewpoint" if not self.active_waypoints else ""
        self.explore_future = self.planning_executor.submit(
            plan_frontier,
            frozenset(self.free_voxel_keys),
            frozenset(self.occupied_voxel_keys),
            self.voxel_resolution_m,
            self._mission_bounds_keys(),
            self.current_position_enu.copy(),
            home.copy(),
            remaining_flight_s=remaining_s,
            position_variance_m2=float(self.odom_position_variance_m2),
            surface_gain_by_key=self._surface_gain_hints(),
            localization_variance_by_key=DistanceVarianceEstimate(
                origin_xyz=tuple(float(value) for value in self.current_position_enu),
                resolution_m=self.voxel_resolution_m,
                base_variance_m2=float(self.odom_position_variance_m2),
                growth_m2_per_m=max(0.0, float(self.get_parameter(
                    "explore_localization_variance_growth_m2_per_m"
                ).value)),
            ),
            visited_goal_keys=frozenset(self.visited_viewpoint_keys),
            config=config,
        )

    def _surface_gain_hints(self) -> dict[tuple[int, int, int], float]:
        """Unfinished target photos supply semantic gain when a target is known."""
        if self.target_observation is None or self.voxel_resolution_m <= 0:
            return {}
        target = self.target_observation.position_enu
        remaining = max(0, int(self.get_parameter("inspection_required_photos").value)
                        - len(self.inspection_captured_indices))
        if remaining == 0:
            return {}
        key = tuple(np.floor(target / self.voxel_resolution_m).astype(int).tolist())
        return {key: float(remaining)}

    def _prepare_return_home(self) -> None:
        if self.current_position_enu is None or self.return_goal_enu is None:
            return
        if self._has_reached(self.return_goal_enu):
            self._enter_landing(self.return_reason or "return home complete")
            return
        if not self._map_is_fresh():
            self.active_waypoints = []
            self.explore_wait_reason = "waiting for fresh 3D map to return"
            return
        if self.active_waypoints and not self._remaining_local_path_is_safe():
            self.active_waypoints = []
            self._publish_event("return_path_blocked", {"map_revision": self.map_revision})

        if self.return_future is not None:
            if not self.return_future.done():
                if not self.active_waypoints:
                    self.explore_wait_reason = "planning known-free 3D return path"
                return
            try:
                path = self.return_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"3D return planning failed: {exc}", throttle_duration_sec=5.0)
                path = []
            self.return_future = None
            self.return_plan_revision = self.map_observation_revision
            if not path or not path_is_known_free(
                self.free_voxel_keys, self.occupied_voxel_keys,
                self.voxel_resolution_m, self._mission_bounds_keys(),
                [self.current_position_enu, *path[1:]], config=self._exploration_config(),
            ):
                self.active_waypoints = []
                self.explore_wait_reason = "no known-free 3D return path; holding"
                if self.return_job_revision != self.map_observation_revision:
                    self.return_plan_stamp_ns = 0
                self._publish_event("return_path_unavailable", {"map_revision": self.map_revision})
                return
            self._set_local_route(path, self._exploration_config().max_local_path_m)
            self.explore_wait_reason = ""
            return

        if self.active_waypoints and self.map_observation_revision == self.return_plan_revision:
            return
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(float(self.get_parameter("explore_min_replan_interval_s").value) * 1e9)
        if now_ns - self.return_plan_stamp_ns < interval_ns:
            return
        self.return_job_revision = self.map_observation_revision
        self.return_plan_stamp_ns = now_ns
        self.return_future = self.planning_executor.submit(
            plan_known_free_path,
            frozenset(self.free_voxel_keys),
            frozenset(self.occupied_voxel_keys),
            self.voxel_resolution_m,
            self._mission_bounds_keys(),
            self.current_position_enu.copy(),
            self.return_goal_enu.copy(),
            self._exploration_config(),
        )
        if not self.active_waypoints:
            self.explore_wait_reason = "planning known-free 3D return path"

    def _preflight_wait_reason(self) -> str:
        reasons: list[str] = []
        if bool(self.failsafe_flags.local_position_invalid):
            reasons.append("local position invalid")
        if bool(self.failsafe_flags.global_position_invalid):
            reasons.append("global position invalid")
        if bool(self.failsafe_flags.home_position_invalid):
            reasons.append("home position invalid")
        if bool(self.failsafe_flags.manual_control_signal_lost):
            reasons.append("manual control signal lost")
        if bool(self.failsafe_flags.remote_id_unhealthy):
            reasons.append("remote ID unhealthy")
        if not reasons:
            reasons.append("PX4 preflight checks not passed")
        return "waiting for PX4 preflight checks: " + ", ".join(reasons)

    def _maybe_replan_path(self) -> None:
        if self.phase != PHASE_TARGETING_CYCLE:
            return
        if not self._map_is_fresh():
            self.active_waypoints = []
            return
        if self.target_future is not None:
            if not self.target_future.done():
                return
            try:
                planned, goal_indices, goal_ids = self.target_future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"3D inspection planning failed: {exc}", throttle_duration_sec=5.0)
                planned, goal_indices, goal_ids = [], set(), {}
            self.target_future = None
            if not planned:
                if self.target_job_revision != self.map_observation_revision:
                    self.last_replan_ns = 0
                    return
                self._enter_return_home("no known-free inspection viewpoint")
                return
            if not path_is_known_free(
                self.free_voxel_keys, self.occupied_voxel_keys,
                self.voxel_resolution_m, self._mission_bounds_keys(),
                [self.current_position_enu, *planned], config=self._exploration_config(),
            ):
                self.active_waypoints = []
                self.last_replan_ns = 0
                return
            previous_count = len(self.active_waypoints)
            self._reset_commanded_setpoint()
            self.active_waypoints = planned
            self.goal_waypoint_indices = goal_indices
            self.goal_waypoint_ids = goal_ids
            self.waypoint_index = 0
            self.target_plan_revision = self.map_observation_revision
            self._publish_planned_path()
            self._publish_event("path_replanned", {
                "previous_waypoints": previous_count,
                "new_waypoints": len(planned),
                "remaining_goals": len(goal_indices),
            })
            return

        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(float(self.get_parameter("replan_interval_s").value) * 1e9)
        if (self.active_waypoints
                and self.target_plan_revision == self.map_observation_revision
                and self._remaining_local_path_is_safe()):
            return
        if self.active_waypoints and now_ns - self.last_replan_ns < interval_ns:
            return
        goals, goal_ids = self._remaining_navigation_goals()
        if not goals:
            self._enter_return_home("inspection route complete")
            return
        self._set_navigation_goals(goals, goal_ids)

    def _remaining_navigation_goals(self) -> tuple[list[np.ndarray], list[int]]:
        if not self.active_waypoints:
            pairs = zip(self.navigation_goals, self.navigation_goal_ids)
            return (
                [goal.copy() for goal, goal_id in pairs
                 if goal_id not in self.inspection_captured_indices],
                [goal_id for goal_id in self.navigation_goal_ids
                 if goal_id not in self.inspection_captured_indices],
            )
        goals: list[np.ndarray] = []
        goal_ids: list[int] = []
        for index in sorted(self.goal_waypoint_indices):
            if index < self.waypoint_index or index >= len(self.active_waypoints):
                continue
            goal_id = self.goal_waypoint_ids.get(index, index)
            if goal_id in self.inspection_captured_indices:
                continue
            goals.append(self.active_waypoints[index].copy())
            goal_ids.append(goal_id)
        return goals, goal_ids

    def _safety_stop_required(self, target_enu: np.ndarray) -> tuple[bool, str]:
        if self.phase not in (PHASE_EXPLORE, PHASE_TARGETING_CYCLE, PHASE_RETURN_HOME):
            return False, ""
        if not self._map_is_fresh():
            return True, "3D observed voxel map missing or stale"
        if not self.active_waypoints:
            return True, self.explore_wait_reason or "waiting for known-free route"
        if not self._remaining_local_path_is_safe():
            return True, "local 3D route entered unknown or occupied space"
        if self.phase != PHASE_TARGETING_CYCLE:
            return False, ""
        if not bool(self.get_parameter("safety_stop_enabled").value):
            return False, ""
        if not self._avoidance_is_enabled():
            return False, ""
        if self.current_position_enu is None or self.grid is None or self.grid_info is None:
            return False, ""

        delta = target_enu[:2] - self.current_position_enu[:2]
        distance = float(np.linalg.norm(delta))
        if distance < 1e-3:
            return False, ""

        direction = delta / distance
        check_distance = min(distance, float(self.get_parameter("safety_stop_distance_m").value))
        if check_distance <= 0.0:
            return False, ""

        step = max(self.grid_info.resolution * 0.5, 0.05)
        half_width = max(0.0, float(self.get_parameter("safety_stop_width_m").value) * 0.5)
        radius_cells = int(math.ceil(half_width / max(self.grid_info.resolution, 1e-6)))
        treat_unknown = bool(self.get_parameter("safety_stop_treat_unknown_as_blocked").value)
        samples = max(1, int(math.ceil(check_distance / step)))
        for index in range(1, samples + 1):
            sample_distance = min(check_distance, index * step)
            sample_xy = self.current_position_enu[:2] + direction * sample_distance
            center = world_to_grid(float(sample_xy[0]), float(sample_xy[1]), self.grid_info)
            blocked, reason = self._grid_neighborhood_blocked(center, radius_cells, treat_unknown)
            if blocked:
                return True, f"{reason} at {sample_distance:.2f}m"
        return False, ""

    def _grid_neighborhood_blocked(
        self,
        center: tuple[int, int],
        radius_cells: int,
        treat_unknown: bool,
    ) -> tuple[bool, str]:
        if self.grid is None or self.grid_info is None:
            return False, ""
        cx, cy = center
        radius_sq = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                x = cx + dx
                y = cy + dy
                if x < 0 or x >= self.grid_info.width or y < 0 or y >= self.grid_info.height:
                    if treat_unknown:
                        return True, "unknown grid edge"
                    continue
                value = int(self.grid[y, x])
                if value >= 50:
                    return True, "occupied obstacle"
                if value < 0 and treat_unknown:
                    return True, "unknown space"
        return False, ""

    def _set_safety_stop(self, active: bool, reason: str) -> None:
        if active == self.safety_stop_active and reason == self.safety_stop_reason:
            return
        self.safety_stop_active = active
        self.safety_stop_reason = reason
        self._publish_event(
            "safety_stop" if active else "safety_clear",
            {"active": active, "reason": reason},
        )

    def _avoidance_is_enabled(self) -> bool:
        if self.avoidance_startup_gate_open:
            return True
        if not bool(self.get_parameter("avoidance_startup_gate_enabled").value):
            self.avoidance_startup_gate_open = True
            return True
        if self.current_position_enu is None:
            return False

        min_height = float(self.get_parameter("avoidance_startup_min_height_m").value)
        max_tilt = float(self.get_parameter("avoidance_startup_max_tilt_rad").value)
        roll, pitch = quaternion_xyzw_to_roll_pitch(self.current_orientation_xyzw)
        tilt = max(abs(roll), abs(pitch))
        ready = self.current_position_enu[2] >= min_height and tilt <= max_tilt
        if ready:
            self.avoidance_startup_gate_open = True
            self.get_logger().info(
                "avoidance startup gate opened: height=%.2fm tilt=%.2frad"
                % (self.current_position_enu[2], tilt)
            )
            return True

        self.get_logger().info(
            "avoidance startup gate waiting: height=%.2fm min=%.2fm tilt=%.2frad max=%.2frad"
            % (self.current_position_enu[2], min_height, tilt, max_tilt),
            throttle_duration_sec=2.0,
        )
        return False

    def _select_target_and_yaw(self) -> tuple[np.ndarray, float]:
        if self.current_position_enu is None:
            return np.zeros(3, dtype=float), 0.0

        if self.phase == PHASE_SAFE_TAKEOFF:
            home = self.home_position_enu if self.home_position_enu is not None else self.current_position_enu
            target = home.copy()
            target[2] = float(self.get_parameter("takeoff_height_m").value)
            return target, float(self.get_parameter("cruise_yaw_enu_rad").value)

        if self.phase == PHASE_HOVER_SCAN:
            anchor = self.initial_scan_anchor_enu
            if anchor is None:
                return self.current_position_enu.copy(), self.current_yaw_enu
            pose_ready, _ = self._initial_scan_pose_ready(hover=False)
            data_ready, _ = self._initial_scan_data_ready()
            if pose_ready and data_ready and self.initial_scan_sequence:
                if self.initial_scan_rotating:
                    return anchor.copy(), self.initial_scan_sequence.target_yaw()
                if self.initial_scan_sequence.step_index == 0:
                    return anchor.copy(), self.initial_scan_sequence.start_yaw_rad
            return anchor.copy(), self.current_yaw_enu

        if self.phase in (PHASE_EXPLORE, PHASE_RETURN_HOME, PHASE_TARGETING_CYCLE):
            target = self._current_nav_target()
            yaw = float(self.get_parameter("cruise_yaw_enu_rad").value)
            if self.phase == PHASE_TARGETING_CYCLE and self.target_observation is not None:
                yaw = self._yaw_to_face(self.target_observation.position_enu)
            elif self.phase == PHASE_EXPLORE:
                yaw = self.explore_goal_yaw_enu
            elif self.phase == PHASE_RETURN_HOME:
                yaw = self._yaw_to_face(target)
            return target, yaw

        return self.current_position_enu.copy(), self.current_yaw_enu

    def _update_phase(self, target: np.ndarray, yaw: float) -> None:
        if self.current_position_enu is None:
            return

        if self.phase == PHASE_SAFE_TAKEOFF:
            if self._has_reached(target, tolerance=0.35):
                self._enter_initial_scan(target)
            return

        if self.phase == PHASE_HOVER_SCAN:
            self._update_initial_scan()
            return

        if self.phase == PHASE_EXPLORE:
            if not self.active_waypoints or not self._has_reached(target):
                return
            if self.waypoint_index < len(self.active_waypoints) - 1:
                self.waypoint_index += 1
                return
            if self.explore_goal_enu is not None and self._has_reached(self.explore_goal_enu):
                if self.explore_viewpoint_entered_ns == 0:
                    self.explore_viewpoint_entered_ns = self.get_clock().now().nanoseconds
                    self.explore_viewpoint_depth_baseline = self.map_fusion_depth_count
                    return
                yaw_error = abs(wrap_pi(self.explore_goal_yaw_enu - self.current_yaw_enu))
                dwell_s = self._age_s(self.explore_viewpoint_entered_ns)
                if (yaw_error > float(self.get_parameter("yaw_acceptance_rad").value)
                        or dwell_s < float(self.get_parameter("explore_viewpoint_dwell_s").value)
                        or self.map_fusion_depth_count <= self.explore_viewpoint_depth_baseline):
                    return
                key = tuple(np.floor(self.explore_goal_enu / self.voxel_resolution_m)
                            .astype(int).tolist())
                self.visited_viewpoint_keys.add(key)
            self.active_waypoints = []
            self.waypoint_index = 0
            self.explore_viewpoint_entered_ns = 0
            return

        if self.phase == PHASE_TARGETING_CYCLE:
            self._update_targeting_cycle(target, yaw)
            return

        if self.phase == PHASE_RETURN_HOME and self.active_waypoints:
            if self._advance_path_if_reached(target):
                self.active_waypoints = []
                self.waypoint_index = 0

    def _reset_initial_scan(self) -> None:
        self.initial_scan_anchor_enu = None
        self.initial_scan_sequence = None
        self.initial_scan_entered_ns = 0
        self.initial_scan_hover_since_ns = 0
        self.initial_scan_rotating = False
        self.initial_scan_wait_reason = ""
        self.initial_scan_depth_baseline = None
        self.initial_scan_lidar_baseline = None
        self.initial_scan_step_depth_baseline = None
        self.initial_scan_step_lidar_baseline = None

    def _enter_initial_scan(self, takeoff_target: np.ndarray) -> None:
        self.phase = PHASE_HOVER_SCAN
        self.initial_scan_anchor_enu = takeoff_target.copy()
        self.initial_scan_sequence = YawScanSequence(
            start_yaw_rad=self.current_yaw_enu,
            steps=max(4, int(self.get_parameter("initial_scan_yaw_steps").value)),
            dwell_s=max(0.0, float(self.get_parameter("initial_scan_yaw_dwell_s").value)),
        )
        self.initial_scan_entered_ns = self.get_clock().now().nanoseconds
        self.initial_scan_hover_since_ns = 0
        self.initial_scan_rotating = False
        if self.map_fusion_status_received:
            self.initial_scan_depth_baseline = self.map_fusion_depth_count
            self.initial_scan_lidar_baseline = self.map_fusion_lidar_count
        self.initial_scan_wait_reason = "waiting for stable hover and first lidar/depth map"
        self._publish_event(
            "phase_changed",
            {
                "phase": self.phase,
                "anchor_enu": self._round_list(self.initial_scan_anchor_enu),
                "yaw_steps": self.initial_scan_sequence.steps,
            },
        )

    def _initial_scan_pose_ready(self, hover: bool) -> tuple[bool, str]:
        if self.current_position_enu is None or self.initial_scan_anchor_enu is None:
            return False, "waiting for takeoff anchor"
        if not self._localization_is_ready():
            return False, "waiting for recent independent localization"
        if not self._tilt_estimate_is_ready():
            return False, "waiting for reliable roll/pitch estimate"
        position_error = float(np.linalg.norm(
            self.current_position_enu - self.initial_scan_anchor_enu
        ))
        if position_error > float(self.get_parameter("initial_scan_max_position_error_m").value):
            return False, "waiting for position to settle at scan anchor"
        roll, pitch = quaternion_xyzw_to_roll_pitch(self.current_orientation_xyzw)
        if max(abs(roll), abs(pitch)) > float(self.get_parameter("initial_scan_max_tilt_rad").value):
            return False, "waiting for level attitude"
        if self.current_speed_mps > float(self.get_parameter("initial_scan_max_speed_mps").value):
            return False, "waiting for low translation speed"
        if (
            hover
            and self.current_yaw_rate_radps
            > float(self.get_parameter("initial_scan_max_hover_yaw_rate_radps").value)
        ):
            return False, "waiting for low yaw rate"
        return True, ""

    def _initial_scan_data_ready(self) -> tuple[bool, str]:
        max_age_s = max(0.1, float(self.get_parameter("initial_scan_max_sensor_age_s").value))
        if (
            self.last_valid_lidar_stamp_ns < self.initial_scan_entered_ns
            or self._age_s(self.last_valid_lidar_stamp_ns) > max_age_s
        ):
            return False, "waiting for valid horizontal lidar scan"
        if (
            self.latest_depth_stamp_ns < self.initial_scan_entered_ns
            or self._age_s(self.latest_depth_stamp_ns) > max_age_s
        ):
            return False, "waiting for forward depth image"
        if (
            self.initial_scan_depth_baseline is None
            or self.initial_scan_lidar_baseline is None
        ):
            return False, "waiting for mapper fusion status"
        if self.map_fusion_lidar_count <= self.initial_scan_lidar_baseline:
            return False, "waiting for horizontal lidar data to be fused into map"
        if self.map_fusion_depth_count <= self.initial_scan_depth_baseline:
            return False, "waiting for forward depth data to be fused into map"
        if (
            self.grid_stamp_ns < self.initial_scan_entered_ns
            or self._age_s(self.grid_stamp_ns) > max_age_s
        ):
            return False, "waiting for fresh occupancy grid"
        minimum = int(self.get_parameter("initial_scan_min_known_grid_cells").value)
        if not has_observed_cells(self.grid, minimum):
            return False, "occupancy grid still has no actual observations"
        if self.grid_info is None or self.initial_scan_anchor_enu is None:
            return False, "waiting for scan anchor in occupancy grid"
        gx, gy = world_to_grid(
            float(self.initial_scan_anchor_enu[0]),
            float(self.initial_scan_anchor_enu[1]),
            self.grid_info,
        )
        if (
            gx < 0 or gx >= self.grid_info.width
            or gy < 0 or gy >= self.grid_info.height
            or int(self.grid[gy, gx]) != 0
        ):
            return False, "scan anchor is not confirmed free in occupancy grid"
        return True, ""

    def _update_initial_scan(self) -> None:
        if self.initial_scan_sequence is None:
            return
        timeout_s = float(self.get_parameter("initial_scan_timeout_s").value)
        if timeout_s > 0.0 and self._age_s(self.initial_scan_entered_ns) > timeout_s:
            self._enter_landing("initial scan timeout: " + self.initial_scan_wait_reason)
            return

        pose_ready, pose_reason = self._initial_scan_pose_ready(hover=not self.initial_scan_rotating)
        data_ready, data_reason = self._initial_scan_data_ready()
        if not pose_ready or not data_ready:
            self.initial_scan_wait_reason = pose_reason or data_reason
            self.initial_scan_hover_since_ns = 0
            self.initial_scan_sequence.update(
                self.current_yaw_enu,
                self.get_clock().now().nanoseconds,
                ready=False,
                tolerance_rad=0.0,
            )
            if self.initial_scan_rotating:
                self.initial_scan_rotating = False
                self._publish_event("initial_scan_paused", {"reason": self.initial_scan_wait_reason})
            return

        now_ns = self.get_clock().now().nanoseconds
        if not self.initial_scan_rotating:
            if self.initial_scan_hover_since_ns == 0:
                self.initial_scan_hover_since_ns = now_ns
            if (
                now_ns - self.initial_scan_hover_since_ns
            ) * 1e-9 < float(self.get_parameter("initial_scan_hover_s").value):
                self.initial_scan_wait_reason = "holding stable hover for lidar contour"
                return
            self.initial_scan_rotating = True
            self.initial_scan_step_depth_baseline = self.map_fusion_depth_count
            self.initial_scan_step_lidar_baseline = self.map_fusion_lidar_count
            self._publish_event(
                "initial_scan_rotation_started",
                {"step": self.initial_scan_sequence.step_index},
            )

        step_has_new_fusion = (
            self.initial_scan_step_depth_baseline is not None
            and self.initial_scan_step_lidar_baseline is not None
            and self.map_fusion_depth_count > self.initial_scan_step_depth_baseline
            and self.map_fusion_lidar_count > self.initial_scan_step_lidar_baseline
        )
        if not step_has_new_fusion:
            self.initial_scan_wait_reason = "waiting for depth and lidar fusion at current heading"
        else:
            self.initial_scan_wait_reason = ""
        previous_step = self.initial_scan_sequence.step_index
        complete = self.initial_scan_sequence.update(
            self.current_yaw_enu,
            now_ns,
            ready=step_has_new_fusion,
            tolerance_rad=max(0.01, float(
                self.get_parameter("initial_scan_yaw_tolerance_rad").value
            )),
        )
        if self.initial_scan_sequence.step_index > previous_step:
            self.initial_scan_step_depth_baseline = self.map_fusion_depth_count
            self.initial_scan_step_lidar_baseline = self.map_fusion_lidar_count
        if complete:
            self._publish_event(
                "initial_scan_complete",
                {"known_grid_cells": self.grid_known_cells},
            )
            self._enter_explore()

    def _enter_explore(self) -> None:
        self.phase = PHASE_EXPLORE
        self.search_started_ns = self.get_clock().now().nanoseconds
        self.active_waypoints = []
        self.navigation_goals = []
        self.navigation_goal_ids = []
        self.goal_waypoint_indices.clear()
        self.goal_waypoint_ids.clear()
        self.waypoint_index = 0
        self.explore_plan_revision = -1
        self.explore_plan_stamp_ns = 0
        self.explore_future = None
        self.explore_goal_enu = None
        self.explore_viewpoint_entered_ns = 0
        self.explore_wait_reason = "waiting for complete observed 3D voxel map"
        self._publish_event("phase_changed", {"phase": self.phase})

    def _enter_targeting_cycle(self) -> None:
        if self.target_observation is None:
            return
        self.phase = PHASE_TARGETING_CYCLE
        self.active_waypoints = []
        self.waypoint_index = 0
        self.goal_waypoint_indices.clear()
        self.goal_waypoint_ids.clear()
        self.target_plan_revision = -1
        self.inspection_captured_indices.clear()
        self.inspection_photo_count = 0
        goals = self._inspection_goals(self.target_observation.position_enu)
        self._set_navigation_goals(goals)
        self._publish_event(
            "target_acquired",
            {
                "target_class": self.target_observation.class_name,
                "target_enu": self._round_list(self.target_observation.position_enu),
                "inspection_waypoints": len(goals),
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
        goal_id = self.goal_waypoint_ids.get(self.waypoint_index, self.waypoint_index)
        if goal_id not in self.inspection_captured_indices:
            if not self._capture_inspection_photo(self.waypoint_index, goal_id):
                return
            self.inspection_captured_indices.add(goal_id)
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
        goal = self._home_air_point()
        self.return_goal_enu = goal.copy() if goal is not None else None
        self.active_waypoints = []
        self.waypoint_index = 0
        self.return_plan_revision = -1
        self.return_plan_stamp_ns = 0
        if self.explore_future is not None:
            self.explore_future.cancel()
        if self.target_future is not None:
            self.target_future.cancel()
        self.explore_future = None
        self.target_future = None
        self.return_future = None
        self.explore_wait_reason = "planning known-free 3D return path"
        self._publish_event("return_home", {
            "reason": reason,
            "home_enu": self._round_list(goal) if goal is not None else None,
        })

    def _enter_landing(self, reason: str) -> None:
        self.phase = PHASE_LANDING
        self.active = False
        self.return_reason = reason
        self.safety_stop_active = False
        self.safety_stop_reason = ""
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

    def _set_navigation_goals(self, goals: list[np.ndarray], goal_ids: list[int] | None = None) -> None:
        self.navigation_goals = [goal.copy() for goal in goals]
        self.navigation_goal_ids = list(goal_ids) if goal_ids is not None else list(range(len(goals)))
        if self.current_position_enu is None or not self._map_is_fresh():
            self.active_waypoints = []
            return
        self.target_job_revision = self.map_observation_revision
        self.last_replan_ns = self.get_clock().now().nanoseconds
        self.target_future = self.planning_executor.submit(
            _plan_safe_goal_sequence,
            frozenset(self.free_voxel_keys),
            frozenset(self.occupied_voxel_keys),
            self.voxel_resolution_m,
            self._mission_bounds_keys(),
            self.current_position_enu.copy(),
            [goal.copy() for goal in goals],
            list(self.navigation_goal_ids),
            self._exploration_config(),
        )

    def _publish_planned_path(self) -> None:
        msg = NavPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        for waypoint in self.active_waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.planned_path_pub.publish(msg)

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

    def _capture_inspection_photo(self, waypoint_index: int, goal_id: int | None = None) -> bool:
        event: dict[str, Any] = {
            "waypoint_index": waypoint_index,
            "photo_index": self.inspection_photo_count + 1,
        }
        if goal_id is not None:
            event["goal_id"] = goal_id
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

    def _slew_setpoint(self, target_enu: np.ndarray, yaw_enu: float) -> tuple[np.ndarray, float]:
        if self.current_position_enu is None:
            return target_enu, yaw_enu
        now_ns = self.get_clock().now().nanoseconds
        if self.commanded_setpoint_enu is None or self.last_setpoint_update_ns <= 0:
            self.commanded_setpoint_enu = self.current_position_enu.copy()
            self.commanded_yaw_enu = self.current_yaw_enu
            self.last_setpoint_update_ns = now_ns

        dt_s = (now_ns - self.last_setpoint_update_ns) * 1e-9
        if dt_s <= 0.0 or dt_s > 1.0:
            dt_s = 0.1
        self.last_setpoint_update_ns = now_ns

        setpoint = self.commanded_setpoint_enu.copy()
        xy_delta = target_enu[:2] - setpoint[:2]
        xy_distance = float(np.linalg.norm(xy_delta))
        xy_speed_parameter = (
            "initial_scan_max_horizontal_setpoint_speed_mps"
            if self.phase == PHASE_HOVER_SCAN
            else "max_horizontal_setpoint_speed_mps"
        )
        max_xy_speed = float(self.get_parameter(xy_speed_parameter).value)
        if self.phase == PHASE_HOVER_SCAN:
            max_xy_speed = max(0.01, max_xy_speed)
        max_xy_step = max_xy_speed * dt_s
        if max_xy_step > 0.0 and xy_distance > max_xy_step:
            setpoint[:2] += xy_delta / xy_distance * max_xy_step
        else:
            setpoint[:2] = target_enu[:2]

        z_delta = float(target_enu[2] - setpoint[2])
        max_z_step = float(self.get_parameter("max_vertical_setpoint_speed_mps").value) * dt_s
        if max_z_step > 0.0 and abs(z_delta) > max_z_step:
            setpoint[2] += math.copysign(max_z_step, z_delta)
        else:
            setpoint[2] = float(target_enu[2])

        yaw_delta = wrap_pi(yaw_enu - self.commanded_yaw_enu)
        yaw_rate_parameter = (
            "initial_scan_yaw_rate_radps"
            if self.phase == PHASE_HOVER_SCAN
            else "max_yaw_rate_radps"
        )
        max_yaw_rate = float(self.get_parameter(yaw_rate_parameter).value)
        if self.phase == PHASE_HOVER_SCAN:
            max_yaw_rate = max(0.01, max_yaw_rate)
        max_yaw_step = max_yaw_rate * dt_s
        if max_yaw_step > 0.0 and abs(yaw_delta) > max_yaw_step:
            self.commanded_yaw_enu = wrap_pi(
                self.commanded_yaw_enu + math.copysign(max_yaw_step, yaw_delta)
            )
        else:
            self.commanded_yaw_enu = wrap_pi(yaw_enu)

        self.commanded_setpoint_enu = setpoint
        return setpoint.copy(), self.commanded_yaw_enu

    def _reset_commanded_setpoint(self) -> None:
        if self.current_position_enu is None:
            self.commanded_setpoint_enu = None
            self.last_setpoint_update_ns = 0
            return
        self.commanded_setpoint_enu = self.current_position_enu.copy()
        self.commanded_yaw_enu = self.current_yaw_enu
        self.last_setpoint_update_ns = self.get_clock().now().nanoseconds

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

    def _publish_status(self, target: np.ndarray | None = None, setpoint: np.ndarray | None = None) -> None:
        odom_age_s = self._age_s(self.last_odom_stamp_ns)
        payload: dict[str, Any] = {
            "phase": self.phase,
            "active": self.active,
            "waypoint_index": self.waypoint_index,
            "waypoint_count": len(self.active_waypoints),
            "nav_state": int(self.vehicle_status.nav_state),
            "arming_state": int(self.vehicle_status.arming_state),
            "pre_flight_checks_pass": bool(self.vehicle_status.pre_flight_checks_pass),
            "accepts_offboard_setpoints": bool(self.vehicle_status.accepts_offboard_setpoints),
            "localization_ready": self._localization_is_ready(),
            "failsafe_flags": {
                "local_position_invalid": bool(self.failsafe_flags.local_position_invalid),
                "global_position_invalid": bool(self.failsafe_flags.global_position_invalid),
                "home_position_invalid": bool(self.failsafe_flags.home_position_invalid),
                "manual_control_signal_lost": bool(self.failsafe_flags.manual_control_signal_lost),
                "offboard_control_signal_lost": bool(self.failsafe_flags.offboard_control_signal_lost),
                "remote_id_unhealthy": bool(self.failsafe_flags.remote_id_unhealthy),
                "mode_req_local_position": int(self.failsafe_flags.mode_req_local_position),
                "mode_req_global_position": int(self.failsafe_flags.mode_req_global_position),
                "mode_req_home_position": int(self.failsafe_flags.mode_req_home_position),
                "mode_req_prevent_arming": int(self.failsafe_flags.mode_req_prevent_arming),
            },
            "odom_age_s": round(odom_age_s, 2) if math.isfinite(odom_age_s) else None,
            "odom_position_variance_m2": round(float(self.odom_position_variance_m2), 3)
            if math.isfinite(self.odom_position_variance_m2)
            else None,
            "odom_height_variance_m2": round(float(self.odom_height_variance_m2), 3)
            if math.isfinite(self.odom_height_variance_m2)
            else None,
            "odom_tilt_variance_rad2": round(float(self.odom_tilt_variance_rad2), 3)
            if math.isfinite(self.odom_tilt_variance_rad2)
            else None,
            "offboard_setpoints_sent": self.offboard_counter,
            "command_wait_reason": self.command_wait_reason,
            "photos": self.inspection_photo_count,
            "safety_stop_active": self.safety_stop_active,
            "safety_stop_reason": self.safety_stop_reason,
            "avoidance_startup_gate_open": self.avoidance_startup_gate_open,
        }
        grid_age_s = self._age_s(self.grid_stamp_ns)
        payload["occupancy_grid_age_s"] = round(grid_age_s, 2) if math.isfinite(grid_age_s) else None
        payload["occupancy_grid_known_cells"] = self.grid_known_cells
        payload["map_fusion_depth_count"] = self.map_fusion_depth_count
        payload["map_fusion_lidar_count"] = self.map_fusion_lidar_count
        payload["map_revision"] = self.map_revision
        payload["map_observation_revision"] = self.map_observation_revision
        payload["observed_voxels_complete"] = self.map_snapshot_complete
        payload["known_free_voxels"] = len(self.free_voxel_keys)
        payload["known_occupied_voxels"] = len(self.occupied_voxel_keys)
        map_age_s = self._age_s(self.map_snapshot_received_ns)
        payload["observed_voxels_age_s"] = round(map_age_s, 2) if math.isfinite(map_age_s) else None
        payload["battery_connected"] = self.battery_connected
        payload["battery_remaining_fraction"] = (
            round(self.battery_remaining_fraction, 3)
            if math.isfinite(self.battery_remaining_fraction) else None
        )
        payload["battery_time_remaining_s"] = (
            round(self.battery_time_remaining_s, 1)
            if math.isfinite(self.battery_time_remaining_s) and self.battery_time_remaining_s > 0
            else None
        )
        if self.phase == PHASE_HOVER_SCAN:
            payload["initial_scan_rotating"] = self.initial_scan_rotating
            payload["initial_scan_wait_reason"] = self.initial_scan_wait_reason
            if self.initial_scan_sequence is not None:
                payload["initial_scan_step"] = self.initial_scan_sequence.step_index
                payload["initial_scan_steps"] = self.initial_scan_sequence.steps
        if self.phase in (PHASE_EXPLORE, PHASE_RETURN_HOME):
            payload["explore_wait_reason"] = self.explore_wait_reason
            payload["remaining_flight_s"] = round(self._remaining_flight_s(), 1)
        if self.phase == PHASE_EXPLORE:
            payload["explore_age_s"] = round(self._age_s(self.search_started_ns), 2)
            payload["explore_score"] = round(self.explore_score, 3)
            if self.explore_goal_enu is not None:
                payload["explore_goal_enu"] = self._round_list(self.explore_goal_enu)
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
            payload["nav_target_enu"] = self._round_list(target)
        if setpoint is not None:
            payload["setpoint_enu"] = self._round_list(setpoint)
        elif self.commanded_setpoint_enu is not None:
            payload["setpoint_enu"] = self._round_list(self.commanded_setpoint_enu)
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
        self._publish_planned_path()
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_event(self, event: str, fields: dict[str, Any]) -> None:
        payload = {"event": event, "phase": self.phase, **fields}
        self.inspection_event_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _age_s(self, stamp_ns: int) -> float:
        if stamp_ns <= 0:
            return math.inf
        return (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9

    @staticmethod
    def _positive_or_inf(value: float) -> float:
        return float(value) if math.isfinite(value) and value > 0.0 else math.inf

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
