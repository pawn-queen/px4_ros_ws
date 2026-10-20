"""Depth/radar/lidar based voxel mapping for the powerplant world."""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import json
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from px4_powerplant_mission.common.frames import quaternion_xyzw_to_roll_pitch, rotate_vector_xyzw
from px4_powerplant_mission.common.qos import reliable_qos_profile, sensor_qos_profile
from px4_powerplant_mission.mapping.voxel_grid import ObservedVoxelSnapshot, VoxelGrid


_NS_PER_SECOND = 1_000_000_000
_POSE_HISTORY_NS = 2 * _NS_PER_SECOND
_MAX_POSE_GAP_NS = 250_000_000
_MAX_SENSOR_LEAD_NS = 2 * _NS_PER_SECOND
_MAX_PENDING_MEASUREMENTS = 32
_CAMERA_INFO_FUTURE_NS = 50_000_000


def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * _NS_PER_SECOND + int(stamp.nanosec)


@dataclass(frozen=True)
class _StampedPose:
    stamp_ns: int
    position: np.ndarray
    orientation_xyzw: np.ndarray


def _interpolate_pose(before: _StampedPose, after: _StampedPose, stamp_ns: int) -> _StampedPose:
    fraction = (stamp_ns - before.stamp_ns) / (after.stamp_ns - before.stamp_ns)
    position = before.position + fraction * (after.position - before.position)
    q0 = before.orientation_xyzw
    q1 = after.orientation_xyzw
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        quaternion = q0 + fraction * (q1 - q0)
    else:
        angle = math.acos(dot)
        denominator = math.sin(angle)
        quaternion = (
            math.sin((1.0 - fraction) * angle) * q0
            + math.sin(fraction * angle) * q1
        ) / denominator
    quaternion /= np.linalg.norm(quaternion)
    return _StampedPose(stamp_ns, position, quaternion)


class _PoseHistory:
    """Bounded odometry history; never extrapolate a sensor measurement."""

    def __init__(self) -> None:
        self.samples: list[_StampedPose] = []

    def add(self, sample: _StampedPose) -> bool:
        """Return true when simulation time jumped backwards and history was reset."""
        jumped = bool(self.samples and sample.stamp_ns < self.samples[-1].stamp_ns - _MAX_POSE_GAP_NS)
        if jumped:
            self.samples.clear()
        stamps = [pose.stamp_ns for pose in self.samples]
        index = bisect_left(stamps, sample.stamp_ns)
        if index < len(self.samples) and self.samples[index].stamp_ns == sample.stamp_ns:
            self.samples[index] = sample
        else:
            self.samples.insert(index, sample)
        cutoff = self.samples[-1].stamp_ns - _POSE_HISTORY_NS
        while self.samples and self.samples[0].stamp_ns < cutoff:
            self.samples.pop(0)
        return jumped

    def at(self, stamp_ns: int) -> _StampedPose | None:
        if not self.samples or stamp_ns < self.samples[0].stamp_ns:
            return None
        stamps = [pose.stamp_ns for pose in self.samples]
        index = bisect_left(stamps, stamp_ns)
        if index < len(self.samples) and self.samples[index].stamp_ns == stamp_ns:
            return self.samples[index]
        if index == 0 or index == len(self.samples):
            return None
        before, after = self.samples[index - 1], self.samples[index]
        if after.stamp_ns - before.stamp_ns > _MAX_POSE_GAP_NS:
            return None
        return _interpolate_pose(before, after, stamp_ns)


@dataclass(frozen=True)
class _PendingMeasurement:
    kind: str
    msg: Image | LaserScan | PointCloud2
    first_stamp_ns: int
    last_stamp_ns: int


class VoxelMapperNode(Node):
    """Build a lightweight occupied voxel map from Gazebo depth/lidar topics."""

    def __init__(self) -> None:
        super().__init__("powerplant_voxel_mapper")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.grid = VoxelGrid(
            float(self.get_parameter("voxel_resolution_m").value),
            hit_log_odds=float(self.get_parameter("hit_log_odds").value),
            miss_log_odds=float(self.get_parameter("miss_log_odds").value),
            occupied_threshold=float(self.get_parameter("occupied_log_odds_threshold").value),
            free_threshold=float(self.get_parameter("free_log_odds_threshold").value),
        )
        self.camera_info_history: deque[CameraInfo] = deque(maxlen=64)
        self.pose_history = _PoseHistory()
        self.pending_measurements: deque[_PendingMeasurement] = deque()
        self.last_selected_depth_stamp_ns: int | None = None
        self.last_selected_scan_stamp_ns: int | None = None
        self.position = np.zeros(3, dtype=float)
        self.orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.have_pose = False
        self.startup_gate_open = not bool(self.get_parameter("startup_gate_enabled").value)
        self.mapping_start_stamp_ns: int | None = None
        self.depth_fused_count = 0
        self.lidar_fused_count = 0
        self.depth_last_sample_stamp_ns = 0
        self.lidar_last_sample_stamp_ns = 0

        self.voxel_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("occupied_voxels_topic").value,
            reliable_qos_profile(),
        )
        self.observed_voxel_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("observed_voxels_topic").value,
            reliable_qos_profile(),
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("occupancy_grid_topic").value,
            reliable_qos_profile(),
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("voxel_marker_topic").value,
            reliable_qos_profile(),
        )
        self.fusion_status_pub = self.create_publisher(
            String,
            self.get_parameter("fusion_status_topic").value,
            reliable_qos_profile(),
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("localized_odom_topic").value,
            self._odom_callback,
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
            self._camera_info_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("laser_scan_topic").value,
            self._laser_scan_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud_topic").value,
            self._pointcloud_callback,
            sensor_qos_profile(),
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish_map)
        self.get_logger().info("voxel mapper node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        self.declare_parameter("depth_camera_info_topic", "/depth_camera/camera_info")
        self.declare_parameter("laser_scan_topic", "/scan")
        self.declare_parameter("pointcloud_topic", "/points")
        self.declare_parameter("occupied_voxels_topic", "/powerplant/map/occupied_voxels")
        self.declare_parameter("observed_voxels_topic", "/powerplant/map/observed_voxels")
        self.declare_parameter("occupancy_grid_topic", "/powerplant/map/occupancy_grid")
        self.declare_parameter("voxel_marker_topic", "/powerplant/map/voxel_markers")
        self.declare_parameter("fusion_status_topic", "/powerplant/map/fusion_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("voxel_resolution_m", 0.25)
        self.declare_parameter("hit_log_odds", 0.85)
        self.declare_parameter("miss_log_odds", -0.4)
        self.declare_parameter("occupied_log_odds_threshold", 0.0)
        self.declare_parameter("free_log_odds_threshold", -0.2)
        self.declare_parameter("ray_clear_enabled", True)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("depth_stride_px", 8)
        self.declare_parameter("depth_fusion_rate_hz", 3.0)
        self.declare_parameter("lidar_fusion_rate_hz", 3.0)
        self.declare_parameter("max_depth_range_m", 18.0)
        self.declare_parameter("min_depth_range_m", 0.25)
        self.declare_parameter("camera_fx", 430.0)
        self.declare_parameter("camera_fy", 430.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        self.declare_parameter("camera_translation_flu", [0.12, -0.03, 0.24])
        self.declare_parameter("lidar_translation_flu", [0.0, 0.0, 0.0])
        self.declare_parameter("pointcloud_frame", "base_link")
        self.declare_parameter("max_publish_points", 60000)
        self.declare_parameter("grid_z_min_m", 0.3)
        self.declare_parameter("grid_z_max_m", 6.0)
        self.declare_parameter("use_flight_height_grid_band", True)
        self.declare_parameter("grid_z_band_below_m", 0.8)
        self.declare_parameter("grid_z_band_above_m", 0.8)
        self.declare_parameter("startup_gate_enabled", True)
        self.declare_parameter("startup_gate_min_height_m", 2.0)
        self.declare_parameter("startup_gate_max_tilt_rad", 0.35)
        self.declare_parameter("max_orientation_variance_rad2", 0.5)
        self.declare_parameter("startup_gate_clear_on_enable", True)
        self.declare_parameter("use_fixed_grid_bounds", False)
        self.declare_parameter("grid_min_x_m", -25.0)
        self.declare_parameter("grid_max_x_m", 35.0)
        self.declare_parameter("grid_min_y_m", -10.0)
        self.declare_parameter("grid_max_y_m", 50.0)

    def _odom_callback(self, msg: Odometry) -> None:
        stamp_ns = _stamp_ns(msg.header.stamp)
        if stamp_ns <= 0:
            self.get_logger().warn("discarding unstamped odometry", throttle_duration_sec=2.0)
            return
        if msg.header.frame_id and msg.header.frame_id != str(self.get_parameter("map_frame").value):
            self.get_logger().warn("odometry frame differs from map frame; no TF transform configured",
                                   throttle_duration_sec=2.0)
            return
        if msg.child_frame_id and msg.child_frame_id != str(self.get_parameter("base_frame").value):
            self.get_logger().warn("odometry child frame differs from base frame; no TF transform configured",
                                   throttle_duration_sec=2.0)
            return

        position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        orientation = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        norm = float(np.linalg.norm(orientation))
        if not np.all(np.isfinite(position)) or not np.isfinite(norm) or norm < 1e-9:
            self.get_logger().warn("discarding invalid odometry pose", throttle_duration_sec=2.0)
            return
        orientation /= norm
        max_orientation_variance = float(self.get_parameter("max_orientation_variance_rad2").value)
        roll_variance = float(msg.pose.covariance[21])
        pitch_variance = float(msg.pose.covariance[28])
        if (
            not np.isfinite(roll_variance)
            or not np.isfinite(pitch_variance)
            or roll_variance < 0.0
            or pitch_variance < 0.0
            or roll_variance > max_orientation_variance
            or pitch_variance > max_orientation_variance
        ):
            self.get_logger().warn(
                "discarding odometry with uncertain roll/pitch covariance",
                throttle_duration_sec=2.0,
            )
            return
        jumped = self.pose_history.add(_StampedPose(stamp_ns, position, orientation))
        if jumped:
            self.pending_measurements.clear()
            self.camera_info_history.clear()
            self.last_selected_depth_stamp_ns = None
            self.last_selected_scan_stamp_ns = None
            self._clear_map()
            self.startup_gate_open = not bool(self.get_parameter("startup_gate_enabled").value)
            self.mapping_start_stamp_ns = None
            self.get_logger().warn("odometry clock jumped backwards; mapping buffers and map cleared")
        latest = self.pose_history.samples[-1]
        self.position = latest.position.copy()
        self.orientation_xyzw = latest.orientation_xyzw.copy()
        self.have_pose = True
        self._drain_pending_measurements()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info_history.append(msg)
        self._drain_pending_measurements()

    def _depth_callback(self, msg: Image) -> None:
        self._queue_measurement("depth", msg)

    def _laser_scan_callback(self, msg: LaserScan) -> None:
        # LaserScan.header.stamp is the first beam. scan_time is the interval
        # between scans, so it is not a substitute for a missing time_increment.
        if not math.isfinite(msg.time_increment) or msg.time_increment < 0.0:
            self.get_logger().warn("discarding scan with invalid time_increment",
                                   throttle_duration_sec=2.0)
            return
        increment_ns = round(msg.time_increment * _NS_PER_SECOND)
        last_offset_ns = max(0, len(msg.ranges) - 1) * increment_ns
        self._queue_measurement("scan", msg, last_offset_ns)

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        self._queue_measurement("cloud", msg)

    def _queue_measurement(
        self,
        kind: str,
        msg: Image | LaserScan | PointCloud2,
        last_offset_ns: int = 0,
    ) -> None:
        if not self._mapping_is_enabled():
            return
        first_stamp_ns = _stamp_ns(msg.header.stamp)
        if first_stamp_ns <= 0:
            self.get_logger().warn(f"discarding unstamped {kind} measurement",
                                   throttle_duration_sec=2.0)
            return
        if self.mapping_start_stamp_ns is not None and first_stamp_ns < self.mapping_start_stamp_ns:
            return
        if self.pose_history.samples:
            oldest = self.pose_history.samples[0].stamp_ns
            newest = self.pose_history.samples[-1].stamp_ns
            if first_stamp_ns < oldest or first_stamp_ns + last_offset_ns > newest + _MAX_SENSOR_LEAD_NS:
                self.get_logger().warn(
                    f"discarding {kind}: sensor stamp outside odometry time history "
                    "(check /clock and use_sim_time)",
                    throttle_duration_sec=2.0,
                )
                return
        if kind in ("depth", "scan"):
            rate_param = "depth_fusion_rate_hz" if kind == "depth" else "lidar_fusion_rate_hz"
            previous_stamp_ns = (
                self.last_selected_depth_stamp_ns if kind == "depth"
                else self.last_selected_scan_stamp_ns
            )
            rate_hz = float(self.get_parameter(rate_param).value)
            if previous_stamp_ns is not None and rate_hz > 0.0:
                min_interval_ns = round(_NS_PER_SECOND / rate_hz)
                if first_stamp_ns - previous_stamp_ns < min_interval_ns:
                    return
            if kind == "depth":
                self.last_selected_depth_stamp_ns = first_stamp_ns
            else:
                self.last_selected_scan_stamp_ns = first_stamp_ns
        self.pending_measurements.append(
            _PendingMeasurement(kind, msg, first_stamp_ns, first_stamp_ns + last_offset_ns)
        )
        if len(self.pending_measurements) > _MAX_PENDING_MEASUREMENTS:
            self.pending_measurements.popleft()
            self.get_logger().warn("mapping sensor queue full; dropping oldest measurement",
                                   throttle_duration_sec=2.0)
        self._drain_pending_measurements()

    def _drain_pending_measurements(self) -> None:
        if not self.pose_history.samples:
            return
        oldest = self.pose_history.samples[0].stamp_ns
        newest = self.pose_history.samples[-1].stamp_ns
        waiting: deque[_PendingMeasurement] = deque()
        while self.pending_measurements:
            measurement = self.pending_measurements.popleft()
            if measurement.first_stamp_ns < oldest:
                self.get_logger().warn("discarding sensor sample without an earlier odometry pose",
                                       throttle_duration_sec=2.0)
                continue
            if measurement.last_stamp_ns > newest:
                if measurement.first_stamp_ns <= newest + _MAX_SENSOR_LEAD_NS:
                    waiting.append(measurement)
                else:
                    self.get_logger().warn("discarding sensor sample too far ahead of odometry",
                                           throttle_duration_sec=2.0)
                continue
            pose = self.pose_history.at(measurement.first_stamp_ns)
            if pose is None:
                self.get_logger().warn("discarding sensor sample across an odometry gap",
                                       throttle_duration_sec=2.0)
                continue
            if measurement.kind == "depth":
                # Give a matching CameraInfo frame time to arrive. Intrinsics
                # fall back to configured values if no info follows soon.
                if (
                    self._matching_camera_info(measurement.msg) is None
                    and newest < measurement.first_stamp_ns + _CAMERA_INFO_FUTURE_NS
                ):
                    waiting.append(measurement)
                    continue
                if self._insert_depth(measurement.msg, pose) > 0:
                    self.depth_fused_count += 1
                    self.depth_last_sample_stamp_ns = measurement.first_stamp_ns
            elif measurement.kind == "scan":
                if self._insert_scan(measurement.msg, measurement.first_stamp_ns) > 0:
                    self.lidar_fused_count += 1
                    self.lidar_last_sample_stamp_ns = measurement.last_stamp_ns
            else:
                self._insert_pointcloud(measurement.msg, pose)
        self.pending_measurements = waiting

    def _matching_camera_info(self, image: Image) -> CameraInfo | None:
        image_stamp_ns = _stamp_ns(image.header.stamp)
        matching = []
        for info in self.camera_info_history:
            if info.width != image.width or info.height != image.height:
                continue
            if info.header.frame_id != image.header.frame_id:
                continue
            if not np.isfinite(info.k[0]) or not np.isfinite(info.k[4]):
                continue
            if info.k[0] <= 0.0 or info.k[4] <= 0.0:
                continue
            info_stamp_ns = _stamp_ns(info.header.stamp)
            if info_stamp_ns <= image_stamp_ns + _CAMERA_INFO_FUTURE_NS:
                matching.append(info)
        if not matching:
            return None
        # CameraInfo describes static calibration. Prefer the latest sample at
        # or before the image; tolerate a small bridge delivery skew.
        preceding = [info for info in matching if _stamp_ns(info.header.stamp) <= image_stamp_ns]
        if preceding:
            return max(preceding, key=lambda info: _stamp_ns(info.header.stamp))
        return min(matching, key=lambda info: _stamp_ns(info.header.stamp))

    def _insert_depth(self, msg: Image, pose: _StampedPose) -> int:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"depth image conversion failed: {exc}")
            return 0
        depth = np.asarray(image, dtype=np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if msg.encoding in ("16UC1", "mono16"):
            depth = depth * 0.001
        info = self._matching_camera_info(msg)
        if info is None and self.camera_info_history:
            self.get_logger().warn(
                "discarding depth image: CameraInfo frame, resolution, or stamp does not match",
                throttle_duration_sec=2.0,
            )
            return 0
        points = self._depth_to_map_points(depth, pose, info)
        if bool(self.get_parameter("ray_clear_enabled").value):
            origin = self._body_to_map(
                np.asarray(self.get_parameter("camera_translation_flu").value, dtype=float),
                pose,
            )
            return self._insert_rays_with_origins(np.broadcast_to(origin, points.shape), points)
        return self.grid.insert_points(points)

    def _depth_to_map_points(
        self,
        depth: np.ndarray,
        pose: _StampedPose,
        camera_info: CameraInfo | None,
    ) -> np.ndarray:
        height, width = depth.shape[:2]
        stride = int(self.get_parameter("depth_stride_px").value)
        min_range = float(self.get_parameter("min_depth_range_m").value)
        max_range = float(self.get_parameter("max_depth_range_m").value)

        if camera_info is not None:
            fx, fy = camera_info.k[0], camera_info.k[4]
            cx, cy = camera_info.k[2], camera_info.k[5]
        else:
            fx = float(self.get_parameter("camera_fx").value)
            fy = float(self.get_parameter("camera_fy").value)
            cx = float(self.get_parameter("camera_cx").value)
            cy = float(self.get_parameter("camera_cy").value)

        ys = np.arange(0, height, stride)
        xs = np.arange(0, width, stride)
        u, v = np.meshgrid(xs, ys)
        z = depth[v, u]
        mask = np.isfinite(z) & (z >= min_range) & (z <= max_range)
        if not np.any(mask):
            return np.empty((0, 3), dtype=float)

        z = z[mask]
        u = u[mask].astype(float)
        v = v[mask].astype(float)
        optical_x = (u - cx) * z / fx
        optical_y = (v - cy) * z / fy
        optical_z = z

        body = np.stack([optical_z, -optical_x, -optical_y], axis=1)
        body += np.asarray(self.get_parameter("camera_translation_flu").value, dtype=float)
        return body @ self._rotation_matrix_xyzw(pose.orientation_xyzw).T + pose.position

    def _insert_scan(self, msg: LaserScan, first_stamp_ns: int) -> int:
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges), dtype=float) * msg.angle_increment
        mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(mask):
            return 0
        increment_ns = round(msg.time_increment * _NS_PER_SECOND)
        lidar_translation = np.asarray(self.get_parameter("lidar_translation_flu").value, dtype=float)
        origins = []
        points = []
        for index in np.flatnonzero(mask):
            beam_pose = self.pose_history.at(first_stamp_ns + int(index) * increment_ns)
            if beam_pose is None:
                self.get_logger().warn("discarding scan: a beam has no bracketing odometry pose",
                                       throttle_duration_sec=2.0)
                return 0
            angle = angles[index]
            body = np.array(
                [ranges[index] * math.cos(angle), ranges[index] * math.sin(angle), 0.0],
                dtype=float,
            ) + lidar_translation
            origins.append(self._body_to_map(lidar_translation, beam_pose))
            points.append(self._body_to_map(body, beam_pose))
        if bool(self.get_parameter("ray_clear_enabled").value):
            return self._insert_rays_with_origins(np.asarray(origins), np.asarray(points))
        return self.grid.insert_points(points)

    def _insert_rays_with_origins(self, origins: np.ndarray, points: np.ndarray) -> int:
        """Integrate moving sensor rays with the same per-frame hit priority as VoxelGrid."""
        if len(points) == 0:
            return 0
        origins = np.asarray(origins, dtype=float)
        points = np.asarray(points, dtype=float)
        finite = np.all(np.isfinite(origins), axis=1) & np.all(np.isfinite(points), axis=1)
        origins, points = origins[finite], points[finite]
        if len(points) == 0:
            return 0

        resolution = self.grid.resolution
        hit_keys = set(map(tuple, np.floor(points / resolution).astype(np.int64).tolist()))
        free_keys: set[tuple[int, int, int]] = set()
        step_m = max(resolution * 0.5, 1e-3)
        # Chunk the ray samples to keep peak memory bounded, including at 35m.
        for start in range(0, len(points), 256):
            chunk_origins = origins[start:start + 256]
            delta = points[start:start + 256] - chunk_origins
            steps = np.maximum(1, np.ceil(np.linalg.norm(delta, axis=1) / step_m).astype(int))
            indices = np.arange(int(np.max(steps)))[:, None]
            sample = (
                chunk_origins[None, :, :]
                + delta[None, :, :] * (indices / steps[None, :])[:, :, None]
            )
            free = np.floor(sample[indices < steps[None, :]] / resolution).astype(np.int64)
            free_keys.update(map(tuple, np.unique(free, axis=0).tolist()))

        for key in free_keys - hit_keys:
            value = self.grid.cells.get(key, 0.0) + self.grid.miss_log_odds
            self.grid.cells[key] = float(np.clip(value, self.grid.min_log_odds, self.grid.max_log_odds))
        for key in hit_keys:
            value = self.grid.cells.get(key, 0.0) + self.grid.hit_log_odds
            self.grid.cells[key] = float(np.clip(value, self.grid.min_log_odds, self.grid.max_log_odds))
        if hit_keys:
            self.grid.revision += 1
        return len(hit_keys)

    def _insert_pointcloud(self, msg: PointCloud2, pose: _StampedPose) -> None:
        configured_frame = str(self.get_parameter("pointcloud_frame").value)
        frame = msg.header.frame_id or configured_frame
        if frame != configured_frame:
            self.get_logger().warn("point cloud frame differs from configured pointcloud_frame",
                                   throttle_duration_sec=2.0)
            return
        map_frame = str(self.get_parameter("map_frame").value)
        if frame not in (map_frame, str(self.get_parameter("base_frame").value)):
            self.get_logger().warn("point cloud frame requires an unavailable sensor extrinsic",
                                   throttle_duration_sec=2.0)
            return
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            arr = np.array([point[0], point[1], point[2]], dtype=float)
            points.append(arr if frame == map_frame else self._body_to_map(arr, pose))
        self.grid.insert_points(points)

    @staticmethod
    def _rotation_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
        x, y, z, w = quaternion
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    @staticmethod
    def _body_to_map(point_flu: np.ndarray, pose: _StampedPose) -> np.ndarray:
        return pose.position + rotate_vector_xyzw(pose.orientation_xyzw, point_flu)

    def _publish_map(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("map_frame").value)
        if not self._mapping_is_enabled():
            self._publish_fusion_status(header)
            return
        max_points = int(self.get_parameter("max_publish_points").value)
        occupied = self.grid.occupied_points(max_points=max_points)
        cloud = point_cloud2.create_cloud_xyz32(header, occupied.tolist())
        self.voxel_pub.publish(cloud)
        snapshot = self.grid.observed_snapshot(max_points=max_points)
        observed_fields = [
            PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z", "state"))
        ]
        self.observed_voxel_pub.publish(
            point_cloud2.create_cloud(header, observed_fields, snapshot.points)
        )
        self._publish_occupancy_grid(header)
        self._publish_marker(header, occupied)
        # Publish status last so subscribers can pair it with the cloud stamp.
        self._publish_fusion_status(header, snapshot)

    def _clear_map(self) -> None:
        self.grid.cells.clear()
        self.grid.revision += 1
        self.depth_fused_count = 0
        self.lidar_fused_count = 0
        self.depth_last_sample_stamp_ns = 0
        self.lidar_last_sample_stamp_ns = 0

    def _publish_fusion_status(
        self,
        header: Header,
        snapshot: ObservedVoxelSnapshot | None = None,
    ) -> None:
        if snapshot is None:
            snapshot = self.grid.observed_snapshot(
                max_points=int(self.get_parameter("max_publish_points").value)
            )
        status = {
            "map_stamp_ns": _stamp_ns(header.stamp),
            "map_revision": self.grid.revision,
            "mapping_enabled": self.startup_gate_open,
            "voxel_resolution_m": self.grid.resolution,
            "observed_bounds_key": snapshot.bounds_key,
            "observed_voxel_count": snapshot.observed_count,
            "observed_voxels_point_count": len(snapshot.points),
            "free_voxel_count": snapshot.free_count,
            "occupied_voxel_count": snapshot.occupied_count,
            "uncertain_voxel_count": snapshot.uncertain_count,
            "observed_voxels_complete": snapshot.complete and self.startup_gate_open,
            "depth_fused_count": self.depth_fused_count,
            "lidar_fused_count": self.lidar_fused_count,
            "depth_last_sample_stamp_ns": self.depth_last_sample_stamp_ns,
            "lidar_last_sample_stamp_ns": self.lidar_last_sample_stamp_ns,
        }
        self.fusion_status_pub.publish(String(data=json.dumps(status)))

    def _mapping_is_enabled(self) -> bool:
        if self.startup_gate_open:
            return True
        if not bool(self.get_parameter("startup_gate_enabled").value):
            self.startup_gate_open = True
            return True
        if not self.have_pose:
            return False

        min_height = float(self.get_parameter("startup_gate_min_height_m").value)
        max_tilt = float(self.get_parameter("startup_gate_max_tilt_rad").value)
        roll, pitch = quaternion_xyzw_to_roll_pitch(self.orientation_xyzw)
        tilt = max(abs(roll), abs(pitch))
        ready = self.position[2] >= min_height and tilt <= max_tilt
        if not ready:
            self.get_logger().info(
                "mapping startup gate waiting: height=%.2fm min=%.2fm tilt=%.2frad max=%.2frad"
                % (self.position[2], min_height, tilt, max_tilt),
                throttle_duration_sec=2.0,
            )
            return False

        if bool(self.get_parameter("startup_gate_clear_on_enable").value):
            self._clear_map()
        self.startup_gate_open = True
        self.mapping_start_stamp_ns = self.pose_history.samples[-1].stamp_ns
        self.get_logger().info(
            "mapping startup gate opened: height=%.2fm tilt=%.2frad" % (self.position[2], tilt)
        )
        return True

    def _publish_occupancy_grid(self, header: Header) -> None:
        z_min, z_max = self._occupancy_grid_z_bounds()
        result = self.grid.to_occupancy_grid_2d(
            z_min,
            z_max,
            bounds_xy=self._fixed_grid_bounds(),
        )
        if result is None:
            return
        grid_array, origin_xy, resolution = result
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = float(resolution)
        msg.info.width = int(grid_array.shape[1])
        msg.info.height = int(grid_array.shape[0])
        msg.info.origin.position.x = float(origin_xy[0])
        msg.info.origin.position.y = float(origin_xy[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_array.reshape(-1).astype(np.int8).tolist()
        self.grid_pub.publish(msg)

    def _occupancy_grid_z_bounds(self) -> tuple[float, float]:
        configured_min = float(self.get_parameter("grid_z_min_m").value)
        configured_max = float(self.get_parameter("grid_z_max_m").value)
        if not bool(self.get_parameter("use_flight_height_grid_band").value) or not self.have_pose:
            return configured_min, configured_max

        below = max(0.0, float(self.get_parameter("grid_z_band_below_m").value))
        above = max(0.0, float(self.get_parameter("grid_z_band_above_m").value))
        z_min = max(configured_min, float(self.position[2]) - below)
        z_max = min(configured_max, float(self.position[2]) + above)
        if z_max <= z_min:
            return configured_min, configured_max
        return z_min, z_max

    def _fixed_grid_bounds(self) -> tuple[float, float, float, float] | None:
        if not bool(self.get_parameter("use_fixed_grid_bounds").value):
            return None
        return (
            float(self.get_parameter("grid_min_x_m").value),
            float(self.get_parameter("grid_min_y_m").value),
            float(self.get_parameter("grid_max_x_m").value),
            float(self.get_parameter("grid_max_y_m").value),
        )

    def _publish_marker(self, header: Header, occupied: np.ndarray) -> None:
        marker = Marker()
        marker.header = header
        marker.ns = "occupied_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        scale = float(self.get_parameter("voxel_resolution_m").value)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.0
        marker.color.g = 0.55
        marker.color.b = 0.9
        marker.color.a = 0.45
        for point in occupied:
            marker.points.append(Point(x=float(point[0]), y=float(point[1]), z=float(point[2])))
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoxelMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
