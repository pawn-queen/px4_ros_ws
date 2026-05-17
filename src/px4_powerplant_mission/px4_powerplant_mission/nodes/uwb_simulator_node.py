"""Simulation-only UWB ranging and trilateration node."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from sensor_msgs.msg import Range
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from px4_powerplant_mission.common.frames import ned_to_enu, normalize_quaternion_xyzw
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile
from px4_powerplant_mission.localization.uwb import parse_anchor_list, simulate_ranges, trilaterate


class UwbSimulatorNode(Node):
    """Create UWB-like ranges from simulation truth and publish a solved pose."""

    def __init__(self) -> None:
        super().__init__("powerplant_uwb_simulator")
        self._declare_parameters()

        anchor_values = list(self.get_parameter("uwb_anchors").value)
        anchor_names = list(self.get_parameter("uwb_anchor_names").value)
        self.anchors = parse_anchor_list(anchor_values, anchor_names)
        self.rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
        self.gazebo_odom_position_enu: np.ndarray | None = None
        self.gazebo_odom_orientation_xyzw: np.ndarray | None = None
        self.gazebo_odom_stamp_ns = 0
        self.gazebo_pose_position_enu: np.ndarray | None = None
        self.gazebo_pose_orientation_xyzw: np.ndarray | None = None
        self.gazebo_pose_stamp_ns = 0
        self.px4_position_enu: np.ndarray | None = None
        self.px4_stamp_ns = 0
        self.last_solution_enu: np.ndarray | None = None
        self._pose_name_fallback_warned = False

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("uwb_pose_topic").value,
            reliable_qos_profile(),
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("anchor_marker_topic").value,
            reliable_qos_profile(),
        )
        self.range_pubs = [
            self.create_publisher(
                Range,
                f"{self.get_parameter('uwb_range_topic_prefix').value}/{anchor.name}",
                reliable_qos_profile(),
            )
            for anchor in self.anchors
        ]

        self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter("truth_vehicle_local_position_topic").value,
            self._vehicle_local_position_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("truth_odom_topic").value,
            self._truth_odom_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            TFMessage,
            self.get_parameter("truth_pose_topic").value,
            self._truth_pose_callback,
            reliable_qos_profile(),
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._timer_callback)
        self.marker_timer = self.create_timer(1.0, self._publish_anchor_markers)
        self.get_logger().info(f"UWB simulator started with {len(self.anchors)} anchors")

    def _declare_parameters(self) -> None:
        self.declare_parameter("truth_vehicle_local_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("truth_odom_topic", "/model/x500_depth/odometry_with_covariance")
        self.declare_parameter("truth_pose_topic", "/world/powerplant/dynamic_pose/info")
        self.declare_parameter("truth_model_name", "x500_depth_0")
        self.declare_parameter("truth_pose_index", 0)
        self.declare_parameter("truth_source", "gazebo_pose")
        self.declare_parameter("truth_timeout_s", 1.0)
        self.declare_parameter("publish_truth_orientation", True)
        self.declare_parameter("truth_yaw_variance_rad2", 0.02)
        self.declare_parameter("uwb_pose_topic", "/uwb/pose")
        self.declare_parameter("uwb_range_topic_prefix", "/uwb/range")
        self.declare_parameter("anchor_marker_topic", "/uwb/anchors")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("range_noise_std_m", 0.08)
        self.declare_parameter("range_min_m", 0.05)
        self.declare_parameter("range_max_m", 80.0)
        self.declare_parameter("max_solution_rms_m", 0.75)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter(
            "uwb_anchors",
            [-10.0, -10.0, 6.0, 34.0, -10.0, 6.0, 34.0, 34.0, 6.0, -10.0, 34.0, 6.0, 12.0, 12.0, 26.0],
        )
        self.declare_parameter(
            "uwb_anchor_names",
            ["anchor_0", "anchor_1", "anchor_2", "anchor_3", "anchor_4"],
        )

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.px4_position_enu = ned_to_enu([msg.x, msg.y, msg.z])
            self.px4_stamp_ns = self.get_clock().now().nanoseconds

    def _truth_odom_callback(self, msg: Odometry) -> None:
        self.gazebo_odom_position_enu = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        q_xyzw = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        if np.all(np.isfinite(q_xyzw)) and np.linalg.norm(q_xyzw) > 1e-6:
            self.gazebo_odom_orientation_xyzw = normalize_quaternion_xyzw(q_xyzw)
        self.gazebo_odom_stamp_ns = self.get_clock().now().nanoseconds

    def _truth_pose_callback(self, msg: TFMessage) -> None:
        transform = self._select_truth_pose_transform(msg)
        if transform is None:
            return

        translation = transform.transform.translation
        position = np.array([translation.x, translation.y, translation.z], dtype=float)
        if not np.all(np.isfinite(position)):
            return

        rotation = transform.transform.rotation
        q_xyzw = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
        if np.all(np.isfinite(q_xyzw)) and np.linalg.norm(q_xyzw) > 1e-6:
            self.gazebo_pose_orientation_xyzw = normalize_quaternion_xyzw(q_xyzw)
        self.gazebo_pose_position_enu = position
        self.gazebo_pose_stamp_ns = self.get_clock().now().nanoseconds

    def _select_truth_pose_transform(self, msg: TFMessage) -> TransformStamped | None:
        if not msg.transforms:
            self.get_logger().warn(
                "Gazebo dynamic pose truth is empty",
                throttle_duration_sec=5.0,
            )
            return None

        model_name = str(self.get_parameter("truth_model_name").value).strip()
        if model_name:
            for transform in msg.transforms:
                if self._matches_truth_model(transform.child_frame_id, model_name):
                    return transform
                if self._matches_truth_model(transform.header.frame_id, model_name):
                    return transform

            if not self._pose_name_fallback_warned:
                self.get_logger().info(
                    "Gazebo dynamic pose bridge did not expose model frame names; "
                    f"using truth_pose_index={self.get_parameter('truth_pose_index').value}",
                )
                self._pose_name_fallback_warned = True

        pose_index = int(self.get_parameter("truth_pose_index").value)
        if 0 <= pose_index < len(msg.transforms):
            return msg.transforms[pose_index]

        self.get_logger().warn(
            f"truth_pose_index={pose_index} is outside dynamic pose message with "
            f"{len(msg.transforms)} transforms",
            throttle_duration_sec=5.0,
        )
        return None

    def _matches_truth_model(self, frame_name: str, model_name: str) -> bool:
        if not frame_name:
            return False
        normalized = frame_name.strip()
        if normalized == model_name:
            return True
        return normalized.endswith(f"::{model_name}") or normalized.endswith(f"/{model_name}")

    def _timer_callback(self) -> None:
        truth = self._current_truth()
        if truth is None:
            return
        truth_position_enu, truth_orientation_xyzw = truth

        noise = float(self.get_parameter("range_noise_std_m").value)
        ranges = simulate_ranges(truth_position_enu, self.anchors, noise, self.rng)
        ranges = [
            float(np.clip(r, self.get_parameter("range_min_m").value, self.get_parameter("range_max_m").value))
            for r in ranges
        ]
        self._publish_ranges(ranges)

        anchor_points = [anchor.position_enu for anchor in self.anchors]
        initial = self.last_solution_enu if self.last_solution_enu is not None else truth_position_enu
        try:
            solution, rms = trilaterate(anchor_points, ranges, initial_enu=initial)
        except ValueError as exc:
            self.get_logger().warn(f"UWB solve skipped: {exc}")
            return
        max_rms = float(self.get_parameter("max_solution_rms_m").value)
        if max_rms > 0.0 and rms > max_rms:
            self.get_logger().warn(
                f"UWB solve skipped: residual RMS {rms:.2f} m > {max_rms:.2f} m",
                throttle_duration_sec=2.0,
            )
            return
        self.last_solution_enu = solution
        self._publish_pose(solution, rms, truth_orientation_xyzw)

    def _publish_ranges(self, ranges: list[float]) -> None:
        now = self.get_clock().now().to_msg()
        for publisher, anchor, value in zip(self.range_pubs, self.anchors, ranges):
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = anchor.name
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.1
            msg.min_range = float(self.get_parameter("range_min_m").value)
            msg.max_range = float(self.get_parameter("range_max_m").value)
            msg.range = float(value)
            publisher.publish(msg)

    def _publish_pose(
        self,
        position: np.ndarray,
        rms: float,
        orientation_xyzw: np.ndarray | None,
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        if bool(self.get_parameter("publish_truth_orientation").value) and orientation_xyzw is not None:
            msg.pose.pose.orientation.x = float(orientation_xyzw[0])
            msg.pose.pose.orientation.y = float(orientation_xyzw[1])
            msg.pose.pose.orientation.z = float(orientation_xyzw[2])
            msg.pose.pose.orientation.w = float(orientation_xyzw[3])
            msg.pose.covariance[35] = float(self.get_parameter("truth_yaw_variance_rad2").value)
        else:
            msg.pose.pose.orientation.w = 1.0
        variance = max(rms * rms, float(self.get_parameter("range_noise_std_m").value) ** 2)
        msg.pose.covariance[0] = variance
        msg.pose.covariance[7] = variance
        msg.pose.covariance[14] = variance * 1.5
        self.pose_pub.publish(msg)

    def _current_truth(self) -> tuple[np.ndarray, np.ndarray | None] | None:
        source = str(self.get_parameter("truth_source").value).lower()
        if source in ("gazebo", "gazebo_pose", "gz", "gz_pose", "dynamic_pose"):
            return self._gazebo_pose_truth()
        if source in ("gazebo_odometry", "gz_odometry"):
            return self._gazebo_odom_truth()
        if source in ("px4", "px4_local_position", "vehicle_local_position"):
            return self._px4_truth()
        if source == "auto":
            return self._gazebo_pose_truth() or self._gazebo_odom_truth() or self._px4_truth()
        self.get_logger().warn(
            f"unknown truth_source '{source}', expected gazebo_pose, gazebo_odometry, "
            "px4_local_position, or auto",
            throttle_duration_sec=5.0,
        )
        return None

    def _gazebo_pose_truth(self) -> tuple[np.ndarray, np.ndarray | None] | None:
        if self.gazebo_pose_position_enu is None or self._is_stale(self.gazebo_pose_stamp_ns):
            self.get_logger().warn(
                "waiting for Gazebo dynamic pose truth; check the ros_gz bridge for "
                f"{self.get_parameter('truth_pose_topic').value}",
                throttle_duration_sec=5.0,
            )
            return None
        return self.gazebo_pose_position_enu, self.gazebo_pose_orientation_xyzw

    def _gazebo_odom_truth(self) -> tuple[np.ndarray, np.ndarray | None] | None:
        if self.gazebo_odom_position_enu is None or self._is_stale(self.gazebo_odom_stamp_ns):
            self.get_logger().warn(
                "waiting for Gazebo odometry truth; check the ros_gz bridge for "
                f"{self.get_parameter('truth_odom_topic').value}",
                throttle_duration_sec=5.0,
            )
            return None
        return self.gazebo_odom_position_enu, self.gazebo_odom_orientation_xyzw

    def _px4_truth(self) -> tuple[np.ndarray, np.ndarray | None] | None:
        if self.px4_position_enu is None or self._is_stale(self.px4_stamp_ns):
            return None
        return self.px4_position_enu, None

    def _is_stale(self, stamp_ns: int) -> bool:
        if stamp_ns <= 0:
            return True
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        return age_s > float(self.get_parameter("truth_timeout_s").value)

    def _publish_anchor_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, anchor in enumerate(self.anchors):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = str(self.get_parameter("map_frame").value)
            marker.ns = "uwb_anchors"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(anchor.position_enu[0])
            marker.pose.position.y = float(anchor.position_enu[1])
            marker.pose.position.z = float(anchor.position_enu[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.r = 0.1
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UwbSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
