"""Simulation-only UWB ranging and trilateration node."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker, MarkerArray

from px4_powerplant_mission.common.frames import ned_to_enu
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
        self.truth_position_enu: np.ndarray | None = None
        self.last_solution_enu: np.ndarray | None = None

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

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._timer_callback)
        self.marker_timer = self.create_timer(1.0, self._publish_anchor_markers)
        self.get_logger().info(f"UWB simulator started with {len(self.anchors)} anchors")

    def _declare_parameters(self) -> None:
        self.declare_parameter("truth_vehicle_local_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("truth_odom_topic", "/model/x500_depth/odometry")
        self.declare_parameter("uwb_pose_topic", "/uwb/pose")
        self.declare_parameter("uwb_range_topic_prefix", "/uwb/range")
        self.declare_parameter("anchor_marker_topic", "/uwb/anchors")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("range_noise_std_m", 0.08)
        self.declare_parameter("range_min_m", 0.05)
        self.declare_parameter("range_max_m", 80.0)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter(
            "uwb_anchors",
            [-8.0, -8.0, 3.0, 8.0, -8.0, 3.0, 8.0, 8.0, 3.0, -8.0, 8.0, 3.0, 0.0, 0.0, 7.0],
        )
        self.declare_parameter(
            "uwb_anchor_names",
            ["anchor_0", "anchor_1", "anchor_2", "anchor_3", "anchor_4"],
        )

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.truth_position_enu = ned_to_enu([msg.x, msg.y, msg.z])

    def _truth_odom_callback(self, msg: Odometry) -> None:
        self.truth_position_enu = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )

    def _timer_callback(self) -> None:
        if self.truth_position_enu is None:
            return

        noise = float(self.get_parameter("range_noise_std_m").value)
        ranges = simulate_ranges(self.truth_position_enu, self.anchors, noise, self.rng)
        ranges = [
            float(np.clip(r, self.get_parameter("range_min_m").value, self.get_parameter("range_max_m").value))
            for r in ranges
        ]
        self._publish_ranges(ranges)

        anchor_points = [anchor.position_enu for anchor in self.anchors]
        initial = self.last_solution_enu if self.last_solution_enu is not None else self.truth_position_enu
        try:
            solution, rms = trilaterate(anchor_points, ranges, initial_enu=initial)
        except ValueError as exc:
            self.get_logger().warn(f"UWB solve skipped: {exc}")
            return
        self.last_solution_enu = solution
        self._publish_pose(solution, rms)

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

    def _publish_pose(self, position: np.ndarray, rms: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.w = 1.0
        variance = max(rms * rms, float(self.get_parameter("range_noise_std_m").value) ** 2)
        msg.pose.covariance[0] = variance
        msg.pose.covariance[7] = variance
        msg.pose.covariance[14] = variance * 1.5
        self.pose_pub.publish(msg)

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

