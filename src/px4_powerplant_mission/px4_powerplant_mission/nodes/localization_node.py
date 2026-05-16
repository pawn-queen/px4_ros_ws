"""Indoor localization node for PX4 Gazebo simulation."""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleImu, VehicleLocalPosition, VehicleOdometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from px4_powerplant_mission.common.frames import (
    frd_to_flu,
    ned_to_enu,
    normalize_quaternion_xyzw,
    quaternion_wxyz_to_xyzw,
    quaternion_xyzw_to_yaw,
    rotate_vector_xyzw,
    yaw_ned_to_enu,
)
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile, sensor_qos_profile
from px4_powerplant_mission.localization.fusion import FusionState


class LocalizationNode(Node):
    """Fuse PX4 local estimates, IMU deltas, depth status, and simulated UWB."""

    def __init__(self) -> None:
        super().__init__("powerplant_localization")
        self._declare_parameters()

        self.state = FusionState()
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_depth_median_m = math.nan
        self.last_depth_stamp_ns = 0
        self.last_uwb_stamp_ns = 0
        self.last_px4_stamp_ns = 0
        self.last_external_yaw_stamp_ns = 0
        self.depth_frame_id = "camera_link"

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("localized_odom_topic").value, reliable_qos_profile()
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("localized_pose_topic").value, reliable_qos_profile()
        )
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, reliable_qos_profile()
        )

        self.create_subscription(
            VehicleImu,
            self.get_parameter("vehicle_imu_topic").value,
            self._vehicle_imu_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter("vehicle_local_position_topic").value,
            self._vehicle_local_position_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            VehicleOdometry,
            self.get_parameter("vehicle_odometry_topic").value,
            self._vehicle_odometry_callback,
            px4_qos_profile(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("uwb_pose_topic").value,
            self._uwb_pose_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_image_topic").value,
            self._depth_callback,
            sensor_qos_profile(),
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish_state)
        self.get_logger().info("powerplant localization node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("vehicle_imu_topic", "/fmu/out/vehicle_imu")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        self.declare_parameter("uwb_pose_topic", "/uwb/pose")
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("localized_pose_topic", "/powerplant/localization/pose")
        self.declare_parameter("status_topic", "/powerplant/localization/status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("px4_position_alpha", 0.20)
        self.declare_parameter("px4_position_alpha_when_uwb_recent", 0.02)
        self.declare_parameter("uwb_position_alpha", 0.25)
        self.declare_parameter("uwb_z_alpha", 0.08)
        self.declare_parameter("uwb_recent_timeout_s", 0.5)
        self.declare_parameter("uwb_outlier_threshold_m", 4.0)
        self.declare_parameter("use_uwb_orientation", True)
        self.declare_parameter("max_uwb_yaw_variance_rad2", 0.5)
        self.declare_parameter("external_yaw_timeout_s", 0.5)
        self.declare_parameter("depth_z_alpha", 0.12)
        self.declare_parameter("use_imu_prediction", False)
        self.declare_parameter("max_integrated_speed_mps", 8.0)
        self.declare_parameter("depth_mount", "forward")
        self.declare_parameter("floor_z_enu_m", 0.0)
        self.declare_parameter("depth_center_fraction", 0.18)

    def _vehicle_imu_callback(self, msg: VehicleImu) -> None:
        if not bool(self.get_parameter("use_imu_prediction").value):
            return
        dt_s = float(msg.delta_velocity_dt) * 1e-6
        if dt_s <= 0.0:
            return

        body_frd_delta = np.asarray(msg.delta_velocity, dtype=float)
        body_flu_delta = frd_to_flu(body_frd_delta)
        world_delta = rotate_vector_xyzw(self.state.orientation_xyzw, body_flu_delta)
        self.state.predict_from_imu_delta(
            world_delta,
            dt_s,
            float(self.get_parameter("max_integrated_speed_mps").value),
        )

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            position = ned_to_enu([msg.x, msg.y, msg.z])
            velocity = None
            if msg.v_xy_valid and msg.v_z_valid:
                velocity = ned_to_enu([msg.vx, msg.vy, msg.vz])
            if not self._external_yaw_is_recent():
                self.state.yaw_enu = yaw_ned_to_enu(float(msg.heading))
            self.state.correct_position(
                position,
                self._px4_position_alpha(),
                velocity,
            )
            self.last_px4_stamp_ns = self.get_clock().now().nanoseconds

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        q_wxyz = np.asarray(msg.q, dtype=float)
        if (
            not self._external_yaw_is_recent()
            and np.all(np.isfinite(q_wxyz))
            and np.linalg.norm(q_wxyz) > 1e-6
        ):
            q_xyzw = normalize_quaternion_xyzw(quaternion_wxyz_to_xyzw(q_wxyz))
            yaw_ned = quaternion_xyzw_to_yaw(q_xyzw)
            self.state.yaw_enu = yaw_ned_to_enu(yaw_ned)

        if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED:
            position = ned_to_enu(msg.position)
            velocity = None
            if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
                velocity = ned_to_enu(msg.velocity)
            self.state.correct_position(
                position,
                self._px4_position_alpha() * 0.5,
                velocity,
            )

    def _uwb_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        if self._is_uwb_outlier(position):
            return
        xy_alpha = float(self.get_parameter("uwb_position_alpha").value)
        z_alpha = float(self.get_parameter("uwb_z_alpha").value)
        self.state.correct_position_axes(position, [xy_alpha, xy_alpha, z_alpha])
        self._maybe_use_uwb_orientation(msg)
        self.last_uwb_stamp_ns = self.get_clock().now().nanoseconds

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

        h, w = depth.shape[:2]
        frac = float(self.get_parameter("depth_center_fraction").value)
        half_h = max(1, int(h * frac * 0.5))
        half_w = max(1, int(w * frac * 0.5))
        center = depth[h // 2 - half_h:h // 2 + half_h, w // 2 - half_w:w // 2 + half_w]
        valid = center[np.isfinite(center) & (center > 0.05) & (center < 100.0)]
        if valid.size == 0:
            return
        self.last_depth_median_m = float(np.median(valid))
        self.last_depth_stamp_ns = self.get_clock().now().nanoseconds
        self.depth_frame_id = msg.header.frame_id or self.depth_frame_id

        if str(self.get_parameter("depth_mount").value).lower() == "downward":
            measured_z = (
                float(self.get_parameter("floor_z_enu_m").value)
                + self.last_depth_median_m
            )
            position = self.state.position_enu.copy()
            position[2] = measured_z
            self.state.correct_position(position, float(self.get_parameter("depth_z_alpha").value))

    def _publish_state(self) -> None:
        if not self.state.has_position:
            return
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = str(self.get_parameter("odom_frame").value)
        odom.child_frame_id = str(self.get_parameter("base_frame").value)
        self._fill_pose_and_twist(odom)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = str(self.get_parameter("map_frame").value)
            transform.child_frame_id = str(self.get_parameter("base_frame").value)
            transform.transform.translation.x = float(self.state.position_enu[0])
            transform.transform.translation.y = float(self.state.position_enu[1])
            transform.transform.translation.z = float(self.state.position_enu[2])
            q = self.state.orientation_xyzw
            transform.transform.rotation.x = float(q[0])
            transform.transform.rotation.y = float(q[1])
            transform.transform.rotation.z = float(q[2])
            transform.transform.rotation.w = float(q[3])
            self.tf_broadcaster.sendTransform(transform)

        self.status_pub.publish(String(data=json.dumps(self._status(), ensure_ascii=False)))

    def _fill_pose_and_twist(self, odom: Odometry) -> None:
        odom.pose.pose.position.x = float(self.state.position_enu[0])
        odom.pose.pose.position.y = float(self.state.position_enu[1])
        odom.pose.pose.position.z = float(self.state.position_enu[2])
        q = self.state.orientation_xyzw
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        odom.twist.twist.linear.x = float(self.state.velocity_enu[0])
        odom.twist.twist.linear.y = float(self.state.velocity_enu[1])
        odom.twist.twist.linear.z = float(self.state.velocity_enu[2])
        odom.pose.covariance[0] = 0.20
        odom.pose.covariance[7] = 0.20
        odom.pose.covariance[14] = 0.35
        odom.pose.covariance[35] = 0.12

    def _status(self) -> dict[str, Any]:
        now_ns = self.get_clock().now().nanoseconds

        def age(stamp_ns: int) -> float | None:
            return None if stamp_ns == 0 else round((now_ns - stamp_ns) * 1e-9, 3)

        return {
            "frame": "ENU/FLU",
            "position_enu_m": [round(float(v), 3) for v in self.state.position_enu],
            "velocity_enu_mps": [round(float(v), 3) for v in self.state.velocity_enu],
            "yaw_enu_rad": round(float(self.state.yaw_enu), 3),
            "depth_median_m": None
            if not math.isfinite(self.last_depth_median_m)
            else round(self.last_depth_median_m, 3),
            "depth_age_s": age(self.last_depth_stamp_ns),
            "uwb_age_s": age(self.last_uwb_stamp_ns),
            "px4_age_s": age(self.last_px4_stamp_ns),
            "external_yaw_age_s": age(self.last_external_yaw_stamp_ns),
        }

    def _px4_position_alpha(self) -> float:
        if self._uwb_is_recent():
            return float(self.get_parameter("px4_position_alpha_when_uwb_recent").value)
        return float(self.get_parameter("px4_position_alpha").value)

    def _uwb_is_recent(self) -> bool:
        if self.last_uwb_stamp_ns <= 0:
            return False
        timeout = float(self.get_parameter("uwb_recent_timeout_s").value)
        return (self.get_clock().now().nanoseconds - self.last_uwb_stamp_ns) * 1e-9 <= timeout

    def _external_yaw_is_recent(self) -> bool:
        if self.last_external_yaw_stamp_ns <= 0:
            return False
        timeout = float(self.get_parameter("external_yaw_timeout_s").value)
        return (self.get_clock().now().nanoseconds - self.last_external_yaw_stamp_ns) * 1e-9 <= timeout

    def _is_uwb_outlier(self, position: np.ndarray) -> bool:
        threshold = float(self.get_parameter("uwb_outlier_threshold_m").value)
        if threshold <= 0.0 or not self.state.has_position or self.last_uwb_stamp_ns <= 0:
            return False
        distance = float(np.linalg.norm(position - self.state.position_enu))
        if distance <= threshold:
            return False
        self.get_logger().warn(
            f"rejecting UWB outlier: jump {distance:.2f} m > {threshold:.2f} m",
            throttle_duration_sec=2.0,
        )
        return True

    def _maybe_use_uwb_orientation(self, msg: PoseWithCovarianceStamped) -> None:
        if not bool(self.get_parameter("use_uwb_orientation").value):
            return
        yaw_variance = float(msg.pose.covariance[35])
        max_variance = float(self.get_parameter("max_uwb_yaw_variance_rad2").value)
        if not math.isfinite(yaw_variance) or yaw_variance <= 0.0 or yaw_variance > max_variance:
            return
        q_xyzw = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(q_xyzw)) or np.linalg.norm(q_xyzw) <= 1e-6:
            return
        self.state.yaw_enu = quaternion_xyzw_to_yaw(normalize_quaternion_xyzw(q_xyzw))
        self.last_external_yaw_stamp_ns = self.get_clock().now().nanoseconds


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
