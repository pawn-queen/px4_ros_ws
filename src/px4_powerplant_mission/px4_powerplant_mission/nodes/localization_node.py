"""Shared navigation adapter and independent UWB health for SITL and hardware."""

from __future__ import annotations

import json
from copy import deepcopy
import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleImu, VehicleLocalPosition, VehicleOdometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from px4_powerplant_mission.common.frames import (
    px4_to_ros_quaternion,
    frd_to_flu,
    ned_to_enu,
    normalize_quaternion_xyzw,
    quaternion_wxyz_to_xyzw,
    quaternion_conjugate_xyzw,
    quaternion_xyzw_to_yaw,
    rotate_vector_xyzw,
    yaw_ned_to_enu,
)
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile, sensor_qos_profile
from px4_powerplant_mission.localization.fusion import FusionState
from px4_powerplant_mission.localization.inputs import UwbInput, declare_input_parameters, set_stamp, stamp_ns


UNKNOWN_TILT_VARIANCE_RAD2 = 1_000_000.0


class LocalizationNode(Node):
    """Publish navigation only; this output must never be used as an EV input."""

    def __init__(self) -> None:
        super().__init__("powerplant_localization")
        self._declare_parameters()

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.navigation_mode = self.get_parameter("navigation_mode").value
        if self.navigation_mode not in ("px4", "adaptive"):
            raise ValueError("navigation_mode must be px4 or adaptive")
        if self.get_parameter("px4_timestamp_domain").value not in ("system", "ros"):
            raise ValueError("px4_timestamp_domain must be system or ros")
        if self.get_parameter("map_frame").value != self.get_parameter("odom_frame").value:
            raise ValueError("map_frame and odom_frame must match the PX4-aligned ENU frame")
        self.input = UwbInput.from_node(
            self, self.get_parameter("uwb_max_position_variance_m2").value,
            self.get_parameter("uwb_max_height_variance_m2").value,
            self.get_parameter("uwb_recent_timeout_s").value)
        self.navigation_odom = None
        self.last_navigation_sample_ns = 0
        self.last_px4_reset_counter = None
        self.px4_attitude = None
        self.tilt_source = "unknown"
        self.state_stamp_ns = 0
        self.last_clock_ns = 0
        self.state = FusionState()
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_depth_median_m = math.nan
        self.last_depth_stamp_ns = 0
        self.last_uwb_stamp_ns = 0
        self.last_uwb_sample_stamp_ns = 0
        self.uwb_quality_valid = False
        self.uwb_rejection_reason = "waiting for UWB"
        self.uwb_last_innovation_nis: float | None = None
        self.uwb_last_measurement_variance_enu_m2: np.ndarray | None = None
        self.position_process_noise_m2ps = max(
            0.0, float(self.get_parameter("position_process_noise_m2ps").value)
        )
        self.last_px4_stamp_ns = 0
        self.last_external_yaw_stamp_ns = 0
        self.last_external_tilt_stamp_ns = 0
        self.roll_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
        self.pitch_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
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

        for name in ("uwb_quality_topic", "uwb_status_topic"):
            self.create_subscription(String, self.get_parameter(name).value,
                                     self._uwb_health_callback, reliable_qos_profile())
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish_state)
        self.get_logger().info("powerplant localization node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("vehicle_imu_topic", "/fmu/out/vehicle_imu")
        self.declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        declare_input_parameters(self)
        self.declare_parameter("navigation_mode", "px4")
        self.declare_parameter("px4_timestamp_domain", "system")
        self.declare_parameter("max_navigation_age_s", 0.5)
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
        self.declare_parameter("use_px4_position_when_uwb_recent", False)
        self.declare_parameter("px4_velocity_alpha", 0.15)
        self.declare_parameter("uwb_position_alpha", 0.25)
        self.declare_parameter("uwb_z_alpha", 0.08)
        self.declare_parameter("uwb_recent_timeout_s", 0.5)
        self.declare_parameter("uwb_outlier_threshold_m", 4.0)
        self.declare_parameter("uwb_max_position_variance_m2", 2.0)
        self.declare_parameter("uwb_max_height_variance_m2", 0.5)
        self.declare_parameter("uwb_nis_threshold", 16.27)
        self.declare_parameter("uwb_recovery_timeout_s", 1.0)
        self.declare_parameter("position_process_noise_m2ps", 0.25)
        self.declare_parameter("use_uwb_orientation", False)
        self.declare_parameter("max_uwb_yaw_variance_rad2", 0.5)
        self.declare_parameter("max_uwb_tilt_variance_rad2", 0.5)
        self.declare_parameter("external_yaw_timeout_s", 0.5)
        self.declare_parameter("independent_position_variance_m2", 0.20)
        self.declare_parameter("independent_height_variance_m2", 0.35)
        self.declare_parameter("fallback_position_variance_m2", 25.0)
        self.declare_parameter("fallback_height_variance_m2", 9.0)
        self.declare_parameter("depth_z_alpha", 0.12)
        self.declare_parameter("use_imu_prediction", False)
        self.declare_parameter("max_integrated_speed_mps", 8.0)
        self.declare_parameter("depth_mount", "forward")
        self.declare_parameter("floor_z_enu_m", 0.0)
        self.declare_parameter("depth_center_fraction", 0.18)

    def _vehicle_imu_callback(self, msg: VehicleImu) -> None:
        if self.navigation_mode != "adaptive":
            return
        if not bool(self.get_parameter("use_imu_prediction").value):
            return
        sample = self._px4_sample_ns(msg)
        if sample <= self.state_stamp_ns:
            return
        dt_s = float(msg.delta_velocity_dt) * 1e-6
        if dt_s <= 0.0:
            return

        self._expire_tilt_if_stale()
        body_frd_delta = np.asarray(msg.delta_velocity, dtype=float)
        body_flu_delta = frd_to_flu(body_frd_delta)
        world_delta = rotate_vector_xyzw(self.state.orientation_xyzw, body_flu_delta)
        self.state.predict_from_imu_delta(
            world_delta,
            dt_s,
            float(self.get_parameter("max_integrated_speed_mps").value),
        )
        self.state_stamp_ns = sample

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if self.navigation_mode != "adaptive":
            return
        sample = self._px4_sample_ns(msg)
        if not sample or sample <= self.state_stamp_ns:
            return
        if msg.xy_valid and msg.z_valid:
            position = ned_to_enu([msg.x, msg.y, msg.z])
            velocity = None
            if msg.v_xy_valid and msg.v_z_valid:
                velocity = ned_to_enu([msg.vx, msg.vy, msg.vz])
            if not self._external_yaw_is_recent():
                self.state.yaw_enu = yaw_ned_to_enu(float(msg.heading))
            self._correct_from_px4_position(position, velocity)
            if not self._uwb_is_recent() or self.get_parameter("use_px4_position_when_uwb_recent").value:
                self.state_stamp_ns = sample
            self.last_px4_stamp_ns = sample

    def _px4_sample_ns(self, msg) -> int:
        now_ns = self.get_clock().now().nanoseconds
        sample = int(msg.timestamp_sample) * 1000
        if sample <= 0:
            return 0
        if (self.get_parameter("px4_timestamp_domain").value == "system"
                and self.get_parameter("use_sim_time").value):
            sample += now_ns - self.system_clock.now().nanoseconds
        age = (now_ns - sample) * 1e-9
        return sample if -0.1 <= age <= self.get_parameter("max_navigation_age_s").value else 0

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        self._check_clock()
        sample = self._px4_sample_ns(msg)
        q = np.asarray(msg.q, dtype=float)
        variance = np.asarray(msg.orientation_variance, dtype=float)
        if (not sample or sample <= self.last_navigation_sample_ns
                or msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or not np.all(np.isfinite(q)) or np.linalg.norm(q) <= 1e-6
                or not np.all(np.isfinite(variance)) or np.any(variance <= 0)):
            return
        orientation = px4_to_ros_quaternion(q)
        self.px4_attitude = (sample, orientation, variance.copy())
        self.last_px4_stamp_ns = sample
        if not self._external_yaw_is_recent():
            self.state.yaw_enu = quaternion_xyzw_to_yaw(orientation)
        if self.tilt_source != "uwb" or not self._external_tilt_is_recent():
            self._invalidate_tilt()  # restores the fresh full PX4 attitude
        position = ned_to_enu(msg.position)
        position_variance = np.asarray(msg.position_variance, dtype=float)[[1, 0, 2]]
        if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(position_variance))
                or np.any(position_variance <= 0)):
            return
        if self.last_px4_reset_counter is not None and msg.reset_counter != self.last_px4_reset_counter:
            # A local-frame jump must not be silently mixed with the old map.
            self.input.valid = False
            self.input.invalid_before_ns = self.get_clock().now().nanoseconds
            self.input.reason = "PX4 estimator reset; awaiting new independent measurement"
            self.navigation_odom = None
        self.last_px4_reset_counter = msg.reset_counter
        self.last_navigation_sample_ns = sample
        velocity = (ned_to_enu(msg.velocity)
                    if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED else None)
        if self.navigation_mode == "adaptive":
            if not self._uwb_is_recent():
                self.state_stamp_ns = sample
            self._correct_from_px4_position(position, velocity, position_alpha_scale=0.5)
            return
        odom = Odometry()
        set_stamp(odom.header.stamp, sample)
        odom.header.frame_id = self.get_parameter("map_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
        q = odom.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, orientation)
        for i, value in zip((0, 7, 14), position_variance):
            odom.pose.covariance[i] = float(value)
        # PX4 attitude uncertainty is in BODY axes; ROS pose uncertainty is
        # in the fixed/world axes. FRD→FLU leaves diagonal body variance intact.
        rotation = np.column_stack([rotate_vector_xyzw(orientation, axis) for axis in np.eye(3)])
        world_variance = rotation @ np.diag(variance) @ rotation.T
        for i in range(3):
            for j in range(3):
                odom.pose.covariance[(i+3)*6+j+3] = float(world_variance[i, j])
        # Odometry twist belongs to child_frame_id (FLU), not the map frame.
        if velocity is not None and np.all(np.isfinite(velocity)):
            body_velocity = rotation.T @ velocity
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
            vvar = np.asarray(msg.velocity_variance)[[1, 0, 2]]
            if np.all(np.isfinite(vvar)) and np.all(vvar > 0):
                vcov = rotation.T @ np.diag(vvar) @ rotation
                for i in range(3):
                    for j in range(3):
                        odom.twist.covariance[i*6+j] = float(vcov[i, j])
            else:
                for i in (0, 7, 14):
                    odom.twist.covariance[i] = 1e6
        else:
            for i in (0, 7, 14):
                odom.twist.covariance[i] = 1e6
        if np.all(np.isfinite(msg.angular_velocity)):
            angular = frd_to_flu(msg.angular_velocity)
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, angular)
        for i in (21, 28, 35):
            odom.twist.covariance[i] = 1e6
        self.navigation_odom = odom

    def _uwb_health_callback(self, msg):
        self.input.health(msg.data, self.get_clock().now().nanoseconds)
        if not self.input.valid:
            self.uwb_quality_valid = False
            self.uwb_rejection_reason = self.input.reason

    def _check_clock(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.last_clock_ns:
            self.navigation_odom = self.px4_attitude = None
            self.last_navigation_sample_ns = self.last_uwb_stamp_ns = 0
            self.last_uwb_sample_stamp_ns = self.state_stamp_ns = 0
            self.last_external_yaw_stamp_ns = self.last_external_tilt_stamp_ns = 0
            self.last_px4_stamp_ns = 0
            self.last_px4_reset_counter = None
            self.tilt_source = "unknown"
            self.state = FusionState()
            self.uwb_quality_valid = False
        self.last_clock_ns = now_ns
        self.input.clock(now_ns)

    def _uwb_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._check_clock()
        received_ns = self.get_clock().now().nanoseconds
        accepted = self.input.accept(msg, received_ns)
        if accepted is None:
            self._reject_uwb(self.input.reason, invalidate_quality=not self.input.valid)
            return
        msg = accepted
        if self.navigation_mode == "px4":
            self.last_uwb_stamp_ns = self.last_uwb_sample_stamp_ns = self.input.last_sample_ns
            self.uwb_quality_valid = True
            self.uwb_rejection_reason = ""
            self.uwb_last_measurement_variance_enu_m2 = np.asarray(msg.pose.covariance)[[0, 7, 14]]
            return
        stamp_sec = int(msg.header.stamp.sec)
        stamp_nanosec = int(msg.header.stamp.nanosec)
        sample_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
        if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000 or sample_ns <= 0:
            self._reject_uwb("missing or invalid measurement stamp", invalidate_quality=False)
            return
        if sample_ns <= self.last_uwb_sample_stamp_ns:
            self._reject_uwb("duplicate or out-of-order measurement stamp", invalidate_quality=False)
            return
        age_s = (received_ns - sample_ns) * 1e-9
        if age_s > float(self.get_parameter("uwb_recent_timeout_s").value) or age_s < -0.1:
            self._reject_uwb(f"invalid measurement age {age_s:.3f} s", invalidate_quality=False)
            return
        if msg.header.frame_id != str(self.get_parameter("map_frame").value):
            self._reject_uwb("position frame does not match map frame")
            return

        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        position_covariance = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)[:3, :3]
        variance = np.diag(position_covariance).copy()
        max_xy = float(self.get_parameter("uwb_max_position_variance_m2").value)
        max_z = float(self.get_parameter("uwb_max_height_variance_m2").value)
        if not np.all(np.isfinite(position)):
            self._reject_uwb("non-finite position")
            return
        # This filter uses only diagonal variances; reject malformed cross-axis
        # covariance instead of silently treating it as a valid diagonal model.
        if (not np.all(np.isfinite(position_covariance))
                or not np.allclose(position_covariance, position_covariance.T, atol=1e-9)
                or np.min(np.linalg.eigvalsh(position_covariance)) < -1e-8
                or np.any(variance <= 0.0)
                or np.any(variance[:2] > max_xy) or variance[2] > max_z):
            self._reject_uwb("invalid or excessive position covariance")
            return

        dt_s = (sample_ns - self.last_uwb_sample_stamp_ns) * 1e-9
        recovery_timeout_s = float(self.get_parameter("uwb_recovery_timeout_s").value)
        recovering = (
            self.last_uwb_sample_stamp_ns <= 0
            or not self._uwb_is_recent()
            or (recovery_timeout_s > 0.0 and dt_s >= recovery_timeout_s)
        )
        if recovering:
            # A PX4 fallback estimate is not an independent prior in the map frame.
            self.state.position_enu = position.copy()
            self.state.position_variance_enu_m2 = variance.copy()
            self.state.has_position = True
            self.uwb_last_innovation_nis = None
        else:
            predicted = self.state.position_enu.copy()
            if not bool(self.get_parameter("use_imu_prediction").value):
                predicted += self.state.velocity_enu * dt_s
            prior_variance = (
                self.state.position_variance_enu_m2
                + self.position_process_noise_m2ps * dt_s
            )
            if (not np.all(np.isfinite(predicted))
                    or not np.all(np.isfinite(prior_variance))
                    or np.any(prior_variance <= 0.0)):
                self._reject_uwb("invalid position prediction")
                return
            innovation = position - predicted
            nis = float(np.sum(innovation * innovation / (prior_variance + variance)))
            self.uwb_last_innovation_nis = nis
            nis_threshold = float(self.get_parameter("uwb_nis_threshold").value)
            if not math.isfinite(nis) or (nis_threshold > 0.0 and nis > nis_threshold):
                self._reject_uwb(f"normalized innovation {nis:.2f} exceeds gate")
                return
            self.state.correct_position_with_variance(
                position, variance, predicted, prior_variance
            )

        # Preserve the original stamp for duplicate detection. A slightly future
        # sample may precede the /clock update, so clamp only the freshness clock.
        self.last_uwb_sample_stamp_ns = sample_ns
        self.last_uwb_stamp_ns = min(sample_ns, received_ns)
        self.uwb_last_measurement_variance_enu_m2 = variance
        self.uwb_quality_valid = True
        self.uwb_rejection_reason = ""
        self.state_stamp_ns = sample_ns
        self._maybe_use_uwb_orientation(msg)

    def _reject_uwb(self, reason: str, *, invalidate_quality: bool = True) -> None:
        if invalidate_quality:
            self.uwb_quality_valid = False
        self.uwb_rejection_reason = reason
        self.get_logger().warn(f"rejecting UWB pose: {reason}", throttle_duration_sec=2.0)

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

        if self.navigation_mode == "adaptive" and str(self.get_parameter("depth_mount").value).lower() == "downward":
            sample = stamp_ns(msg.header.stamp)
            age = (self.get_clock().now().nanoseconds - sample) * 1e-9
            if sample <= self.state_stamp_ns or not 0 <= age <= self.get_parameter("max_navigation_age_s").value:
                return
            measured_z = (
                float(self.get_parameter("floor_z_enu_m").value)
                + self.last_depth_median_m
            )
            position = self.state.position_enu.copy()
            position[2] = measured_z
            self.state.correct_position(position, float(self.get_parameter("depth_z_alpha").value))
            self.state_stamp_ns = sample

    def _publish_state(self) -> None:
        self._check_clock()
        self.status_pub.publish(String(data=json.dumps(self._status(), ensure_ascii=False)))
        now_ns = self.get_clock().now().nanoseconds
        if self.navigation_mode == "px4":
            if self.navigation_odom is None:
                return
            age = (now_ns - self.last_navigation_sample_ns) * 1e-9
            if not 0 <= age <= self.get_parameter("max_navigation_age_s").value:
                return
            odom = deepcopy(self.navigation_odom)
            # A fresh PX4 output does not prove that the independent input lives.
            if not self._has_independent_position():
                for i in (0, 7):
                    odom.pose.covariance[i] = max(odom.pose.covariance[i], self.get_parameter("fallback_position_variance_m2").value)
                odom.pose.covariance[14] = max(odom.pose.covariance[14], self.get_parameter("fallback_height_variance_m2").value)
        else:
            if not self.state.has_position or self.state_stamp_ns <= 0:
                return
            if not 0 <= (now_ns-self.state_stamp_ns)*1e-9 <= self.get_parameter("max_navigation_age_s").value:
                return
            odom = Odometry()
            set_stamp(odom.header.stamp, self.state_stamp_ns)
            odom.header.frame_id = self.get_parameter("odom_frame").value
            odom.child_frame_id = self.get_parameter("base_frame").value
            self._fill_pose_and_twist(odom)
        self.odom_pub.publish(odom)
        pose = PoseStamped()
        pose.header, pose.pose = odom.header, odom.pose.pose
        self.pose_pub.publish(pose)
        if self.get_parameter("publish_tf").value:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            p = odom.pose.pose.position
            transform.transform.translation.x = p.x
            transform.transform.translation.y = p.y
            transform.transform.translation.z = p.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _fill_pose_and_twist(self, odom: Odometry) -> None:
        self._expire_tilt_if_stale()
        odom.pose.pose.position.x = float(self.state.position_enu[0])
        odom.pose.pose.position.y = float(self.state.position_enu[1])
        odom.pose.pose.position.z = float(self.state.position_enu[2])
        q = self.state.orientation_xyzw
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        body_velocity = rotate_vector_xyzw(quaternion_conjugate_xyzw(q), self.state.velocity_enu)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
        for i in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[i] = 1e6
        position_variance = LocalizationNode._position_covariance(self)
        odom.pose.covariance[0] = float(position_variance[0])
        odom.pose.covariance[7] = float(position_variance[1])
        odom.pose.covariance[14] = float(position_variance[2])
        tilt_valid = self.state.has_tilt and self._external_tilt_is_recent()
        odom.pose.covariance[21] = (
            self.roll_variance_rad2 if tilt_valid else UNKNOWN_TILT_VARIANCE_RAD2
        )
        odom.pose.covariance[28] = (
            self.pitch_variance_rad2 if tilt_valid else UNKNOWN_TILT_VARIANCE_RAD2
        )
        odom.pose.covariance[35] = 0.12

    def _position_covariance(self) -> np.ndarray:
        def fallback() -> np.ndarray:
            return np.array(
                [
                    float(self.get_parameter("fallback_position_variance_m2").value),
                    float(self.get_parameter("fallback_position_variance_m2").value),
                    float(self.get_parameter("fallback_height_variance_m2").value),
                ],
                dtype=float,
            )

        if not self._has_independent_position():
            return fallback()
        age_s = max(
            0.0,
            (self.get_clock().now().nanoseconds - self.last_uwb_stamp_ns) * 1e-9,
        )
        process_variance = max(
            0.0, float(getattr(self, "position_process_noise_m2ps", 0.25))
        ) * age_s
        posterior = np.asarray(self.state.position_variance_enu_m2, dtype=float)
        measurement = getattr(self, "uwb_last_measurement_variance_enu_m2", None)
        if measurement is None:
            measurement = np.zeros(3, dtype=float)
        floor = np.array(
            [
                float(self.get_parameter("independent_position_variance_m2").value),
                float(self.get_parameter("independent_position_variance_m2").value),
                float(self.get_parameter("independent_height_variance_m2").value),
            ],
            dtype=float,
        )
        result = np.maximum(np.maximum(posterior + process_variance, measurement), floor)
        if not np.all(np.isfinite(result)) or np.any(result <= 0.0):
            return fallback()
        return result

    def _status(self) -> dict[str, Any]:
        now_ns = self.get_clock().now().nanoseconds

        def age(stamp_ns: int) -> float | None:
            return None if stamp_ns == 0 else round((now_ns - stamp_ns) * 1e-9, 3)

        position = self.state.position_enu
        velocity = self.state.velocity_enu
        yaw = self.state.yaw_enu
        variance = LocalizationNode._position_covariance(self)
        if self.navigation_mode == "px4" and self.navigation_odom is not None:
            p = self.navigation_odom.pose.pose.position
            position = [p.x, p.y, p.z]
            q = self.navigation_odom.pose.pose.orientation
            orientation = [q.x, q.y, q.z, q.w]
            yaw = quaternion_xyzw_to_yaw(orientation)
            v = self.navigation_odom.twist.twist.linear
            velocity = rotate_vector_xyzw(orientation, [v.x, v.y, v.z])
            if self._has_independent_position():
                variance = np.asarray(self.navigation_odom.pose.covariance)[[0, 7, 14]]
        return {
            "frame": "ENU/FLU",
            "navigation_mode": self.navigation_mode,
            "navigation_source": "PX4 EKF" if self.navigation_mode == "px4" else "adaptive (may contain PX4 information)",
            "external_vision_source": "independent UWB topic, never this odometry",
            "navigation_sample_ns": self.last_navigation_sample_ns if self.navigation_mode == "px4" else self.state_stamp_ns,
            "uwb_input_healthy": self.input.healthy(self.get_clock().now().nanoseconds),
            "uwb_input_reason": self.input.reason,
            "px4_reset_counter": self.last_px4_reset_counter,
            "tilt_source": self.tilt_source,
            "position_enu_m": [round(float(v), 3) for v in position],
            "velocity_enu_mps": [round(float(v), 3) for v in velocity],
            "yaw_enu_rad": round(float(yaw), 3),
            "depth_median_m": None
            if not math.isfinite(self.last_depth_median_m)
            else round(self.last_depth_median_m, 3),
            "depth_age_s": age(self.last_depth_stamp_ns),
            "uwb_age_s": age(self.last_uwb_stamp_ns),
            "uwb_quality_valid": self.uwb_quality_valid,
            "uwb_rejection_reason": self.uwb_rejection_reason,
            "uwb_last_innovation_nis": (
                None if self.uwb_last_innovation_nis is None
                else round(self.uwb_last_innovation_nis, 3)
            ),
            "uwb_measurement_variance_enu_m2": (
                None if self.uwb_last_measurement_variance_enu_m2 is None
                else [round(float(v), 4) for v in self.uwb_last_measurement_variance_enu_m2]
            ),
            "position_variance_enu_m2": [
                round(float(v), 4) for v in variance
            ],
            "has_independent_position": self._has_independent_position(),
            "px4_age_s": age(self.last_px4_stamp_ns),
            "external_yaw_age_s": age(self.last_external_yaw_stamp_ns),
            "external_tilt_age_s": age(self.last_external_tilt_stamp_ns),
            "has_reliable_tilt": self.state.has_tilt and self._external_tilt_is_recent(),
        }

    def _correct_from_px4_position(
        self,
        position: np.ndarray,
        velocity: np.ndarray | None,
        position_alpha_scale: float = 1.0,
    ) -> None:
        if self._uwb_is_recent() and not bool(self.get_parameter("use_px4_position_when_uwb_recent").value):
            self._correct_velocity_from_px4(velocity)
            return
        self.state.correct_position(
            position,
            self._px4_position_alpha() * position_alpha_scale,
            velocity,
        )

    def _correct_velocity_from_px4(self, velocity: np.ndarray | None) -> None:
        if velocity is None or not np.all(np.isfinite(velocity)):
            return
        alpha = float(np.clip(float(self.get_parameter("px4_velocity_alpha").value), 0.0, 1.0))
        self.state.velocity_enu = (1.0 - alpha) * self.state.velocity_enu + alpha * velocity

    def _has_independent_position(self) -> bool:
        return self.uwb_quality_valid and self._uwb_is_recent() and self.input.healthy(self.get_clock().now().nanoseconds)

    def _px4_position_alpha(self) -> float:
        if self._uwb_is_recent():
            return float(self.get_parameter("px4_position_alpha_when_uwb_recent").value)
        return float(self.get_parameter("px4_position_alpha").value)

    def _uwb_is_recent(self) -> bool:
        if self.last_uwb_stamp_ns <= 0:
            return False
        timeout = float(self.get_parameter("uwb_recent_timeout_s").value)
        age_s = (self.get_clock().now().nanoseconds - self.last_uwb_stamp_ns) * 1e-9
        return 0.0 <= age_s <= timeout

    def _external_yaw_is_recent(self) -> bool:
        if self.last_external_yaw_stamp_ns <= 0:
            return False
        timeout = float(self.get_parameter("external_yaw_timeout_s").value)
        age_s = (self.get_clock().now().nanoseconds - self.last_external_yaw_stamp_ns) * 1e-9
        return 0.0 <= age_s <= timeout

    def _external_tilt_is_recent(self) -> bool:
        if self.last_external_tilt_stamp_ns <= 0:
            return False
        age_s = (self.get_clock().now().nanoseconds - self.last_external_tilt_stamp_ns) * 1e-9
        timeout = float(self.get_parameter("uwb_recent_timeout_s").value)
        return 0.0 <= age_s <= timeout

    def _expire_tilt_if_stale(self) -> None:
        if not self._external_tilt_is_recent():
            self._invalidate_tilt()

    def _invalidate_tilt(self) -> None:
        self.state.clear_tilt()
        self.tilt_source = "unknown"
        self.last_external_tilt_stamp_ns = 0
        self.roll_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
        self.pitch_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
        if self.px4_attitude is not None:
            stamp, q, variance = self.px4_attitude
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            if 0 <= age <= self.get_parameter("max_navigation_age_s").value:
                self.state.correct_tilt(q)
                self.tilt_source = "px4"
                self.last_external_tilt_stamp_ns = stamp
                self.roll_variance_rad2, self.pitch_variance_rad2 = map(float, variance[:2])

    def _maybe_use_uwb_orientation(self, msg: PoseWithCovarianceStamped) -> None:
        if not bool(self.get_parameter("use_uwb_orientation").value):
            self._invalidate_tilt()
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
            self._invalidate_tilt()
            return
        q_xyzw = normalize_quaternion_xyzw(q_xyzw)

        # An identity quaternion without positive covariance is the UWB node's
        # explicit marker for missing truth orientation, not a level vehicle.
        yaw_variance = float(msg.pose.covariance[35])
        max_yaw_variance = float(self.get_parameter("max_uwb_yaw_variance_rad2").value)
        if math.isfinite(yaw_variance) and 0.0 < yaw_variance <= max_yaw_variance:
            self.state.yaw_enu = quaternion_xyzw_to_yaw(q_xyzw)
            self.last_external_yaw_stamp_ns = self.last_uwb_stamp_ns

        roll_variance = float(msg.pose.covariance[21])
        pitch_variance = float(msg.pose.covariance[28])
        max_tilt_variance = float(self.get_parameter("max_uwb_tilt_variance_rad2").value)
        tilt_valid = all(
            math.isfinite(variance) and 0.0 < variance <= max_tilt_variance
            for variance in (roll_variance, pitch_variance)
        )
        if tilt_valid and self.state.correct_tilt(q_xyzw):
            self.tilt_source = "uwb"
            self.last_external_tilt_stamp_ns = self.last_uwb_stamp_ns
            self.roll_variance_rad2 = roll_variance
            self.pitch_variance_rad2 = pitch_variance
        else:
            self._invalidate_tilt()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
