"""Feed independent UWB measurements to PX4, never navigation estimates."""
from __future__ import annotations

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from px4_powerplant_mission.common.frames import enu_to_ned, ros_to_px4_quaternion, rotate_vector_xyzw
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile
from px4_powerplant_mission.localization.inputs import UwbInput, declare_input_parameters, stamp_ns


class ExternalVisionBridgeNode(Node):
    """One input sample produces at most one external position measurement."""

    def __init__(self) -> None:
        super().__init__("powerplant_external_vision_bridge")
        self._declare_parameters()
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.input = UwbInput.from_node(
            self, self.get_parameter("max_publish_position_variance_m2").value,
            self.get_parameter("max_publish_height_variance_m2").value,
            self.get_parameter("max_measurement_age_s").value,
        )
        if self.get_parameter("px4_timestamp_domain").value not in ("system", "ros"):
            raise ValueError("px4_timestamp_domain must be system or ros")
        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry, self.get_parameter("visual_odometry_topic").value, px4_qos_profile())
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("uwb_pose_topic").value,
                                 self._pose_callback, reliable_qos_profile())
        for name in ("uwb_quality_topic", "uwb_status_topic"):
            self.create_subscription(String, self.get_parameter(name).value,
                                     self._health_callback, reliable_qos_profile())
        # Observe clock rewinds even while measurements stop.
        self.create_timer(.1, lambda: self.input.clock(self.get_clock().now().nanoseconds))
        self.get_logger().info("external vision uses independent UWB input; navigation odometry is not subscribed")

    def _declare_parameters(self):
        declare_input_parameters(self)
        for name, value in {
            "map_frame": "map",
            "visual_odometry_topic": "/fmu/in/vehicle_visual_odometry",
            "max_measurement_age_s": .5,
            "max_publish_position_variance_m2": 2.,
            "max_publish_height_variance_m2": .5,
            "use_independent_orientation": False,
            "max_orientation_variance_rad2": .5,
            "px4_timestamp_domain": "system",
        }.items():
            self.declare_parameter(name, value)

    def _health_callback(self, msg):
        self.input.health(msg.data, self.get_clock().now().nanoseconds)

    def _pose_callback(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        sample = self.input.accept(msg, now_ns)
        if sample is None:
            return
        # XRCE performs system↔PX4 boot time synchronization. Do not subtract
        # boot offsets here. In SITL, carry the ROS sample's age into system time.
        transport_now = (self.system_clock.now().nanoseconds
                         if self.get_parameter("px4_timestamp_domain").value == "system"
                         and self.get_parameter("use_sim_time").value else now_ns)
        sample_ns = transport_now - (now_ns - stamp_ns(sample.header.stamp))
        if sample_ns <= 0:
            return
        p = sample.pose.pose.position
        visual = VehicleOdometry()
        visual.timestamp = transport_now // 1000
        visual.timestamp_sample = sample_ns // 1000
        visual.pose_frame = VehicleOdometry.POSE_FRAME_NED
        visual.position = [float(v) for v in enu_to_ned([p.x, p.y, p.z])]
        cov = sample.pose.covariance
        visual.position_variance = [float(cov[i]) for i in (7, 0, 14)]
        # A single tag has neither attitude nor velocity. NaN is PX4's unknown
        # marker. Never manufacture yaw from a PX4 estimate or identity pose.
        visual.q = [math.nan] * 4
        visual.orientation_variance = [math.nan] * 3
        q = sample.pose.pose.orientation
        attitude = np.array([q.x, q.y, q.z, q.w])
        variances = np.array([cov[i] for i in (21, 28, 35)])
        world_covariance = np.asarray(cov).reshape(6, 6)[3:, 3:]
        maximum = self.get_parameter("max_orientation_variance_rad2").value
        if (self.get_parameter("use_independent_orientation").value
                and np.all(np.isfinite(attitude)) and np.linalg.norm(attitude) > 1e-6
                and np.all(np.isfinite(variances)) and np.all(variances > 0)
                and np.all(variances <= maximum)
                and np.all(np.isfinite(world_covariance))
                and np.allclose(world_covariance, world_covariance.T, atol=1e-9)
                and np.min(np.linalg.eigvalsh(world_covariance)) >= 0):
            visual.q = ros_to_px4_quaternion(attitude).tolist()
            rotation = np.column_stack([rotate_vector_xyzw(attitude, axis) for axis in np.eye(3)])
            visual.orientation_variance = np.diag(rotation.T @ world_covariance @ rotation).tolist()
        visual.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        visual.velocity = [math.nan] * 3
        visual.velocity_variance = [math.nan] * 3
        visual.angular_velocity = [math.nan] * 3
        visual.reset_counter = self.input.reset_counter
        visual.quality = 0  # no calibrated 0–100 quality model
        self.visual_odom_pub.publish(visual)


def main(args=None):
    rclpy.init(args=args)
    node = ExternalVisionBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
