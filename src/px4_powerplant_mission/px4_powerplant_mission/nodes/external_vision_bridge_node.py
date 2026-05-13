"""Bridge the ROS indoor localization estimate into PX4 external vision."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node

from px4_powerplant_mission.common.frames import (
    enu_to_ned,
    quaternion_xyzw_to_yaw,
    yaw_enu_to_ned,
    yaw_to_quaternion_xyzw,
)
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile


class ExternalVisionBridgeNode(Node):
    """Publish localized ROS ENU odometry as PX4 NED visual odometry."""

    def __init__(self) -> None:
        super().__init__("powerplant_external_vision_bridge")
        self._declare_parameters()

        self.visual_odom_pub = self.create_publisher(
            VehicleOdometry,
            self.get_parameter("visual_odometry_topic").value,
            px4_qos_profile(),
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("localized_odom_topic").value,
            self._odom_callback,
            reliable_qos_profile(),
        )
        self.get_logger().info("PX4 external vision bridge started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("visual_odometry_topic", "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("use_velocity", False)
        self.declare_parameter("position_variance_m2", 0.08)
        self.declare_parameter("height_variance_m2", 0.12)
        self.declare_parameter("yaw_variance_rad2", 0.05)
        self.declare_parameter("velocity_variance_m2ps2", 0.25)

    def _odom_callback(self, msg: Odometry) -> None:
        position_enu = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(position_enu)):
            return

        q_enu_xyzw = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        yaw_ned = yaw_enu_to_ned(quaternion_xyzw_to_yaw(q_enu_xyzw))
        q_ned_xyzw = yaw_to_quaternion_xyzw(yaw_ned)

        visual = VehicleOdometry()
        now_us = self._timestamp_us()
        visual.timestamp = now_us
        visual.timestamp_sample = now_us
        visual.pose_frame = VehicleOdometry.POSE_FRAME_NED
        visual.position = self._float32_list(enu_to_ned(position_enu))
        visual.q = [
            float(q_ned_xyzw[3]),
            float(q_ned_xyzw[0]),
            float(q_ned_xyzw[1]),
            float(q_ned_xyzw[2]),
        ]

        if bool(self.get_parameter("use_velocity").value):
            velocity_enu = np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ],
                dtype=float,
            )
            visual.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            visual.velocity = self._float32_list(enu_to_ned(velocity_enu))
            velocity_variance = self._velocity_variance(msg)
        else:
            visual.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            visual.velocity = [math.nan, math.nan, math.nan]
            velocity_variance = [math.nan, math.nan, math.nan]

        visual.angular_velocity = [math.nan, math.nan, math.nan]
        visual.position_variance = self._position_variance(msg)
        visual.orientation_variance = [
            float(self.get_parameter("yaw_variance_rad2").value),
            float(self.get_parameter("yaw_variance_rad2").value),
            float(self._positive_or_default(msg.pose.covariance[35], "yaw_variance_rad2")),
        ]
        visual.velocity_variance = velocity_variance
        visual.reset_counter = 0
        visual.quality = 100
        self.visual_odom_pub.publish(visual)

    def _position_variance(self, msg: Odometry) -> list[float]:
        horizontal_default = float(self.get_parameter("position_variance_m2").value)
        height_default = float(self.get_parameter("height_variance_m2").value)
        east_var = self._positive_or_value(msg.pose.covariance[0], horizontal_default)
        north_var = self._positive_or_value(msg.pose.covariance[7], horizontal_default)
        up_var = self._positive_or_value(msg.pose.covariance[14], height_default)
        return [float(north_var), float(east_var), float(up_var)]

    def _velocity_variance(self, msg: Odometry) -> list[float]:
        default = float(self.get_parameter("velocity_variance_m2ps2").value)
        east_var = self._positive_or_value(msg.twist.covariance[0], default)
        north_var = self._positive_or_value(msg.twist.covariance[7], default)
        up_var = self._positive_or_value(msg.twist.covariance[14], default)
        return [float(north_var), float(east_var), float(up_var)]

    def _positive_or_default(self, value: float, parameter_name: str) -> float:
        return self._positive_or_value(value, float(self.get_parameter(parameter_name).value))

    @staticmethod
    def _positive_or_value(value: float, default: float) -> float:
        if math.isfinite(value) and value > 0.0:
            return float(value)
        return float(default)

    @staticmethod
    def _float32_list(values: Sequence[float]) -> list[float]:
        return [float(value) for value in values]

    def _timestamp_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExternalVisionBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
