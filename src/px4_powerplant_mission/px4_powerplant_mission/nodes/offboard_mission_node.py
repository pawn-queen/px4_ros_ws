"""PX4 offboard mission controller with simple waypoint/path planning support."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from px4_powerplant_mission.common.frames import enu_to_ned, yaw_enu_to_ned
from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile
from px4_powerplant_mission.path_planning.grid_astar import GridInfo, plan_astar


class OffboardMissionNode(Node):
    """Publish PX4 offboard setpoints for a powerplant inspection mission."""

    def __init__(self) -> None:
        super().__init__("powerplant_offboard_mission")
        self._declare_parameters()
        self.current_position_enu: np.ndarray | None = None
        self.current_yaw_enu = 0.0
        self.vehicle_status = VehicleStatus()
        self.offboard_counter = 0
        self.active = bool(self.get_parameter("auto_start").value)
        self.phase = "takeoff" if self.active else "idle"
        self.base_waypoints = self._parse_waypoints(self.get_parameter("mission_waypoints_enu").value)
        self.active_waypoints = list(self.base_waypoints)
        self.waypoint_index = 0
        self.grid: np.ndarray | None = None
        self.grid_info: GridInfo | None = None

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
            OccupancyGrid,
            self.get_parameter("occupancy_grid_topic").value,
            self._occupancy_grid_callback,
            reliable_qos_profile(),
        )

        self.create_service(Trigger, "start_powerplant_mission", self._start_mission)
        self.create_service(Trigger, "land_powerplant_mission", self._land_mission)
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info("offboard mission node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("occupancy_grid_topic", "/powerplant/map/occupancy_grid")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("mission_status_topic", "/powerplant/control/mission_status")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("takeoff_height_m", 2.5)
        self.declare_parameter("target_acceptance_m", 0.45)
        self.declare_parameter("cruise_yaw_enu_rad", 0.0)
        self.declare_parameter("use_occupancy_grid_planner", True)
        self.declare_parameter("planner_inflation_radius_m", 0.6)
        self.declare_parameter("path_waypoint_stride", 4)
        self.declare_parameter(
            "mission_waypoints_enu",
            [
                0.0, 0.0, 2.5,
                5.0, 0.0, 2.5,
                5.0, 5.0, 2.5,
                0.0, 5.0, 2.5,
                0.0, 0.0, 2.5,
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

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

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

    def _start_mission(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.active = True
        self.phase = "takeoff"
        self.offboard_counter = 0
        self.waypoint_index = 0
        self.active_waypoints = list(self.base_waypoints)
        response.success = True
        response.message = "mission started"
        return response

    def _land_mission(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.phase = "landing"
        self.active = False
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        response.success = True
        response.message = "land command sent"
        return response

    def _timer_callback(self) -> None:
        self._publish_offboard_heartbeat()
        if self.current_position_enu is None:
            return
        if not self.active:
            self._publish_status()
            return

        if self.offboard_counter == 10:
            self._engage_offboard()
            if bool(self.get_parameter("arm_on_start").value):
                self._arm()
        self.offboard_counter = min(self.offboard_counter + 1, 1000)

        target = self._select_target()
        self._publish_position_setpoint(target, float(self.get_parameter("cruise_yaw_enu_rad").value))
        self._advance_if_reached(target)
        self._publish_status(target)

    def _select_target(self) -> np.ndarray:
        takeoff_height = float(self.get_parameter("takeoff_height_m").value)
        if self.phase == "takeoff":
            target = self.current_position_enu.copy()
            target[2] = takeoff_height
            if abs(self.current_position_enu[2] - takeoff_height) < 0.35:
                self.phase = "mission"
                self._refresh_planned_path()
            return target

        if not self.active_waypoints:
            return self.current_position_enu.copy()
        self.waypoint_index = min(self.waypoint_index, len(self.active_waypoints) - 1)
        return self.active_waypoints[self.waypoint_index]

    def _advance_if_reached(self, target: np.ndarray) -> None:
        if self.current_position_enu is None or self.phase != "mission":
            return
        distance = float(np.linalg.norm(self.current_position_enu - target))
        if distance > float(self.get_parameter("target_acceptance_m").value):
            return
        if self.waypoint_index < len(self.active_waypoints) - 1:
            self.waypoint_index += 1
        else:
            self.phase = "hold"
            self.active = False

    def _refresh_planned_path(self) -> None:
        if (
            not bool(self.get_parameter("use_occupancy_grid_planner").value)
            or self.grid is None
            or self.grid_info is None
            or self.current_position_enu is None
            or not self.base_waypoints
        ):
            self.active_waypoints = list(self.base_waypoints)
            return

        planned: list[np.ndarray] = []
        start = self.current_position_enu
        stride = max(1, int(self.get_parameter("path_waypoint_stride").value))
        for goal in self.base_waypoints:
            path_xy = plan_astar(
                self.grid,
                self.grid_info,
                start[:2],
                goal[:2],
                inflation_radius_m=float(self.get_parameter("planner_inflation_radius_m").value),
            )
            if not path_xy:
                planned.append(goal)
                start = goal
                continue
            for x, y in path_xy[::stride]:
                planned.append(np.array([x, y, goal[2]], dtype=float))
            planned.append(goal)
            start = goal
        self.active_waypoints = planned or list(self.base_waypoints)
        self.waypoint_index = 0

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
        fields = [
            f"phase={self.phase}",
            f"active={self.active}",
            f"wp={self.waypoint_index}/{max(len(self.active_waypoints) - 1, 0)}",
            f"nav_state={self.vehicle_status.nav_state}",
        ]
        if target is not None:
            fields.append(f"target_enu={[round(float(v), 2) for v in target]}")
        self.status_pub.publish(String(data=", ".join(fields)))

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


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

