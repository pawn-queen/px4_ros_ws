"""Mission state transitions and map contracts for autonomous exploration."""

from concurrent.futures import Future
from types import SimpleNamespace
import json

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from px4_powerplant_mission.common.initial_scan import YawScanSequence
from px4_powerplant_mission.nodes import offboard_mission_node as mission


def test_completed_hover_scan_enters_explore_without_preset_waypoints():
    class FakeController:
        phase = mission.PHASE_HOVER_SCAN
        initial_scan_sequence = YawScanSequence(0.0, 4, 0.0)
        initial_scan_rotating = True
        initial_scan_entered_ns = 1_000_000_000
        initial_scan_hover_since_ns = 1_000_000_000
        initial_scan_wait_reason = ""
        initial_scan_step_depth_baseline = 1
        initial_scan_step_lidar_baseline = 1
        map_fusion_depth_count = 2
        map_fusion_lidar_count = 2
        grid_known_cells = 12
        current_yaw_enu = 0.0
        now_ns = 2_000_000_000
        entered_explore = False

        def get_parameter(self, name):
            return SimpleNamespace(value={
                "initial_scan_timeout_s": 90.0,
                "initial_scan_yaw_tolerance_rad": 0.12,
            }[name])

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

        def _age_s(self, stamp_ns):
            return (self.now_ns - stamp_ns) * 1e-9

        def _initial_scan_pose_ready(self, hover):
            return True, ""

        def _initial_scan_data_ready(self):
            return True, ""

        def _publish_event(self, name, fields):
            pass

        def _enter_explore(self):
            self.entered_explore = True
            self.phase = mission.PHASE_EXPLORE

        def _enter_global_search(self):
            raise AssertionError("preset waypoint search must not run after hover scan")

    controller = FakeController()
    controller.initial_scan_sequence.step_index = 3
    mission.OffboardMissionNode._update_initial_scan(controller)
    controller.now_ns += 100_000_000
    mission.OffboardMissionNode._update_initial_scan(controller)
    assert controller.entered_explore
    assert controller.phase == "EXPLORE"


@pytest.fixture
def controller_node():
    rclpy.init(args=[])
    node = mission.OffboardMissionNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _observed_cloud(seconds: int):
    header = Header(stamp=Time(sec=seconds), frame_id="map")
    fields = [
        PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z", "state"))
    ]
    return point_cloud2.create_cloud(
        header, fields,
        [(0.125, 0.125, 5.125, 0.0), (0.375, 0.125, 5.125, 1.0)],
    )


def _observed_status(seconds: int, *, complete: bool = True):
    return String(data=json.dumps({
        "map_stamp_ns": seconds * 1_000_000_000,
        "map_revision": 7,
        "mapping_enabled": True,
        "voxel_resolution_m": 0.25,
        "observed_bounds_key": [0, 0, 20, 1, 0, 20],
        "observed_voxel_count": 2,
        "observed_voxels_point_count": 2,
        "free_voxel_count": 1,
        "occupied_voxel_count": 1,
        "uncertain_voxel_count": 0,
        "observed_voxels_complete": complete,
        "depth_fused_count": 2,
        "lidar_fused_count": 2,
    }))


def test_voxel_snapshot_requires_matching_stamp_and_complete_cloud(controller_node):
    node = controller_node
    node._observed_voxels_callback(_observed_cloud(2))
    node._map_fusion_status_callback(_observed_status(3))
    assert node.map_revision == -1
    assert not node.map_snapshot_complete

    node._map_fusion_status_callback(_observed_status(2))
    assert node.map_snapshot_complete
    assert node.map_snapshot_stamp_ns == 2_000_000_000
    assert node.free_voxel_keys == {(0, 0, 20)}
    assert node.occupied_voxel_keys == {(1, 0, 20)}
    assert node._map_is_fresh()  # Gazebo header time may differ from the control clock.

    node._observed_voxels_callback(_observed_cloud(4))
    node._map_fusion_status_callback(_observed_status(4, complete=False))
    assert not node.map_snapshot_complete
    assert node.map_revision == 7


def test_explore_entry_discards_stale_configured_route(controller_node):
    node = controller_node
    stale = np.array([25.0, 25.0, 5.0])
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.active_waypoints = [stale]
    node.navigation_goals = [stale]
    node._enter_explore()

    assert node.phase == "EXPLORE"
    assert node.active_waypoints == []
    assert node.navigation_goals == []
    target, _ = node._select_target_and_yaw()
    assert np.allclose(target, node.current_position_enu)


def test_yolo_target_preempts_explore_even_when_current_path_is_blocked(
    controller_node, monkeypatch
):
    node = controller_node
    node.active = True
    node.phase = mission.PHASE_EXPLORE
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.target_observation = mission.TargetObservation(
        class_name="gas_cylinders",
        confidence=0.9,
        bbox_xyxy=[0.0, 0.0, 10.0, 10.0],
        position_enu=np.array([3.0, 0.0, 5.0]),
        stamp_ns=node.get_clock().now().nanoseconds,
    )

    def safe_inspection_route(goals):
        assert goals
        node.active_waypoints = [node.current_position_enu.copy()]
        node.goal_waypoint_indices = {0}

    monkeypatch.setattr(node, "_set_navigation_goals", safe_inspection_route)
    monkeypatch.setattr(node, "_safety_stop_required", lambda target: (True, "path blocked"))
    monkeypatch.setattr(node, "_maybe_engage_offboard", lambda: None)
    node._timer_callback()
    assert node.phase == mission.PHASE_TARGETING_CYCLE


def test_return_home_holds_without_known_free_route(controller_node, monkeypatch):
    node = controller_node
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.home_position_enu = np.array([5.0, 0.0, 0.0])
    node.phase = mission.PHASE_EXPLORE
    def unsafe_legacy_route(goals):
        raise AssertionError("legacy planner can fall back to a direct unsafe route")

    monkeypatch.setattr(node, "_set_navigation_goals", unsafe_legacy_route)

    node._enter_return_home("exploration budget exhausted")
    monkeypatch.setattr(node, "_map_is_fresh", lambda: True)
    node.voxel_resolution_m = 0.25
    node.map_observation_revision = 1
    node.return_job_revision = 1
    node.return_future = Future()
    node.return_future.set_result([])
    node._prepare_return_home()
    assert node.phase == mission.PHASE_RETURN_HOME
    assert node.active_waypoints == []
    target, _ = node._select_target_and_yaw()
    assert np.allclose(target, node.current_position_enu)


def test_explore_holds_when_3d_snapshot_is_missing(controller_node):
    node = controller_node
    node.phase = mission.PHASE_EXPLORE
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.home_position_enu = np.array([0.0, 0.0, 0.0])
    node.active_waypoints = [np.array([2.0, 0.0, 5.0])]
    node.map_snapshot_complete = False

    node._prepare_exploration()
    target, _ = node._select_target_and_yaw()
    assert node.active_waypoints == []
    assert np.allclose(target, node.current_position_enu)
    assert "3D voxel" in node.explore_wait_reason


def test_controller_publishes_only_a_short_local_route(controller_node):
    node = controller_node
    start = np.array([0.0, 0.0, 5.0])
    node.current_position_enu = start
    distant_goal = np.array([5.0, 0.0, 5.0])

    node._set_local_route([start, np.array([1.0, 0.0, 5.0]), distant_goal], 2.0)
    assert node.active_waypoints
    assert not np.allclose(node.active_waypoints[-1], distant_goal)
    assert np.linalg.norm(node.active_waypoints[-1] - start) <= 2.0 + 1e-9


def test_completed_async_frontier_plan_publishes_only_local_segment(controller_node, monkeypatch):
    from px4_powerplant_mission.path_planning.frontier_3d import ExplorationPlan

    node = controller_node
    start = np.array([0.125, 0.125, 5.125])
    endpoint = np.array([0.625, 0.125, 5.125])
    node.phase = mission.PHASE_EXPLORE
    node.current_position_enu = start
    node.home_position_enu = np.array([0.125, 0.125, 0.0])
    node.odom_position_variance_m2 = 0.2
    node.voxel_resolution_m = 0.25
    node.map_observation_revision = 1
    node.explore_job_revision = 1
    node.explore_future = Future()
    node.explore_future.set_result(ExplorationPlan(
        goal_xyz=np.array([3.125, 0.125, 5.125]),
        goal_yaw_rad=1.0,
        local_path_xyz=[start, endpoint],
        frontier_key=(13, 0, 20),
        score=3.0,
        visible_unknown_count=12,
        surface_gain=0.0,
        outbound_time_s=4.0,
        return_time_s=5.0,
        clearance_m=0.5,
        localization_variance_m2=0.2,
        reserve_margin_s=60.0,
    ))
    monkeypatch.setattr(node, "_map_is_fresh", lambda: True)
    monkeypatch.setattr(node, "_localization_is_ready", lambda: True)
    monkeypatch.setattr(node, "_battery_is_low", lambda: False)
    monkeypatch.setattr(node, "_remaining_flight_s", lambda: 100.0)
    monkeypatch.setattr(mission, "path_is_known_free", lambda *args, **kwargs: True)

    node._prepare_exploration()
    assert node.explore_future is None
    assert len(node.active_waypoints) == 1
    assert np.allclose(node.active_waypoints[0], endpoint)
    assert node.explore_plan_revision == 1


def test_yolo_entry_clears_prior_explore_motion_while_photo_route_plans(
    controller_node, monkeypatch
):
    node = controller_node
    node.phase = mission.PHASE_EXPLORE
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.active_waypoints = [np.array([2.0, 0.0, 5.0])]
    node.target_observation = mission.TargetObservation(
        class_name="gas_cylinders", confidence=0.9,
        bbox_xyxy=[0.0, 0.0, 10.0, 10.0],
        position_enu=np.array([3.0, 0.0, 5.0]),
        stamp_ns=node.get_clock().now().nanoseconds,
    )
    monkeypatch.setattr(node, "_set_navigation_goals", lambda goals: None)
    node._enter_targeting_cycle()

    assert node.phase == mission.PHASE_TARGETING_CYCLE
    assert node.active_waypoints == []
    assert node.waypoint_index == 0


def test_completed_async_return_plan_stays_on_short_known_free_segment(
    controller_node, monkeypatch
):
    node = controller_node
    node.current_position_enu = np.array([0.125, 0.125, 5.125])
    node.home_position_enu = np.array([4.125, 0.125, 0.0])
    node.voxel_resolution_m = 0.25
    node.map_observation_revision = 2
    node._enter_return_home("inspection complete")
    node.return_job_revision = 2
    node.return_future = Future()
    node.return_future.set_result([
        node.current_position_enu.copy(),
        np.array([1.125, 0.125, 5.125]),
        np.array([4.125, 0.125, 5.0]),
    ])
    monkeypatch.setattr(node, "_map_is_fresh", lambda: True)
    monkeypatch.setattr(mission, "path_is_known_free", lambda *args, **kwargs: True)
    node._prepare_return_home()

    assert node.phase == mission.PHASE_RETURN_HOME
    assert node.active_waypoints
    assert np.linalg.norm(node.active_waypoints[-1] - node.current_position_enu) <= 2.0 + 1e-9


def test_reached_viewpoint_waits_for_yaw_and_new_depth_fusion(controller_node):
    node = controller_node
    goal = np.array([0.125, 0.125, 5.125])
    node.phase = mission.PHASE_EXPLORE
    node.current_position_enu = goal.copy()
    node.current_yaw_enu = 0.0
    node.explore_goal_yaw_enu = 0.0
    node.explore_goal_enu = goal.copy()
    node.voxel_resolution_m = 0.25
    node.active_waypoints = [goal.copy()]
    node.map_fusion_depth_count = 2

    node._update_phase(goal, 0.0)
    assert node.active_waypoints
    node.explore_viewpoint_entered_ns = node.get_clock().now().nanoseconds - 1_000_000_000
    node._update_phase(goal, 0.0)
    assert node.active_waypoints  # A fresh camera fusion is still required.
    node.map_fusion_depth_count = 3
    node._update_phase(goal, 0.0)
    assert node.active_waypoints == []
    assert (0, 0, 20) in node.visited_viewpoint_keys


def test_unconnected_battery_uses_mission_time_budget(controller_node):
    node = controller_node
    node.mission_started_ns = node.get_clock().now().nanoseconds - 10_000_000_000
    node.battery_connected = False
    node.battery_remaining_fraction = 0.0
    node.battery_time_remaining_s = 1.0
    node.battery_stamp_ns = node.get_clock().now().nanoseconds

    assert not node._battery_is_low()
    assert node._remaining_flight_s() > 280.0

    node.battery_connected = True
    assert node._battery_is_low()
    assert node._remaining_flight_s() <= 1.0


def test_explore_map_requires_recent_fused_camera_and_lidar(controller_node):
    node = controller_node
    now_ns = node.get_clock().now().nanoseconds
    node.map_snapshot_complete = True
    node.map_snapshot_received_ns = now_ns
    node.voxel_resolution_m = 0.25
    node.free_voxel_keys = {(0, 0, 20)}
    node.map_fusion_lidar_received_ns = now_ns
    node.map_fusion_depth_received_ns = now_ns - 4_000_000_000
    assert not node._map_is_fresh()
    node.map_fusion_depth_received_ns = now_ns
    assert node._map_is_fresh()


def test_async_inspection_route_keeps_photo_state_machine(controller_node, monkeypatch):
    node = controller_node
    goal = np.array([0.125, 0.125, 5.125])
    node.phase = mission.PHASE_TARGETING_CYCLE
    node.current_position_enu = goal.copy()
    node.current_yaw_enu = 0.0
    node.voxel_resolution_m = 0.25
    node.map_observation_revision = 3
    node.target_job_revision = 3
    node.target_future = Future()
    node.target_future.set_result(([goal.copy()], {0}, {0: 0}))
    node.target_observation = mission.TargetObservation(
        class_name="gas_cylinders", confidence=0.9,
        bbox_xyxy=[0.0, 0.0, 10.0, 10.0],
        position_enu=np.array([2.0, 0.125, 5.125]),
        stamp_ns=node.get_clock().now().nanoseconds,
    )
    monkeypatch.setattr(node, "_map_is_fresh", lambda: True)
    monkeypatch.setattr(mission, "path_is_known_free", lambda *args, **kwargs: True)
    monkeypatch.setattr(node, "_capture_inspection_photo", lambda index, goal_id: True)

    node._maybe_replan_path()
    assert len(node.active_waypoints) == 1
    node._update_targeting_cycle(goal, 0.0)
    assert node.inspection_photo_count == 1
    assert node.phase == mission.PHASE_RETURN_HOME


def test_return_home_enters_landing_at_home_air_point(controller_node):
    node = controller_node
    node.current_position_enu = np.array([0.0, 0.0, 5.0])
    node.home_position_enu = np.array([0.0, 0.0, 0.0])
    node._enter_return_home("inspection complete")
    node._prepare_return_home()
    assert node.phase == mission.PHASE_LANDING
    assert node.landing_command_sent
