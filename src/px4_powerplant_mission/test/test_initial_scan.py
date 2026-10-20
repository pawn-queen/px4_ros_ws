import math
from types import SimpleNamespace

import numpy as np

from px4_powerplant_mission.common.frames import wrap_pi
from px4_powerplant_mission.common.initial_scan import YawScanSequence, has_observed_cells
from px4_powerplant_mission.nodes.offboard_mission_node import (
    OffboardMissionNode,
    PHASE_HOVER_SCAN,
    PHASE_SAFE_TAKEOFF,
)
from px4_powerplant_mission.path_planning.grid_astar import GridInfo


def test_yaw_scan_completes_only_after_all_wrapped_headings():
    scan = YawScanSequence(start_yaw_rad=3.0, steps=12, dwell_s=0.2)
    assert scan.target_yaw() < -2.0  # The first step crosses +pi to -pi.
    for step in range(12):
        target = scan.target_yaw()
        now_ns = (step + 1) * 1_000_000_000
        assert not scan.update(target, now_ns, ready=True, tolerance_rad=0.12)
        assert scan.step_index == step
        done = scan.update(target, now_ns + 200_000_000, ready=True, tolerance_rad=0.12)
        assert done is (step == 11)
    assert scan.complete
    assert math.isclose(wrap_pi(scan.target_yaw() - 3.0), 0.0, abs_tol=1e-9)


def test_yaw_scan_requires_continuous_stable_dwell():
    scan = YawScanSequence(start_yaw_rad=0.0, steps=4, dwell_s=0.3)
    target = scan.target_yaw()
    scan.update(target, 1_000_000_000, ready=True, tolerance_rad=0.1)
    scan.update(target, 1_200_000_000, ready=False, tolerance_rad=0.1)
    scan.update(target, 1_300_000_000, ready=True, tolerance_rad=0.1)
    assert scan.step_index == 0
    scan.update(target, 1_600_000_000, ready=True, tolerance_rad=0.1)
    assert scan.step_index == 1


def test_all_unknown_grid_is_not_a_map_observation():
    unknown = np.full((4, 4), -1, dtype=np.int8)
    assert not has_observed_cells(unknown, minimum=1)
    unknown[0, 0] = 0
    assert has_observed_cells(unknown, minimum=1)
    assert not has_observed_cells(unknown, minimum=2)


def test_takeoff_enters_initial_scan_before_search():
    class FakeController:
        phase = PHASE_SAFE_TAKEOFF
        current_position_enu = np.array([0.0, 0.0, 5.0])
        entered_scan = False

        def _has_reached(self, target, tolerance):
            return True

        def _enter_initial_scan(self, target):
            self.entered_scan = True
            self.phase = PHASE_HOVER_SCAN

    controller = FakeController()
    OffboardMissionNode._update_phase(controller, np.array([0.0, 0.0, 5.0]), 0.0)
    assert controller.entered_scan
    assert controller.phase == PHASE_HOVER_SCAN


def test_fresh_but_all_unknown_grid_blocks_scan():
    class FakeController:
        initial_scan_entered_ns = 1_000_000_000
        last_valid_lidar_stamp_ns = 1_500_000_000
        latest_depth_stamp_ns = 1_500_000_000
        grid_stamp_ns = 1_500_000_000
        initial_scan_depth_baseline = 1
        initial_scan_lidar_baseline = 1
        map_fusion_depth_count = 2
        map_fusion_lidar_count = 2
        grid = np.full((5, 5), -1, dtype=np.int8)
        grid_info = GridInfo(origin_x=0.0, origin_y=0.0, resolution=1.0, width=5, height=5)
        initial_scan_anchor_enu = np.array([0.5, 0.5, 5.0])

        def get_parameter(self, name):
            return SimpleNamespace(value={
                "initial_scan_max_sensor_age_s": 2.0,
                "initial_scan_min_known_grid_cells": 2,
            }[name])

        def _age_s(self, stamp_ns):
            return 0.5

    controller = FakeController()
    ready, reason = OffboardMissionNode._initial_scan_data_ready(controller)
    assert not ready
    assert "no actual observations" in reason
    controller.grid[0, :2] = 0
    ready, reason = OffboardMissionNode._initial_scan_data_ready(controller)
    assert ready
    assert reason == ""
    controller.grid[1, 0] = 0
    controller.grid[0, 0] = -1
    ready, reason = OffboardMissionNode._initial_scan_data_ready(controller)
    assert not ready
    assert "anchor" in reason
    controller.grid[0, 0] = 0
    controller.map_fusion_depth_count = 1
    ready, reason = OffboardMissionNode._initial_scan_data_ready(controller)
    assert not ready
    assert "depth data" in reason


def test_scan_waits_for_level_stable_hover():
    class FakeController:
        current_position_enu = np.array([0.0, 0.0, 5.0])
        initial_scan_anchor_enu = np.array([0.0, 0.0, 5.0])
        current_orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
        current_speed_mps = 0.1
        current_yaw_rate_radps = 0.2

        def _localization_is_ready(self):
            return True

        def _tilt_estimate_is_ready(self):
            return True

        def get_parameter(self, name):
            return SimpleNamespace(value={
                "initial_scan_max_position_error_m": 0.5,
                "initial_scan_max_tilt_rad": 0.35,
                "initial_scan_max_speed_mps": 0.35,
                "initial_scan_max_hover_yaw_rate_radps": 0.12,
            }[name])

    controller = FakeController()
    ready, reason = OffboardMissionNode._initial_scan_pose_ready(controller, hover=True)
    assert not ready
    assert "yaw rate" in reason
    ready, _ = OffboardMissionNode._initial_scan_pose_ready(controller, hover=False)
    assert ready
    controller.current_yaw_rate_radps = 0.0
    controller.current_position_enu = np.array([0.6, 0.0, 5.0])
    ready, reason = OffboardMissionNode._initial_scan_pose_ready(controller, hover=True)
    assert not ready
    assert "position" in reason


def test_scan_position_drift_targets_anchor():
    class FakeController:
        phase = PHASE_HOVER_SCAN
        current_position_enu = np.array([0.6, 0.0, 5.0])
        current_yaw_enu = 0.4
        initial_scan_anchor_enu = np.array([0.0, 0.0, 5.0])
        initial_scan_rotating = False
        initial_scan_sequence = YawScanSequence(0.4, 12, 0.3)

        def _initial_scan_pose_ready(self, hover):
            return False, "waiting for position to settle at scan anchor"

        def _initial_scan_data_ready(self):
            return True, ""

    target, yaw = OffboardMissionNode._select_target_and_yaw(FakeController())
    assert np.allclose(target, [0.0, 0.0, 5.0])
    assert math.isclose(yaw, 0.4)


def test_takeoff_waits_for_reliable_roll_pitch_estimate():
    class FakeController:
        current_position_enu = np.array([0.0, 0.0, 0.0])
        command_wait_reason = ""

        def _localization_is_ready(self):
            return True

        def _tilt_estimate_is_ready(self):
            return False

    controller = FakeController()
    assert not OffboardMissionNode._ready_to_initialize_mission(controller)
    assert "roll/pitch" in controller.command_wait_reason


def test_each_scan_heading_requires_new_mapper_fusion():
    class FakeController:
        initial_scan_sequence = YawScanSequence(0.0, 4, 0.0)
        initial_scan_rotating = True
        initial_scan_entered_ns = 1_000_000_000
        initial_scan_hover_since_ns = 0
        initial_scan_wait_reason = ""
        initial_scan_step_depth_baseline = 1
        initial_scan_step_lidar_baseline = 1
        map_fusion_depth_count = 1
        map_fusion_lidar_count = 1
        current_yaw_enu = math.pi / 2.0
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

    controller = FakeController()
    OffboardMissionNode._update_initial_scan(controller)
    assert controller.initial_scan_sequence.step_index == 0
    assert "fusion" in controller.initial_scan_wait_reason
    controller.map_fusion_depth_count = 2
    controller.map_fusion_lidar_count = 2
    OffboardMissionNode._update_initial_scan(controller)
    controller.now_ns += 100_000_000
    OffboardMissionNode._update_initial_scan(controller)
    assert controller.initial_scan_sequence.step_index == 1
    assert not controller.entered_explore
