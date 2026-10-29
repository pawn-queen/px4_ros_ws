"""Behavior checks for simulated UWB measurement validity and timing."""

from types import SimpleNamespace

import numpy as np

from px4_powerplant_mission.localization.uwb import Anchor
from px4_powerplant_mission.nodes import uwb_simulator_node as uwb_module
from px4_powerplant_mission.nodes.uwb_simulator_node import UwbSimulatorNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeUwb:
    _time_msg = staticmethod(UwbSimulatorNode._time_msg)
    _publish_ranges = UwbSimulatorNode._publish_ranges
    _publish_pose = UwbSimulatorNode._publish_pose
    _publish_quality = UwbSimulatorNode._publish_quality

    def __init__(self):
        positions = [
            (-3.0, -3.0, 2.0),
            (3.0, -3.0, 2.0),
            (3.0, 3.0, 2.0),
            (-3.0, 3.0, 2.0),
            (0.0, 0.0, 5.0),
        ]
        self.anchors = [
            Anchor(f"anchor_{index}", np.asarray(position, dtype=float))
            for index, position in enumerate(positions)
        ]
        self.range_pubs = [_Publisher() for _ in self.anchors]
        self.pose_pub = _Publisher()
        self.quality_pub = _Publisher()
        self.parameters = {
            "range_min_m": 0.05,
            "range_max_m": 80.0,
            "map_frame": "map",
            "publish_truth_orientation": False,
            "truth_yaw_variance_rad2": 0.02,
            "range_noise_std_m": 0.08,
            "truth_timeout_s": 1.0,
            "range_positive_bias_probability": 0.0,
            "range_positive_bias_m": 0.0,
            "max_solution_rms_m": 0.75,
            "max_geometry_condition": 1000.0,
        }
        self.now_ns = 2_000_000_000
        self.truth_sample_ns = 1_800_000_000
        self.truth_position = np.array([0.8, -0.4, 1.7])
        self.last_processed_truth_sample_ns = 0
        self.last_solution_sample_ns = 0
        self.last_solution_enu = None
        self.rng = np.random.default_rng(42)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

    def get_logger(self):
        return SimpleNamespace(warn=lambda *args, **kwargs: None)

    def _current_truth(self):
        return self.truth_position, None, self.truth_sample_ns


def test_invalid_or_saturated_ranges_are_not_published():
    fake = _FakeUwb()
    UwbSimulatorNode._publish_ranges(
        fake, [2.5, float("nan"), 0.05, 80.0, 80.1], 1_800_000_000
    )

    assert [len(publisher.messages) for publisher in fake.range_pubs] == [1, 0, 0, 0, 0]
    valid = fake.range_pubs[0].messages[0]
    assert valid.header.frame_id == "anchor_0"
    assert valid.header.stamp.sec == 1
    assert valid.header.stamp.nanosec == 800_000_000
    assert valid.range == 2.5


def test_repeated_or_older_truth_stamp_does_not_repeat_solve_or_publish(monkeypatch):
    fake = _FakeUwb()
    solve_calls = []
    covariance = np.diag([0.1, 0.2, 0.3])

    monkeypatch.setattr(
        uwb_module,
        "simulate_ranges",
        lambda position, anchors, noise, rng: [2.5] * len(anchors),
    )

    def solve_once(*args, **kwargs):
        solve_calls.append(1)
        return SimpleNamespace(
            position_enu=fake.truth_position.copy(),
            residual_rms_m=0.04,
            position_covariance_m2=covariance,
            used_indices=(0, 1, 2, 3, 4),
            rejected_indices=(),
            geometry_condition=5.0,
        )

    monkeypatch.setattr(uwb_module, "solve_uwb_ranges", solve_once)
    UwbSimulatorNode._timer_callback(fake)
    assert len(solve_calls) == 1
    assert [len(pub.messages) for pub in fake.range_pubs] == [1] * 5
    assert len(fake.pose_pub.messages) == 1
    assert len(fake.quality_pub.messages) == 1

    UwbSimulatorNode._timer_callback(fake)
    fake.truth_sample_ns -= 100_000_000
    UwbSimulatorNode._timer_callback(fake)
    assert len(solve_calls) == 1
    assert [len(pub.messages) for pub in fake.range_pubs] == [1] * 5
    assert len(fake.pose_pub.messages) == 1
    assert len(fake.quality_pub.messages) == 1

    fake.truth_sample_ns += 200_000_000
    UwbSimulatorNode._timer_callback(fake)
    assert len(solve_calls) == 2
    assert len(fake.pose_pub.messages) == 2


def test_published_pose_uses_solved_full_position_covariance():
    fake = _FakeUwb()
    covariance = np.array(
        [
            [0.11, 0.01, -0.02],
            [0.01, 0.22, 0.03],
            [-0.02, 0.03, 0.44],
        ]
    )
    UwbSimulatorNode._publish_pose(
        fake,
        np.array([1.0, 2.0, 3.0]),
        0.9,
        None,
        1_800_000_000,
        position_covariance_m2=covariance,
    )

    msg = fake.pose_pub.messages[0]
    published = np.array(
        [[msg.pose.covariance[row * 6 + column] for column in range(3)] for row in range(3)]
    )
    assert np.allclose(published, covariance)
    assert msg.pose.covariance[0] != 0.9**2
