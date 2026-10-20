"""Behavioral tests for timestamped, quality-aware UWB position fusion."""

from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped

from px4_powerplant_mission.localization.fusion import FusionState
from px4_powerplant_mission.nodes.localization_node import LocalizationNode


class _FakeLogger:
    def warn(self, *_args, **_kwargs):
        pass


class _FakeLocalization:
    _uwb_pose_callback = LocalizationNode._uwb_pose_callback
    _reject_uwb = LocalizationNode._reject_uwb
    _uwb_is_recent = LocalizationNode._uwb_is_recent
    _has_independent_position = LocalizationNode._has_independent_position
    _position_covariance = LocalizationNode._position_covariance
    _correct_from_px4_position = LocalizationNode._correct_from_px4_position
    _correct_velocity_from_px4 = LocalizationNode._correct_velocity_from_px4
    _px4_position_alpha = LocalizationNode._px4_position_alpha

    def __init__(self):
        self.now_ns = 10_000_000_000
        self.state = FusionState()
        self.last_uwb_stamp_ns = 0
        self.last_uwb_sample_stamp_ns = 0
        self.uwb_quality_valid = False
        self.uwb_rejection_reason = ""
        self.uwb_last_innovation_nis = None
        self.uwb_last_measurement_variance_enu_m2 = None
        self.position_process_noise_m2ps = 0.25
        self.parameters = {
            "map_frame": "map",
            "uwb_recent_timeout_s": 0.5,
            "uwb_max_position_variance_m2": 2.0,
            "uwb_max_height_variance_m2": 2.0,
            "uwb_nis_threshold": 16.27,
            "uwb_recovery_timeout_s": 1.0,
            "use_imu_prediction": False,
            "independent_position_variance_m2": 0.2,
            "independent_height_variance_m2": 0.35,
            "fallback_position_variance_m2": 25.0,
            "fallback_height_variance_m2": 9.0,
            "use_px4_position_when_uwb_recent": False,
            "px4_position_alpha_when_uwb_recent": 0.02,
            "px4_position_alpha": 0.2,
            "px4_velocity_alpha": 0.15,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

    def get_logger(self):
        return _FakeLogger()

    def _maybe_use_uwb_orientation(self, _msg):
        pass


def _pose(stamp_ns, position=(0.0, 0.0, 0.0), variance=(0.5, 0.5, 0.5)):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
    msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14] = variance
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_per_axis_gain_uses_predicted_and_measured_variance():
    state = FusionState()
    state.has_position = True
    state.position_variance_enu_m2[:] = 1.0
    gain = state.correct_position_with_variance(
        [1.0, 1.0, 1.0], [1.0, 3.0, 0.5]
    )
    assert gain == pytest.approx([0.5, 0.25, 2.0 / 3.0])
    assert state.position_enu == pytest.approx(gain)
    assert state.position_variance_enu_m2 == pytest.approx(
        [0.5, 0.75, 1.0 / 3.0]
    )


@pytest.mark.parametrize(
    "stamp_ns, variance",
    [
        (0, (0.5, 0.5, 0.5)),
        (9_900_000_000, (0.0, 0.5, 0.5)),
        (9_900_000_000, (float("nan"), 0.5, 0.5)),
        (9_900_000_000, (float("inf"), 0.5, 0.5)),
        (9_900_000_000, (2.1, 0.5, 0.5)),
        (9_900_000_000, (0.5, 0.5, 2.1)),
        (9_000_000_000, (0.5, 0.5, 0.5)),
        (10_200_000_000, (0.5, 0.5, 0.5)),
    ],
)
def test_invalid_time_or_variance_cannot_create_independent_position(stamp_ns, variance):
    node = _FakeLocalization()
    node._uwb_pose_callback(_pose(stamp_ns, variance=variance))
    assert node.last_uwb_stamp_ns == 0
    assert not node._has_independent_position()
    assert node._position_covariance() == pytest.approx([25.0, 25.0, 9.0])


def test_duplicate_and_out_of_order_samples_preserve_previous_quality_until_timeout():
    node = _FakeLocalization()
    node._uwb_pose_callback(_pose(9_900_000_000))
    assert node._has_independent_position()
    assert node.last_uwb_sample_stamp_ns == 9_900_000_000

    node._uwb_pose_callback(_pose(9_900_000_000, position=(2.0, 0.0, 0.0)))
    assert node._has_independent_position()
    assert node.last_uwb_sample_stamp_ns == 9_900_000_000
    assert node.state.position_enu == pytest.approx([0.0, 0.0, 0.0])
    assert node._position_covariance() == pytest.approx([0.525, 0.525, 0.525])

    node._uwb_pose_callback(_pose(9_800_000_000))
    assert "out-of-order" in node.uwb_rejection_reason
    assert node._has_independent_position()
    node.now_ns = 10_100_000_000
    node._uwb_pose_callback(_pose(10_050_000_000, position=(0.1, 0.0, 0.0)))
    assert node._has_independent_position()


def test_lower_measurement_variance_gets_larger_gain_and_output_reflects_quality():
    precise = _FakeLocalization()
    coarse = _FakeLocalization()
    for node in (precise, coarse):
        node._uwb_pose_callback(_pose(9_950_000_000))
        node.now_ns = 10_050_000_000

    precise._uwb_pose_callback(
        _pose(10_050_000_000, position=(0.2, 0.0, 0.0), variance=(0.05, 0.05, 0.05))
    )
    coarse._uwb_pose_callback(
        _pose(10_050_000_000, position=(0.2, 0.0, 0.0), variance=(1.5, 1.5, 1.5))
    )
    assert precise.state.position_enu[0] > coarse.state.position_enu[0]
    assert precise._position_covariance() == pytest.approx([0.2, 0.2, 0.35])
    assert coarse._position_covariance() == pytest.approx([1.5, 1.5, 1.5])


def test_nis_rejects_jump_and_recovery_can_reinitialize_without_px4_prior():
    node = _FakeLocalization()
    node._uwb_pose_callback(_pose(9_950_000_000, position=(0.0, 0.0, 0.0)))
    node.now_ns = 10_050_000_000
    node._uwb_pose_callback(
        _pose(10_050_000_000, position=(10.0, 0.0, 0.0), variance=(0.1, 0.1, 0.1))
    )
    assert "normalized innovation" in node.uwb_rejection_reason
    assert node.last_uwb_sample_stamp_ns == 9_950_000_000
    assert node.state.position_enu == pytest.approx([0.0, 0.0, 0.0])
    assert node._position_covariance() == pytest.approx([25.0, 25.0, 9.0])

    node.now_ns = 11_200_000_000
    node._correct_from_px4_position(np.array([100.0, 0.0, 0.0]), None)
    assert node.state.position_enu[0] > 0.0
    assert not node._has_independent_position()
    assert node._position_covariance() == pytest.approx([25.0, 25.0, 9.0])

    node._uwb_pose_callback(
        _pose(11_200_000_000, position=(5.0, 0.0, 0.0), variance=(0.4, 0.4, 0.4))
    )
    assert node.state.position_enu == pytest.approx([5.0, 0.0, 0.0])
    assert node._has_independent_position()
    assert node._position_covariance() == pytest.approx([0.4, 0.4, 0.4])


def test_wrong_frame_and_asymmetric_covariance_fail_closed():
    node = _FakeLocalization()
    node._uwb_pose_callback(_pose(9_900_000_000))
    assert node._has_independent_position()

    node.now_ns = 10_050_000_000
    wrong_frame = _pose(10_050_000_000)
    wrong_frame.header.frame_id = "odom"
    node._uwb_pose_callback(wrong_frame)
    assert not node._has_independent_position()
    assert node.last_uwb_sample_stamp_ns == 9_900_000_000
    assert node._position_covariance() == pytest.approx([25.0, 25.0, 9.0])

    node.now_ns = 10_100_000_000
    asymmetric = _pose(10_100_000_000)
    asymmetric.pose.covariance[1] = 0.2
    node._uwb_pose_callback(asymmetric)
    assert not node._has_independent_position()
    assert "covariance" in node.uwb_rejection_reason
    assert node.last_uwb_sample_stamp_ns == 9_900_000_000
