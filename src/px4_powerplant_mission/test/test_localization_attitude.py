import math
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from px4_powerplant_mission.common.frames import (
    quaternion_xyzw_to_roll_pitch,
    quaternion_xyzw_to_yaw,
)
from px4_powerplant_mission.localization.fusion import FusionState
from px4_powerplant_mission.nodes.localization_node import (
    LocalizationNode,
    UNKNOWN_TILT_VARIANCE_RAD2,
)
from px4_powerplant_mission.nodes.uwb_simulator_node import (
    UwbSimulatorNode,
    UNKNOWN_ORIENTATION_VARIANCE_RAD2,
)


def _quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


class _FakeLocalization:
    _external_tilt_is_recent = LocalizationNode._external_tilt_is_recent
    _expire_tilt_if_stale = LocalizationNode._expire_tilt_if_stale
    _invalidate_tilt = LocalizationNode._invalidate_tilt

    def __init__(self):
        self.px4_attitude = None
        self.state = FusionState()
        self.state.has_position = True
        self.now_ns = 2_000_000_000
        self.last_uwb_stamp_ns = self.now_ns
        self.last_external_yaw_stamp_ns = 0
        self.last_external_tilt_stamp_ns = 0
        self.roll_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
        self.pitch_variance_rad2 = UNKNOWN_TILT_VARIANCE_RAD2
        self.parameters = {
            "use_uwb_orientation": True,
            "max_uwb_yaw_variance_rad2": 0.5,
            "max_uwb_tilt_variance_rad2": 0.5,
            "uwb_recent_timeout_s": 0.5,
            "independent_position_variance_m2": 0.2,
            "independent_height_variance_m2": 0.35,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))

    def _has_independent_position(self):
        return True


class _FakePublisher:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


class _FakeUwb:
    _time_msg = staticmethod(UwbSimulatorNode._time_msg)
    _stamp_ns = staticmethod(UwbSimulatorNode._stamp_ns)

    def __init__(self):
        self.pose_pub = _FakePublisher()
        self.parameters = {
            "map_frame": "map",
            "publish_truth_orientation": True,
            "truth_yaw_variance_rad2": 0.02,
            "range_noise_std_m": 0.08,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


def test_fusion_preserves_roll_pitch_when_yaw_source_changes():
    roll, pitch, yaw = 0.21, -0.14, 1.1
    state = FusionState()
    assert state.correct_tilt(_quaternion(roll, pitch, yaw))
    state.yaw_enu = yaw
    assert np.allclose(state.orientation_xyzw, _quaternion(roll, pitch, yaw), atol=1e-12)

    state.yaw_enu = -0.7
    actual_roll, actual_pitch = quaternion_xyzw_to_roll_pitch(state.orientation_xyzw)
    assert math.isclose(actual_roll, roll, abs_tol=1e-12)
    assert math.isclose(actual_pitch, pitch, abs_tol=1e-12)
    assert math.isclose(quaternion_xyzw_to_yaw(state.orientation_xyzw), -0.7, abs_tol=1e-12)


def test_localization_uses_full_uwb_attitude_and_marks_stale_tilt_unknown():
    fake = _FakeLocalization()
    q = _quaternion(0.18, -0.11, 0.9)
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.orientation.x = float(q[0])
    msg.pose.pose.orientation.y = float(q[1])
    msg.pose.pose.orientation.z = float(q[2])
    msg.pose.pose.orientation.w = float(q[3])
    msg.pose.covariance[21] = 0.02
    msg.pose.covariance[28] = 0.03
    msg.pose.covariance[35] = 0.02

    LocalizationNode._maybe_use_uwb_orientation(fake, msg)
    odom = Odometry()
    LocalizationNode._fill_pose_and_twist(fake, odom)
    roll, pitch = quaternion_xyzw_to_roll_pitch([
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,
    ])
    assert math.isclose(roll, 0.18, abs_tol=1e-12)
    assert math.isclose(pitch, -0.11, abs_tol=1e-12)
    assert odom.pose.covariance[21] == 0.02
    assert odom.pose.covariance[28] == 0.03

    fake.now_ns += 600_000_000
    stale_odom = Odometry()
    LocalizationNode._fill_pose_and_twist(fake, stale_odom)
    assert not fake.state.has_tilt
    assert stale_odom.pose.covariance[21] == UNKNOWN_TILT_VARIANCE_RAD2
    assert stale_odom.pose.covariance[28] == UNKNOWN_TILT_VARIANCE_RAD2
    assert quaternion_xyzw_to_roll_pitch([
        stale_odom.pose.pose.orientation.x,
        stale_odom.pose.pose.orientation.y,
        stale_odom.pose.pose.orientation.z,
        stale_odom.pose.pose.orientation.w,
    ]) == (0.0, 0.0)


def test_uwb_marks_missing_orientation_unknown_and_localization_rejects_it():
    fake_uwb = _FakeUwb()
    q = _quaternion(0.2, -0.1, 0.5)
    UwbSimulatorNode._publish_pose(fake_uwb, np.array([1.0, 2.0, 3.0]), 0.1, q, 2_000_000_000)
    valid = fake_uwb.pose_pub.last
    assert np.allclose(
        [valid.pose.pose.orientation.x, valid.pose.pose.orientation.y,
         valid.pose.pose.orientation.z, valid.pose.pose.orientation.w],
        q,
    )
    assert [valid.pose.covariance[index] for index in (21, 28, 35)] == [0.02] * 3

    fake_uwb.parameters["publish_truth_orientation"] = False
    UwbSimulatorNode._publish_pose(fake_uwb, np.array([1.0, 2.0, 3.0]), 0.1, q, 2_000_000_000)
    missing = fake_uwb.pose_pub.last
    assert missing.pose.pose.orientation.w == 1.0
    assert [missing.pose.covariance[index] for index in (21, 28, 35)] == [
        UNKNOWN_ORIENTATION_VARIANCE_RAD2
    ] * 3
    fake_localization = _FakeLocalization()
    assert fake_localization.state.correct_tilt(q)
    LocalizationNode._maybe_use_uwb_orientation(fake_localization, missing)
    assert not fake_localization.state.has_tilt
    odom = Odometry()
    LocalizationNode._fill_pose_and_twist(fake_localization, odom)
    assert odom.pose.covariance[21] == UNKNOWN_TILT_VARIANCE_RAD2


def test_invalid_new_gazebo_orientation_does_not_reuse_old_truth_attitude():
    fake = _FakeUwb()
    fake.now_ns = 2_000_000_000
    fake.gazebo_pose_orientation_xyzw = _quaternion(0.2, 0.1, 0.5)
    fake._select_truth_pose_transform = lambda msg: msg.transforms[0]
    fake.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=fake.now_ns))

    transform = TransformStamped()
    transform.header.stamp.sec = 2
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 0.0
    # An all-zero quaternion is invalid; a previous sample must not survive it.
    UwbSimulatorNode._truth_pose_callback(fake, TFMessage(transforms=[transform]))
    assert fake.gazebo_pose_orientation_xyzw is None
