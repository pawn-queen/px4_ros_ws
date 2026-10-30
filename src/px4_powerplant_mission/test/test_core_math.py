import math

import numpy as np

from px4_powerplant_mission.common.frames import (
    enu_to_ned,
    ned_to_enu,
    quaternion_xyzw_to_roll_pitch,
    yaw_enu_to_ned,
    yaw_ned_to_enu,
)
from px4_powerplant_mission.localization.uwb import trilaterate


def test_ned_enu_roundtrip():
    ned = np.array([1.0, 2.0, -3.0])
    assert np.allclose(enu_to_ned(ned_to_enu(ned)), ned)


def test_yaw_roundtrip():
    yaw = 0.7
    assert math.isclose(yaw_enu_to_ned(yaw_ned_to_enu(yaw)), yaw)


def test_roll_pitch_from_quaternion():
    roll = 0.2
    pitch = -0.15
    cy = math.cos(0.0)
    sy = math.sin(0.0)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]

    solved_roll, solved_pitch = quaternion_xyzw_to_roll_pitch(q)

    assert math.isclose(solved_roll, roll, abs_tol=1e-6)
    assert math.isclose(solved_pitch, pitch, abs_tol=1e-6)


def test_uwb_trilateration():
    anchors = np.array(
        [
            [-3.0, -3.0, 2.0],
            [3.0, -3.0, 2.0],
            [3.0, 3.0, 2.0],
            [-3.0, 3.0, 2.0],
            [0.0, 0.0, 5.0],
        ]
    )
    truth = np.array([0.8, -0.4, 1.7])
    ranges = np.linalg.norm(anchors - truth, axis=1)
    solution, rms = trilaterate(anchors, ranges, initial_enu=[0.0, 0.0, 1.0])
    assert np.allclose(solution, truth, atol=1e-2)
    assert rms < 1e-3
