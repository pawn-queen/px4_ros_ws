import math

import numpy as np

from px4_powerplant_mission.common.frames import enu_to_ned, ned_to_enu, yaw_enu_to_ned, yaw_ned_to_enu
from px4_powerplant_mission.localization.uwb import trilaterate
from px4_powerplant_mission.path_planning.grid_astar import GridInfo, plan_astar


def test_ned_enu_roundtrip():
    ned = np.array([1.0, 2.0, -3.0])
    assert np.allclose(enu_to_ned(ned_to_enu(ned)), ned)


def test_yaw_roundtrip():
    yaw = 0.7
    assert math.isclose(yaw_enu_to_ned(yaw_ned_to_enu(yaw)), yaw)


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


def test_astar_avoids_obstacle_gap():
    grid = np.zeros((10, 10), dtype=np.int8)
    grid[5, :] = 100
    grid[5, 4] = 0
    info = GridInfo(origin_x=0.0, origin_y=0.0, resolution=1.0, width=10, height=10)
    path = plan_astar(grid, info, start_xy=[1.5, 1.5], goal_xy=[8.5, 8.5], inflation_radius_m=0.0)
    assert path
    assert any(4.0 <= x <= 5.0 and 5.0 <= y <= 6.0 for x, y in path)

