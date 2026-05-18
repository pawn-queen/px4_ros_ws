import math

import numpy as np

from px4_powerplant_mission.common.frames import enu_to_ned, ned_to_enu, yaw_enu_to_ned, yaw_ned_to_enu
from px4_powerplant_mission.localization.uwb import trilaterate
from px4_powerplant_mission.mapping.voxel_grid import VoxelGrid
from px4_powerplant_mission.path_planning.grid_astar import GridInfo, plan_astar
from px4_powerplant_mission.path_planning.grid_astar import world_to_grid


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


def test_voxel_grid_ray_clearing_marks_free_unknown_and_occupied():
    grid = VoxelGrid(1.0)
    grid.insert_rays([0.5, 0.5, 0.5], [[3.5, 0.5, 0.5]])

    result = grid.to_occupancy_grid_2d(0.0, 1.0, bounds_xy=(0.0, 0.0, 5.0, 1.0))

    assert result is not None
    occupancy, _, _ = result
    assert occupancy[0, 0] == 0
    assert occupancy[0, 1] == 0
    assert occupancy[0, 3] == 100
    assert occupancy[0, 4] == -1


def test_astar_avoids_unknown_when_unknown_is_costly():
    grid = np.zeros((5, 7), dtype=np.int8)
    grid[2, 1:6] = -1
    info = GridInfo(origin_x=0.0, origin_y=0.0, resolution=1.0, width=7, height=5)

    path = plan_astar(
        grid,
        info,
        start_xy=[0.5, 2.5],
        goal_xy=[6.5, 2.5],
        inflation_radius_m=0.0,
        unknown_policy="cost",
        unknown_cost=10.0,
    )

    assert path
    cells = [world_to_grid(x, y, info) for x, y in path]
    assert not any(y == 2 and 1 <= x <= 5 for x, y in cells)


def test_astar_can_forbid_unknown_cells():
    grid = np.zeros((5, 7), dtype=np.int8)
    grid[2, 1:6] = -1
    info = GridInfo(origin_x=0.0, origin_y=0.0, resolution=1.0, width=7, height=5)

    path = plan_astar(
        grid,
        info,
        start_xy=[0.5, 2.5],
        goal_xy=[6.5, 2.5],
        inflation_radius_m=0.0,
        unknown_policy="occupied",
    )

    assert path
    cells = [world_to_grid(x, y, info) for x, y in path]
    assert not any(y == 2 and 1 <= x <= 5 for x, y in cells)
