"""2D A* planner on nav_msgs/OccupancyGrid style arrays."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class GridInfo:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int


NEIGHBORS = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
]


def world_to_grid(x: float, y: float, info: GridInfo) -> tuple[int, int]:
    return (
        int((x - info.origin_x) / info.resolution),
        int((y - info.origin_y) / info.resolution),
    )


def grid_to_world(ix: int, iy: int, info: GridInfo) -> tuple[float, float]:
    return (
        info.origin_x + (ix + 0.5) * info.resolution,
        info.origin_y + (iy + 0.5) * info.resolution,
    )


def inflate_obstacles(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return grid.copy()
    inflated = grid.copy()
    occupied = np.argwhere(grid > 50)
    height, width = grid.shape
    for y, x in occupied:
        ymin = max(0, y - radius_cells)
        ymax = min(height, y + radius_cells + 1)
        xmin = max(0, x - radius_cells)
        xmax = min(width, x + radius_cells + 1)
        inflated[ymin:ymax, xmin:xmax] = 100
    return inflated


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def plan_astar(
    grid: np.ndarray,
    info: GridInfo,
    start_xy: Iterable[float],
    goal_xy: Iterable[float],
    obstacle_threshold: int = 50,
    inflation_radius_m: float = 0.4,
    unknown_policy: str = "free",
    unknown_cost: float = 4.0,
) -> list[tuple[float, float]]:
    """Plan a 2D path. Returns an empty list when no valid path exists."""

    unknown_policy = unknown_policy.lower()
    if unknown_policy not in ("free", "cost", "occupied"):
        unknown_policy = "free"
    unknown_cost = max(1.0, float(unknown_cost))
    radius_cells = int(math.ceil(inflation_radius_m / max(info.resolution, 1e-6)))
    costmap = inflate_obstacles(grid, radius_cells)
    start = world_to_grid(float(start_xy[0]), float(start_xy[1]), info)
    goal = world_to_grid(float(goal_xy[0]), float(goal_xy[1]), info)

    def in_bounds(cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < info.width and 0 <= y < info.height

    def traversable(cell: tuple[int, int]) -> bool:
        x, y = cell
        value = int(costmap[y, x])
        if value >= obstacle_threshold:
            return False
        if value < 0 and unknown_policy == "occupied":
            return False
        return True

    def traversal_cost(cell: tuple[int, int], step_cost: float) -> float:
        x, y = cell
        if int(costmap[y, x]) < 0 and unknown_policy == "cost":
            return step_cost * unknown_cost
        return step_cost

    if not in_bounds(start) or not in_bounds(goal):
        return []
    if not traversable(start) or not traversable(goal):
        return []

    frontier: list[tuple[float, tuple[int, int]]] = []
    heappush(frontier, (0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far = {start: 0.0}

    while frontier:
        _, current = heappop(frontier)
        if current == goal:
            cells = _reconstruct(came_from, current)
            return [grid_to_world(ix, iy, info) for ix, iy in cells]

        for dx, dy, step_cost in NEIGHBORS:
            nxt = (current[0] + dx, current[1] + dy)
            if not in_bounds(nxt) or not traversable(nxt):
                continue
            new_cost = cost_so_far[current] + traversal_cost(nxt, step_cost)
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + _heuristic(nxt, goal)
                heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    return []
