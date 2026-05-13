"""A compact probabilistic voxel grid for simulation mapping."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np


@dataclass
class VoxelGrid:
    resolution: float
    hit_log_odds: float = 0.85
    min_log_odds: float = -4.0
    max_log_odds: float = 4.0
    occupied_threshold: float = 0.0
    cells: dict[tuple[int, int, int], float] = field(default_factory=dict)

    def key_from_point(self, point: Sequence[float]) -> tuple[int, int, int]:
        arr = np.asarray(point, dtype=float)
        return tuple(np.floor(arr / self.resolution).astype(int).tolist())

    def center_from_key(self, key: tuple[int, int, int]) -> np.ndarray:
        return (np.asarray(key, dtype=float) + 0.5) * self.resolution

    def insert_points(self, points_enu: Iterable[Sequence[float]]) -> int:
        inserted = 0
        for point in points_enu:
            arr = np.asarray(point, dtype=float)
            if not np.all(np.isfinite(arr)):
                continue
            key = self.key_from_point(arr)
            value = self.cells.get(key, 0.0) + self.hit_log_odds
            self.cells[key] = float(np.clip(value, self.min_log_odds, self.max_log_odds))
            inserted += 1
        return inserted

    def occupied_points(self, max_points: int | None = None) -> np.ndarray:
        keys = [key for key, value in self.cells.items() if value >= self.occupied_threshold]
        if max_points is not None and len(keys) > max_points:
            step = max(1, len(keys) // max_points)
            keys = keys[::step][:max_points]
        if not keys:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray([self.center_from_key(key) for key in keys], dtype=np.float32)

    def to_occupancy_grid_2d(
        self,
        z_min: float,
        z_max: float,
        padding_m: float = 1.0,
    ) -> tuple[np.ndarray, tuple[float, float], float] | None:
        occupied = [
            self.center_from_key(key)
            for key, value in self.cells.items()
            if value >= self.occupied_threshold and z_min <= self.center_from_key(key)[2] <= z_max
        ]
        if not occupied:
            return None

        points = np.asarray(occupied, dtype=float)
        min_xy = np.floor((np.min(points[:, :2], axis=0) - padding_m) / self.resolution) * self.resolution
        max_xy = np.ceil((np.max(points[:, :2], axis=0) + padding_m) / self.resolution) * self.resolution
        width = int(max(1, np.ceil((max_xy[0] - min_xy[0]) / self.resolution)))
        height = int(max(1, np.ceil((max_xy[1] - min_xy[1]) / self.resolution)))
        grid = np.zeros((height, width), dtype=np.int8)

        for point in points:
            ix = int((point[0] - min_xy[0]) / self.resolution)
            iy = int((point[1] - min_xy[1]) / self.resolution)
            if 0 <= ix < width and 0 <= iy < height:
                grid[iy, ix] = 100
        return grid, (float(min_xy[0]), float(min_xy[1])), self.resolution

