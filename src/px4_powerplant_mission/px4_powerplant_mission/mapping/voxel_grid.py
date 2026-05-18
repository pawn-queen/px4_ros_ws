"""A compact probabilistic voxel grid for simulation mapping."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Sequence

import numpy as np


@dataclass
class VoxelGrid:
    resolution: float
    hit_log_odds: float = 0.85
    miss_log_odds: float = -0.4
    min_log_odds: float = -4.0
    max_log_odds: float = 4.0
    occupied_threshold: float = 0.0
    free_threshold: float = -0.2
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

    def insert_rays(
        self,
        origin_enu: Sequence[float],
        hit_points_enu: Iterable[Sequence[float]],
    ) -> int:
        origin = np.asarray(origin_enu, dtype=float)
        if not np.all(np.isfinite(origin)):
            return 0

        hit_keys: set[tuple[int, int, int]] = set()
        free_keys: set[tuple[int, int, int]] = set()
        for point in hit_points_enu:
            hit = np.asarray(point, dtype=float)
            if not np.all(np.isfinite(hit)):
                continue
            hit_key = self.key_from_point(hit)
            hit_keys.add(hit_key)
            free_keys.update(self._ray_free_keys(origin, hit, hit_key))

        for key in free_keys - hit_keys:
            value = self.cells.get(key, 0.0) + self.miss_log_odds
            self.cells[key] = float(np.clip(value, self.min_log_odds, self.max_log_odds))

        for key in hit_keys:
            value = self.cells.get(key, 0.0) + self.hit_log_odds
            self.cells[key] = float(np.clip(value, self.min_log_odds, self.max_log_odds))
        return len(hit_keys)

    def _ray_free_keys(
        self,
        origin: np.ndarray,
        hit: np.ndarray,
        hit_key: tuple[int, int, int],
    ) -> list[tuple[int, int, int]]:
        delta = hit - origin
        distance = float(np.linalg.norm(delta))
        if distance < 1e-6:
            return []

        step_m = max(self.resolution * 0.5, 1e-3)
        steps = max(1, int(math.ceil(distance / step_m)))
        keys: list[tuple[int, int, int]] = []
        previous: tuple[int, int, int] | None = None
        for index in range(steps):
            point = origin + delta * (index / steps)
            key = self.key_from_point(point)
            if key != previous and key != hit_key:
                keys.append(key)
            previous = key
        return keys

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
        bounds_xy: tuple[float, float, float, float] | None = None,
    ) -> tuple[np.ndarray, tuple[float, float], float] | None:
        occupied = []
        free = []
        for key, value in self.cells.items():
            point = self.center_from_key(key)
            if not z_min <= point[2] <= z_max:
                continue
            if value >= self.occupied_threshold:
                occupied.append(point)
            elif value <= self.free_threshold:
                free.append(point)

        if bounds_xy is not None:
            min_xy = np.asarray([bounds_xy[0], bounds_xy[1]], dtype=float)
            max_xy = np.asarray([bounds_xy[2], bounds_xy[3]], dtype=float)
            if not np.all(np.isfinite(min_xy)) or not np.all(np.isfinite(max_xy)):
                return None
            if np.any(max_xy <= min_xy):
                return None
        else:
            observed = occupied + free
            if not observed:
                return None
            points = np.asarray(observed, dtype=float)
            min_xy = np.floor((np.min(points[:, :2], axis=0) - padding_m) / self.resolution) * self.resolution
            max_xy = np.ceil((np.max(points[:, :2], axis=0) + padding_m) / self.resolution) * self.resolution

        width = int(max(1, np.ceil((max_xy[0] - min_xy[0]) / self.resolution)))
        height = int(max(1, np.ceil((max_xy[1] - min_xy[1]) / self.resolution)))
        grid = np.full((height, width), -1, dtype=np.int8)

        for point in free:
            ix = int((point[0] - min_xy[0]) / self.resolution)
            iy = int((point[1] - min_xy[1]) / self.resolution)
            if 0 <= ix < width and 0 <= iy < height:
                grid[iy, ix] = 0
        for point in occupied:
            ix = int((point[0] - min_xy[0]) / self.resolution)
            iy = int((point[1] - min_xy[1]) / self.resolution)
            if 0 <= ix < width and 0 <= iy < height:
                grid[iy, ix] = 100
        return grid, (float(min_xy[0]), float(min_xy[1])), self.resolution
