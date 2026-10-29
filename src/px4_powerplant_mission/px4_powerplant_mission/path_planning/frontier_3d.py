"""Conservative local exploration in a sparse, observed 3D voxel map.

Only keys explicitly marked free can carry the vehicle. Missing keys are
unknown, including keys between sparse sensor rays. ``bounds_keys`` is the
permitted mission volume (inclusive), not the current observation envelope.
All coordinates are ENU metres.
"""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
import math
from typing import Iterable, Mapping, Sequence

import numpy as np


Key = tuple[int, int, int]
Bounds = tuple[Key, Key]
_NEIGHBORS: tuple[Key, ...] = (
    (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
)


@dataclass(frozen=True)
class ExplorationConfig:
    """Planner limits; ``clearance_radius_m`` is a known-free centre radius.

    For a larger airframe, increase the radius to include the vehicle radius,
    localization uncertainty and a margin. Every voxel centre inside that
    radius must be observed free. Snap limits never authorize unknown travel.
    """

    clearance_radius_m: float = 0.35
    start_snap_radius_m: float = 0.35
    goal_snap_radius_m: float = 0.35
    max_search_radius_m: float = 10.0
    sensor_range_m: float = 8.0
    max_local_path_m: float = 2.0
    min_goal_distance_m: float = 0.75
    candidate_spacing_m: float = 0.75
    max_candidates: int = 96
    max_frontier_targets: int = 400
    cruise_speed_mps: float = 1.0
    return_speed_mps: float = 1.0
    reserve_s: float = 30.0
    max_position_variance_m2: float = 4.0
    camera_horizontal_fov_rad: float = math.radians(75.0)
    camera_vertical_fov_rad: float = math.radians(60.0)
    yaw_bins: int = 12
    information_weight: float = 3.0
    surface_weight: float = 2.0
    travel_time_weight: float = 0.12
    clearance_weight: float = 0.5
    localization_weight: float = 0.5
    reserve_weight: float = 0.5


@dataclass
class ExplorationPlan:
    goal_xyz: np.ndarray
    goal_yaw_rad: float
    local_path_xyz: list[np.ndarray]
    frontier_key: Key
    score: float
    visible_unknown_count: int
    surface_gain: float
    outbound_time_s: float
    return_time_s: float
    clearance_m: float
    localization_variance_m2: float
    reserve_margin_s: float


def _point(point: Sequence[float]) -> np.ndarray:
    result = np.asarray(point, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("expected a finite ENU XYZ point")
    return result


def _key(point: np.ndarray, resolution_m: float) -> Key:
    return tuple(np.floor(point / resolution_m).astype(int).tolist())


def _center(key: Key, resolution_m: float) -> np.ndarray:
    return (np.asarray(key, dtype=float) + 0.5) * resolution_m


def _bounds(bounds_keys: Sequence[Sequence[int]]) -> Bounds:
    if len(bounds_keys) != 2 or any(len(part) != 3 for part in bounds_keys):
        raise ValueError("bounds_keys must contain inclusive min and max XYZ keys")
    low, high = tuple(int(v) for v in bounds_keys[0]), tuple(int(v) for v in bounds_keys[1])
    if any(a > b for a, b in zip(low, high)):
        raise ValueError("bounds_keys minimum exceeds maximum")
    return low, high


def _inside(key: Key, bounds: Bounds) -> bool:
    return all(bounds[0][axis] <= key[axis] <= bounds[1][axis] for axis in range(3))


def _add(a: Key, b: Key) -> Key:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _validate(resolution_m: float, config: ExplorationConfig) -> None:
    if not math.isfinite(resolution_m) or resolution_m <= 0:
        raise ValueError("resolution_m must be positive")
    for name in (
        "clearance_radius_m", "start_snap_radius_m", "goal_snap_radius_m",
        "max_search_radius_m", "sensor_range_m", "max_local_path_m",
        "min_goal_distance_m", "candidate_spacing_m", "cruise_speed_mps",
        "return_speed_mps", "reserve_s", "max_position_variance_m2",
    ):
        value = getattr(config, name)
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
    if config.cruise_speed_mps <= 0 or config.return_speed_mps <= 0:
        raise ValueError("travel speeds must be positive")
    if config.max_candidates < 1 or config.max_frontier_targets < 1 or config.yaw_bins < 1:
        raise ValueError("candidate, target, and yaw limits must be positive")


def extract_frontiers(
    free_keys: Iterable[Key],
    occupied_keys: Iterable[Key],
    bounds_keys: Sequence[Sequence[int]],
) -> dict[Key, set[Key]]:
    """Map each known free boundary key to adjacent unknown keys in bounds."""

    bounds = _bounds(bounds_keys)
    occupied = set(occupied_keys)
    free = set(free_keys) - occupied
    result: dict[Key, set[Key]] = {}
    for key in free:
        if not _inside(key, bounds):
            continue
        unknown = {
            neighbor for offset in _NEIGHBORS
            if _inside((neighbor := _add(key, offset)), bounds)
            and neighbor not in free and neighbor not in occupied
        }
        if unknown:
            result[key] = unknown
    return result


def _clearance_offsets(radius_m: float, resolution_m: float) -> list[Key]:
    extent = int(math.ceil(radius_m / resolution_m))
    return [
        (x, y, z)
        for x in range(-extent, extent + 1)
        for y in range(-extent, extent + 1)
        for z in range(-extent, extent + 1)
        if (x * x + y * y + z * z) * resolution_m ** 2 <= radius_m ** 2 + 1e-12
    ]


def _safe_keys(free: set[Key], bounds: Bounds, resolution_m: float, radius_m: float) -> set[Key]:
    offsets = _clearance_offsets(radius_m, resolution_m)
    return {
        key for key in free
        if all((neighbor := _add(key, offset)) in free and _inside(neighbor, bounds)
               for offset in offsets)
    }


def _segment_safe(start: np.ndarray, end: np.ndarray, safe: set[Key], resolution_m: float) -> bool:
    length = float(np.linalg.norm(end - start))
    steps = max(1, int(math.ceil(length / (resolution_m * 0.25))))
    return all(_key(start + (end - start) * (step / steps), resolution_m) in safe
               for step in range(steps + 1))


def _snap_endpoint(
    point: np.ndarray, safe: set[Key], resolution_m: float, max_distance_m: float,
) -> Key | None:
    direct = _key(point, resolution_m)
    if direct in safe:
        return direct
    if max_distance_m <= 0:
        return None
    nearby = sorted(
        (key for key in safe if np.linalg.norm(_center(key, resolution_m) - point) <= max_distance_m),
        key=lambda key: (float(np.linalg.norm(_center(key, resolution_m) - point)), key),
    )
    for key in nearby:
        if _segment_safe(point, _center(key, resolution_m), safe, resolution_m):
            return key
    return None


def _dijkstra(safe: set[Key], source: Key, *, limit_m: float | None, resolution_m: float):
    distances: dict[Key, float] = {source: 0.0}
    previous: dict[Key, Key] = {}
    heap: list[tuple[float, Key]] = [(0.0, source)]
    while heap:
        distance, current = heappop(heap)
        if distance > distances[current] + 1e-9:
            continue
        for offset in _NEIGHBORS:
            nxt = _add(current, offset)
            next_distance = distance + resolution_m
            if nxt not in safe or (limit_m is not None and next_distance > limit_m + 1e-9):
                continue
            if next_distance < distances.get(nxt, math.inf):
                distances[nxt] = next_distance
                previous[nxt] = current
                heappush(heap, (next_distance, nxt))
    return distances, previous


def _route(previous: Mapping[Key, Key], start: Key, goal: Key) -> list[Key]:
    route = [goal]
    while route[-1] != start:
        nxt = previous.get(route[-1])
        if nxt is None:
            return []
        route.append(nxt)
    route.reverse()
    return route


def _route_points(
    route: list[Key], start: np.ndarray, goal: np.ndarray, safe: set[Key], resolution_m: float,
) -> list[np.ndarray]:
    if not route:
        return []
    centres = [_center(key, resolution_m) for key in route]
    if not _segment_safe(start, centres[0], safe, resolution_m):
        return []
    if not _segment_safe(centres[-1], goal, safe, resolution_m):
        return []
    result = [start.copy()]
    for centre in centres:
        if np.linalg.norm(centre - result[-1]) > 1e-9:
            result.append(centre)
    if np.linalg.norm(goal - result[-1]) > 1e-9:
        result.append(goal.copy())
    return result


def plan_known_free_path(
    free_keys: Iterable[Key],
    occupied_keys: Iterable[Key],
    resolution_m: float,
    bounds_keys: Sequence[Sequence[int]],
    start_xyz: Sequence[float],
    goal_xyz: Sequence[float],
    config: ExplorationConfig | None = None,
) -> list[np.ndarray]:
    """Return a full 3D known-free route, including endpoints, or ``[]``.

    Snapping is limited by ``start_snap_radius_m`` and ``goal_snap_radius_m``.
    Even a short snap must remain entirely in safe, observed free voxels.
    """

    config = config or ExplorationConfig()
    _validate(resolution_m, config)
    bounds = _bounds(bounds_keys)
    occupied = set(occupied_keys)
    free = {key for key in free_keys if key not in occupied and _inside(key, bounds)}
    safe = _safe_keys(free, bounds, resolution_m, config.clearance_radius_m)
    start, goal = _point(start_xyz), _point(goal_xyz)
    start_key = _snap_endpoint(start, safe, resolution_m, config.start_snap_radius_m)
    goal_key = _snap_endpoint(goal, safe, resolution_m, config.goal_snap_radius_m)
    if start_key is None or goal_key is None:
        return []
    if start_key == goal_key:
        return _route_points([start_key], start, goal, safe, resolution_m)
    distances, previous = _dijkstra(safe, start_key, limit_m=None, resolution_m=resolution_m)
    if goal_key not in distances:
        return []
    return _route_points(_route(previous, start_key, goal_key), start, goal, safe, resolution_m)


def path_is_known_free(
    free_keys: Iterable[Key],
    occupied_keys: Iterable[Key],
    resolution_m: float,
    bounds_keys: Sequence[Sequence[int]],
    path_xyz: Iterable[Sequence[float]],
    config: ExplorationConfig | None = None,
) -> bool:
    """Check a current local path against the latest observed voxel map.

    Prepend the current position before the remaining waypoints on each
    control cycle. Unknown voxels invalidate even a short segment.
    """

    config = config or ExplorationConfig()
    _validate(resolution_m, config)
    bounds = _bounds(bounds_keys)
    occupied = occupied_keys if isinstance(occupied_keys, (set, frozenset)) else set(occupied_keys)
    free = free_keys if isinstance(free_keys, (set, frozenset)) else set(free_keys)
    points = [_point(point) for point in path_xyz]
    if not points:
        return False
    offsets = _clearance_offsets(config.clearance_radius_m, resolution_m)
    checked: dict[Key, bool] = {}

    def point_is_safe(point: np.ndarray) -> bool:
        key = _key(point, resolution_m)
        if key not in checked:
            checked[key] = all(
                (neighbor := _add(key, offset)) in free
                and neighbor not in occupied and _inside(neighbor, bounds)
                for offset in offsets
            )
        return checked[key]

    if not point_is_safe(points[0]):
        return False
    for start, end in zip(points, points[1:]):
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(math.ceil(distance / (resolution_m * 0.25))))
        if any(
            not point_is_safe(start + (end - start) * (index / steps))
            for index in range(steps + 1)
        ):
            return False
    return True


def _first_non_free(
    origin: np.ndarray, target_key: Key, free: set[Key], resolution_m: float,
) -> Key | None:
    """Return the first non-free voxel on the line toward a target centre."""

    gx, gy, gz = float(origin[0]) / resolution_m, float(origin[1]) / resolution_m, float(origin[2]) / resolution_m
    x, y, z = math.floor(gx), math.floor(gy), math.floor(gz)
    tx, ty, tz = target_key
    dx, dy, dz = tx + 0.5 - gx, ty + 0.5 - gy, tz + 0.5 - gz

    sx = 1 if dx > 0 else -1 if dx < 0 else 0
    sy = 1 if dy > 0 else -1 if dy < 0 else 0
    sz = 1 if dz > 0 else -1 if dz < 0 else 0
    stepx = 1.0 / abs(dx) if sx else math.inf
    stepy = 1.0 / abs(dy) if sy else math.inf
    stepz = 1.0 / abs(dz) if sz else math.inf
    nextx = ((x + 1 - gx) if sx > 0 else (gx - x)) * stepx if sx else math.inf
    nexty = ((y + 1 - gy) if sy > 0 else (gy - y)) * stepy if sy else math.inf
    nextz = ((z + 1 - gz) if sz > 0 else (gz - z)) * stepz if sz else math.inf

    for _ in range(abs(tx - x) + abs(ty - y) + abs(tz - z) + 3):
        key = (x, y, z)
        if key == target_key or key not in free:
            return key
        next_t = min(nextx, nexty, nextz)
        if next_t >= 1.0 + 1e-12:
            return None
        if nextx <= next_t + 1e-12:
            x += sx
            nextx += stepx
        if nexty <= next_t + 1e-12:
            y += sy
            nexty += stepy
        if nextz <= next_t + 1e-12:
            z += sz
            nextz += stepz
    return None


def _visible(
    origin: np.ndarray, target_key: Key, free: set[Key], resolution_m: float,
) -> bool:
    return _first_non_free(origin, target_key, free, resolution_m) == target_key


def _view_gain(
    origin: np.ndarray,
    target_keys: list[Key],
    target_points: np.ndarray,
    target_gains: np.ndarray,
    target_is_unknown: np.ndarray,
    frontier_unknown_keys: set[Key],
    free: set[Key],
    resolution_m: float,
    config: ExplorationConfig,
) -> tuple[int, float, float, Key | None]:
    """Choose yaw from the first unknown voxels actually seen along rays."""

    delta = target_points - origin
    distances = np.linalg.norm(delta, axis=1)
    horizontal = np.hypot(delta[:, 0], delta[:, 1])
    elevations = np.arctan2(delta[:, 2], horizontal)
    geometric = (
        (distances > 1e-9)
        & (distances <= config.sensor_range_m)
        & (np.abs(elevations) <= config.camera_vertical_fov_rad * 0.5)
    )

    first_unknown: set[Key] = set()
    visible_surfaces: list[int] = []
    for index in np.flatnonzero(geometric):
        index = int(index)
        first_key = _first_non_free(origin, target_keys[index], free, resolution_m)
        if first_key is None:
            continue
        if target_is_unknown[index]:
            if first_key in frontier_unknown_keys:
                first_unknown.add(first_key)
        elif first_key == target_keys[index]:
            visible_surfaces.append(index)
    if not first_unknown and not visible_surfaces:
        return 0, 0.0, 0.0, None

    visible_keys = sorted(first_unknown) + [target_keys[index] for index in visible_surfaces]
    visible_points = np.asarray([_center(key, resolution_m) for key in visible_keys])
    visible_gains = np.asarray(
        [0.0] * len(first_unknown)
        + [float(target_gains[index]) for index in visible_surfaces],
    )
    is_unknown = np.asarray(
        [True] * len(first_unknown) + [False] * len(visible_surfaces),
    )
    delta = visible_points - origin
    distances = np.linalg.norm(delta, axis=1)
    azimuths = np.arctan2(delta[:, 1], delta[:, 0])
    elevations = np.arctan2(delta[:, 2], np.hypot(delta[:, 0], delta[:, 1]))
    valid = (distances <= config.sensor_range_m) & (
        np.abs(elevations) <= config.camera_vertical_fov_rad * 0.5
    )
    if not np.any(valid):
        return 0, 0.0, 0.0, None
    azimuths = azimuths[valid]
    distances = distances[valid]
    is_unknown = is_unknown[valid]
    visible_gains = visible_gains[valid]
    valid_keys = [key for key, keep in zip(visible_keys, valid) if keep]

    yaw_options = np.linspace(-math.pi, math.pi, config.yaw_bins, endpoint=False)
    angular_error = np.abs(
        (azimuths[:, None] - yaw_options[None, :] + math.pi) % (2.0 * math.pi) - math.pi
    )
    in_view = angular_error <= config.camera_horizontal_fov_rad * 0.5
    unknown_counts = np.sum(in_view & is_unknown[:, None], axis=0)
    surface_gains = np.sum(in_view * visible_gains[:, None], axis=0)
    scores = (
        config.information_weight * np.log1p(unknown_counts)
        + config.surface_weight * np.log1p(surface_gains)
    )
    best_index = int(np.argmax(scores))
    frontier_indices = np.flatnonzero(in_view[:, best_index] & is_unknown)
    if len(frontier_indices):
        closest = int(frontier_indices[np.argmin(distances[frontier_indices])])
        frontier_key = valid_keys[closest]
    else:
        frontier_key = None
    return (
        int(unknown_counts[best_index]),
        float(surface_gains[best_index]),
        float(yaw_options[best_index]),
        frontier_key,
    )


def _clearance_m(key: Key, free: set[Key], bounds: Bounds, resolution_m: float, cap_m: float) -> float:
    extent = max(1, int(math.ceil(cap_m / resolution_m)))
    closest = cap_m
    for x in range(-extent, extent + 1):
        for y in range(-extent, extent + 1):
            for z in range(-extent, extent + 1):
                distance = math.sqrt(x * x + y * y + z * z) * resolution_m
                if distance >= closest:
                    continue
                neighbor = _add(key, (x, y, z))
                if neighbor not in free or not _inside(neighbor, bounds):
                    closest = distance
    return closest


def _local_segment(path: list[np.ndarray], max_length_m: float, safe: set[Key], resolution_m: float) -> list[np.ndarray]:
    if not path:
        return []
    result = [path[0].copy()]
    travelled = 0.0
    for point in path[1:]:
        length = float(np.linalg.norm(point - result[-1]))
        if travelled + length <= max_length_m + 1e-9:
            result.append(point.copy())
            travelled += length
            continue
        remaining = max_length_m - travelled
        if remaining > 1e-6 and length > 1e-9:
            end = result[-1] + (point - result[-1]) * (remaining / length)
            if _segment_safe(result[-1], end, safe, resolution_m):
                result.append(end)
        break
    return result


def plan_frontier(
    free_keys: Iterable[Key],
    occupied_keys: Iterable[Key],
    resolution_m: float,
    bounds_keys: Sequence[Sequence[int]],
    current_xyz: Sequence[float],
    return_xyz: Sequence[float],
    *,
    remaining_flight_s: float,
    position_variance_m2: float = 0.0,
    surface_gain_by_key: Mapping[Key, float] | None = None,
    localization_variance_by_key: Mapping[Key, float] | None = None,
    visited_goal_keys: Iterable[Key] | None = None,
    config: ExplorationConfig | None = None,
) -> ExplorationPlan | None:
    """Select a reachable safe view, with enough time for known-free return.

    ``None`` means hold/reobserve: no safe start, frontier, view, route, or
    return budget. Only ``local_path_xyz`` should be executed; call again on
    fresh observations or after reaching that short segment's end.
    """

    config = config or ExplorationConfig()
    _validate(resolution_m, config)
    if (not math.isfinite(remaining_flight_s) or remaining_flight_s <= 0
            or not math.isfinite(position_variance_m2) or position_variance_m2 < 0
            or position_variance_m2 > config.max_position_variance_m2):
        return None
    bounds = _bounds(bounds_keys)
    occupied = set(occupied_keys)
    free = {key for key in free_keys if key not in occupied and _inside(key, bounds)}
    frontiers = extract_frontiers(free, occupied, bounds)
    if not frontiers:
        return None
    safe = _safe_keys(free, bounds, resolution_m, config.clearance_radius_m)
    current, return_point = _point(current_xyz), _point(return_xyz)
    current_key = _snap_endpoint(current, safe, resolution_m, config.start_snap_radius_m)
    return_key = _snap_endpoint(return_point, safe, resolution_m, config.goal_snap_radius_m)
    if current_key is None or return_key is None:
        return None
    outward, previous = _dijkstra(
        safe, current_key, limit_m=config.max_search_radius_m, resolution_m=resolution_m,
    )
    homeward, home_previous = _dijkstra(safe, return_key, limit_m=None, resolution_m=resolution_m)
    reachable = set(outward) & set(homeward)
    if not reachable:
        return None

    unknown = set().union(*frontiers.values())
    unknown_targets = sorted(
        unknown,
        key=lambda key: (float(np.linalg.norm(_center(key, resolution_m) - current)), key),
    )[:config.max_frontier_targets]
    if not unknown_targets:
        return None
    target_array = np.asarray([_center(key, resolution_m) for key in unknown_targets])
    surface_gain_by_key = surface_gain_by_key or {}
    localization_variance_by_key = localization_variance_by_key or {}
    surface_items = [
        (key, float(gain)) for key, gain in surface_gain_by_key.items()
        if math.isfinite(float(gain)) and float(gain) > 0
    ]
    surface_array = np.asarray([_center(key, resolution_m) for key, _ in surface_items])
    surface_values = np.asarray([gain for _, gain in surface_items])
    view_keys = unknown_targets + [key for key, _ in surface_items]
    view_points = np.asarray([_center(key, resolution_m) for key in view_keys])
    view_gains = np.concatenate((np.ones(len(unknown_targets)), surface_values))
    view_is_unknown = np.concatenate((
        np.ones(len(unknown_targets), dtype=bool),
        np.zeros(len(surface_items), dtype=bool),
    ))
    start_offset_m = float(np.linalg.norm(current - _center(current_key, resolution_m)))
    return_offset_m = float(np.linalg.norm(return_point - _center(return_key, resolution_m)))
    spacing_cells = max(1, int(round(config.candidate_spacing_m / resolution_m)))
    spatial_candidates: dict[Key, tuple[float, Key]] = {}
    visited = set(visited_goal_keys) if visited_goal_keys is not None else set()
    for key in reachable:
        if key in visited:
            continue
        point = _center(key, resolution_m)
        if np.linalg.norm(point - current) < config.min_goal_distance_m:
            continue
        outbound_s = (outward[key] + start_offset_m) / config.cruise_speed_mps
        return_s = (homeward[key] + return_offset_m) / config.return_speed_mps
        if outbound_s + return_s + config.reserve_s > remaining_flight_s:
            continue
        local_variance = float(localization_variance_by_key.get(key, position_variance_m2))
        if not math.isfinite(local_variance) or local_variance < 0 or local_variance > config.max_position_variance_m2:
            continue
        distances = np.linalg.norm(target_array - point, axis=1)
        unknown_potential = int(np.count_nonzero(distances <= config.sensor_range_m))
        if not unknown_potential:
            continue
        surface_potential = 0.0
        if len(surface_items):
            surface_distances = np.linalg.norm(surface_array - point, axis=1)
            surface_potential = float(np.sum(surface_values[surface_distances <= config.sensor_range_m]))
        bucket = tuple(axis // spacing_cells for axis in key)
        priority = (
            - config.information_weight * math.log1p(unknown_potential)
            - config.surface_weight * math.log1p(surface_potential)
            + config.travel_time_weight * (outbound_s + return_s)
            + config.localization_weight * max(position_variance_m2, local_variance)
        )
        if bucket not in spatial_candidates or priority < spatial_candidates[bucket][0]:
            spatial_candidates[bucket] = priority, key
    candidate_keys = [item[1] for item in sorted(spatial_candidates.values())[:config.max_candidates]]

    best: ExplorationPlan | None = None
    clearance_cache: dict[Key, float] = {}
    for key in candidate_keys:
        point = _center(key, resolution_m)
        route_keys = _route(previous, current_key, key)
        return_route_keys = _route(home_previous, return_key, key)
        if not route_keys or not return_route_keys:
            continue
        outbound_time_s = (outward[key] + start_offset_m) / config.cruise_speed_mps
        return_time_s = (homeward[key] + return_offset_m) / config.return_speed_mps
        reserve_margin_s = remaining_flight_s - outbound_time_s - return_time_s - config.reserve_s
        if reserve_margin_s < 0:
            continue
        path_variances = [
            float(localization_variance_by_key.get(route_key, position_variance_m2))
            for route_key in set(route_keys) | set(return_route_keys)
        ]
        if any(not math.isfinite(value) or value < 0 for value in path_variances):
            continue
        variance = max([position_variance_m2, *path_variances])
        if variance > config.max_position_variance_m2:
            continue
        visible, surface_gain, yaw, frontier_key = _view_gain(
            point, view_keys, view_points, view_gains, view_is_unknown,
            unknown, free, resolution_m, config,
        )
        if visible < 1 or frontier_key is None:
            continue
        for route_key in set(route_keys) | set(return_route_keys):
            if route_key not in clearance_cache:
                clearance_cache[route_key] = _clearance_m(
                    route_key, free, bounds, resolution_m,
                    max(config.clearance_radius_m + resolution_m, 1.0),
                )
        clearance = min(
            clearance_cache[route_key]
            for route_key in set(route_keys) | set(return_route_keys)
        )
        score = (
            config.information_weight * math.log1p(visible)
            + config.surface_weight * math.log1p(surface_gain)
            - config.travel_time_weight * (outbound_time_s + return_time_s)
            + config.clearance_weight * clearance
            - config.localization_weight * variance * (1.0 + outbound_time_s + return_time_s)
            + config.reserve_weight * reserve_margin_s / max(remaining_flight_s, 1.0)
        )
        if best is not None and score <= best.score:
            continue
        full_path = _route_points(route_keys, current, point, safe, resolution_m)
        local_path = _local_segment(full_path, config.max_local_path_m, safe, resolution_m)
        if len(local_path) < 2 or np.linalg.norm(local_path[-1] - local_path[0]) < 1e-6:
            continue
        best = ExplorationPlan(
            goal_xyz=point,
            goal_yaw_rad=yaw,
            local_path_xyz=local_path,
            frontier_key=frontier_key,
            score=score,
            visible_unknown_count=visible,
            surface_gain=surface_gain,
            outbound_time_s=outbound_time_s,
            return_time_s=return_time_s,
            clearance_m=clearance,
            localization_variance_m2=variance,
            reserve_margin_s=reserve_margin_s,
        )
    return best
