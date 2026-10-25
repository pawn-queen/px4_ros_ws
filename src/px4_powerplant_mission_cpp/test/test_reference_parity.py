"""Retained geometry contracts plus an independent exact-ray oracle for P1.

P1 deliberately changes ray cells and flight-time scoring. Legacy planner
scenarios run with explicit zero observation/negligible yaw cost; production
budgets and calibrated views have separate native P1 tests.
"""
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest
from px4_powerplant_mission.mapping.voxel_grid import VoxelGrid
from px4_powerplant_mission.path_planning import frontier_3d as reference

ROOT = Path(__file__).resolve().parents[2]


def encode(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (set, frozenset)):
        return list(value)
    raise TypeError(type(value))


@pytest.fixture(scope="module")
def native():
    executable = os.environ.get("POWERPLANT_CORE_PROBE", str(ROOT.parent / "build/px4_powerplant_mission_cpp/core_probe"))
    proc = subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def call(**payload):
        proc.stdin.write(json.dumps(payload, default=encode, allow_nan=False) + "\n")
        proc.stdin.flush()
        output = proc.stdout.readline()
        assert output, f"native probe exited: {proc.poll()}"
        result = json.loads(output)
        if isinstance(result, dict) and "error" in result:
            raise ValueError(result["error"])
        return result

    yield call
    proc.stdin.close()
    proc.wait(timeout=5)
    assert proc.returncode == 0


def adapters(native):
    def common(op, free, occupied, resolution, bounds, config=None, **kw):
        cfg = dataclasses.asdict(config or reference.ExplorationConfig())
        if op == "plan":
            cfg.update(yaw_rate_radps=1e300, viewpoint_dwell_s=0.)
        return native(op=op, free=free, occupied=occupied, resolution=resolution,
                      bounds=bounds, config=cfg, **kw)

    def plan_path(free, occupied, resolution, bounds, start, goal, config=None):
        return [np.array(p) for p in common("path", free, occupied, resolution, bounds, config, start=start, goal=goal)]

    def safe(free, occupied, resolution, bounds, path, config=None):
        return common("safe", free, occupied, resolution, bounds, config, path=path)

    def frontiers(free, occupied, bounds):
        return {tuple(item["key"]): {tuple(k) for k in item["unknown"]}
                for item in common("frontiers", free, occupied, 1.0, bounds)}

    def plan(free, occupied, resolution, bounds, current, home, *, remaining_flight_s,
             position_variance_m2=0, surface_gain_by_key=None, localization_variance_by_key=None,
             visited_goal_keys=None, config=None):
        result = common("plan", free, occupied, resolution, bounds, config, start=current, goal=home,
                        remaining=remaining_flight_s, variance=position_variance_m2,
                        surfaces=[[*k, v] for k, v in (surface_gain_by_key or {}).items()],
                        variances=[[*k, v] for k, v in (localization_variance_by_key or {}).items()],
                        visited=visited_goal_keys or [])
        if result is None:
            return None
        result["goal_xyz"] = np.array(result["goal_xyz"])
        result["local_path_xyz"] = [np.array(p) for p in result["local_path_xyz"]]
        result["frontier_key"] = tuple(result["frontier_key"])
        return SimpleNamespace(**result)

    return dict(plan_known_free_path=plan_path, path_is_known_free=safe,
                extract_frontiers=frontiers, plan_frontier=plan)


spec = importlib.util.spec_from_file_location("retained_frontier_scenarios", ROOT / "px4_powerplant_mission/test/test_frontier_3d.py")
scenarios = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenarios)
SCENARIOS = [name for name in vars(scenarios) if name.startswith("test_")
             and name not in {"test_visibility_ray_stops_at_first_unknown_voxel", "test_view_gain_counts_first_unknown_on_slab_surface"}]


@pytest.mark.parametrize("name", SCENARIOS)
def test_original_planner_scenarios_on_cpp(native, monkeypatch, name):
    for key, value in adapters(native).items():
        monkeypatch.setattr(scenarios, key, value)
    getattr(scenarios, name)()


@pytest.mark.parametrize("seed", range(8))
def test_exact_voxel_rays_match_independent_plane_intersection_oracle(native, seed):
    rng = np.random.default_rng(seed)
    grid = VoxelGrid(.25)
    frames = []
    for _ in range(4):
        origin = rng.uniform(-1, 1, 3)
        hits = rng.uniform(-3, 3, (24, 3))
        hits = np.concatenate([hits, hits[:4]])  # duplicate rays must update once per frame
        frames.append(dict(origin=origin, points=hits))
        # Independent reference: collect every segment/grid-plane intersection,
        # sort globally, then classify interval midpoints. No DDA step recurrence
        # or legacy fixed-distance sampler is used here.
        hit_keys = {tuple(np.floor(hit/.25).astype(int)) for hit in hits}
        free_keys = set()
        for hit in hits:
            delta = hit-origin
            if np.all(delta == 0):
                continue
            crossings = {0., 1.}
            for axis in range(3):
                if delta[axis] == 0:
                    continue
                lo, hi = sorted((origin[axis]/.25, hit[axis]/.25))
                for plane in range(int(np.floor(lo))+1, int(np.ceil(hi))):
                    t = (plane*.25-origin[axis])/delta[axis]
                    if 0 < t < 1:
                        crossings.add(float(t))
            times = sorted(crossings)
            for begin, end in zip(times, times[1:]):
                if end > begin:
                    point = origin+delta*((begin+end)*.5)
                    free_keys.add(tuple(np.floor(point/.25).astype(int)))
        for key in free_keys-hit_keys:
            grid.cells[key] = float(np.clip(grid.cells.get(key,0.)+grid.miss_log_odds,grid.min_log_odds,grid.max_log_odds))
        for key in hit_keys:
            grid.cells[key] = float(np.clip(grid.cells.get(key,0.)+grid.hit_log_odds,grid.min_log_odds,grid.max_log_odds))
        grid.revision += 1
    result = native(op="voxel", resolution=.25, frames=frames, limit=5,
                    grid_bounds=[-4, -4, 4, 4], zmin=-2.0, zmax=2.0)
    actual = {tuple(row[:3]): row[3] for row in result["cells"]}
    assert actual.keys() == grid.cells.keys()
    for key in grid.cells:
        assert actual[key] == pytest.approx(grid.cells[key], abs=1e-12)
    snapshot = grid.observed_snapshot(5)
    assert result["complete"] == snapshot.complete
    assert result["free"] == snapshot.free_count
    assert result["occupied"] == snapshot.occupied_count
    assert result["uncertain"] == snapshot.uncertain_count
    expected, _, _ = grid.to_occupancy_grid_2d(-2, 2, bounds_xy=(-4, -4, 4, 4))
    assert result["grid"] == expected.reshape(-1).tolist()
    assert result["revision"] == grid.revision


@pytest.mark.parametrize("seed", range(8))
def test_random_routes_match_python(native, seed):
    rng = np.random.default_rng(seed)
    free = {(x, y, z) for x in range(-3, 4) for y in range(-3, 4) for z in range(3)}
    occupied = {key for key in sorted(free) if rng.random() < .18}
    start, goal = (-2.5, -2.5, .5), (3.5, 3.5, 2.5)
    occupied -= {(-3, -3, 0), (3, 3, 2)}
    free -= occupied
    bounds = ((-3, -3, 0), (3, 3, 2))
    config = reference.ExplorationConfig(clearance_radius_m=0)
    expected = reference.plan_known_free_path(free, occupied, 1, bounds, start, goal, config)
    actual = adapters(native)["plan_known_free_path"](free, occupied, 1, bounds, start, goal, config)
    assert bool(actual) == bool(expected)
    if actual:
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        assert reference.path_is_known_free(free, occupied, 1, bounds, actual, config)


def test_hit_priority_unknown_and_invalid_configuration(native):
    result = native(op="voxel", resolution=1.0, frames=[dict(origin=[.5, .5, .5], points=[[2.5, .5, .5], [4.5, .5, .5]])])
    cells = {tuple(row[:3]): row[3] for row in result["cells"]}
    assert cells[(2, 0, 0)] == pytest.approx(.85)
    assert cells[(3, 0, 0)] == pytest.approx(-.4)
    with pytest.raises(ValueError):
        adapters(native)["plan_known_free_path"]({(0, 0, 0)}, set(), 0, ((0, 0, 0), (1, 1, 1)), [.5]*3, [.5]*3)
