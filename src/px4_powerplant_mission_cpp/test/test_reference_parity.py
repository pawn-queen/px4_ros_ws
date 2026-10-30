"""Frozen pre-deletion reference contracts for the native planner and voxel grid.

The JSON fixtures under ``test/fixtures`` were generated from the retained Python
implementation before it was retired.  They preserve the old geometry/planner
behaviour without importing Python runtime code in this package's tests.
"""
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / "fixtures"


def load(name):
    return json.loads((FIXTURES / name).read_text())


@pytest.fixture(scope="module")
def native():
    executable = os.environ.get(
        "POWERPLANT_CORE_PROBE",
        str(ROOT.parent / "build/px4_powerplant_mission_cpp/core_probe"),
    )
    proc = subprocess.Popen(
        [executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
    )

    def call(**payload):
        proc.stdin.write(json.dumps(payload, allow_nan=False) + "\n")
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


def test_frozen_frontier_extraction(native):
    case = load("planner_cases.json")["frontier"]
    result = native(
        op="frontiers",
        free=case["free"],
        occupied=case["occupied"],
        resolution=1.0,
        bounds=case["bounds"],
    )
    actual = {tuple(item["key"]): {tuple(k) for k in item["unknown"]} for item in result}
    expected = {tuple(k): {tuple(v) for v in values} for k, values in case["expected"]}
    assert actual == expected


def test_frozen_random_routes(native):
    for case in load("route_cases.json")["cases"]:
        result = native(
            op="path",
            free=case["free"],
            occupied=case["occupied"],
            resolution=1,
            bounds=case["bounds"],
            start=case["start"],
            goal=case["goal"],
            config={"clearance_radius_m": 0},
        )
        expected = case["expected"]
        assert len(result) == len(expected), case["seed"]
        if expected:
            np.testing.assert_allclose(result, expected, atol=1e-9)
            assert native(
                op="safe",
                free=case["free"],
                occupied=case["occupied"],
                resolution=1,
                bounds=case["bounds"],
                path=result,
                config={"clearance_radius_m": 0},
            ) is True


def test_frozen_planner_cases(native):
    for case in load("planner_cases.json")["plans"]:
        result = native(
            op="plan",
            free=case["free"],
            occupied=case["occupied"],
            resolution=1.0,
            bounds=case["bounds"],
            start=case["start"],
            goal=case["goal"],
            remaining=case["remaining"],
            variance=case["variance"],
            config=case["config"],
            surfaces=case["surface"],
            variances=case["variances"],
            visited=case["visited"],
        )
        expected = case["expected"]
        if expected is None:
            assert result in (None, {}), case["name"]
            continue
        assert result, case["name"]
        np.testing.assert_allclose(result["goal_xyz"], expected["goal_xyz"], atol=1e-9)
        assert tuple(result["frontier_key"]) == tuple(expected["frontier_key"]), case["name"]
        assert result["visible_unknown_count"] == expected["visible_unknown_count"]
        assert result["surface_gain"] == pytest.approx(expected["surface_gain"], abs=1e-9)
        np.testing.assert_allclose(
            result["local_path_xyz"], expected["local_path_xyz"], atol=1e-9
        )


def test_frozen_voxel_rays(native):
    for case in load("ray_cases.json")["cases"]:
        result = native(
            op="voxel",
            resolution=0.25,
            frames=case["frames"],
            limit=5,
            grid_bounds=[-4, -4, 4, 4],
            zmin=-2.0,
            zmax=2.0,
        )
        expected = case["expected"]
        actual = {tuple(row[:3]): row[3] for row in result["cells"]}
        expected_cells = {tuple(row[:3]): row[3] for row in expected["cells"]}
        assert actual.keys() == expected_cells.keys(), case["seed"]
        for key, value in expected_cells.items():
            assert actual[key] == pytest.approx(value, abs=1e-12), (case["seed"], key)
        assert result["complete"] == expected["complete"]
        assert result["free"] == expected["free"]
        assert result["occupied"] == expected["occupied"]
        assert result["uncertain"] == expected["uncertain"]
        assert result["grid"] == expected["grid"]
        assert result["revision"] == expected["revision"]


def test_frozen_hit_priority_and_invalid_configuration(native):
    result = native(
        op="voxel",
        resolution=1.0,
        frames=[dict(origin=[0.5, 0.5, 0.5], points=[[2.5, 0.5, 0.5], [4.5, 0.5, 0.5]])],
    )
    cells = {tuple(row[:3]): row[3] for row in result["cells"]}
    assert cells[(2, 0, 0)] == pytest.approx(0.85)
    assert cells[(3, 0, 0)] == pytest.approx(-0.4)
    with pytest.raises(ValueError):
        native(
            op="path",
            free=[[0, 0, 0]],
            occupied=[],
            resolution=0,
            bounds=[[0, 0, 0], [1, 1, 1]],
            start=[0.5, 0.5, 0.5],
            goal=[0.5, 0.5, 0.5],
            config={"clearance_radius_m": 0},
        )
