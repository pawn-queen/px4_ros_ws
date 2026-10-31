"""原生 runner 入口的只读资产绑定约束；不启动 ROS、Gazebo 或飞行。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

REPO = Path(__file__).resolve().parents[2]
ENTRY = REPO / "scripts/ci/issue3_acceptance.sh"


def asset_program():
    marker = 'python3 - "$workspace" "$px4_root" "$output" <<\'PY\'\n'
    return ENTRY.read_text().split(marker, 1)[1].split('\nPY\n', 1)[0]


def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


@pytest.fixture
def assets(tmp_path):
    workspace, px4, out = (tmp_path / name for name in ("workspace", "px4", "out"))
    root = px4 / "Tools/simulation/gz"
    root.mkdir(parents=True)
    out.mkdir()
    contents = {"worlds/powerplant.sdf": b"scene"}
    contents.update({f"models/{name}/model.sdf": name.encode() for name in
                     ("x500_depth", "x500", "x500_base", "OakD-Lite")})
    for name, data in contents.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    git(root, "init", "-q")
    git(root, "add", ".")
    git(root, "-c", "user.name=Fixture", "-c", "user.email=fixture@example.invalid",
        "commit", "-qm", "fixture")
    source = workspace / "src/px4_powerplant_mission_cpp/config/powerplant_assets.json"
    source.parent.mkdir(parents=True)
    payload = {"schema_version": 1, "model_git_commit": "0" * 40,
               "asset_commit_pending": False,
               "files": [{"path": "worlds/powerplant.sdf", "sha256": hashlib.sha256(b"scene").hexdigest()}]}
    source.write_text(json.dumps(payload))
    body = workspace / "validation/evidence/sitl_body_envelope.json"
    body.parent.mkdir(parents=True)
    body.write_text(json.dumps({"source_sdf_sha256": {
        "/original/" + name: hashlib.sha256(data).hexdigest()
        for name, data in contents.items() if name.startswith("models/")}}))
    return workspace, px4, out, root, source, payload


def run_binding(assets):
    workspace, px4, out, *_ = assets
    return subprocess.run([sys.executable, "-c", asset_program(), str(workspace), str(px4), str(out)],
                          capture_output=True, text=True)


def test_exact_bytes_allow_run_local_commit_binding_without_mutation(assets):
    _, _, out, root, source, _ = assets
    original = source.read_bytes()
    result = run_binding(assets)
    assert result.returncode == 0, result.stderr
    audit = json.loads((out / "asset_binding_audit.json").read_text())
    assert audit["rebound"] is True
    assert audit["actual_model_commit"] == git(root, "rev-parse", "HEAD")
    assert len(audit["verified_files"]) == 5
    assert source.read_bytes() == original
    assert git(root, "status", "--porcelain") == ""


def test_scene_hash_mismatch_is_rejected(assets):
    _, _, out, root, *_ = assets
    (root / "worlds/powerplant.sdf").write_bytes(b"other scene")
    result = run_binding(assets)
    assert result.returncode != 0
    assert "asset hash mismatch" in result.stderr
    assert not (out / "powerplant_assets.json").exists()


def test_matching_lock_cannot_bind_dirty_asset_to_head(assets):
    _, _, out, root, source, payload = assets
    (root / "worlds/powerplant.sdf").write_bytes(b"uncommitted")
    payload["files"][0]["sha256"] = hashlib.sha256(b"uncommitted").hexdigest()
    source.write_text(json.dumps(payload))
    result = run_binding(assets)
    assert result.returncode != 0
    assert "differs from checked-out model commit" in result.stderr
    assert not (out / "powerplant_assets.json").exists()


def test_body_audit_mismatch_is_rejected(assets):
    _, _, out, root, *_ = assets
    (root / "models/x500_depth/model.sdf").write_bytes(b"changed body")
    result = run_binding(assets)
    assert result.returncode != 0
    assert "asset hash mismatch: models/x500_depth/model.sdf" in result.stderr
    assert not (out / "powerplant_assets.json").exists()


def test_escaping_scene_path_is_rejected(assets):
    _, _, _, _, source, payload = assets
    payload["files"][0]["path"] = "../outside"
    source.write_text(json.dumps(payload))
    result = run_binding(assets)
    assert result.returncode != 0
    assert "invalid asset path" in result.stderr


def test_mission_requires_explicit_frozen_profile_before_any_setup():
    import os
    env = dict(os.environ)
    env.pop("CONTROLLER_PROFILE", None)
    result = subprocess.run(["bash", str(ENTRY), "mission"], env=env, capture_output=True, text=True)
    assert result.returncode == 2
    assert "requires an explicit frozen" in result.stderr
