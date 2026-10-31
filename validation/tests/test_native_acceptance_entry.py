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


def dds_program():
    return ENTRY.read_text().split("<<'PYDDS'\n", 1)[1].split("\nPYDDS\n", 1)[0]


@pytest.fixture
def dds_fallback(tmp_path, monkeypatch):
    """运行真实 shell 内嵌程序，只替换网络和系统解包边界。"""
    import platform

    payloads = {"dds-one": b"pinned package one", "dds-two": b"pinned package two"}
    lock = tmp_path / "ros_dependencies.json"
    lock.write_text(json.dumps({"packages": [{
        "package": name, "version": "fixture-1", "sha256": hashlib.sha256(data).hexdigest(),
        "url": f"https://mirrors.ustc.edu.cn/ros2/ubuntu/pool/{name}.deb",
    } for name, data in payloads.items()]}))
    output = tmp_path / "output"
    output.mkdir()
    prefix = tmp_path / "user-prefix"
    real_read_text = Path.read_text

    def read_text(path, *args, **kwargs):
        if path == Path("/etc/os-release"):
            return 'VERSION_CODENAME="noble"\n'
        return real_read_text(path, *args, **kwargs)

    monkeypatch.setattr(Path, "read_text", read_text)
    monkeypatch.setattr(platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(sys, "argv", ["dds-fallback", str(lock), str(prefix), str(output)])
    return {"lock": lock, "prefix": prefix, "output": output, "payloads": payloads, "events": []}


def execute_dds(fixture, monkeypatch, download):
    def run(command, **kwargs):
        if command[0] == "curl":
            # HTTPS keeps certificate checks; public HTTP is the pinned official source.
            url = command[-1]
            protocol = "=https" if url.startswith("https://") else "=http"
            if protocol == "=http":
                assert url.startswith("http://packages.ros.org/ros2/ubuntu/")
            assert command[command.index("--proto") + 1] == protocol
            assert command[command.index("--proto-redir") + 1] == protocol
            assert "--insecure" not in command and "-k" not in command
            url = command[-1]
            returncode, data = download(url)
            fixture["events"].append(("curl", url, returncode))
            if data is not None:
                Path(command[command.index("--output") + 1]).write_bytes(data)
            return subprocess.CompletedProcess(command, returncode)
        assert command[:2] == ["dpkg-deb", "-x"]
        assert kwargs.get("check") is True
        fixture["events"].append(("extract", Path(command[2]).name, Path(command[2]).read_bytes()))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", run)
    exec(compile(dds_program(), str(ENTRY) + ":PYDDS", "exec"), {})


def test_dds_extracts_only_after_every_package_passes_original_sha(dds_fallback, monkeypatch):
    def download(url):
        if "ustc.edu.cn" in url and url.endswith("dds-one.deb"):
            return 35, None  # 原站连接失败后，使用镜像；并非忽略 TLS 校验。
        name = Path(url).stem
        return 0, dds_fallback["payloads"][name]

    execute_dds(dds_fallback, monkeypatch, download)
    events = dds_fallback["events"]
    assert [event[0] for event in events] == ["curl", "curl", "curl", "extract", "extract"]
    assert {event[2] for event in events if event[0] == "extract"} == set(dds_fallback["payloads"].values())
    receipts = json.loads((dds_fallback["output"] / "middleware_fallback_receipts.json").read_text())
    assert len(receipts) == 2
    assert receipts[0]["url"].startswith("https://mirrors.tuna.tsinghua.edu.cn/")
    assert all(row["sha256"] == hashlib.sha256(dds_fallback["payloads"][row["package"]]).hexdigest()
               for row in receipts)


def test_dds_later_hash_mismatch_prevents_even_prior_verified_package_extraction(dds_fallback, monkeypatch):
    def download(url):
        name = Path(url).stem
        return 0, dds_fallback["payloads"][name] if name == "dds-one" else b"tampered package"

    with pytest.raises(SystemExit, match="SHA256 mismatch: dds-two"):
        execute_dds(dds_fallback, monkeypatch, download)
    assert [event[0] for event in dds_fallback["events"]] == ["curl", "curl"]
    assert not dds_fallback["prefix"].exists()
    assert not (dds_fallback["output"] / "middleware_fallback_receipts.json").exists()


def test_dds_all_download_channels_fail_without_extraction(dds_fallback, monkeypatch):
    with pytest.raises(SystemExit, match="locked DDS dependency unavailable: dds-one"):
        execute_dds(dds_fallback, monkeypatch, lambda url: (35, None))
    events = dds_fallback["events"]
    assert [event[0] for event in events] == ["curl", "curl", "curl"]
    assert "ustc.edu.cn" in events[0][1] and "tuna.tsinghua.edu.cn" in events[1][1]
    assert not dds_fallback["prefix"].exists()
    assert not (dds_fallback["output"] / "middleware_fallback_receipts.json").exists()


def test_dds_official_public_fallback_requires_original_bytes(dds_fallback, monkeypatch):
    def download(url):
        if url.startswith("https://"):
            return 35, None
        return 0, dds_fallback["payloads"][Path(url).stem]

    execute_dds(dds_fallback, monkeypatch, download)
    receipts = json.loads((dds_fallback["output"] / "middleware_fallback_receipts.json").read_text())
    assert len(receipts) == 2
    assert all(row["url"].startswith("http://packages.ros.org/ros2/ubuntu/") for row in receipts)
    assert sum(event[0] == "extract" for event in dds_fallback["events"]) == 2


def test_dds_official_public_fallback_rejects_corrupt_bytes(dds_fallback, monkeypatch):
    def download(url):
        return (35, None) if url.startswith("https://") else (0, b"corrupt package")

    with pytest.raises(SystemExit, match="SHA256 mismatch: dds-one"):
        execute_dds(dds_fallback, monkeypatch, download)
    assert not any(event[0] == "extract" for event in dds_fallback["events"])
