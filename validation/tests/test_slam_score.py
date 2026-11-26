"""Synthetic geometry exercises the evaluator. These fixtures are not SITL evidence."""

import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

SCORE = Path(__file__).resolve().parents[1] / "slam" / "score.py"
spec = importlib.util.spec_from_file_location("slam_score", SCORE)
score = importlib.util.module_from_spec(spec)
spec.loader.exec_module(score)
SEC = 1_000_000_000


def jsonl(path, records):
    path.write_text("".join(json.dumps(r) + "\n" for r in records))


def experiment(tmp, name, split):
    directory = tmp / name
    directory.mkdir()
    bag = hashlib.sha256(name.encode()).hexdigest()
    windows = {"align": (0, 1), "first_a": (1, 2.4), "b": (3, 4.4), "occlusion": (5, 6),
               "negative": (7, 8), "revisit": (9, 11)}
    score.write_report(directory / "windows.json", {"events": [
        {"name": k, "start_ns": int(start * SEC), "end_ns": int(end * SEC)}
        for k, (start, end) in windows.items()]})
    stamps = np.arange(111) * (SEC // 10)
    np.save(directory / "depth_stamps.npy", stamps)
    np.save(directory / "a_visible_stamps.npy", stamps[((stamps >= SEC) & (stamps <= int(2.4*SEC))) |
                                                         (stamps >= 9*SEC)])
    np.save(directory / "c_visible_stamps.npy", stamps[(stamps >= 7*SEC) & (stamps <= 8*SEC)])
    truth = np.interp(stamps / SEC, [0, 2.4, 3, 4.4, 6, 8, 9.0, 11], [0, 0, 3, 3, 6, 6, 0, 0])
    (directory / "truth.csv").write_text("stamp_ns,x,y,z,qx,qy,qz,qw\n" + "".join(
        f"{s},{x},0,0,0,0,0,1\n" for s, x in zip(stamps, truth)))
    score.write_report(directory / "capture.json", {"split": split, "capture_id": name,
        "bag_sha256": bag, "passed": True, "b_visibility_checks": 20, "b_visible_after_window": 0})
    reference = np.array([[3, y * .1, z * .1] for y in range(4) for z in range(4)])
    np.save(directory / "ref.npy", reference)
    # Rotate/translate all estimates away from truth to exercise full initial yaw alignment.
    pose_rows = [{"sample_ns": int(s), "position": [1, -x + 2, 3],
                  "quaternion_wxyz": [2**-.5, 0, 0, -2**-.5]} for s, x in zip(stamps, truth)]
    estimated = np.c_[1 + reference[:, 1], 2 - reference[:, 0], 3 + reference[:, 2]]
    jsonl(directory / "kf.jsonl", [{"id": i, "sample_ns": int(t * SEC)} for i,t in [(1,2),(2,4),(3,9.2),(4,7)]])
    rounds = []
    for index in range(1 if split == "online" else 3):
        runs = {}
        for mode in ("on", "off"):
            prefix = f"{index}-{mode}"
            jsonl(directory / (prefix + ".jsonl"), pose_rows)
            events = [{"sample_ns": int(5.2 * SEC), "type": "lost"},
                      {"sample_ns": int(5.2 * SEC), "type": "masked_loss"},
                      {"sample_ns": int(6.1 * SEC), "type": "unmasked"},
                      {"sample_ns": int(7 * SEC), "type": "tracking"}]
            if mode == "on":
                events += [{"sample_ns": int(9.2 * SEC), "type": kind, "current_id": 3, "matched_id": 1}
                           for kind in ("loop", "relocalized")]
            jsonl(directory / (prefix + "-events.jsonl"), events)
            before = estimated + [0, .2 + .01 * index, 0]
            after = estimated + [0, .01 + .002 * index, 0] if mode == "on" else before
            np.save(directory / (prefix + "-before.npy"), before)
            np.save(directory / (prefix + "-after.npy"), after)
            runs[mode] = {"run_id": name + "-" + prefix, "poses": prefix + ".jsonl",
                          "trajectory": prefix + ".jsonl", "events": prefix + "-events.jsonl",
                          "keyframes": "kf.jsonl", "b_before": prefix + "-before.npy",
                          "b_after": prefix + "-after.npy", "b_source_ids": [2],
                          "timing": {"raw_depth_frames": 101, "registered_frames": 101,
                                     "registration_missed_frames": 0, "tracked_frames": 101,
                                     "real_time_factor": .8, "sim_hz": 10,
                                     "max_frame_gap_s": .1, "registered_receipt_latency_p95_s": .02,
                                     "odom_receipt_latency_p95_s": .03}}
        rounds.append(runs)
    evidence = {"capture_id": name, "split": split, "bag_sha256": bag,
                "truth": "truth.csv", "reference_b": "ref.npy", "windows": "windows.json",
                "capture_manifest": "capture.json", "voxel_m": .25, "rounds": rounds}
    path = directory / "evidence.json"
    score.write_report(path, evidence)
    return path


def online_evidence(blind):
    evidence = score.read_json(blind)
    pair = evidence["rounds"][0]
    pose_path = blind.parent / "online-poses.jsonl"
    shutil.copyfile(blind.parent / pair["on"]["poses"], pose_path)
    pair["on"].update(run_id="online", poses=pose_path.name)
    live = blind.parent / "online_evidence.json"
    score.write_report(live, dict(evidence, split="online", rounds=[pair]))
    return live


def test_freeze_and_score_blind_and_online(tmp_path):
    cal = experiment(tmp_path, "cal", "calibration")
    blind = experiment(tmp_path, "blind", "blind")
    live = online_evidence(blind)
    frozen = tmp_path / "thresholds.json"
    result = score.freeze([cal], frozen)
    limit = result["limits"]["on_b_after_p95_m"]
    assert limit["limit"] == pytest.approx(.018)
    report = score.evaluate([blind, live], frozen, tmp_path / "report.json", tmp_path / "plot.png")
    assert report["passed"]
    assert report["captures"][0]["rounds"][0]["metrics"]["on_live_ate_m"] < 1e-10
    assert (tmp_path / "plot.png").stat().st_size > 1000
    report_round = report["captures"][0]["rounds"][0]
    assert report_round["metrics"]["relocalize_delay_s"] == pytest.approx(3.1)
    assert report_round["metrics"]["on_longest_lost_s"] == pytest.approx(1.8)
    assert report_round["counts"]["on"] == {"negative_keyframe_pairs": 2, "false_loops": 0}
    assert report_round["timing"]["on"]["odom_receipt_latency_p95_s"] == .03
    assert (tmp_path / "plot-capture1-run1-trajectory.png").stat().st_size > 1000
    assert (tmp_path / "plot-capture1-run1-surface-B.png").stat().st_size > 1000


@pytest.mark.parametrize("failure", ["tracking_only", "wrong_old_keyframe", "false_positive", "B_cleared",
                                     "B_misaligned", "B_seen_again", "mixed_split", "repeated_run"])
def test_corrupt_evidence_cannot_pass(tmp_path, failure):
    path = experiment(tmp_path, "cal", "calibration")
    d = path.parent
    on_path = d / "0-on-events.jsonl"
    records = score.rows(on_path)
    if failure == "tracking_only":
        records[-1]["type"] = "tracking"
    elif failure == "wrong_old_keyframe":
        records[-1]["matched_id"] = 2
    elif failure == "false_positive":
        with (d / "kf.jsonl").open("a") as stream:
            stream.write(json.dumps({"id": 10, "sample_ns": 7 * SEC}) + "\n")
        records.append({"sample_ns": 7 * SEC, "current_id": 10, "matched_id": 1, "type": "loop"})
        records.sort(key=score.time_ns)
    elif failure == "B_cleared":
        np.save(d / "0-on-after.npy", np.empty((0, 3)))
    elif failure == "B_misaligned":
        np.save(d / "0-on-after.npy", np.load(d / "0-on-before.npy"))
    elif failure == "B_seen_again":
        cap = score.read_json(d / "capture.json")
        cap["b_visible_after_window"] = 2
        score.write_report(d / "capture.json", cap)
    elif failure in ("mixed_split", "repeated_run"):
        evidence = score.read_json(path)
        if failure == "mixed_split":
            evidence["split"] = "blind"
        else:
            evidence["rounds"][1] = evidence["rounds"][0]
        score.write_report(path, evidence)
    jsonl(on_path, records)
    with pytest.raises(score.EvidenceError):
        score.freeze([path], tmp_path / "thresholds.json")
    assert not (tmp_path / "thresholds.json").exists()


def test_calibration_cannot_be_used_as_blind_and_failures_are_reported(tmp_path):
    cal = experiment(tmp_path, "cal", "calibration")
    blind = experiment(tmp_path, "blind", "blind")
    thresholds = tmp_path / "thresholds.json"
    score.freeze([cal], thresholds)
    with pytest.raises(score.EvidenceError):
        score.evaluate([cal], thresholds)
    # Keep a valid match but delay the first observed LOST on a single replay.
    path = blind.parent / "0-on-events.jsonl"
    events = score.rows(path)
    for event in events[:2]:
        event["sample_ns"] += 300_000_000
    jsonl(path, events)
    report = score.evaluate([blind, online_evidence(blind)], thresholds)
    assert not report["passed"]
    assert not report["captures"][0]["rounds"][0]["checks"]["limit_lost_delay_s"]


def test_noise_larger_than_correction_prevents_freeze(tmp_path):
    path = experiment(tmp_path, "cal", "calibration")
    before = np.load(path.parent / "0-on-before.npy")
    np.save(path.parent / "0-on-after.npy", before + [0, -.001, 0])
    with pytest.raises(score.EvidenceError, match="repeat variation"):
        score.freeze([path], tmp_path / "limits.json")


def test_revisit_to_lookalike_is_a_false_loop_but_within_lookalike_is_valid(tmp_path):
    path = experiment(tmp_path, "cal", "calibration")
    events_path = path.parent / "0-on-events.jsonl"
    keyframes = path.parent / "kf.jsonl"
    jsonl(keyframes, score.rows(keyframes) + [{"id": 5, "sample_ns": int(7.5*SEC)}])
    events = score.rows(events_path) + [{"type":"loop","sample_ns":int(7.5*SEC),"current_id":5,"matched_id":4}]
    jsonl(events_path, sorted(events, key=score.time_ns))
    first = score.score_capture(path)["rounds"][0]
    assert first["counts"]["on"] == {"negative_keyframe_pairs":4,"false_loops":0}
    assert first["passed"]
    events.append({"type":"loop","sample_ns":int(9.2*SEC),"current_id":3,"matched_id":4})
    jsonl(events_path, sorted(events, key=score.time_ns))
    first = score.score_capture(path)["rounds"][0]
    assert first["counts"]["on"]["false_loops"] == 1
    assert not first["passed"]


def test_missing_negative_views_and_unpaired_online_evidence(tmp_path):
    cal = experiment(tmp_path, "cal", "calibration")
    blind = experiment(tmp_path, "blind", "blind")
    frozen = tmp_path / "limits.json"
    score.freeze([cal], frozen)
    with pytest.raises(score.EvidenceError, match="online flight"):
        score.evaluate([blind], frozen)
    np.save(blind.parent / "a_visible_stamps.npy", np.array([2*SEC], dtype=np.int64))
    assert not score.score_capture(blind)["rounds"][0]["checks"]["on_loop_to_first_A"]
