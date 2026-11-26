"""Convert recorded graph revisions and independent scene observations to trial evidence."""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def read_lines(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def dump_lines(path, rows):
    with path.open("x") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    return str(path)


def pose_record(stamp, p):
    return {"sample_ns": int(stamp), "position": list(p[:3]),
            "quaternion_wxyz": [p[6], *p[3:6]]}


def transform(points, pose):
    return Rotation.from_quat(pose[3:]).apply(points) + pose[:3]


def corrected_pose(p, offset):
    rot = Rotation.from_quat(offset[3:]) * Rotation.from_quat(p[3:])
    return [*transform(p[:3], offset), *rot.as_quat()]


class Truth:
    def __init__(self, path):
        with path.open() as stream:
            rows = sorted({int(r["stamp_ns"]): [float(r[k]) for k in
                           ("x", "y", "z", "qx", "qy", "qz", "qw")]
                           for r in csv.DictReader(stream)}.items())
        if len(rows) < 2:
            raise ValueError("missing named Gazebo model trajectory")
        self.stamps = np.array([ns for ns, _ in rows], dtype=np.int64)
        self.poses = np.array([p for _, p in rows])
        self.seconds = (self.stamps-self.stamps[0])*1.e-9
        self.rot = Slerp(self.seconds, Rotation.from_quat(self.poses[:, 3:]))

    def at(self, stamp):
        t = (stamp-self.stamps[0])*1.e-9
        if t < 0 or t > self.seconds[-1]:
            raise ValueError("keyframe outside truth time range")
        pos = [np.interp(t, self.seconds, axis) for axis in self.poses[:, :3].T]
        return [*pos, *self.rot(t).as_quat()]


def run_evidence(run_dir, capture):
    """Project exactly the same retained B pixels using two actual graph revisions."""
    manifest_path = run_dir / "run.json"
    if manifest_path.exists():
        return json.loads(manifest_path.read_text())
    windows = {w["name"]: (w["start_ns"], w["end_ns"])
               for w in json.loads((capture/"events.json").read_text())["events"]}
    graphs = sorted(read_lines(run_dir/"graphs.jsonl"), key=lambda r: r["ns"])
    infos = sorted(read_lines(run_dir/"info.jsonl"), key=lambda row: row["ns"])
    times = {}
    for row in infos:
        if row["id"] > 0:
            times.setdefault(row["id"], row["ns"])
    # ref_id is the new node, including when the optimized graph has older nodes.
    graph_ids = {int(i) for g in graphs for i in g["poses"]}
    times = {i: ns for i, ns in times.items() if i in graph_ids}
    before = [g for g in graphs if g["ns"] < windows["revisit"][0]]
    after = [g for g in graphs if windows["revisit"][0] <= g["ns"] <= windows["revisit"][1]]
    if not before or not after or not times:
        raise ValueError(f"{run_dir}: missing graph before or after revisit")
    last = after[-1]
    ref = np.load(capture/"b_reference.npy", allow_pickle=False)
    if ref.ndim != 2 or ref.shape[1] != 3 or not len(ref):
        raise ValueError("empty B scene reference")
    tree = cKDTree(ref)
    truth = Truth(capture/"truth.csv")
    pre_points, post_points, source_ids = [], [], []
    for i, stamp in sorted(times.items()):
        if not windows["b"][0] <= stamp <= windows["b"][1]:
            continue
        saved = run_dir/"depth_samples"/f"{stamp}.npy"
        if str(i) not in before[-1]["poses"]:
            continue
        if not saved.is_file():
            raise ValueError(f"{run_dir}: missing original depth pixels for B keyframe {i}")
        points = np.load(saved, allow_pickle=False)
        # Select the predeclared B patch in truth space, identically for both
        # graph revisions, before looking at estimated surface errors.
        dist, _ = tree.query(transform(points, truth.at(stamp)))
        patch = points[dist <= 0.25]
        if len(patch):
            if str(i) not in last["poses"]:
                raise ValueError(f"{run_dir}: final graph dropped historical B keyframe {i}")
            pre_points.append(transform(patch, before[-1]["poses"][str(i)]))
            post_points.append(transform(patch, last["poses"][str(i)]))
            source_ids.append(i)
    if not source_ids:
        raise ValueError(f"{run_dir}: no retained historical depth observation of B")
    for label, data in (("before", pre_points), ("after", post_points)):
        np.save(run_dir/f"b_{label}.npy", np.concatenate(data), allow_pickle=False)

    events = []
    statuses = sorted(read_lines(run_dir/"status.jsonl"), key=lambda r: r["stamp_ns"])
    tracked = {s["stamp_ns"] for s in statuses if s["tracking"] == "TRACKING"}
    masked_loss_ns = None
    last_tracking = None
    for row in statuses:
        stamp = row["stamp_ns"]
        state = row["tracking"].lower()
        if (masked_loss_ns is None and row["masked"] is True and state == "lost"
                and windows["occlusion"][0] <= stamp <= windows["occlusion"][1]):
            events.append({"type": "masked_loss", "sample_ns": stamp})
            masked_loss_ns = stamp
        if state in ("lost", "tracking") and state != last_tracking:
            events.append({"type": state, "sample_ns": stamp})
            last_tracking = state
    # A recognized loop must also be an accepted non-neighbour edge in the graph.
    edges = {tuple(sorted(pair)) for g in graphs if g["ns"] <= windows["revisit"][1]
             for pair in g["loops"]}
    for info in infos:
        i, j = info["id"], info["loop"]
        if (i in times and j in times and info["ns"] == times[i] and times[j] < times[i]
                and tuple(sorted((i,j))) in edges):
            item = {"current_id": i, "matched_id": j, "sample_ns": info["ns"]}
            events.append(dict(type="loop", **item))
            if (any(e["type"] == "lost" and windows["occlusion"][0] <= e["sample_ns"] < info["ns"]
                    for e in events) and times[j] < windows["occlusion"][0]):
                events.append(dict(type="relocalized", **item))

    odoms = sorted(read_lines(run_dir/"odom.jsonl"), key=lambda r: r["ns"])
    graph_idx = 0
    identity = [0., 0., 0., 0., 0., 0., 1.]
    poses = []
    for odom in odoms:
        # RTAB-Map can also publish an odometry message for a failed registration.
        if not windows["first_a"][0] <= odom["ns"] <= windows["revisit"][1] or odom["ns"] not in tracked:
            continue
        while graph_idx+1 < len(graphs) and graphs[graph_idx+1]["ns"] <= odom["ns"]:
            graph_idx += 1
        g = graphs[graph_idx]
        offset = g["map_to_odom"] if g["ns"] <= odom["ns"] else identity
        poses.append(pose_record(odom["ns"], corrected_pose(odom["p"], offset)))
    # Include timing from both clock domains. Native-rate gaps are measured in
    # simulation time. Wall reception includes actual observer/transport load.
    frames = [f for f in read_lines(run_dir/"depth.jsonl")
              if windows["first_a"][0] <= f["ns"] <= windows["revisit"][1]]
    if len(frames) < 2:
        raise ValueError("insufficient registered frames")
    stamps = np.unique([f["ns"] for f in frames])
    if masked_loss_ns is not None:
        # These samples come from the registered input copy. An occluded frame
        # contains no depth points, even while the raw camera continues to run.
        for stamp in stamps[stamps > masked_loss_ns]:
            depth = run_dir/"depth_samples"/f"{stamp}.npy"
            if depth.is_file() and len(np.load(depth, mmap_mode="r", allow_pickle=False)):
                events.append({"type": "unmasked", "sample_ns": int(stamp)})
                break
    duration = (stamps[-1]-stamps[0])*1.e-9
    wall = (frames[-1]["steady_ns"]-frames[0]["steady_ns"])*1.e-9
    if duration <= 0 or wall <= 0:
        raise ValueError("stationary experiment clock")
    exposures = np.load(capture/"depth_stamps.npy", allow_pickle=False)
    eligible = exposures[(exposures >= windows["first_a"][0]) & (exposures <= windows["revisit"][1])]
    if not len(eligible) or not poses:
        raise ValueError("no raw input or valid estimator output in the observation windows")
    scored_odoms = [o for o in odoms if o["ns"] in tracked
                    and windows["first_a"][0] <= o["ns"] <= windows["revisit"][1]]
    timing = {"raw_depth_frames": len(eligible), "registered_frames": len(stamps),
              "registration_missed_frames": len(np.setdiff1d(eligible, stamps)),
              "tracked_frames": len(poses), "sim_hz": (len(stamps)-1)/duration,
              "real_time_factor": duration/wall,
              "max_frame_gap_s": float(np.diff(stamps).max())*1.e-9,
              "odom_receipt_latency_p95_s": float(np.percentile(
                  [(o["receipt_sim_ns"]-o["ns"])*1.e-9 for o in scored_odoms],95)),
              "registered_receipt_latency_p95_s": float(np.percentile(
                  [(f["sim_ns"]-f["ns"])*1.e-9 for f in frames],95))}
    result = {
        "run_id": run_dir.name, "poses": dump_lines(run_dir/"poses.jsonl", poses),
        "events": dump_lines(run_dir/"measured_events.jsonl", sorted(events, key=lambda e: e["sample_ns"])),
        "keyframes": dump_lines(run_dir/"keyframes.jsonl",
                                 ({"id": i, "sample_ns": ns} for i, ns in times.items())),
        "trajectory": dump_lines(run_dir/"optimized.jsonl", (
            pose_record(ns, last["poses"][str(i)]) for i,ns in sorted(times.items(),key=lambda p:p[1])
            if str(i) in last["poses"] and windows["align"][0] <= ns <= windows["revisit"][1])),
        "b_before": str(run_dir/"b_before.npy"), "b_after": str(run_dir/"b_after.npy"),
        "b_source_ids": source_ids, "timing": timing,
    }
    manifest_path.write_text(json.dumps(result, indent=2, allow_nan=False))
    return result
