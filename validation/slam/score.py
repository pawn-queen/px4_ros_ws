"""Independent scoring of the local powerplant SLAM experiment (no ROS runtime)."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


class EvidenceError(ValueError):
    """An input cannot support the requested experiment."""


def require(condition, message):
    if not condition:
        raise EvidenceError(message)


def read_json(path):
    return json.loads(Path(path).read_text())


def rows(path):
    with Path(path).open() as stream:
        if Path(path).suffix == ".csv":
            return list(csv.DictReader(stream))
        return [json.loads(line) for line in stream if line.strip()]


def time_ns(row):
    for field in ("sample_ns", "stamp_ns"):
        if field in row:
            return int(row[field])
    raise EvidenceError("sample timestamp is missing")


def positions(path):
    records = rows(path)
    require(len(records) >= 2, f"{path}: need at least two poses")
    stamps = np.asarray([time_ns(r) for r in records], dtype=np.int64)
    xyz, xyzw = [], []
    for r in records:
        if "position" in r:
            xyz.append(r["position"])
            w, x, y, z = r["quaternion_wxyz"]
            xyzw.append([x, y, z, w])
        else:
            xyz.append([r[k] for k in ("x", "y", "z")])
            xyzw.append([r[k] for k in ("qx", "qy", "qz", "qw")])
    xyz, xyzw = np.asarray(xyz, dtype=float), np.asarray(xyzw, dtype=float)
    require(xyz.shape == (len(stamps), 3) and np.isfinite(xyz).all()
            and np.isfinite(xyzw).all(), f"{path}: invalid pose")
    require(np.all(stamps >= 0) and np.all(np.diff(stamps) > 0), f"{path}: invalid pose timeline")
    require(np.allclose(np.linalg.norm(xyzw, axis=1), 1.0, atol=0.01), f"{path}: invalid quaternion")
    return stamps, xyz, Rotation.from_quat(xyzw)


def samples(source, targets):
    stamps, xyz, rotations = source
    require(len(targets) > 0 and min(targets) >= stamps[0] and max(targets) <= stamps[-1],
            "reference poses do not bracket the observation timeline")
    seconds = (stamps - stamps[0]) * 1e-9
    query = (np.asarray(targets, dtype=np.int64) - stamps[0]) * 1e-9
    return (np.column_stack([np.interp(query, seconds, axis) for axis in xyz.T]),
            Slerp(seconds, rotations)(query))


def heading(rotations):
    direction = rotations.apply([1.0, 0, 0])
    return np.arctan2(direction[..., 1], direction[..., 0])


def wrapped(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def within(stamps, window):
    return (stamps >= window[0]) & (stamps <= window[1])


def exposure_times(path):
    stamps = np.load(path, allow_pickle=False)
    require(stamps.ndim == 1 and stamps.dtype == np.int64 and len(stamps) > 0
            and np.all(stamps >= 0) and np.all(np.diff(stamps) > 0),
            f"{path}: invalid exposure timestamps")
    return stamps


def initial_alignment(truth, estimate, window):
    stamps, xyz, rotations = estimate
    initial = within(stamps, window)
    require(initial.sum() >= 2, "initial alignment window has fewer than two estimates")
    real, real_rot = samples(truth, stamps[initial])
    delta = wrapped(heading(real_rot) - heading(rotations[initial]))
    rotation = Rotation.from_rotvec([0, 0, math.atan2(np.sin(delta).mean(), np.cos(delta).mean())])
    return rotation, np.mean(real - rotation.apply(xyz[initial]), axis=0)


def transformed(points, transform):
    rotation, shift = transform
    return rotation.apply(points) + shift


def rmse(array):
    return float(np.sqrt(np.mean(np.square(array))))


def metric(value):
    require(np.isfinite(value), "nonfinite score")
    return float(value)


def cloud(path):
    points = np.load(path, allow_pickle=False)
    require(points.ndim == 2 and points.shape[1] == 3 and len(points) >= 3
            and np.isfinite(points).all(), f"{path}: B surface requires finite Nx3 points")
    return points


def surface_metrics(points, reference, voxel_m):
    residuals = cKDTree(reference).query(points)[0]
    reference_distances = cKDTree(points).query(reference)[0]
    return float(np.percentile(residuals, 95)), float(np.mean(reference_distances <= voxel_m))


def event_windows(path):
    items = read_json(path)["events"]
    windows = {}
    for event in items:
        name = event["name"]
        if name not in ("align", "first_a", "b", "occlusion", "negative", "revisit"):
            continue
        require(name not in windows, f"duplicate window {name}")
        windows[name] = (int(event["start_ns"]), int(event["end_ns"]))
        require(windows[name][0] < windows[name][1], f"empty window {name}")
    order = ["first_a", "b", "occlusion", "negative", "revisit"]
    require(set(windows) == {"align", *order}, "incomplete route windows")
    require(all(windows[a][1] <= windows[b][0] for a, b in zip(order, order[1:])),
            "route windows are out of order")
    require(windows["align"][1] <= windows["occlusion"][0], "alignment must precede occlusion")
    return windows


def pose_metrics(truth, raw, transform, label, need_rpe):
    stamps, xyz, rotations = raw
    reference, ref_rot = samples(truth, stamps)
    aligned = transformed(xyz, transform)
    distances = np.linalg.norm(aligned - reference, axis=1)
    result = {label + "_ate_m": rmse(distances)}
    if need_rpe:
        # Compare relative body-frame translations over a fixed one-second interval.
        # A missing run of estimates must not be bridged by long extrapolation.
        i = np.arange(len(stamps))
        j = np.searchsorted(stamps, stamps + 1_000_000_000)
        valid = j < len(stamps)
        i, j = i[valid], j[valid]
        valid = stamps[j] - stamps[i] <= 1_200_000_000
        i, j = i[valid], j[valid]
        require(len(i) > 0, "no observed one-second RPE intervals")
        estimated_step = rotations[i].inv().apply(xyz[j] - xyz[i])
        reference_step = ref_rot[i].inv().apply(reference[j] - reference[i])
        result[label + "_rpe_1s_m"] = rmse(np.linalg.norm(estimated_step - reference_step, axis=1))
        result[label + "_heading_rmse_rad"] = rmse(wrapped(heading(transform[0] * rotations) - heading(ref_rot)))
    return result


def run_evidence(base, config):
    keyframes = {}
    for r in rows(base / config["keyframes"]):
        node = int(r["id"])
        require(node > 0 and node not in keyframes, "invalid or duplicate keyframe id")
        keyframes[node] = time_ns(r)
    require(keyframes, "empty keyframe table")
    events = rows(base / config["events"])
    event_times = [time_ns(e) for e in events]
    require(all(b >= a for a, b in zip(event_times, event_times[1:])), "unordered events")
    for event in events:
        if event["type"] in ("loop", "relocalized"):
            current, matched = int(event["current_id"]), int(event["matched_id"])
            require(current in keyframes and matched in keyframes and current != matched,
                    "match does not identify two existing keyframes")
            require(keyframes[matched] < keyframes[current], "match target is not historical")
            require(keyframes[current] == time_ns(event), "event stamp differs from current keyframe")
    poses = positions(base / config["poses"])
    return keyframes, events, poses


def matched_history(truth, keyframes, events, windows, visible_a, kind):
    """The old keyframe must be inside the scored first visit and the new one inside the revisit."""
    positives = []
    for event in events:
        if event["type"] != kind or not within(time_ns(event), windows["revisit"]):
            continue
        matched = keyframes[int(event["matched_id"])]
        if not within(matched, windows["first_a"]) or not {matched, time_ns(event)} <= visible_a:
            continue
        p, r = samples(truth, [matched, time_ns(event)])
        positives.append((time_ns(event), float(np.linalg.norm(p[1] - p[0])),
                          float(abs(wrapped(np.diff(heading(r))[0])))))
    return positives


def check_relocalization(truth, windows, data, visible_a):
    keyframes, events, poses = data
    checks = {}
    lost = [time_ns(e) for e in events if e["type"] == "masked_loss" and within(time_ns(e), windows["occlusion"])]
    recovered_input = [time_ns(e) for e in events if e["type"] == "unmasked" and lost and time_ns(e) > lost[0]]
    positives = [m for m in matched_history(truth, keyframes, events, windows, visible_a, "relocalized")
                 if recovered_input and m[0] >= recovered_input[0]]
    checks["lost_during_mask"] = bool(lost)
    checks["vision_restored_after_loss"] = bool(recovered_input)
    checks["historical_A_relocalized"] = bool(positives)
    # Consecutive tracking in an unregistered local odom frame cannot prove recovery in the old map.
    if not lost or not positives:
        return checks, {}
    confirmed = positives[0][0]
    after = within(poses[0], [confirmed, windows["revisit"][1]])
    checks["relocalization_stays_tracked"] = bool(after.sum() >= 3
        and poses[0][after][-1] - confirmed >= 1_000_000_000 and not any(
        e["type"] == "lost" and confirmed <= time_ns(e) <= windows["revisit"][1] for e in events))
    metrics = {"lost_delay_s": (lost[0] - windows["occlusion"][0]) * 1e-9,
               "relocalize_delay_s": (confirmed - recovered_input[0]) * 1e-9,
               "relocalize_match_distance_m": positives[0][1],
               "relocalize_match_heading_rad": positives[0][2]}
    if after.sum() >= 3:
        transform = initial_alignment(truth, poses, windows["align"])
        real, orientation = samples(truth, poses[0][after])
        error = np.linalg.norm(transformed(poses[1][after], transform) - real, axis=1)
        angle = np.abs(wrapped(heading(transform[0] * poses[2][after]) - heading(orientation)))
        metrics["relocalized_position_max_m"] = float(error.max())
        metrics["relocalized_heading_max_rad"] = float(angle.max())
    return checks, metrics


def mode_scores(base, run, data, mode, truth, reference, windows, voxel, exposures, visible_a, visible_c):
    keyframes, events, poses = data
    checks, metrics = {}, {}
    transform = initial_alignment(truth, poses, windows["align"])
    metrics.update(pose_metrics(truth, poses, transform, "live", True))
    graph = positions(base / run["trajectory"])
    metrics.update(pose_metrics(truth, graph, transform, "graph", False))
    # The native bridge provides exact-synchronized colour/depth samples. Use the
    # same raw exposure denominator for all replays, including missed callbacks.
    start, end = windows["first_a"][0], windows["revisit"][1]
    inputs = exposures[within(exposures, (start, end))]
    require(len(inputs) > 0, "missing scored raw exposure timestamps")
    tracked = poses[0][within(poses[0], (start, end))]
    require(np.isin(tracked, inputs).all(), "tracking output does not identify a raw exposure")
    metrics["tracking_coverage"] = len(tracked) / len(inputs)
    state_events = sorted((time_ns(e), e["type"]) for e in events if e["type"] in ("lost","tracking"))
    metrics["longest_lost_s"] = max((max(0, min(end, state_events[i+1][0] if i+1 < len(state_events) else end)
                                          - max(start, t)) * 1e-9
                                     for i, (t, state) in enumerate(state_events) if state == "lost"), default=0.)
    # Per-window coverage exposes dropped stretches hidden by ATE on the output-only samples.
    for name in ("first_a", "b", "revisit"):
        start, end = windows[name]
        times = poses[0][within(poses[0], (start, end))]
        checks[name + "_pose_observed"] = len(times) >= 2
        metrics[name + "_max_gap_s"] = float(np.diff(np.r_[start, times, end]).max()) * 1e-9
    ids = run["b_source_ids"]
    checks["b_sources_from_first_pass"] = (len(set(ids)) == len(ids) and bool(ids)
        and all(i in keyframes and within(keyframes[i], windows["b"]) for i in ids))
    before, after = cloud(base / run["b_before"]), cloud(base / run["b_after"])
    require(before.shape == after.shape, "B projection lost or gained historical samples")
    before, after = transformed(before, transform), transformed(after, transform)
    before_error, before_coverage = surface_metrics(before, reference, voxel)
    after_error, after_coverage = surface_metrics(after, reference, voxel)
    metrics.update(b_before_p95_m=before_error, b_after_p95_m=after_error,
                   b_before_coverage=before_coverage, b_after_coverage=after_coverage)
    checks["b_remains_covered"] = after_coverage > 0 and after_coverage >= before_coverage
    matches = matched_history(truth, keyframes, events, windows, visible_a, "loop")
    a_nodes = {node for node, ns in keyframes.items() if ns in visible_a}
    c_nodes = {node for node, ns in keyframes.items() if ns in visible_c}
    negatives = len(a_nodes) * len(c_nodes)
    wrong_pairs = {tuple(sorted((e["current_id"], e["matched_id"]))) for e in events
                   if e["type"] in ("loop", "relocalized") and (
                       (e["current_id"] in a_nodes and e["matched_id"] in c_nodes)
                       or (e["current_id"] in c_nodes and e["matched_id"] in a_nodes))}
    if mode == "on":
        checks["negative_examples_observed"] = negatives > 0
        checks["loop_to_first_A"] = bool(matches)
        checks["b_error_improved"] = after_error < before_error
        checks["no_negative_match"] = not wrong_pairs
        if matches:
            metrics["loop_match_distance_m"] = max(m[1] for m in matches)
            metrics["loop_match_heading_rad"] = max(m[2] for m in matches)
        metrics["b_correction_m"] = float(np.percentile(np.linalg.norm(after - before, axis=1), 95))
    else:
        checks["loop_disabled"] = not any(e["type"] == "loop" for e in events)
    return checks, metrics, {"negative_keyframe_pairs": negatives, "false_loops": len(wrong_pairs)}


def grade_round(base, round_spec, truth, reference, windows, voxel, exposures, visible_a, visible_c):
    on_data = run_evidence(base, round_spec["on"])
    off_data = run_evidence(base, round_spec["off"])
    checks, metrics, counts, timing = {}, {}, {}, {}
    for mode, data in (("on", on_data), ("off", off_data)):
        c, m, counts[mode] = mode_scores(base, round_spec[mode], data, mode, truth, reference, windows,
                                         voxel, exposures, visible_a, visible_c)
        timing[mode] = round_spec[mode]["timing"]
        checks.update({mode + "_" + name: bool(value) for name, value in c.items()})
        metrics.update({mode + "_" + name: metric(value) for name, value in m.items()})
    recovery = run_evidence(base, round_spec["lost"]) if "lost" in round_spec else on_data
    c, m = check_relocalization(truth, windows, recovery, visible_a)
    checks.update(c)
    metrics.update(m)
    # Comparing the identical observations before/after AND a separate loop-disabled replay
    # excludes clearing the map and a blanket rigid alignment as explanations for the gain.
    gain = metrics["off_b_after_p95_m"] - metrics["on_b_after_p95_m"]
    checks["b_better_than_off"] = gain > 0
    metrics["b_gain_over_off_m"] = gain
    metrics["on_b_gain_over_before_m"] = metrics["on_b_before_p95_m"] - metrics["on_b_after_p95_m"]
    return {"run_ids": {mode: cfg["run_id"] for mode, cfg in round_spec.items() if mode in ("on","off","lost")},
            "checks": checks, "metrics": metrics, "counts": counts, "timing": timing,
            "passed": all(checks.values())}


def score_capture(path):
    path = Path(path).resolve()
    data = read_json(path)
    base = path.parent
    split = data["split"]
    require(split in ("calibration", "blind", "online"), "unknown split")
    require(bool(data["capture_id"]), "capture ID is missing")
    digest = data["bag_sha256"]
    require(isinstance(digest, str) and len(digest) == 64 and all(c in "0123456789abcdef" for c in digest),
            "invalid bag digest")
    capture = read_json(base / data["capture_manifest"])
    require(capture["bag_sha256"] == digest and capture["capture_id"] == data["capture_id"]
            and (capture["split"] == split or (split == "online" and capture["split"] == "blind")),
            "capture metadata does not identify this bag")
    require(capture.get("passed") is True, "incomplete or failed native capture")
    require(type(capture["b_visibility_checks"]) is int and capture["b_visibility_checks"] > 0
            and capture["b_visible_after_window"] == 0, "historical B revisit visibility is unverified")
    voxel = float(data["voxel_m"])
    require(math.isfinite(voxel) and voxel > 0, "invalid voxel resolution")
    truth = positions(base / data["truth"])
    reference = cloud(base / data["reference_b"])
    windows = event_windows(base / data["windows"])
    capture_dir = (base / data["capture_manifest"]).parent
    exposures = exposure_times(capture_dir / "depth_stamps.npy")
    a = exposure_times(capture_dir / "a_visible_stamps.npy")
    c = exposure_times(capture_dir / "c_visible_stamps.npy")
    require(np.isin(a, exposures).all() and np.isin(c, exposures).all()
            and np.all(within(a, windows["first_a"]) | within(a, windows["revisit"]))
            and np.all(within(c, windows["negative"])), "invalid independent visibility annotations")
    visible_a, visible_c = set(a.tolist()), set(c.tolist())
    require(all(truth[0][0] <= a < b <= truth[0][-1] for a,b in windows.values()),
            "route windows lack truth coverage")
    first_a, _ = samples(truth, [sum(windows["first_a"]) // 2])
    negative, _ = samples(truth, [sum(windows["negative"]) // 2])
    revisit, _ = samples(truth, [sum(windows["revisit"]) // 2])
    require(np.linalg.norm(first_a - revisit) < np.linalg.norm(first_a - negative),
            "negative control is not separated from the A revisit in independent truth")
    require(len(data["rounds"]) == (1 if split == "online" else 3),
            f"{split}: incorrect number of independent rounds")
    run_ids, pose_files = set(), set()
    for round_spec in data["rounds"]:
        for mode in ("on", "off", "lost"):
            if mode not in round_spec:
                continue
            run = round_spec[mode]
            filename = (base / run["poses"]).resolve()
            require(run["run_id"] and run["run_id"] not in run_ids and filename not in pose_files,
                    "reused run id or output in the same dataset")
            run_ids.add(run["run_id"])
            pose_files.add(filename)
    rounds = [grade_round(base, r, truth, reference, windows, voxel, exposures, visible_a, visible_c)
              for r in data["rounds"]]
    return {"capture_id": data["capture_id"], "split": split, "bag_sha256": digest, "rounds": rounds,
            "voxel_m": voxel, "passed": all(r["passed"] for r in rounds)}


def greater_is_better(name):
    return name.endswith("_coverage") or name in ("b_gain_over_off_m", "on_b_gain_over_before_m")


def limits(runs):
    keys = set(runs[0]["metrics"])
    require(all(set(r["metrics"]) == keys for r in runs), "incomplete calibration metrics")
    bounds = {}
    for key in sorted(keys - {"on_b_correction_m"}):
        values = [r["metrics"][key] for r in runs]
        span = max(values) - min(values)
        lower = greater_is_better(key)
        bounds[key] = {"direction": "min" if lower else "max",
                       "limit": min(values) - span if lower else max(values) + span,
                       "calibration_min": min(values), "calibration_max": max(values)}
    return bounds


def write_report(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


GAINS = ("b_gain_over_off_m", "on_b_gain_over_before_m")


def repeat_noise(capture):
    # Include covariance in the paired effect as well as scatter in each surface.
    return max(float(np.ptp([r["metrics"][key] for r in capture["rounds"]]))
               for key in (*GAINS, "off_b_after_p95_m", "on_b_after_p95_m", "on_b_before_p95_m"))


def freeze(evidence_paths, threshold_path, plot=None):
    require(len(evidence_paths) > 0, "no calibration data")
    captures = [score_capture(p) for p in evidence_paths]
    require(all(c["split"] == "calibration" for c in captures), "freeze requires calibration split")
    failures = sorted({k for c in captures for r in c["rounds"] for k,v in r["checks"].items() if not v})
    require(not failures, "calibration failed structural checks: " + ", ".join(failures))
    bags = [c["bag_sha256"] for c in captures]
    require(len(set(bags)) == len(bags), "duplicate calibration bag")
    resolution = {c["voxel_m"] for c in captures}
    require(len(resolution) == 1, "mixed scoring resolution")
    # Keep repeatability distinct from differences between independent flights.
    # Both the paired effect and the individually measured surfaces can fluctuate.
    observed_noise = max(repeat_noise(c) for c in captures)
    require(all(r["metrics"][key] > observed_noise for c in captures for r in c["rounds"] for key in GAINS),
            "B improvement does not exceed calibration repeat variation")
    result = {"schema_version": 2, "method": "worst_plus_range", "b_repeat_noise_m": observed_noise,
              "calibration_ids": [c["capture_id"] for c in captures], "calibration_bags": bags,
              "voxel_m": resolution.pop(), "limits": limits([r for c in captures for r in c["rounds"]]),
              "calibration": captures}
    threshold_path = Path(threshold_path)
    threshold_path.parent.mkdir(parents=True, exist_ok=True)
    if threshold_path.exists():
        require(read_json(threshold_path) == result, "existing frozen thresholds differ")
    else:
        with threshold_path.open("x") as out:
            json.dump(result, out, ensure_ascii=False, indent=2, allow_nan=False)
            out.write("\n")
    if plot:
        plot_results(captures, evidence_paths, Path(plot))
    return result


def evaluate(evidence_paths, threshold_path, output=None, plot=None):
    threshold_path = Path(threshold_path)
    frozen = read_json(threshold_path)
    require(frozen["schema_version"] == 2 and frozen["method"] == "worst_plus_range"
            and bool(frozen["limits"]), "unsupported frozen thresholds")
    require(evidence_paths, "no evaluation data")
    captures = [score_capture(path) for path in evidence_paths]
    blind = {c["bag_sha256"]: c for c in captures if c["split"] == "blind"}
    online = {c["bag_sha256"]: c for c in captures if c["split"] == "online"}
    require(blind and blind.keys() == online.keys() and len(captures) == 2 * len(blind),
            "evaluation requires one blind replay set and one online flight for the same held-out bag")
    for bag, flight in online.items():
        require(flight["capture_id"] == blind[bag]["capture_id"]
                and flight["rounds"][0]["run_ids"]["on"] not in {
                    r["run_ids"]["on"] for r in blind[bag]["rounds"]}, "online evidence must come from the original flight")
    for capture in captures:
        require(capture["split"] in ("blind", "online"), "evaluation needs held-out data")
        require(capture["bag_sha256"] not in frozen["calibration_bags"]
                and capture["capture_id"] not in frozen["calibration_ids"], "calibration leakage")
        require(capture["voxel_m"] == frozen["voxel_m"], "evaluation resolution differs from calibration")
        noise = max(frozen["b_repeat_noise_m"], repeat_noise(blind[capture["bag_sha256"]]))
        capture["b_repeat_noise_m"] = noise
        for run in capture["rounds"]:
            run["checks"]["b_gain_exceeds_repeat_noise"] = all(
                run["metrics"][key] > noise for key in GAINS)
            for name, bound in frozen["limits"].items():
                value = run["metrics"].get(name)
                run["checks"]["limit_" + name] = value is not None and (
                    value >= bound["limit"] if bound["direction"] == "min" else value <= bound["limit"])
            run["passed"] = all(run["checks"].values())
        capture["passed"] = all(r["passed"] for r in capture["rounds"])
    result = {"schema_version": 2, "thresholds_sha256": hashlib.sha256(threshold_path.read_bytes()).hexdigest(),
              "captures": captures, "passed": all(c["passed"] for c in captures)}
    if output:
        write_report(output, result)
    if plot:
        plot_results(captures, evidence_paths, Path(plot))
    return result


def plot_results(captures, evidence_paths, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels, on, off, after = [], [], [], []
    for c in captures:
        for index, run in enumerate(c["rounds"], 1):
            labels.append(f"{c['capture_id']} / {c['split']} {index}")
            m = run["metrics"]
            on.append(m["on_b_before_p95_m"])
            after.append(m["on_b_after_p95_m"])
            off.append(m["off_b_after_p95_m"])
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, max(2.8, 0.9 * len(labels))))
    y = np.arange(len(labels))
    for vals, marker, title in ((on, "s", "Before loop"), (after, "o", "After loop"), (off, "x", "Loop disabled")):
        ax.scatter(vals, y, marker=marker, label=title)
    ax.set_yticks(y, labels)
    ax.set_xlabel("Historical B surface error, p95 (m)")
    ax.legend(loc="best")
    ax.grid(axis="x", alpha=.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    for idx, evidence_path in enumerate(evidence_paths, 1):
        evidence_path = Path(evidence_path).resolve()
        base = evidence_path.parent
        manifest = read_json(evidence_path)
        truth = positions(base / manifest["truth"])
        windows = event_windows(base / manifest["windows"])
        reference = cloud(base / manifest["reference_b"])
        for repeat, run in enumerate(manifest["rounds"], 1):
            # Number the output by capture and round so that no first-run example can hide a failure.
            prefix = path.with_name(f"{path.stem}-capture{idx}-run{repeat}")
            fig, ax = plt.subplots(figsize=(7, 6))
            ax.plot(*truth[1][:, :2].T, color="#222222", label="Gazebo truth", lw=2)
            palette = {"off": "#bb6250", "on": "#137daa"}
            for mode in ("off", "on"):
                cfg = run[mode]
                raw = positions(base / cfg["poses"])
                transform = initial_alignment(truth, raw, windows["align"])
                graph = transformed(positions(base / cfg["trajectory"])[1], transform)
                ax.plot(*graph[:, :2].T, color=palette[mode], lw=1.4,
                        label=f"Final graph, loop {mode}", ls="--" if mode == "off" else "-")
                if mode == "on":
                    ax.plot(*transformed(raw[1], transform)[:, :2].T, color="#35a785", alpha=.65,
                            lw=.9, label="Online pose, loop on")
            ax.set(xlabel="East (m)", ylabel="North (m)", title=f'{manifest["capture_id"]} / {manifest["split"]} / {repeat}')
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(alpha=.2)
            ax.legend()
            fig.tight_layout()
            fig.savefig(prefix.with_name(prefix.name + "-trajectory" + path.suffix), dpi=160)
            plt.close(fig)

            on_run = run["on"]
            transform = initial_alignment(truth, positions(base / on_run["poses"]), windows["align"])
            before = transformed(cloud(base / on_run["b_before"]), transform)
            after = transformed(cloud(base / on_run["b_after"]), transform)
            fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.3))
            for ax, (first, second) in zip(axes, ((0, 1), (0, 2), (1, 2))):
                for pts, title, color, marker in (
                        (reference, "STL reference", "#262626", "."),
                        (before, "Before loop", "#c4734e", "+"),
                        (after, "After loop", "#007fa8", ".")):
                    # For rendering only, keep deterministic samples. Scores use every point.
                    view = pts[::max(1, (len(pts) + 4999) // 5000)]
                    ax.scatter(view[:, first], view[:, second], s=6, color=color, alpha=.55,
                               marker=marker, label=title)
                axis = ["East (m)", "North (m)", "Up (m)"]
                ax.set(xlabel=axis[first], ylabel=axis[second])
                ax.set_aspect("equal", adjustable="datalim")
                ax.grid(alpha=.2)
            axes[-1].legend()
            fig.suptitle(f'{manifest["capture_id"]} / {manifest["split"]} / {repeat} — historical B')
            fig.tight_layout()
            fig.savefig(prefix.with_name(prefix.name + "-surface-B" + path.suffix), dpi=160)
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    cal = sub.add_parser("freeze")
    cal.add_argument("--evidence", type=Path, action="append", required=True)
    cal.add_argument("--output", type=Path, required=True)
    cal.add_argument("--plot", type=Path)
    ev = sub.add_parser("evaluate")
    ev.add_argument("--evidence", type=Path, action="append", required=True)
    ev.add_argument("--thresholds", type=Path, required=True)
    ev.add_argument("--output", type=Path, required=True)
    ev.add_argument("--plot", type=Path)
    args = parser.parse_args()
    try:
        if args.command == "freeze":
            freeze(args.evidence, args.output, args.plot)
            return 0
        result = evaluate(args.evidence, args.thresholds, args.output, args.plot)
        return 0 if result["passed"] else 1
    except (ValueError, KeyError, TypeError, OSError) as error:
        if args.command == "evaluate":
            write_report(args.output, {"passed": False, "error": str(error)})
        parser.exit(2, f"Invalid evidence: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
