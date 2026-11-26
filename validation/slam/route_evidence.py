"""Native MCAP measurements of sensor timing and actual landmark visibility."""
import csv
import json
import struct

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from validation.slam.route import CAMERA, sha, visible_points


def header_stamp(raw):
    # ROS sensor_msgs header stamp follows the four-byte CDR encapsulation.
    if len(raw) < 12 or raw[:2] not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("invalid CDR encapsulation")
    seconds, ns = struct.unpack_from("<iI" if raw[1] else ">iI", raw, 4)
    if ns >= 1_000_000_000 or seconds < 0:
        raise ValueError("invalid sensor sample stamp")
    return seconds * 1_000_000_000 + ns


class Truth:
    def __init__(self, path):
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        self.times = np.array([int(r["stamp_ns"]) for r in rows], dtype=np.int64)
        if len(self.times) < 2 or np.any(np.diff(self.times) <= 0):
            raise ValueError("need ordered Gazebo model truth")
        self.positions = np.array([[float(r[k]) for k in ("x","y","z")] for r in rows])
        quats = np.array([[float(r[k]) for k in ("qx","qy","qz","qw")] for r in rows])
        self.rotation = Slerp((self.times-self.times[0])*1e-9, Rotation.from_quat(quats))

    def pose(self, stamp):
        times = self.times
        if stamp < times[0] or stamp > times[-1]:
            raise ValueError("truth does not cover the scored exposure")
        right = int(np.clip(np.searchsorted(times, stamp), 1, len(times)-1))
        left = right - 1
        if times[right] - times[left] > 150_000_000:
            raise ValueError("truth gap exceeds 150 ms at a scored exposure")
        alpha = (stamp-times[left])/(times[right]-times[left])
        origin = self.positions[left] + alpha * (self.positions[right]-self.positions[left])
        return origin, self.rotation((stamp-times[0])*1e-9).as_matrix()


def patch_returns(patch, position, rotation, intrinsics, msg):
    """Test first-hit STL points against independent native, unregistered depth pixels."""
    points = patch[visible_points(patch, position, rotation, intrinsics)]
    if len(points) < 8:
        return False
    fx, fy, cx, cy, width, height = intrinsics
    forward, left, up = ((points - position) @ rotation - CAMERA).T
    u = np.clip(np.rint(cx - fx * left / forward).astype(int), 0, width-1)
    v = np.clip(np.rint(cy - fy * up / forward).astype(int), 0, height-1)
    if msg.encoding != "32FC1" or (msg.width, msg.height) != (width, height):
        raise ValueError("unexpected native depth format or size")
    depth = np.ndarray((height, width), np.dtype(">f4" if msg.is_bigendian else "<f4"), msg.data,
                       strides=(msg.step, 4))
    # Require returns on multiple distinct pixels. Allow subpixel parallax at mesh boundaries.
    supported = np.abs(depth[v, u] - forward) <= .20
    return len(np.unique(v[supported] * width + u[supported])) >= 8 and supported.sum() >= .15 * len(patch)


def within(stamp, window):
    return window[0] <= stamp <= window[1]


def timing(samples, hz):
    gaps = np.diff(np.asarray(samples, dtype=np.int64)) * 1e-9
    if len(gaps) == 0 or np.any(gaps <= 0):
        raise ValueError("missing, duplicate or regressing native sensor stamps")
    return {"messages": len(samples), "effective_hz": float(1. / gaps.mean()), "max_gap_s": float(gaps.max()),
            "expected_hz": hz, "estimated_missing_periods": int(np.maximum(0, np.rint(gaps*hz)-1).sum())}


def bag_evidence(output, topics, windows):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from sensor_msgs.msg import CameraInfo, Image

    raw = output / "raw"
    meta = yaml.safe_load((raw/"metadata.yaml").read_text())["rosbag2_bagfile_information"]
    if meta["storage_identifier"] != "mcap" or len(meta["relative_file_paths"]) != 1:
        raise ValueError("expected one native MCAP")
    counts = {row["topic_metadata"]["name"]: row["message_count"] for row in meta["topics_with_message_count"]}
    if set(counts) != set(topics) or any(counts[t] <= 0 for t in topics):
        raise ValueError(f"raw MCAP topic counts differ from the sensor-only contract: {counts}")

    def messages(selected):
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(raw), storage_id="mcap"), ConverterOptions("", ""))
        reader.set_filter(StorageFilter(topics=selected))
        while reader.has_next():
            yield reader.read_next()

    plan = json.loads((output/"route.json").read_text())
    profile = plan["native_camera_profile"]
    info = {}
    for topic, data, _ in messages([name + "/camera_info" for name in profile]):
        name = topic.removesuffix("/camera_info")
        msg = deserialize_message(data, CameraInfo)
        values = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)
        if (not np.isfinite(values).all() or min(values) <= 0 or (name in info and info[name] != values)
                or values[-2:] != (profile[name]["width"], profile[name]["height"])):
            raise ValueError(f"wrong native resolution, invalid or changing calibration on {name}")
        info[name] = values
    if info.keys() != profile.keys():
        raise ValueError("missing recorded native camera intrinsics")

    truth = Truth(output/"truth.csv")
    patches = {key: np.load(output/f"{key}_reference.npy", allow_pickle=False) for key in ("a","b","c")}
    view = {key: [] for key in ("first_a","b","negative","revisit")}
    checked = {key: 0 for key in view}
    later_count = later_seen = 0
    stamps = {name: [] for name in [*profile, "/slam/input/imu"]}
    for name, data, _ in messages(list(stamps)):
        stamp = header_stamp(data)
        stamps[name].append(stamp)
        if name != "/depth_camera":
            continue
        later = windows["occlusion"][1] < stamp <= windows["revisit"][1]
        phase = next((key for key in view if within(stamp, windows[key])), None)
        if not phase and not later:
            continue
        origin, rotation = truth.pose(stamp)
        if later:
            later_count += 1
            if visible_points(patches["b"], origin, rotation, info[name]).any():
                later_seen += 1
        if phase:
            checked[phase] += 1
            target = {"first_a":"a", "revisit":"a", "negative":"c", "b":"b"}[phase]
            msg = deserialize_message(data, Image)
            if patch_returns(patches[target], origin, rotation, info[name], msg):
                view[phase].append(stamp)
    for phase, hits in view.items():
        if len(hits) < 2 or len(hits) < checked[phase] / 2:
            raise ValueError(f"insufficient native depth support for {phase}: {len(hits)}/{checked[phase]}")
    if not later_count:
        raise ValueError("no post-occlusion exposures")

    rates = {name: timing(stamps[name], settings["hz"]) for name, settings in profile.items()}
    rates["/slam/input/imu"] = timing(stamps["/slam/input/imu"], plan["imu_hz"])
    np.save(output/"depth_stamps.npy", np.asarray(stamps["/depth_camera"], dtype=np.int64))
    np.save(output/"a_visible_stamps.npy", np.asarray(view["first_a"] + view["revisit"], dtype=np.int64))
    np.save(output/"c_visible_stamps.npy", np.asarray(view["negative"], dtype=np.int64))
    filename = meta["relative_file_paths"][0]
    return {"bag_sha256": sha(raw/filename), "bag_directory": "raw", "bag_file": filename,
            "topic_counts": counts, "sensor_timing": rates, "recorded_intrinsics": info,
            "landmark_exposures": {key: {"checked": checked[key], "visible": len(hits)} for key, hits in view.items()},
            "b_visibility_checks": later_count, "b_visible_after_window": later_seen,
            "b_first_view_exposures": checked["b"], "b_first_view_depth_confirmed": len(view["b"]),
            "b_visibility_method": "native first-hit depth support; conservative frustum on every post-mask exposure"}
