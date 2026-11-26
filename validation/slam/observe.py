"""Record SLAM estimates separately from the measurements and scoring truth."""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


def sample_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_row(pose):
    t, q = pose.position, pose.orientation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]


def transform_row(tf):
    t, q = tf.translation, tf.rotation
    return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile, DurabilityPolicy,
                            ReliabilityPolicy)
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import Info, MapGraph
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    points_dir = args.out / "depth_samples"
    points_dir.mkdir()
    rclpy.init()
    node = Node("slam_experiment_observer", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    streams = ExitStack()
    files = {name: streams.enter_context((args.out / (name + ".jsonl")).open("x", buffering=1))
             for name in ("odom", "status", "info", "graphs", "depth")}
    calibration = {}
    def write(name, record):
        files[name].write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")

    def odom(msg):
        p = pose_row(msg.pose.pose)
        if np.all(np.isfinite(p)) and np.linalg.norm(p[3:]) > 0.5:
            write("odom", {"ns": sample_ns(msg.header.stamp), "p": p,
                           "receipt_sim_ns": node.get_clock().now().nanoseconds})

    def status(msg):
        write("status", json.loads(msg.data))

    def info(msg):
        write("info", {"ns": sample_ns(msg.header.stamp), "id": msg.ref_id,
                       "loop": msg.loop_closure_id, "proximity": msg.proximity_detection_id})

    def graph(msg):
        poses = {str(i): pose_row(p) for i, p in zip(msg.poses_id, msg.poses)}
        write("graphs", {"ns": sample_ns(msg.header.stamp), "poses": poses,
                         "map_to_odom": transform_row(msg.map_to_odom),
                         "loops": [[link.from_id, link.to_id] for link in msg.links
                                   if link.type == 1]})

    def extrinsic(msg):
        for tf in msg.transforms:
            if (tf.header.frame_id, tf.child_frame_id) == ("slam_body", "slam_camera_optical"):
                calibration["optical"] = transform_row(tf.transform)

    def image(msg):
        stamp = sample_ns(msg.header.stamp)
        now = node.get_clock().now().nanoseconds
        write("depth", {"ns": stamp, "sim_ns": now, "steady_ns": time.monotonic_ns()})
        k = calibration.get("info")
        mount = calibration.get("optical")
        if not mount or k is None or (k.width, k.height, k.header.frame_id) != (
                msg.width, msg.height, msg.header.frame_id):
            return
        if msg.encoding != "32FC1" or len(msg.data) < msg.height * msg.step:
            raise ValueError("registered depth must be a valid 32FC1 image")
        z = np.ndarray((msg.height, msg.width), dtype=">f4" if msg.is_bigendian else "<f4",
                       buffer=msg.data, strides=(msg.step, 4))
        stride = max(1, min(msg.height, msg.width) // 48)
        v, u = np.mgrid[stride//2:msg.height:stride, stride//2:msg.width:stride]
        d = z[v, u]
        good = np.isfinite(d) & (d > 0)
        u, v, d = u[good], v[good], d[good]
        optical = np.column_stack(((u-k.k[2])*d/k.k[0], (v-k.k[5])*d/k.k[4], d))
        body = Rotation.from_quat(mount[3:]).apply(optical) + mount[:3]
        np.save(points_dir / str(stamp), body.astype(np.float32), allow_pickle=False)

    static_qos = QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    for kind, topic, handler, qos in (
            (Odometry, "/slam/odometry", odom, qos_profile_sensor_data),
            (Info, "/slam/info", info, 50), (MapGraph, "/slam/mapGraph", graph, 10),
            (String, "/slam/status", status, 100),
            (TFMessage, "/slam/tf_static", extrinsic, static_qos),
            (CameraInfo, "/slam/rgb/camera_info", lambda m: calibration.update(info=m), qos_profile_sensor_data),
            (Image, "/slam/depth_registered/image", image, qos_profile_sensor_data)):
        node.create_subscription(kind, topic, handler, qos)
    def check_discovery():
        # Replay starts only after both sides have discovered the estimator.
        if (all(node.count_publishers(t) for t in ("/slam/status", "/slam/mapGraph", "/slam/odometry"))
                and node.count_subscribers("/camera") and node.count_subscribers("/slam/input/imu")):
            (args.out / "ready").touch()
            timer.cancel()

    timer = node.create_timer(.2, check_discovery, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        streams.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
