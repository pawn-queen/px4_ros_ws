#!/usr/bin/env python3
"""Capture the prescribed flight on an already running native powerplant/x500_depth SITL stack."""
import argparse
import csv
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from validation.slam.route import check_loaded_model, prepare, save_json, sha
from validation.slam.route_evidence import bag_evidence

TOPICS = ("/clock", "/camera", "/camera/camera_info", "/depth_camera", "/depth_camera/camera_info",
          "/slam/input/imu", "/slam/input/occluded")
SETPOINT = "/fmu/in/trajectory_setpoint"
MODE = "/fmu/in/offboard_control_mode"
TRUTH = "/world/powerplant/dynamic_pose/info"


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def bounded_step(current, target, maximum):
    difference = target - current
    distance = float(np.linalg.norm(difference))
    return target.copy() if distance <= maximum else current + difference * (maximum / distance)


def run_flight(output, plan, manifest):
    import rclpy
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.sdf_generator_config_pb2 import SdfGeneratorConfig
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from px4_msgs import msg as px
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("native_slam_capture", parameter_overrides=[Parameter("use_sim_time", value=True)])
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    mode = node.create_publisher(px.OffboardControlMode, MODE, qos)
    setpoint = node.create_publisher(px.TrajectorySetpoint, SETPOINT, qos)
    commands = node.create_publisher(px.VehicleCommand, "/fmu/in/vehicle_command", qos)
    mask = node.create_publisher(Bool, TOPICS[-1], 10)
    state = {}
    receipts = {}
    events = []
    def subscribe(topic, cls, key):
        number = getattr(cls, "MESSAGE_VERSION", 0)
        def receive(message):
            state[key] = message
            receipts[key] = time.monotonic()
        node.create_subscription(cls, "/fmu/out/" + topic + (f"_v{number}" if number else ""), receive, qos)
    subscribe("vehicle_status", px.VehicleStatus, "status")
    subscribe("vehicle_local_position", px.VehicleLocalPosition, "pos")
    subscribe("vehicle_land_detected", px.VehicleLandDetected, "land")
    subscribe("estimator_status_flags", px.EstimatorStatusFlags, "aiding")

    def now():
        return node.get_clock().now().nanoseconds
    def log(text):
        print(f"{now()*1e-9:.2f} {text}", flush=True)
    def exclusive():
        return all(node.count_publishers(topic) == 1 for topic in (MODE, SETPOINT))
    def enu():
        p = state["pos"]
        return np.array([p.y, p.x, -p.z], dtype=float)
    def attitude_yaw():
        return wrap(math.pi/2 - state["pos"].heading)
    def ground():
        return ("status" in state and state["status"].arming_state == px.VehicleStatus.ARMING_STATE_DISARMED
                and "land" in state and state["land"].landed)
    def armed():
        return "status" in state and state["status"].arming_state == px.VehicleStatus.ARMING_STATE_ARMED
    def offboard():
        return state["status"].nav_state == px.VehicleStatus.NAVIGATION_STATE_OFFBOARD
    def healthy_position():
        p = state.get("pos")
        return (p is not None and p.xy_valid and p.z_valid and np.isfinite(enu()).all()
                and math.isfinite(p.heading) and time.monotonic() - receipts["pos"] < 3.)
    def command(code, a=0., b=0.):
        status = state["status"]
        msg = px.VehicleCommand()
        msg.timestamp = now() // 1000
        msg.target_system, msg.target_component = status.system_id, status.component_id
        msg.source_system, msg.source_component = 1, 191
        msg.from_external = True
        msg.command, msg.param1, msg.param2 = code, float(a), float(b)
        commands.publish(msg)

    # Gazebo truth is a separate recording channel. No pose values enter PX4 setpoints.
    gz = GzNode()
    request = SdfGeneratorConfig()
    request.global_entity_gen_config.expand_include_tags.data = True
    request.global_entity_gen_config.resources_use_absolute_paths.data = True
    ok, snapshot = gz.request(f"/world/{plan['world']}/generate_world_sdf", request,
                              SdfGeneratorConfig, StringMsg, 15000)
    if not ok:
        raise ValueError("Gazebo did not return its live SDF")
    manifest["loaded_profile"] = check_loaded_model(snapshot.data, plan)
    loaded_sdf = output / "loaded_world.sdf"
    loaded_sdf.write_text(snapshot.data)
    manifest["loaded_sdf_sha256"] = sha(loaded_sdf)
    truth_csv = (output / "truth.csv").open("x", newline="")
    writer = csv.writer(truth_csv)
    writer.writerow(("stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"))
    truth_lock = threading.Lock()
    truth_count, truth_last, entity = 0, 0, None
    truth_error = None
    def record_truth(packet):
        nonlocal truth_count, truth_last, entity, truth_error
        for pose in packet.pose:
            if pose.name != "x500_depth_0":
                continue
            stamp = packet.header.stamp.sec * 10**9 + packet.header.stamp.nsec
            q, p = pose.orientation, pose.position
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            with truth_lock:
                if truth_csv.closed:
                    return
                if (entity is not None and entity != pose.id) or stamp < truth_last:
                    truth_error = "native truth model or clock reset"
                if stamp > truth_last and all(math.isfinite(float(v)) for v in values):
                    writer.writerow((stamp, *values))
                    truth_last, entity = stamp, pose.id
                    truth_count += 1
            return
    if not gz.subscribe(Pose_V, TRUTH, record_truth):
        raise RuntimeError("cannot subscribe to the native Gazebo model truth")

    enabled = masked = mask_enabled = airborne = arm_sent = False
    ref = np.zeros(3)
    yaw = 0.
    last_send = last_mask = last_graph_check = -1
    recorder = None
    recorder_log = None
    def spin(check_flight=False):
        nonlocal last_send, last_mask, last_graph_check
        rclpy.spin_once(node, timeout_sec=.01)
        stamp = now()
        if stamp < last_send or truth_error:
            raise RuntimeError(truth_error or "simulation clock rewound")
        if recorder and recorder.poll() is not None:
            raise RuntimeError("raw bag recorder exited during capture")
        if mask_enabled and stamp-last_mask >= 200_000_000:
            mask.publish(Bool(data=masked))
            last_mask = stamp
        if enabled:
            if stamp-last_graph_check >= 500_000_000:
                last_graph_check = stamp
                if not exclusive():
                    raise RuntimeError("additional PX4 setpoint or mode publisher")
            if check_flight and (not armed() or not offboard() or not healthy_position() or state["status"].failsafe):
                raise RuntimeError("PX4 left the healthy armed Offboard state")
            if stamp-last_send >= 25_000_000:
                last_send = stamp
                m = px.OffboardControlMode()
                m.timestamp, m.position = stamp//1000, True
                mode.publish(m)
                sp = px.TrajectorySetpoint()
                sp.timestamp = stamp//1000
                sp.position = [float(ref[1]), float(ref[0]), float(-ref[2])]
                sp.velocity = sp.acceleration = sp.jerk = [math.nan] * 3
                sp.yaw, sp.yawspeed = wrap(math.pi/2-yaw), math.nan
                setpoint.publish(sp)
    def wait(condition, wall_budget, label):
        end = time.monotonic() + wall_budget
        while time.monotonic() < end:
            spin()
            if condition():
                return
        raise RuntimeError("timeout waiting for " + label + "; " + str({
            "topics": list(state), "truth_samples": truth_count, "sim_ns": now(),
            "preflight": getattr(state.get("status"), "pre_flight_checks_pass", None),
            "ev_pos": getattr(state.get("aiding"), "cs_ev_pos", None),
        }))
    def change(code, a, b, predicate, label, timeout):
        last = -10.
        def reached():
            nonlocal last
            if predicate():
                return True
            if time.monotonic()-last > 1.5:
                command(code, a, b)
                last = time.monotonic()
            return False
        wait(reached, timeout, label)

    def land():
        nonlocal enabled
        enabled = False
        # Keep requesting NAV_LAND until the flight controller acknowledges the mode.
        # Never force disarm. Landing completion requires a fresh landed sample AND disarm.
        log("request NAV_LAND")
        landing_start = time.monotonic()
        end = landing_start+120.
        last_command = -10.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if (ground() and time.monotonic()-landing_start > 2.
                    and all(receipts[k] > landing_start and time.monotonic()-receipts[k]<5. for k in ("land","status"))):
                return True
            if ("status" in state and state["status"].nav_state != px.VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                    and time.monotonic() - last_command >= 1.):
                command(px.VehicleCommand.VEHICLE_CMD_NAV_LAND)
                last_command = time.monotonic()
        return False

    def window(name, start, end, point, angle):
        events.append({"name": name, "start_ns": start, "end_ns": end, "waypoint": name,
                       "position_enu_m": list(point), "yaw_rad": angle})
        save_json(output / "events.json", {"clock": "gazebo_sim_ns", "events": events})

    try:
        # Arming checks are evaluated for the *current* flight mode. SITL boots in
        # AUTO_LOITER, whose check set needs a home position this stack never has,
        # so pre_flight_checks_pass cannot be required before the mode switch.
        # It is required after the Offboard request, where the arm command follows.
        wait(lambda: now()>0 and truth_count>=3 and healthy_position() and ground() and
             state.get("aiding") is not None and state["aiding"].cs_ev_pos,
             180., "clock, named Gazebo model, grounded UWB/PX4 and EV aiding")
        if not exclusive():
            raise RuntimeError("start native capture with use_control:=false; another publisher owns Offboard")
        origin = enu()
        if np.linalg.norm(origin[:2]) > .5 or abs(origin[2]) > .45:
            raise RuntimeError("PX4 local origin is outside the audited native spawn column")
        ref, yaw = origin.copy(), attitude_yaw()
        recorder_log = (output / "record.log").open("x")
        recorder = subprocess.Popen(["ros2","bag","record","--use-sim-time","-s","mcap","-o",str(output/"raw"),
                                     "--topics", *TOPICS], stdout=recorder_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        # Hold the first gate message until the recorder and raw producers are active.
        # Brief raw subscriptions only check message headers and are removed before flight.
        samples = {}
        subscriptions = []
        def check_first(topic, cls):
            def receive(msg):
                if topic in samples:
                    return
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if stamp <= 0:
                    raise ValueError(f"invalid native sensor stamp on {topic}")
                if cls != Imu:
                    profile = plan["native_camera_profile"][topic.removesuffix("/camera_info")]
                    if (msg.width, msg.height) != (profile["width"], profile["height"]):
                        raise ValueError(f"native image shape mismatch on {topic}")
                samples[topic] = stamp
            subscriptions.append(node.create_subscription(cls, topic, receive, qos_profile_sensor_data))
        for topic, cls in [(TOPICS[1], Image), (TOPICS[2], CameraInfo), (TOPICS[3], Image),
                           (TOPICS[4], CameraInfo), (TOPICS[5], Imu)]:
            check_first(topic, cls)
        wait(lambda: all(any(e.node_name.startswith("rosbag2_recorder")
                             for e in node.get_subscriptions_info_by_topic(topic)) for topic in TOPICS)
             and all(node.count_publishers(topic)>0 for topic in TOPICS)
             and len(samples) == len(subscriptions), 90., "raw recorder discovery and native sensor samples")
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        manifest["initial_sensor_stamps_ns"] = samples
        mask_enabled = True
        mask.publish(Bool(data=False))
        manifest["gate_open_sim_ns"] = now()
        warm = now()
        enabled = True
        wait(lambda: now()-warm >= 2_000_000_000, 30., "Offboard setpoint warmup")
        change(px.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6., offboard, "Offboard mode", 25.)
        wait(lambda: state["status"].pre_flight_checks_pass, 30.,
             "preflight checks in the requested Offboard mode")
        arm_sent = True
        change(px.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1., 0., armed, "PX4 armed", 25.)
        airborne = True
        mask.publish(Bool(data=False))
        for segment in plan["segments"]:
            name, coord, degrees = segment["name"], segment["position_enu_m"], segment["yaw_deg"]
            dwell, use_mask = segment["dwell_s"], segment["occluded"]
            target = np.array(coord)
            angle = math.radians(degrees)
            masked = use_mask
            mask.publish(Bool(data=masked))
            start = last = now()
            entry = settled = None
            leg_deadline = time.monotonic() + 180.
            start_ref = ref.copy()
            speed = plan["approach_speed_mps"] if name in ("first_a", "revisit") else plan["cruise_speed_mps"]
            log("leg " + name)
            while time.monotonic() < leg_deadline:
                spin(check_flight=True)
                stamp = now()
                dt = max(0., min(.12, (stamp-last)*1e-9))
                last = stamp
                ref = target.copy() if name=="takeoff" else bounded_step(ref, target, speed*dt)
                yaw = wrap(yaw + np.clip(wrap(angle-yaw), -.42*dt, .42*dt))
                actual = enu()
                # Geometric clearance above the takeoff area assumes <=0.6 m lateral tracking error.
                horizontal = start_ref[:2] + np.clip(np.dot(actual[:2]-start_ref[:2], target[:2]-start_ref[:2]) /
                                 max(1e-9, np.sum((target[:2]-start_ref[:2])**2)), 0., 1.)*(target[:2]-start_ref[:2])
                if np.linalg.norm(actual[:2]-horizontal) > .65 or actual[2] > 3.48:
                    raise RuntimeError("PX4 departed the audited route envelope on " + name)
                if name!="takeoff" and actual[2]<2.48:
                    raise RuntimeError("PX4 below route clearance on " + name)
                distance = np.linalg.norm(actual-target)
                diff = abs(wrap(angle-attitude_yaw()))
                if entry is None and distance < 1.5 and diff < .32:
                    entry = stamp
                if distance < .32 and diff < .18 and np.linalg.norm(ref-target)<.02:
                    if settled is None:
                        settled = stamp
                    if (stamp-settled)*1e-9 >= dwell:
                        break
                else:
                    settled = None
            else:
                raise RuntimeError("failed to reach and dwell at " + name)
            if name in ("first_a","b","occlusion","negative","revisit"):
                window(name, start if name == "occlusion" else entry, stamp, coord, angle)
                if name == "first_a":
                    window("align", settled, stamp, coord, angle)
        manifest["route_completed"] = True
    finally:
        enabled = False
        try:
            if airborne or arm_sent:
                manifest["landed_disarmed"] = land()
                if not manifest["landed_disarmed"]:
                    manifest["landing_error"] = "landed and disarmed acknowledgement missing"
        finally:
            if recorder and recorder.poll() is None:
                os.killpg(recorder.pid, signal.SIGINT)
                try:
                    recorder.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    recorder.terminate()
                    recorder.wait(timeout=15)
                    manifest["recorder_error"] = "recorder did not finalize after SIGINT"
            if recorder_log:
                recorder_log.close()
            gz.unsubscribe(TRUTH)
            with truth_lock:
                truth_csv.close()
            manifest.update({"truth_samples": truth_count, "truth_entity_id": entity,
                             "truth_topic": TRUTH, "truth_model": "x500_depth_0",
                             "windows": {e["name"]: [e["start_ns"], e["end_ns"]] for e in events}})
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--px4-root", type=Path, default=Path(os.getenv("PX4_ROOT", str(Path.home()/"drone-simulation"))))
    parser.add_argument("--split", choices=["calibration", "blind"], required=True)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any((output/item).exists() for item in ("raw","truth.csv","manifest.json")):
        parser.error("choose a directory without previous capture evidence")
    plan = prepare(args.px4_root.resolve(), args.split, output)
    if args.prepare_only:
        print(output / "route.json")
        return 0
    manifest = {"schema_version": 1, "split": args.split, "capture_id": str(uuid.uuid4()),
                "route_completed": False, "landed_disarmed": False, "passed": False,
                "topics": list(TOPICS),
                **{key: plan[key] for key in ("world_sha256", "assets_sha256", "model_sha256", "camera_sha256")},
                "route_sha256": sha(output / "route.json")}
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    old_signal = signal.signal(signal.SIGTERM, terminate)
    try:
        run_flight(output, plan, manifest)
        manifest.update(bag_evidence(output, TOPICS, manifest["windows"]))
        manifest["passed"] = bool(manifest["route_completed"] and manifest["landed_disarmed"]
                                  and not manifest.get("recorder_error") and manifest["b_visibility_checks"]>0
                                  and manifest["b_visible_after_window"]==0)
    except (KeyboardInterrupt, Exception) as error:
        manifest["error"] = f"{type(error).__name__}: {error}"
        print(manifest["error"], file=sys.stderr)
    finally:
        signal.signal(signal.SIGTERM, old_signal)
        save_json(output / "manifest.json", manifest)
    return 0 if manifest["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
