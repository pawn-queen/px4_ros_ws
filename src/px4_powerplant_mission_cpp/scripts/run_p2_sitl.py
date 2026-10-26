#!/usr/bin/env python3
"""Isolated PX4/Gazebo P2 acceptance run; never connects to an existing flight.

Source ROS, the workspace and scripts/p0_ros_env.sh before running. --prepare-only
writes the resolved configuration/commands without starting simulation or ROS.
Detection replay projects a known *test object* into genuine Gazebo RGB frames;
it is explicitly not a test of YOLO inference or unknown-target discovery.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
from collections import OrderedDict, deque
from datetime import datetime, timezone
from functools import partial
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET


def json_dumps(value, **kwargs):
    """Preserve numeric/boolean values from NumPy in every evidence file.

    Conversion is representational only: a failed numpy.bool_ remains false;
    unsupported object types still raise instead of being stringified.
    """
    def encode_numpy(obj):
        import numpy as np
        if isinstance(obj, np.generic): return obj.item()
        if isinstance(obj, np.ndarray): return obj.tolist()
        raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")
    return json.dumps(value, default=encode_numpy, **kwargs)


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
EXPECTED_PHASES = ["START", "SAFE_TAKEOFF", "HOVER_SCAN", "EXPLORE",
                   "TARGETING_CYCLE", "RETURN_HOME", "LANDING", "COMPLETE"]


def takeoff_startup_evidence(parameters, observed_px4_parameters):
    """Compare configured bootstrap timing with actual read-only PX4 values.

    This verifies configuration only; it does not infer actual ramp state or
    replace the measured tracking/stop checks during flight.
    """
    height = parameters.get("motion_takeoff_bootstrap_height_m")
    duration = parameters.get("motion_takeoff_bootstrap_duration_s")
    tolerance = parameters.get("motion_tracking_position_tolerance_m")
    reaction = parameters.get("motion_reaction_time_s")
    def observed_value(name):
        entry = observed_px4_parameters.get(name) if isinstance(observed_px4_parameters, dict) else None
        return entry.get("value") if isinstance(entry, dict) else None
    spool = observed_value("COM_SPOOLUP_TIME")
    ramp = observed_value("MPC_TKO_RAMP_T")

    def finite_number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    height_valid = (finite_number(height) and finite_number(tolerance)
                    and 0 < height <= tolerance)
    duration_valid = finite_number(duration) and duration > 0
    reaction_valid = finite_number(reaction) and reaction > 0
    measured_valid = (finite_number(spool) and finite_number(ramp)
                      and spool >= 0 and ramp >= 0)
    required_duration = spool+ramp+reaction if measured_valid and reaction_valid else None
    duration_covers_startup = (duration_valid and required_duration is not None
                              and duration >= required_duration)
    reasons = []
    if not height_valid:
        reasons.append("bootstrap height must be positive and no greater than position tracking tolerance")
    if not duration_valid:
        reasons.append("bootstrap duration must be positive and finite")
    if not reaction_valid:
        reasons.append("motion reaction budget is unavailable or invalid")
    if not measured_valid:
        reasons.append("actual PX4 spool/ramp parameters are unavailable or invalid")
    elif not duration_covers_startup:
        reasons.append("bootstrap duration does not cover actual PX4 spool+ramp+reaction budget")
    return {"passed": bool(height_valid and duration_covers_startup), "reasons": reasons,
            "scope": "configuration compared with read-only PX4 parameters; actual response requires flight evidence",
            "bootstrap_reference": "current_continuous_authorized_hold_after_arming",
            "bootstrap_height_m": height, "bootstrap_minimum_duration_s": duration,
            "position_tracking_tolerance_m": tolerance, "reaction_budget_s": reaction,
            "actual_px4_spoolup_time_s": spool, "actual_px4_takeoff_ramp_time_s": ramp,
            "required_minimum_duration_s": required_duration,
            "height_within_tracking_tolerance": bool(height_valid),
            "duration_covers_px4_startup_and_reaction": bool(duration_covers_startup),
            "px4_parameters_modified_for_bootstrap": False,
            "actual_takeoff_ramp_state_observed": False}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--px4-root", type=Path, default=Path("/home/pawn/drone-simulation"))
    p.add_argument("--scenario", choices=("room", "two_rooms"), default="room")
    p.add_argument("--detection-mode", choices=("replay", "yolo", "none"), default="replay")
    p.add_argument("--sensor-profile", choices=("native", "cpu"), default="native",
                   help="cpu copies the camera/vehicle models into this run and reduces sensor load")
    p.add_argument("--cpu-camera-rate-hz", type=float, default=10.0,
                   help="positive RGB/depth rate for the isolated cpu profile; native keeps its original SDF")
    p.add_argument("--cpu-lidar-rate-hz", type=float, default=15.0,
                   help="positive GPU lidar rate for the isolated cpu profile")
    p.add_argument("--image-chain-only", action="store_true",
                   help="yolo only: validate genuine inference/source-image linkage without requesting START")
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--timeout", type=float, default=300)
    p.add_argument("--startup-timeout", type=float, default=90)
    p.add_argument("--ros-domain", type=int, default=25)
    p.add_argument("--agent-port", type=int, default=8889)
    p.add_argument("--inject-obstacle", action="store_true")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--prepare-only", action="store_true")
    args = p.parse_args()
    if not 0 <= args.ros_domain <= 232 or not 1024 <= args.agent_port <= 65535:
        p.error("choose a ROS domain in [0,232] and an unprivileged UDP port")
    if args.agent_port in (14550, 18570):
        p.error("the Agent port must not overlap the isolated SITL MAVLink/GCS ports")
    if args.timeout <= 0 or args.startup_timeout <= 0:
        p.error("timeouts must be positive")
    if not math.isfinite(args.cpu_camera_rate_hz) or args.cpu_camera_rate_hz<=0:
        p.error("--cpu-camera-rate-hz must be a finite positive number")
    if args.sensor_profile!="cpu" and args.cpu_camera_rate_hz!=10.0:
        p.error("a nondefault --cpu-camera-rate-hz requires --sensor-profile cpu")
    if not math.isfinite(args.cpu_lidar_rate_hz) or args.cpu_lidar_rate_hz<=0:
        p.error("--cpu-lidar-rate-hz must be a finite positive number")
    if args.sensor_profile!="cpu" and args.cpu_lidar_rate_hz!=15.0:
        p.error("a nondefault --cpu-lidar-rate-hz requires --sensor-profile cpu")
    if args.image_chain_only and args.detection_mode != "yolo":
        p.error("--image-chain-only requires --detection-mode yolo")
    if args.image_chain_only and args.inject_obstacle:
        p.error("--image-chain-only does not execute a flight or inject obstacles")
    if args.output_dir is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output_dir = WORKSPACE / "log" / f"p2_sitl_{args.scenario}_{stamp}"
    args.output_dir = args.output_dir.expanduser().resolve()
    args.px4_root = args.px4_root.expanduser().resolve()
    return args


def sensor_profile(args, runtime):
    """Resolve explicit sensor load without changing the external PX4 tree."""
    original = args.px4_root / "Tools/simulation/gz/models/OakD-Lite"
    original_sdf = original / "model.sdf"
    original_bytes = original_sdf.read_bytes()
    original_vehicle = args.px4_root / "Tools/simulation/gz/models/x500_depth"
    original_vehicle_sdf = original_vehicle / "model.sdf"
    original_vehicle_bytes = original_vehicle_sdf.read_bytes()
    effective_vehicle_sdf = original_vehicle_sdf
    resources = None
    effective_sdf = original_sdf
    if args.sensor_profile == "cpu":
        resources = runtime / "models"
        copied = resources / "OakD-Lite"
        shutil.copytree(original, copied)
        effective_sdf = copied / "model.sdf"
        tree = ET.parse(effective_sdf)
        settings = {"IMX214": (640,360,args.cpu_camera_rate_hz), "StereoOV7251": (320,240,args.cpu_camera_rate_hz)}
        found = set()
        for sensor in tree.getroot().findall("./model/link/sensor"):
            if sensor.get("name") not in settings: continue
            name = sensor.get("name")
            width,height,rate = settings[name]
            required = [sensor.find("camera/image/width"),sensor.find("camera/image/height"),sensor.find("update_rate")]
            if any(element is None for element in required):
                raise RuntimeError(f"cannot configure isolated CPU camera profile: {name}")
            for element,value in zip(required,(width,height,rate)): element.text=str(value)
            found.add(name)
        if found != set(settings): raise RuntimeError("unexpected OakD camera model; refusing partial CPU override")
        tree.write(effective_sdf, encoding="utf-8", xml_declaration=True)
        copied_vehicle = resources / "x500_depth"
        shutil.copytree(original_vehicle,copied_vehicle)
        effective_vehicle_sdf = copied_vehicle / "model.sdf"
        vehicle_tree = ET.parse(effective_vehicle_sdf)
        lidar = vehicle_tree.getroot().find("./model/link/sensor[@name='mapping_lidar']/update_rate")
        if lidar is None: raise RuntimeError("expected mapping_lidar missing; refusing partial CPU override")
        lidar.text=str(args.cpu_lidar_rate_hz)
        vehicle_tree.write(effective_vehicle_sdf,encoding="utf-8",xml_declaration=True)
    tree = ET.parse(effective_sdf)
    cameras = {}
    for sensor in tree.getroot().findall("./model/link/sensor"):
        if sensor.get("name") not in ("IMX214","StereoOV7251"): continue
        name = "rgb" if sensor.get("name")=="IMX214" else "depth"
        cameras[name] = {"sensor_name":sensor.get("name"),"width":int(sensor.findtext("camera/image/width")),
            "height":int(sensor.findtext("camera/image/height")),"configured_update_rate_hz":float(sensor.findtext("update_rate")),
            "horizontal_fov_rad":float(sensor.findtext("camera/horizontal_fov")),"sensor_pose":sensor.findtext("pose")}
    lidar_tree = ET.parse(effective_vehicle_sdf)
    # ElementTree's limited XPath does not accept nested attribute predicates.
    lidar_link = next((link for link in lidar_tree.getroot().findall("./model/link")
                       if link.find("sensor[@name='mapping_lidar']") is not None),None)
    if lidar_link is None: raise RuntimeError("mapping_lidar link is unavailable for sensor manifest")
    lidar = lidar_link.find("sensor[@name='mapping_lidar']")
    cameras["lidar"]={"sensor_name":lidar.get("name"),"sensor_type":lidar.get("type"),
        "configured_update_rate_hz":float(lidar.findtext("update_rate")),"sensor_pose":lidar.findtext("pose"),
        "link_pose":lidar_link.findtext("pose"),"link_pose_relative_to":lidar_link.find("pose").get("relative_to"),
        "horizontal":{key:lidar.findtext("ray/scan/horizontal/"+key) for key in ("samples","resolution","min_angle","max_angle")},
        "vertical":{key:lidar.findtext("ray/scan/vertical/"+key) for key in ("samples","resolution","min_angle","max_angle")},
        "range":{key:lidar.findtext("ray/range/"+key) for key in ("min","max","resolution")}}
    metadata = {"name":args.sensor_profile,"parameter_source":str(effective_sdf),
                "original_model_directory":str(original),"copied_model_directory":str(effective_sdf.parent) if resources else None,
                "original_model_sdf_sha256":hashlib.sha256(original_bytes).hexdigest(),
                "effective_model_sdf_sha256":hashlib.sha256(effective_sdf.read_bytes()).hexdigest(),
                "configured_sensors":cameras,
                "cpu_camera_rate_requested_hz":args.cpu_camera_rate_hz if args.sensor_profile=="cpu" else None,
                "cpu_lidar_rate_requested_hz":args.cpu_lidar_rate_hz if args.sensor_profile=="cpu" else None,
                "vehicle_model":{"original_directory":str(original_vehicle),"effective_directory":str(effective_vehicle_sdf.parent),
                    "original_model_sdf_sha256":hashlib.sha256(original_vehicle_bytes).hexdigest(),
                    "effective_model_sdf_sha256":hashlib.sha256(effective_vehicle_sdf.read_bytes()).hexdigest()},
                "performance_scope":"native_sensor_load" if args.sensor_profile=="native" else "cpu_reduced_sensor_load",
                "external_checkout_modified":False}
    return resources,metadata


def verified_body_envelope(models, parameters, audit_path):
    audit=json.loads(audit_path.read_text())
    expected={"x500_depth","x500","x500_base","OakD-Lite"}
    hashes=audit.get("source_sdf_sha256",{})
    if {Path(path).parent.name for path in hashes} != expected:
        raise RuntimeError("incomplete SDF body audit; rerun derive_sitl_body_envelope.py")
    for path,digest in hashes.items():
        actual=models/Path(path).parent.name/"model.sdf"
        if hashlib.sha256(actual.read_bytes()).hexdigest()!=digest:
            raise RuntimeError(f"SDF model changed since body audit: {actual}")
    values=[float(parameters[key]) for key in (
        "motion_body_radius_m","motion_body_z_min_m","motion_body_z_max_m","initial_scan_max_tilt_rad")]
    radius,lower,upper,tilt=values
    if (not all(math.isfinite(v) for v in values) or tilt<0
            or tilt>audit["total_tilt_bound_rad"]
            or radius<audit["maximum_horizontal_radius_m"]
            or lower>audit["minimum_z_m"] or upper<audit["maximum_z_m"]):
        raise RuntimeError("SITL motion body/tilt parameters do not contain the audited SDF envelope")
    return {"report":str(audit_path),"model_hashes_match":True,"profile_contains_audited_geometry":True,
            "configured_radius_m":radius,"configured_z_min_m":lower,"configured_z_max_m":upper,
            "configured_tilt_rad":tilt,"minimum_geometric_z_m":audit["minimum_z_m"],
            "scope":audit["scope"]}


def prepare(args):
    import yaml
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    runtime = output / "runtime" / f"run-{time.time_ns()}"
    runtime.mkdir(parents=True, exist_ok=True)
    local_models,sensor_metadata = sensor_profile(args,runtime)
    build = args.px4_root / "build/px4_sitl_default"
    binary = build / "bin/px4"
    world = PACKAGE / "test/worlds" / f"p2_{args.scenario}.sdf"
    ET.parse(world)
    config = yaml.safe_load((WORKSPACE / "src/px4_powerplant_mission/config/powerplant.yaml").read_text())
    mission = config["powerplant_offboard_mission"]["ros__parameters"]
    profile = yaml.safe_load((WORKSPACE/"src/px4_powerplant_mission/config/p2_simulation.yaml").read_text())["powerplant_offboard_mission"]["ros__parameters"]
    body_audit=verified_body_envelope(args.px4_root/"Tools/simulation/gz/models",{**mission,**profile},
                                      PACKAGE/"research_sources/sitl_body_envelope.json")
    mission.update({
        "auto_start": False, "takeoff_region_confirmed": True,
        "takeoff_region_radius_m": 1.5, "takeoff_region_height_m": 4.2,
        "takeoff_height_m": 3.0, "return_home_height_m": 3.0,
        "inspection_altitude_m": 3.0, "inspection_photo_dir": str(output / "photos"),
        "explore_min_x_m": -5.5, "explore_max_x_m": 9.5 if args.scenario == "two_rooms" else 5.5,
        "explore_min_y_m": -5.5, "explore_max_y_m": 5.5,
        "explore_min_z_m": 1.0, "explore_max_z_m": 6.0,
        "explore_max_duration_s": args.timeout, "initial_scan_timeout_s": 100.0,
    })
    mapper = config["powerplant_voxel_mapper"]["ros__parameters"]
    # This isolated lockstep simulator and PX4 share the Gazebo boot clock.
    # Hardware retains system↔boot synchronization in the shared defaults.
    config["powerplant_localization"]["ros__parameters"]["px4_timestamp_domain"] = "ros"
    config["powerplant_external_vision_bridge"]["ros__parameters"]["px4_timestamp_domain"] = "ros"
    mapper.update({"grid_min_x_m": -6.0, "grid_max_x_m": 10.0,
                   "grid_min_y_m": -6.0, "grid_max_y_m": 6.0,
                   "grid_z_min_m": 1.0, "grid_z_max_m": 6.0})
    config_path = runtime / "powerplant_p2_sitl.yaml"
    config_path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    rootfs = runtime / "px4_rootfs"
    rootfs.mkdir(exist_ok=True)
    etc = rootfs / "etc"
    if not etc.exists() and (build / "etc").is_dir():
        etc.symlink_to(build / "etc", target_is_directory=True)
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": str(args.ros_domain), "ROS2CLI_ENABLE_DAEMON": "0",
        "GZ_PARTITION": f"powerplant-p2-{os.getpid()}", "GZ_IP": "127.0.0.1",
        "PX4_SIM_MODEL": "gz_x500_depth", "PX4_SYS_AUTOSTART": "4002",
        "PX4_GZ_STANDALONE": "1", "PX4_GZ_WORLD": "powerplant",
        "PX4_GZ_MODEL_POSE": "0,0,0.1,0,0,0", "PX4_GZ_NO_FOLLOW": "1",
        "PX4_GZ_MODELS": str(local_models if local_models else args.px4_root / "Tools/simulation/gz/models"),
        "HEADLESS": "1", "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1",
        "PX4_PARAM_UXRCE_DDS_PRT": str(args.agent_port),
        "PX4_PARAM_UXRCE_DDS_SYNCT": "0",
        "PX4_UXRCE_DDS_PORT": str(args.agent_port),
        "PX4_PARAM_UXRCE_DDS_DOM_ID": str(args.ros_domain),
        "PX4_PARAM_UXRCE_DDS_AG_IP": "2130706433",
        "PX4_PARAM_EKF2_EV_CTRL": "11", "PX4_PARAM_EKF2_EV_QMIN": "0",
        "PX4_PARAM_EKF2_HGT_REF": "3", "PX4_PARAM_EKF2_GPS_CTRL": "0",
        "PX4_PARAM_SYS_HAS_GPS": "0", "PX4_PARAM_SYS_HAS_MAG": "0",
        "PX4_PARAM_EKF2_MAG_TYPE": "5", "PX4_PARAM_SIM_GZ_EN_ODOM": "1",
        "PX4_PARAM_COM_RC_IN_MODE": "4", "PX4_PARAM_COM_ARM_WO_GPS": "2",
        "PX4_PARAM_COM_ARM_ODID": "0", "PX4_PARAM_COM_HOME_IN_AIR": "1",
        # This failsafe change is confined to the fresh SITL rootfs/process.
        "PX4_PARAM_COM_OBL_RC_ACT": "4", "PX4_PARAM_COM_OF_LOSS_T": ".5",
    })
    resources = [str(local_models) if local_models else "", str(args.px4_root / "Tools/simulation/gz/models"), str(world.parent), env.get("GZ_SIM_RESOURCE_PATH", "")]
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(x for x in resources if x)
    plugins = str(build / "src/modules/simulation/gz_plugins")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        x for x in (plugins, env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")) if x)
    gz = ["gz", "sim", "-r", "-s", "--headless-rendering", str(world)]
    commands = [
        ("agent", ["MicroXRCEAgent", "udp4", "-p", str(args.agent_port)], runtime),
        ("gazebo", gz, runtime),
        ("px4", [str(binary), "-d", "-i", "0", "-w", str(rootfs), str(build / "etc")], rootfs),
        ("gcs", [sys.executable, str(PACKAGE / "scripts/p2_sitl_gcs.py"),
                 "--output-dir", str(output), "--startup-timeout", str(args.startup_timeout)], runtime),
        ("sensors", ["ros2", "launch", "px4_powerplant_mission", "gz_sensor_bridge.launch.py"], runtime),
        ("mission", ["ros2", "launch", "px4_powerplant_mission", "powerplant_system.launch.py",
                     f"config_file:={config_path}", "localization_profile:=simulation",
                     "use_control:=true", f"use_yolo:={'true' if args.detection_mode == 'yolo' else 'false'}"], runtime),
    ]
    if args.gui:
        commands.insert(2, ("gui", ["gz", "sim", "-g"], runtime))
    missing = [x for x in ("MicroXRCEAgent", "gz", "ros2") if not shutil.which(x)]
    if not binary.is_file(): missing.append(str(binary))
    for name in ("rclpy", "yaml", "numpy", "px4_msgs", "gz.transport13", "gz.msgs10.pose_v_pb2", "pymavlink.dialects.v20.common"):
        try:
            available = importlib.util.find_spec(name) is not None
        except ModuleNotFoundError:
            available = False
        if not available: missing.append(f"Python module {name}")
    manifest = {
        "scenario": args.scenario, "detection_mode": args.detection_mode,
        "acceptance_scope":"yolo_image_chain_only" if args.image_chain_only else "full_mission",
        "sensor_profile":sensor_metadata,
        "body_envelope_audit":body_audit,
        "synthetic_detection_replay": args.detection_mode == "replay",
        "world": str(world), "config": str(config_path), "rootfs": str(rootfs),
        "ros_domain": args.ros_domain, "agent_port": args.agent_port,
        "gcs_link": {"listen_endpoint": ["127.0.0.1", 14550], "expected_px4_endpoint": ["127.0.0.1", 18570],
                     "verified_peer_required_before_transmit": True, "heartbeat_period_wall_s": 1.0,
                     "expected_px4_NAV_DLL_ACT": 2,
                     "px4_datalink_checks_disabled": False, "sends_arming_mode_or_trajectory_commands": False,
                     "health_file": str(output / "gcs_health.json"),
                     "heartbeat_evidence_file": str(output / "gcs_heartbeats.jsonl")},
        "gz_partition": env["GZ_PARTITION"], "missing_dependencies": missing,
        "commands": [{"name": name, "argv": argv, "cwd": str(cwd)} for name, argv, cwd in commands],
        "test_only_confirmation": {"radius_m": 1.5, "height_m": 4.2, "relative_takeoff_m": 3.0},
        "takeoff_startup_configuration": takeoff_startup_evidence({**mission, **profile}, {}),
    }
    (output / "manifest.json").write_text(json_dumps(manifest, ensure_ascii=False, indent=2))
    return env, commands, manifest


class Processes:
    def __init__(self, output, env):
        self.output, self.env, self.children = output, env, []

    def start(self, name, command, cwd):
        log = (self.output / f"{name}.log").open("w")
        child = subprocess.Popen(command, cwd=cwd, env=self.env, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((name, child, log))

    def failed(self):
        return [f"{name} exited {child.returncode}" for name, child, _ in self.children
                if child.poll() is not None]

    def close(self):
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for _, child, _ in reversed(self.children):
                if child.poll() is None:
                    try: os.killpg(child.pid, sig)
                    except ProcessLookupError: pass
            deadline = time.monotonic() + (3 if sig != signal.SIGKILL else 1)
            while time.monotonic() < deadline and any(p.poll() is None for _, p, _ in self.children):
                time.sleep(.1)
        for _, child, log in self.children:
            child.wait(timeout=2)
            log.close()


def monitor(args, processes):
    # Set the isolated partition before importing Transport: binding defaults
    # may construct NodeOptions while the module is being initialized.
    os.environ["GZ_PARTITION"] = processes.env["GZ_PARTITION"]
    os.environ["GZ_IP"] = processes.env["GZ_IP"]
    import numpy as np
    import yaml
    import rclpy
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
    from rosgraph_msgs.msg import Clock
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def rotation(q):
        w, x, y, z = q / np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

    class Monitor(Node):
        def __init__(self):
            super().__init__("powerplant_p2_sitl_acceptance")
            self.latest, self.events, self.phases = {}, [], []
            self.poses, self.images = deque(maxlen=120), deque(maxlen=3)
            self.rgb_info = self.depth_info = None
            self.sensor_observations = {}
            # Retain hashes and small metadata, never a history of full RGB.
            self.yolo_raw = OrderedDict()
            self.yolo_sources = OrderedDict()
            self.yolo_json = OrderedDict()
            self.yolo_seen = OrderedDict()
            self.yolo_result_count = self.yolo_target_count = 0
            self.yolo_pair_count = self.yolo_success_count = 0
            self.yolo_metadata_errors = self.yolo_pixel_errors = self.yolo_invalid_messages = 0
            self.yolo_expired_sources = self.yolo_expired_results = 0
            self.yolo_file = (args.output_dir / "yolo_image_chain.jsonl").open("w")
            self.truth_poses, self.depth_images = deque(maxlen=360), deque(maxlen=2)
            self.truth_pending = deque(maxlen=360)
            self.truth_lock = threading.Lock()
            self.truth_callback_messages = self.truth_named_messages = 0
            self.truth_entity_id = None
            self.truth_file = (args.output_dir / "truth.jsonl").open("w")
            self.reference_errors, self.body_reference_errors = [], []
            self.reference_offsets = []
            self.depth_truth_residuals, self.depth_navigation_residuals = [], []
            self.calibration_frames = 0
            self.last_calibration_time = 0.0
            self.calibration_file = (args.output_dir / "calibration.jsonl").open("w")
            profile_path = WORKSPACE / "src/px4_powerplant_mission/config/p2_simulation.yaml"
            profile = yaml.safe_load(profile_path.read_text())["powerplant_offboard_mission"]["ros__parameters"]
            shared = yaml.safe_load((WORKSPACE / "src/px4_powerplant_mission/config/powerplant.yaml").read_text())["powerplant_offboard_mission"]["ros__parameters"]
            self.motion_parameters = {**shared,**profile}
            self.position_tolerance = float(self.motion_parameters["motion_tracking_position_tolerance_m"])
            self.velocity_tolerance = float(self.motion_parameters["motion_tracking_velocity_tolerance_mps"])
            self.sigma_multiplier = float(self.motion_parameters["motion_sigma_multiplier"])
            self.reaction_budget = float(self.motion_parameters["motion_reaction_time_s"])
            self.depth_mount = np.asarray(profile["camera_translation_flu"],dtype=float)
            self.rgb_mount = np.asarray(profile["rgb_camera_translation_flu"],dtype=float)
            def mount_rotation(rpy):
                roll,pitch,yaw = rpy
                cr,sr,cp,sp,cy,sy = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
                return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                                 [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
            self.depth_mount_rotation = mount_rotation(profile["camera_rotation_rpy_rad"])
            self.rgb_mount_rotation = mount_rotation(profile["rgb_camera_rotation_rpy_rad"])
            self.scene_boxes = []
            self.scene_cylinders = []
            world = ET.parse(PACKAGE / "test/worlds" / f"p2_{args.scenario}.sdf").getroot()
            for model in world.findall("./world/model"):
                name = model.get("name", "")
                model_pose = np.asarray([float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()])
                if np.any(np.abs(model_pose[3:]) > 1e-9):
                    raise RuntimeError("calibration scene requires axis-aligned static primitives")
                for link in model.findall("link"):
                    link_pose = np.asarray([float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()])
                    for visual in link.findall("visual"):
                        visual_pose = np.asarray([float(v) for v in visual.findtext("pose", "0 0 0 0 0 0").split()])
                        if np.any(np.abs(link_pose[3:]) > 1e-9) or np.any(np.abs(visual_pose[3:]) > 1e-9):
                            raise RuntimeError("calibration visual must be axis aligned")
                        center = model_pose[:3] + link_pose[:3] + visual_pose[:3]
                        box = visual.find("geometry/box/size")
                        cylinder = visual.find("geometry/cylinder")
                        if box is not None:
                            half = np.asarray([float(v) for v in box.text.split()]) / 2
                            is_wall = name.startswith(("floor", "ceiling", "wall", "divider", "partition"))
                            self.scene_boxes.append((center-half, center+half, is_wall, name))
                        elif cylinder is not None:
                            self.scene_cylinders.append((center, float(cylinder.findtext("radius")),
                                                        float(cylinder.findtext("length"))/2))
            self.start_future = None
            self.started = False
            self.last_start_try = 0.0
            self.start_ready_epoch = self.start_ready_clock_ns = self.start_ready_wall = None
            self.explore_since = None
            self.replay_count = self.photo_count = 0
            self.last_replay_time = 0.0
            self.setpoints, self.bad_pva = [], 0
            self.brake_seen = self.stop_seen = False
            self.obstacle_time = None
            self.obstacle_process = None
            self.obstacle_injection = None
            self.braking_first = self.braking_stop = None
            self.motion_status_rows = deque(maxlen=128)
            self.braking_status_rows = []
            self.max_regions = self.max_portals = 0
            self.latest_velocity = np.zeros(3)
            self.measured_vxy = self.measured_vz = 0.0
            self.tracking_errors, self.velocity_errors = [], []
            self.navigation_rows, self.clock_rows = [], []
            self.clock_ns = self.clock_wall = None
            self.tracking_file = (args.output_dir / "tracking.jsonl").open("w")
            self.latest_position_sigma = self.latest_velocity_sigma = None
            self.pose_file = (args.output_dir / "navigation.jsonl").open("w")
            self.event_file = (args.output_dir / "events.jsonl").open("w")
            self.state_file = (args.output_dir / "states.jsonl").open("w")
            self.pva_file = (args.output_dir / "setpoints.jsonl").open("w")
            self.input_health = {}
            self.health_file = (args.output_dir / "input_health.jsonl").open("w")
            reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, "/powerplant/control/mission_status", self.status, reliable)
            self.create_subscription(Clock, "/clock", self.clock_sample, qos_profile_sensor_data)
            self.create_subscription(String, "/powerplant/control/inspection_events", self.event, reliable)
            for name, topic in (("localization", "/powerplant/localization/status"),
                                ("uwb_quality", "/uwb/quality"),
                                ("uwb_status", "/uwb/status"),
                                ("fusion", "/powerplant/map/fusion_status")):
                self.create_subscription(String, topic, partial(self.health_sample,name), reliable)
            self.create_subscription(Odometry, "/powerplant/localization/odom", self.pose, reliable)
            self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.pva, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/camera/camera_info", lambda m: setattr(self, "rgb_info", m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/depth_camera/camera_info", lambda m: setattr(self, "depth_info", m), qos_profile_sensor_data)
            self.create_subscription(Image, "/depth_camera", lambda m: self.sensor_image("depth",m), qos_profile_sensor_data)
            self.create_subscription(Image, "/camera", lambda m: self.sensor_image("rgb",m), qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/mapping_lidar", self.lidar_sample, qos_profile_sensor_data)
            if args.detection_mode == "yolo":
                self.create_subscription(Image, "/powerplant/perception/yolo_source_image", self.yolo_source, reliable)
                self.create_subscription(String, "/powerplant/perception/yolo_detections", self.yolo_detection, reliable)
            # Jazzy's Pose_V -> TF bridge can discard both pose names and the
            # outer sampling stamp. Subscribe to the original protobuf instead;
            # never infer model identity from its position in an anonymous TF.
            self.gz_truth_node = GzNode()
            if not self.gz_truth_node.subscribe(Pose_V, "/world/powerplant/dynamic_pose/info", self.truth):
                raise RuntimeError("cannot subscribe to isolated Gazebo model truth")
            if args.detection_mode == "replay":
                self.source_pub = self.create_publisher(Image, "/powerplant/perception/yolo_source_image", reliable)
                self.detection_pub = self.create_publisher(String, "/powerplant/perception/yolo_detections", reliable)
            self.start_client = self.create_client(Trigger, "/start_powerplant_mission")
            self.land_client = self.create_client(Trigger, "/land_powerplant_mission")

        def sensor_image(self, kind, msg):
            now,sample=time.monotonic(),stamp(msg)
            entry=self.sensor_observations.get(kind)
            if entry is None:
                entry={"width":msg.width,"height":msg.height,"encoding":msg.encoding,
                    "received_frames":0,"unique_exposures":0,"first_sample_ns":sample,
                    "first_wall":now,"last_sample_ns":None,"nonmonotonic_samples":0}
                self.sensor_observations[kind]=entry
            entry["received_frames"]+=1
            if entry["last_sample_ns"] is None or sample>entry["last_sample_ns"]:
                entry["unique_exposures"]+=1
            elif sample<entry["last_sample_ns"]: entry["nonmonotonic_samples"]+=1
            entry["last_sample_ns"],entry["last_wall"]=sample,now
            if kind=="depth": self.depth_images.append(msg)
            elif args.detection_mode=="replay": self.images.append(msg)
            if kind=="rgb" and args.detection_mode=="yolo":
                self.yolo_raw[sample]=self.image_fingerprint(msg)
                self.trim_yolo_cache()
                self.pair_yolo(sample)

        def health_sample(self, name, msg):
            try: payload=json.loads(msg.data)
            except (ValueError,TypeError): return
            keys=("status_stamp_ns","sample_stamp_ns","navigation_sample_ns",
                  "uwb_input_healthy","uwb_input_reason","has_independent_position",
                  "uwb_rejection_reason","px4_reset_counter","localization_reason",
                  "localization_healthy","accepted","reason","kind")
            row={key:payload[key] for key in keys if key in payload}
            previous=self.input_health.get(name,{})
            self.input_health[name]=row
            ignored=("status_stamp_ns","sample_stamp_ns","navigation_sample_ns")
            if {k:v for k,v in row.items() if k not in ignored} != {k:v for k,v in previous.items() if k not in ignored}:
                self.health_file.write(json_dumps({"source":name,"wall":time.monotonic(),**row})+"\n")
                self.health_file.flush()

        def image_fingerprint(self, msg):
            return {"sample_ns":stamp(msg),"frame_id":msg.header.frame_id,
                    "width":msg.width,"height":msg.height,"encoding":msg.encoding,
                    "step":msg.step,"is_bigendian":msg.is_bigendian,"bytes":len(msg.data),
                    "sha256":hashlib.sha256(memoryview(msg.data)).hexdigest(),"receipt_wall":time.monotonic()}

        def trim_yolo_cache(self):
            now=time.monotonic()
            for cache,limit,kind in ((self.yolo_raw,512,"raw"),(self.yolo_sources,32,"source"),(self.yolo_json,32,"result")):
                while cache and (len(cache)>limit or now-next(iter(cache.values()))["receipt_wall"]>15):
                    sample,_=cache.popitem(last=False)
                    if sample in self.yolo_seen: continue
                    if kind=="source": self.yolo_expired_sources+=1
                    elif kind=="result": self.yolo_expired_results+=1
            while len(self.yolo_seen)>512: self.yolo_seen.popitem(last=False)

        def yolo_source(self, msg):
            if stamp(msg)<=0 or msg.width<=0 or msg.height<=0 or len(msg.data)==0:
                self.yolo_invalid_messages+=1
                return
            sample=stamp(msg)
            self.yolo_sources[sample]=self.image_fingerprint(msg)
            self.trim_yolo_cache()
            self.pair_yolo(sample)

        def yolo_detection(self, msg):
            try:
                if len(msg.data)>1_048_576: raise ValueError("oversize YOLO JSON")
                payload=json.loads(msg.data)
                sec,nanosec=payload["stamp"]["sec"],payload["stamp"]["nanosec"]
                width,height=payload["image_width"],payload["image_height"]
                if (any(type(value) is not int for value in (sec,nanosec,width,height))
                        or sec<0 or not 0<=nanosec<1_000_000_000): raise ValueError("noncanonical YOLO exposure metadata")
                sample=sec*1_000_000_000+nanosec
                frame=payload["frame_id"]
                detections=payload["detections"]
                if (sample<=0 or width<=0 or height<=0 or not isinstance(frame,str)
                        or not isinstance(detections,list)): raise ValueError("invalid YOLO exposure metadata")
            except (ValueError,TypeError,KeyError):
                self.yolo_invalid_messages+=1
                return
            if sample not in self.yolo_json and sample not in self.yolo_seen:
                self.yolo_result_count+=1
                self.yolo_target_count+=len(detections)
            self.yolo_json[sample]={"sample_ns":sample,"frame_id":frame,"width":width,"height":height,
                                  "detections":len(detections),"receipt_wall":time.monotonic()}
            self.trim_yolo_cache()
            self.pair_yolo(sample)

        def pair_yolo(self, sample):
            if sample in self.yolo_seen: return
            raw,source,result=self.yolo_raw.get(sample),self.yolo_sources.get(sample),self.yolo_json.get(sample)
            if raw is None or source is None or result is None: return
            metadata=all(raw[key]==source[key]==result[key] for key in ("sample_ns","frame_id","width","height"))
            pixels=all(raw[key]==source[key] for key in ("encoding","step","is_bigendian","bytes","sha256"))
            self.yolo_pair_count+=1
            self.yolo_success_count+=bool(metadata and pixels)
            self.yolo_metadata_errors+=not metadata
            self.yolo_pixel_errors+=not pixels
            self.yolo_seen[sample]=True
            self.yolo_file.write(json_dumps({"sample_ns":sample,"metadata_equal":metadata,
                "source_pixels_equal_original":pixels,"original_rgb":raw,"yolo_source":source,
                "inference_result":result,"detection_mode":"real_yolo","synthetic_detection_replay":False})+"\n")
            self.yolo_file.flush()

        def yolo_chain_evidence(self):
            checks={"real_yolo_mode":args.detection_mode=="yolo",
                    "at_least_three_real_inference_results":self.yolo_result_count>=3,
                    "at_least_three_strict_exposure_pairs":self.yolo_success_count>=3,
                    "source_json_original_stamp_frame_dimensions_equal":self.yolo_pair_count>=3 and self.yolo_metadata_errors==0,
                    "source_pixels_unannotated_equal_original":self.yolo_pair_count>=3 and self.yolo_pixel_errors==0,
                    "no_malformed_image_or_json":self.yolo_invalid_messages==0,
                    "no_expired_source_or_result_evidence":self.yolo_expired_sources==0 and self.yolo_expired_results==0}
            return {"passed":all(checks.values()),"checks":checks,
                    "real_inference_result_count":self.yolo_result_count,"detected_object_count":self.yolo_target_count,
                    "paired_exposures":self.yolo_pair_count,"successful_strict_pairs":self.yolo_success_count,
                    "metadata_mismatches":self.yolo_metadata_errors,"pixel_mismatches":self.yolo_pixel_errors,
                    "malformed_messages":self.yolo_invalid_messages,
                    "expired_unpaired_sources":self.yolo_expired_sources,"expired_unpaired_results":self.yolo_expired_results,
                    "pending_source_exposures":sum(sample not in self.yolo_seen for sample in self.yolo_sources),
                    "pending_result_exposures":sum(sample not in self.yolo_seen for sample in self.yolo_json),
                    "raw_cache_limit":512,"source_result_cache_limit":32,"cache_max_age_wall_s":15,
                    "comparison":"exact_bytes_via_sha256_and_image_layout_metadata",
                    "scope":"inference_exposure_and_original_pixel_transport; detection_accuracy_and_flight_are_separate"}

        def lidar_sample(self, msg):
            now,sample=time.monotonic(),stamp(msg)
            entry=self.sensor_observations.get("lidar")
            if entry is None:
                entry={"range_samples":len(msg.ranges),"angle_min_rad":msg.angle_min,
                    "angle_max_rad":msg.angle_max,"angle_increment_rad":msg.angle_increment,
                    "range_min_m":msg.range_min,"range_max_m":msg.range_max,
                    "received_frames":0,"unique_exposures":0,"first_sample_ns":sample,
                    "first_wall":now,"last_sample_ns":None,"nonmonotonic_samples":0}
                self.sensor_observations["lidar"]=entry
            entry["received_frames"]+=1
            if entry["last_sample_ns"] is None or sample>entry["last_sample_ns"]:
                entry["unique_exposures"]+=1
            elif sample<entry["last_sample_ns"]: entry["nonmonotonic_samples"]+=1
            entry["last_sample_ns"],entry["last_wall"]=sample,now

        def sensor_evidence(self):
            result={}
            for kind,entry in self.sensor_observations.items():
                observed=dict(entry)
                span=(entry["last_sample_ns"]-entry["first_sample_ns"])*1e-9
                wall_span=entry["last_wall"]-entry["first_wall"]
                observed["observed_unique_exposure_rate_sim_hz"]=(entry["unique_exposures"]-1)/span if span>0 else None
                observed["observed_received_rate_wall_hz"]=(entry["received_frames"]-1)/wall_span if wall_span>0 else None
                result[kind]=observed
            return result

        def status(self, msg):
            try: state = json.loads(msg.data)
            except ValueError: return
            now = time.monotonic()
            self.latest = state
            self.update_start_readiness()
            self.state_file.write(json_dumps({"wall": now, **state}) + "\n")
            motion_snapshot = {"receipt_wall":now,"receipt_ros_ns":self.clock_ns,
                **{key:state.get(key) for key in (
                    "motion_mode","motion_reason","motion_stopped","motion_stop_time_s",
                    "motion_stop_certificate_valid","motion_stopping_envelope_enu",
                    "motion_stop_margin_lower_bound_m","odom_sample_stamp_ns",
                    "localization_epoch","map_epoch","map_observation_revision")}}
            self.motion_status_rows.append(motion_snapshot)
            if self.obstacle_injection is not None and self.braking_stop is None:
                self.braking_status_rows.append(motion_snapshot)
            phase = state.get("phase")
            if phase and (not self.phases or phase != self.phases[-1]):
                self.phases.append(phase)
                print(json_dumps({"phase": phase, "motion": state.get("motion_mode"),
                                  "reason": state.get("motion_reason")}), flush=True)
                if phase == "EXPLORE" and self.explore_since is None:
                    self.explore_since = time.monotonic()
            if state.get("motion_mode") == "BRAKING": self.brake_seen = True
            if self.brake_seen and state.get("motion_stopped") and np.linalg.norm(self.latest_velocity) <= .12:
                self.stop_seen = True
            if self.obstacle_injection is not None:
                if state.get("motion_mode") == "BRAKING" and self.braking_first is None:
                    self.braking_first = motion_snapshot
                    self.event_file.write(json_dumps({"event":"test_first_braking_report",**motion_snapshot})+"\n")
                if (self.braking_first is not None and self.braking_stop is None
                        and state.get("motion_mode")=="HOLD" and state.get("motion_stopped")
                        and self.navigation_rows and self.navigation_rows[-1]["covariance_valid"]
                        and now-self.navigation_rows[-1]["wall"]<=self.reaction_budget
                        and np.linalg.norm(self.latest_velocity)<=.12):
                    self.braking_stop = {**motion_snapshot,
                        "navigation_sample_ns":self.navigation_rows[-1]["sample_ns"],
                        "navigation_receipt_wall":self.navigation_rows[-1]["wall"],
                        "measured_velocity_enu_mps":self.latest_velocity.tolist()}
                    self.event_file.write(json_dumps({"event":"test_measured_stopped_report",**self.braking_stop})+"\n")
            self.max_regions = max(self.max_regions, int(state.get("region_count", 0)))
            self.max_portals = max(self.max_portals, int(state.get("portal_count", 0)))

        def event(self, msg):
            try: event = json.loads(msg.data)
            except ValueError: return
            self.events.append(event)
            self.event_file.write(json_dumps({"wall": time.monotonic(), **event}) + "\n")
            self.event_file.flush()
            if (event.get("event") == "inspection_photo" and event.get("saved")
                    and event.get("quality_accepted") and event.get("coverage_committed")):
                self.photo_count += 1

        def execution_active(self):
            return (self.latest.get("phase") in ("SAFE_TAKEOFF","HOVER_SCAN","EXPLORE","TARGETING_CYCLE","RETURN_HOME")
                    and self.latest.get("active")
                    and self.latest.get("arming_state")==VehicleStatus.ARMING_STATE_ARMED
                    and self.latest.get("nav_state")==VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def clock_sample(self, msg):
            self.clock_ns = msg.clock.sec*1_000_000_000+msg.clock.nanosec
            self.clock_wall = time.monotonic()
            if self.execution_active():
                self.clock_rows.append({"wall":self.clock_wall,"sample_ns":self.clock_ns})

        def pose(self, msg):
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            quat = np.array([q.w, q.x, q.y, q.z])
            point = np.array([p.x, p.y, p.z])
            if np.isfinite(quat).all() and np.linalg.norm(quat) > .5 and np.isfinite(point).all():
                self.poses.append((stamp(msg), point, quat / np.linalg.norm(quat)))
                attitude = rotation(quat)
                self.latest_velocity = attitude @ np.array([v.x, v.y, v.z])
                pc=np.asarray(msg.pose.covariance).reshape(6,6)[:3,:3]
                vc_flu=np.asarray(msg.twist.covariance).reshape(6,6)[:3,:3]
                vc=attitude @ vc_flu @ attitude.T
                frame_valid=(msg.header.frame_id==self.motion_parameters["map_frame"]
                             and msg.child_frame_id==self.motion_parameters["base_frame"])
                def valid_covariance(covariance):
                    return (np.isfinite(covariance).all()
                            and np.diag(covariance).min()>0
                            and np.max(np.abs(covariance-covariance.T))<=1e-10
                            and np.linalg.eigvalsh(covariance).min()>=-1e-16)
                covariance_valid=(frame_valid and valid_covariance(pc)
                                  and valid_covariance(vc_flu) and valid_covariance(vc))
                self.latest_position_sigma=float(np.sqrt(max(0,np.linalg.eigvalsh(pc).max()))) if covariance_valid else None
                self.latest_velocity_sigma=float(np.sqrt(max(0,np.linalg.eigvalsh(vc).max()))) if covariance_valid else None
                # Match the executor's outward certificate quantization while
                # preserving all original covariance values as separate evidence.
                def certificate_sigma(value):
                    return np.maximum(value,np.ceil(np.asarray(value)*1000)/1000)
                position_certificate_sigma=float(certificate_sigma(self.latest_position_sigma)) if covariance_valid else None
                velocity_certificate_sigma=float(certificate_sigma(self.latest_velocity_sigma)) if covariance_valid else None
                position_axis_sigma=certificate_sigma(np.sqrt(np.diag(pc))).tolist() if covariance_valid else None
                velocity_axis_sigma=certificate_sigma(np.sqrt(np.diag(vc))).tolist() if covariance_valid else None
                wall=time.monotonic()
                row={"sample_ns":stamp(msg),"wall":wall,"position":point.tolist(),
                     "velocity":self.latest_velocity.tolist(),"position_sigma":self.latest_position_sigma,
                     "velocity_sigma":self.latest_velocity_sigma,"covariance_valid":covariance_valid,
                     "frame_id":msg.header.frame_id,"child_frame_id":msg.child_frame_id,
                     "navigation_frame_valid":frame_valid,
                     "orientation_wxyz":(quat/np.linalg.norm(quat)).tolist(),
                     "position_covariance_enu_m2":pc.tolist(),
                     "linear_velocity_covariance_flu_m2ps2":vc_flu.tolist(),
                     "linear_velocity_covariance_enu_m2ps2":vc.tolist(),
                     "position_sigma_certificate_m":position_certificate_sigma,
                     "velocity_sigma_certificate_mps":velocity_certificate_sigma,
                     "position_axis_sigma_certificate_m":position_axis_sigma,
                     "velocity_axis_sigma_certificate_mps":velocity_axis_sigma,
                     "execution_active":bool(self.execution_active()),"phase":self.latest.get("phase"),
                     "clock_sample_age_s":(self.clock_ns-stamp(msg))*1e-9 if self.clock_ns is not None else None,
                     "clock_receipt_age_wall_s":wall-self.clock_wall if self.clock_wall is not None else None}
                self.navigation_rows.append(row)
                # Preserve invalid numeric evidence as JSON null, accompanied
                # by covariance_valid=false, rather than emitting NaN/Infinity.
                self.pose_file.write(json_dumps(clean_json(row),allow_nan=False)+"\n")
                if row["execution_active"]:
                    self.measured_vxy=max(self.measured_vxy,float(np.linalg.norm(self.latest_velocity[:2])))
                    self.measured_vz=max(self.measured_vz,abs(float(self.latest_velocity[2])))

        def pva(self, msg):
            row = {"wall": time.monotonic(), "timestamp": msg.timestamp,
                   "position": list(msg.position), "velocity": list(msg.velocity),
                   "acceleration": list(msg.acceleration), "yaw": float(msg.yaw),
                   "yawspeed": float(msg.yawspeed),"execution_active":bool(self.execution_active())}
            values = row["position"] + row["velocity"] + row["acceleration"] + [row["yaw"], row["yawspeed"]]
            self.bad_pva += not all(math.isfinite(x) for x in values)
            self.setpoints.append(row)
            self.pva_file.write(json_dumps(row) + "\n")

        def execution_evidence(self):
            # Compare at the navigation *sampling* time using two enclosing
            # references. Callback arrival order and a nearest-future sample do
            # not define the reference the vehicle was supposed to follow.
            refs=sorted(self.setpoints,key=lambda r:r["timestamp"])
            times=[r["timestamp"]*1000 for r in refs]
            active=[r for r in self.navigation_rows if r["execution_active"]]
            certificates=[]
            unmatched=excluded=0
            for row in active:
                sample=row["sample_ns"]
                index=bisect.bisect_right(times,sample)
                if index==0 or index>=len(times):
                    # NAV_LAND owns subsequent motion. The last 10 Hz task
                    # status can lag the end of the 50 Hz reference stream.
                    if times and sample>times[-1] and self.latest.get("phase") in ("LANDING","COMPLETE"):
                        excluded+=1
                    else: unmatched+=1
                    continue
                before,after=refs[index-1],refs[index]
                dt=(times[index]-times[index-1])*1e-9
                if dt<=0 or dt>self.reaction_budget:
                    unmatched+=1
                    continue
                alpha=(sample-times[index-1])/(times[index]-times[index-1])
                def enu(field):
                    value=(1-alpha)*np.asarray(before[field])+alpha*np.asarray(after[field])
                    return value[[1,0,2]]*np.array([1,1,-1])
                position_error=np.asarray(row["position"])-enu("position")
                velocity_error=np.asarray(row["velocity"])-enu("velocity")
                pe=float(np.linalg.norm(position_error))
                ve=float(np.linalg.norm(velocity_error))
                pb=(self.position_tolerance+self.sigma_multiplier*row["position_sigma_certificate_m"]) if row["covariance_valid"] else None
                vb=(self.velocity_tolerance+self.sigma_multiplier*row["velocity_sigma_certificate_mps"]) if row["covariance_valid"] else None
                position_axis_bound=(self.position_tolerance+self.sigma_multiplier*np.asarray(row["position_axis_sigma_certificate_m"])) if row["covariance_valid"] else None
                velocity_axis_bound=(self.velocity_tolerance+self.sigma_multiplier*np.asarray(row["velocity_axis_sigma_certificate_mps"])) if row["covariance_valid"] else None
                position_axes_accepted=(position_axis_bound is not None and bool(np.all(np.abs(position_error)<=position_axis_bound)))
                velocity_axes_accepted=(velocity_axis_bound is not None and bool(np.all(np.abs(velocity_error)<=velocity_axis_bound)))
                certificate={"sample_ns":sample,"wall":row["wall"],"phase":row["phase"],
                    "position_error_m":pe,"velocity_error_mps":ve,"position_bound_m":pb,
                    "velocity_bound_mps":vb,"position_margin_m":pb-pe if pb is not None else None,
                    "velocity_margin_mps":vb-ve if vb is not None else None,
                    "position_error_enu_m":position_error.tolist(),
                    "velocity_error_enu_mps":velocity_error.tolist(),
                    "position_axis_bound_m":position_axis_bound.tolist() if position_axis_bound is not None else None,
                    "velocity_axis_bound_mps":velocity_axis_bound.tolist() if velocity_axis_bound is not None else None,
                    "position_axis_margin_m":(position_axis_bound-np.abs(position_error)).tolist() if position_axis_bound is not None else None,
                    "velocity_axis_margin_mps":(velocity_axis_bound-np.abs(velocity_error)).tolist() if velocity_axis_bound is not None else None,
                    "position_axes_accepted":position_axes_accepted,
                    "velocity_axes_accepted":velocity_axes_accepted,
                    "reference_bracket_s":dt,"position_sigma_m":row["position_sigma"],
                    "velocity_sigma_mps":row["velocity_sigma"],
                    "position_sigma_certificate_m":row["position_sigma_certificate_m"],
                    "velocity_sigma_certificate_mps":row["velocity_sigma_certificate_mps"],
                    "position_axis_sigma_certificate_m":row["position_axis_sigma_certificate_m"],
                    "velocity_axis_sigma_certificate_mps":row["velocity_axis_sigma_certificate_mps"],
                    "accepted":pb is not None and vb is not None and pe<=pb and ve<=vb
                               and position_axes_accepted and velocity_axes_accepted}
                certificates.append(certificate)
                self.tracking_file.write(json_dumps(certificate)+"\n")
            self.tracking_file.flush()
            self.tracking_errors=[c["position_error_m"] for c in certificates]
            self.velocity_errors=[c["velocity_error_mps"] for c in certificates]
            def summary(values):
                values=list(values)
                return {"count":len(values),"max":max(values) if values else None,"min":min(values) if values else None,
                        "p95":float(np.percentile(values,95)) if values else None,
                        "p99":float(np.percentile(values,99)) if values else None}
            def intervals(rows,field,scale=1):
                return [(b[field]-a[field])*scale for a,b in zip(rows,rows[1:])]
            execution_pva=[r for r in self.setpoints if r["execution_active"]]
            nav_wall=summary(intervals(active,"wall"))
            nav_sample=summary(intervals(active,"sample_ns",1e-9))
            clock_wall=summary(intervals(self.clock_rows,"wall"))
            clock_sample=summary(intervals(self.clock_rows,"sample_ns",1e-9))
            clock_age=summary(r["clock_sample_age_s"] for r in active if r["clock_sample_age_s"] is not None)
            receipt_age=summary(r["clock_receipt_age_wall_s"] for r in active if r["clock_receipt_age_wall_s"] is not None)
            paired_intervals=[abs((b["sample_ns"]-a["sample_ns"])*1e-9-(b["wall"]-a["wall"]))
                              for a,b in zip(self.clock_rows,self.clock_rows[1:])]
            return {"navigation_samples":len(active),"tracking_certificates":len(certificates),
                    "unmatched_navigation_samples":unmatched,"excluded_after_reference_end":excluded,
                    "tracking_violations":sum(not c["accepted"] for c in certificates),
                    "axis_tracking_violations":sum(not(c["position_axes_accepted"] and c["velocity_axes_accepted"]) for c in certificates),
                    "minimum_position_margin_m":min((c["position_margin_m"] for c in certificates if c["position_margin_m"] is not None),default=None),
                    "minimum_velocity_margin_mps":min((c["velocity_margin_mps"] for c in certificates if c["velocity_margin_mps"] is not None),default=None),
                    "position_error_m":summary(self.tracking_errors),"velocity_error_mps":summary(self.velocity_errors),
                    "publication_wall_interval_s":summary(intervals(execution_pva,"wall")),
                    "publication_sample_interval_s":summary(intervals(execution_pva,"timestamp",1e-6)),
                    "navigation_wall_interval_s":nav_wall,"navigation_sample_interval_s":nav_sample,
                    "clock_wall_interval_s":clock_wall,"clock_sample_interval_s":clock_sample,
                    "clock_wall_interval_difference_s":summary(paired_intervals),
                    "navigation_clock_sample_age_s":clock_age,"navigation_clock_receipt_age_wall_s":receipt_age,
                    "reaction_budget_s":self.reaction_budget,
                    "position_tolerance_m":self.position_tolerance,"velocity_tolerance_mps":self.velocity_tolerance,
                    "sigma_multiplier":self.sigma_multiplier}

        def update_start_readiness(self):
            epoch=self.latest.get("localization_epoch")
            ready=(self.latest.get("localization_ready") and self.latest.get("vehicle_status_fresh")
                   and epoch is not None and self.clock_ns is not None)
            if not ready:
                self.start_ready_epoch=self.start_ready_clock_ns=self.start_ready_wall=None
                return
            if (epoch!=self.start_ready_epoch or self.start_ready_clock_ns is None
                    or self.clock_ns<self.start_ready_clock_ns):
                self.start_ready_epoch=epoch
                self.start_ready_clock_ns=self.clock_ns
                self.start_ready_wall=time.monotonic()

        def gcs_evidence(self):
            try:
                gcs = json.loads((args.output_dir / "gcs_health.json").read_text())
                if not isinstance(gcs, dict):
                    raise ValueError("GCS health must be an object")
            except (OSError, ValueError):
                gcs = {"link_healthy": False, "failure": "GCS health evidence unavailable"}
            updated = gcs.get("updated_wall_monotonic")
            gcs["evidence_fresh_at_report"] = (isinstance(updated, (int, float))
                and math.isfinite(updated) and 0 <= time.monotonic()-updated <= 3)
            observed = gcs.get("read_only_px4_parameters", {})
            policy = observed.get("NAV_DLL_ACT", {}) if isinstance(observed, dict) else {}
            gcs["px4_datalink_policy_verified"] = (isinstance(policy, dict) and policy.get("value") == 2)
            return gcs

        def start_if_ready(self):
            if args.image_chain_only: return
            self.update_start_readiness()
            if self.started: return
            if self.start_future is not None:
                if not self.start_future.done(): return
                result = self.start_future.result()
                self.start_future = None
                if result and result.success:
                    self.started = True
                    event={"event":"start_service_accepted","message":result.message,"wall":time.monotonic()}
                    self.events.append(event)
                    self.event_file.write(json_dumps(event)+"\n")
                    self.event_file.flush()
                return
            now = time.monotonic()
            if now - self.last_start_try < 1: return
            # START establishes the stationary Offboard warmup. The controller
            # checks actual PX4 preflight after entering Offboard and before
            # arming; the previous manual/POSCTL mode need not be armable.
            # This is a test-initialization wait, not a relaxation of P0. Both
            # clocks must accumulate a stable ready interval in the same epoch.
            stable=(self.start_ready_clock_ns is not None and self.start_ready_wall is not None
                    and self.clock_ns-self.start_ready_clock_ns>=3_000_000_000
                    and now-self.start_ready_wall>=4.0)
            if (stable and self.latest.get("localization_ready") and self.latest.get("vehicle_status_fresh")
                    and self.start_client.service_is_ready()):
                self.last_start_try = now
                gcs = self.gcs_evidence()
                startup = takeoff_startup_evidence(self.motion_parameters, gcs.get("read_only_px4_parameters", {}))
                if not (gcs.get("link_healthy") and gcs["evidence_fresh_at_report"]
                        and gcs["px4_datalink_policy_verified"] and startup["passed"]):
                    return
                self.event_file.write(json_dumps({"event":"test_start_readiness_dwell","wall":now,
                    "localization_epoch":self.start_ready_epoch,
                    "takeoff_startup_validation":startup,
                    "ready_sim_s":(self.clock_ns-self.start_ready_clock_ns)*1e-9,
                    "ready_wall_s":now-self.start_ready_wall})+"\n")
                self.event_file.flush()
                self.start_future = self.start_client.call_async(Trigger.Request())

        def truth(self, msg):
            # This callback runs on Gazebo Transport's thread. Only copy the
            # exact named model origin and its original protobuf sampling time;
            # validation, file I/O and ROS bookkeeping stay on the monitor thread.
            self.truth_callback_messages += 1
            for pose in msg.pose:
                if pose.name != "x500_depth_0": continue
                sample=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nsec
                row={"sample_ns":sample,"wall":time.monotonic(),"name":pose.name,"entity_id":pose.id,
                     "position":[pose.position.x,pose.position.y,pose.position.z],
                     "quaternion_wxyz":[pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z],
                     "source":"gazebo_pose_v","reference":"model_origin",
                     "gz_partition":processes.env["GZ_PARTITION"]}
                with self.truth_lock:
                    self.truth_pending.append(row)
                    self.truth_named_messages += 1
                break

        def consume_truth(self):
            with self.truth_lock:
                pending=list(self.truth_pending)
                self.truth_pending.clear()
            for row in pending:
                self.truth_file.write(json_dumps(row)+"\n")
                point=np.asarray(row["position"])
                quat=np.asarray(row["quaternion_wxyz"])
                if (row["sample_ns"]>0 and np.isfinite(point).all()
                        and np.isfinite(quat).all() and np.linalg.norm(quat)>.5):
                    self.truth_entity_id=row["entity_id"]
                    self.truth_poses.append((row["sample_ns"],point,quat/np.linalg.norm(quat)))
            if pending: self.truth_file.flush()

        def pose_at(self, sample, history=None):
            points = list(self.poses if history is None else history)
            for (ta, pa, qa), (tb, pb, qb) in zip(points, points[1:]):
                if ta <= sample <= tb and 0 < tb - ta <= 250_000_000:
                    u = (sample - ta) / (tb - ta)
                    dot = float(np.dot(qa, qb))
                    if dot < 0: qb, dot = -qb, -dot
                    if dot > .9995: q = qa + u * (qb - qa)
                    else:
                        angle = math.acos(max(-1, min(1, dot)))
                        q = (math.sin((1-u)*angle)*qa + math.sin(u*angle)*qb) / math.sin(angle)
                    return pa + u * (pb-pa), q / np.linalg.norm(q)
            return None

        def first_static_hit(self, origin, ray):
            closest, wall = math.inf, False
            for lower, upper, is_wall, _ in self.scene_boxes:
                near, far = -math.inf, math.inf
                for axis in range(3):
                    if abs(ray[axis]) < 1e-10:
                        if not lower[axis] <= origin[axis] <= upper[axis]:
                            far = -math.inf
                            break
                    else:
                        a, b = (lower[axis]-origin[axis])/ray[axis], (upper[axis]-origin[axis])/ray[axis]
                        near, far = max(near, min(a,b)), min(far, max(a,b))
                hit = near if near > 0 else far
                if far >= max(0, near) and 0 < hit < closest:
                    closest, wall = hit, is_wall
            # The target cylinder and checkers are genuine visual occluders.
            # Rays hitting them cannot be used as known-wall calibration evidence.
            for center, radius, half_height in self.scene_cylinders:
                delta = origin-center
                a, b = float(np.dot(ray[:2],ray[:2])), float(2*np.dot(delta[:2],ray[:2]))
                c = float(np.dot(delta[:2],delta[:2])-radius*radius)
                disc = b*b-4*a*c
                hits = []
                if a > 1e-12 and disc >= 0:
                    for t in ((-b-math.sqrt(disc))/(2*a), (-b+math.sqrt(disc))/(2*a)):
                        if t > 0 and abs(delta[2]+t*ray[2]) <= half_height: hits.append(t)
                if abs(ray[2]) > 1e-10:
                    for z in (-half_height,half_height):
                        t = (z-delta[2])/ray[2]
                        if t > 0 and np.linalg.norm(delta[:2]+t*ray[:2]) <= radius: hits.append(t)
                if hits and min(hits) < closest: closest, wall = min(hits), False
            return closest if wall else None

        def calibrate(self):
            now = time.monotonic()
            if (not self.latest.get("localization_ready") or now-self.last_calibration_time < .5
                    or not self.depth_images or self.depth_info is None):
                return
            image = self.depth_images[0]
            nav, truth = self.pose_at(stamp(image)), self.pose_at(stamp(image), self.truth_poses)
            if nav is None or truth is None:
                oldest = max(self.poses[0][0] if self.poses else 0,
                             self.truth_poses[0][0] if self.truth_poses else 0)
                if stamp(image) < oldest: self.depth_images.popleft()
                return
            self.depth_images.popleft()
            self.last_calibration_time = now
            info = self.depth_info
            if (info.width, info.height) != (image.width, image.height) or info.k[0] <= 0 or info.k[4] <= 0:
                return
            if image.encoding not in ("32FC1", "16UC1"): return
            dtype = np.dtype((">" if image.is_bigendian else "<") + ("f4" if image.encoding=="32FC1" else "u2"))
            if image.step < image.width*dtype.itemsize or len(image.data) < image.step*image.height: return
            data = np.ndarray((image.height,image.width), dtype=dtype, buffer=image.data,
                              strides=(image.step,dtype.itemsize))
            point, quat = nav
            truth_point, truth_quat = truth
            r, truth_r = rotation(quat), rotation(truth_quat)
            origin, truth_origin = point+r@self.depth_mount, truth_point+truth_r@self.depth_mount
            offset = point-truth_point
            self.reference_offsets.append(offset.tolist())
            self.reference_errors.append(float(np.linalg.norm(offset)))
            self.body_reference_errors.append(float(np.linalg.norm(point-(truth_point+truth_r@np.array([0,0,.24])))))
            truth_residuals, nav_residuals = [], []
            for v in np.linspace(8,image.height-9,9).astype(int):
                for u in np.linspace(8,image.width-9,12).astype(int):
                    depth = float(data[v,u]) * (.001 if image.encoding=="16UC1" else 1)
                    if not math.isfinite(depth) or depth <= .1: continue
                    ray = np.array([1.,-(u-info.k[2])/info.k[0],-(v-info.k[5])/info.k[4]])
                    expected = self.first_static_hit(truth_origin,truth_r@self.depth_mount_rotation@ray)
                    # Keep correspondence defined by truth, never select by the
                    # observed residual (which would hide a wrong extrinsic).
                    if expected is None or expected > 20: continue
                    truth_residuals.append(abs(depth-expected))
                    nav_expected = self.first_static_hit(origin,r@self.depth_mount_rotation@ray)
                    if nav_expected is not None: nav_residuals.append(abs(depth-nav_expected))
            self.depth_truth_residuals.extend(truth_residuals)
            self.depth_navigation_residuals.extend(nav_residuals)
            self.calibration_frames += 1
            self.calibration_file.write(json_dumps({"sample_ns":stamp(image), "wall":now,
                "navigation_minus_model_origin_m":offset.tolist(),
                "model_origin_error_m":self.reference_errors[-1],
                "incorrect_base_link_reference_error_m":self.body_reference_errors[-1],
                "depth_truth_residuals_m":truth_residuals,
                "depth_navigation_residuals_m":nav_residuals})+"\n")
            self.calibration_file.flush()

        def replay(self):
            if args.detection_mode != "replay" or self.explore_since is None or self.rgb_info is None:
                return
            if time.monotonic()-self.last_replay_time < .15: return
            if time.monotonic() - self.explore_since < 3 or not self.images or self.max_regions < 1: return
            if args.inject_obstacle and not self.stop_seen: return
            image = self.images[0]
            pose = self.pose_at(stamp(image))
            if pose is None:
                if self.poses and stamp(image) < self.poses[0][0]: self.images.popleft()
                return
            self.images.popleft()
            point, quat = pose
            info = self.rgb_info
            if info.width != image.width or info.height != image.height or info.k[0] <= 0 or info.k[4] <= 0:
                return
            pixels = []
            for x in (2.63, 3.35):
                for y in (-.35, .35):
                    for z in (2.1, 3.9):
                        local = self.rgb_mount_rotation.T @ (rotation(quat).T @ (np.array([x, y, z]) - point) - self.rgb_mount)
                        if local[0] <= .1: return
                        pixels.append((info.k[2] - info.k[0]*local[1]/local[0],
                                       info.k[5] - info.k[4]*local[2]/local[0]))
            x0, y0 = np.min(pixels, axis=0)
            x1, y1 = np.max(pixels, axis=0)
            if x1 < 0 or x0 >= image.width or y1 < 0 or y0 >= image.height: return
            bbox = [max(0, float(x0)), max(0, float(y0)),
                    min(image.width-1, float(x1)), min(image.height-1, float(y1))]
            if (bbox[2]-bbox[0])*(bbox[3]-bbox[1]) < 1024: return
            payload = {"stamp": {"sec": image.header.stamp.sec, "nanosec": image.header.stamp.nanosec},
                       "frame_id": image.header.frame_id, "image_width": image.width,
                       "image_height": image.height,
                       "detections": [{"class_id": 0, "class_name": "gas_cylinders",
                                       "confidence": .99, "bbox_xyxy": bbox}]}
            self.source_pub.publish(image)
            self.detection_pub.publish(String(data=json_dumps(payload)))
            self.last_replay_time = time.monotonic()
            self.replay_count += 1

        def inject(self):
            if not args.inject_obstacle: return
            if self.obstacle_process is not None:
                if self.obstacle_process.poll() is None:
                    if time.monotonic()-self.obstacle_time>4:
                        self.obstacle_process.kill()
                        self.obstacle_process.communicate(timeout=1)
                        self.obstacle_process=None
                        raise RuntimeError("Gazebo obstacle injection timed out")
                    return
                stdout,stderr=self.obstacle_process.communicate()
                returncode=self.obstacle_process.returncode
                self.obstacle_process=None
                self.obstacle_injection.update({"response_receipt_wall":time.monotonic(),
                    "response_receipt_ros_ns":self.clock_ns,
                    "service_accepted":returncode==0 and "true" in stdout.lower()})
                self.event_file.write(json_dumps({"event":"test_obstacle_service_response",**self.obstacle_injection})+"\n")
                if not self.obstacle_injection["service_accepted"]:
                    raise RuntimeError("Gazebo obstacle injection failed: "+stdout+stderr)
                return
            if (self.obstacle_time is not None or self.latest.get("phase")!="EXPLORE"
                    or self.latest.get("motion_mode")!="TRACKING"
                    or not self.latest.get("motion_stop_certificate_valid")):
                return
            velocity = self.latest_velocity[:2]
            speed = float(np.linalg.norm(velocity))
            if (speed<.15 or not self.poses or not self.navigation_rows
                    or not self.navigation_rows[-1]["covariance_valid"]
                    or time.monotonic()-self.navigation_rows[-1]["wall"]>self.reaction_budget):
                return
            point = self.poses[-1][1]
            obstacle = point[:2] + 2.0 * velocity / speed
            heading = math.atan2(velocity[1], velocity[0])
            request = (f'name: "p2_brake_obstacle", position: {{x: {obstacle[0]}, y: {obstacle[1]}, z: 3}}, '
                       f'orientation: {{z: {math.sin(heading/2)}, w: {math.cos(heading/2)}}}')
            self.obstacle_time = time.monotonic()
            self.brake_seen = self.stop_seen = False
            self.obstacle_injection={"request_started_wall":self.obstacle_time,
                "request_started_ros_ns":self.clock_ns,"service_accepted":False,
                "obstacle_position_enu_m":[float(obstacle[0]),float(obstacle[1]),3.0],
                "navigation_at_request":self.navigation_rows[-1],
                "pre_request_motion_report":self.motion_status_rows[-1] if self.motion_status_rows else None,
                "insertion_time_source":"unknown_within_gazebo_service_request_response_window"}
            # The service client must not block receipt of navigation/PVA while
            # measuring reaction latency. Poll its short response in this loop.
            self.obstacle_process=subprocess.Popen(
                ["gz","service","-s","/world/powerplant/set_pose",
                 "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean",
                 "--timeout","2000","--req",request],stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,text=True,env=processes.env)
            self.event_file.write(json_dumps(clean_json({"event":"test_obstacle_request_started",**self.obstacle_injection}),allow_nan=False)+"\n")

        def braking_evidence(self):
            if not args.inject_obstacle: return None
            injection=self.obstacle_injection
            first,stopped=self.braking_first,self.braking_stop
            start_sample=injection.get("request_started_ros_ns") if injection else None
            stop_sample=stopped.get("navigation_sample_ns") if stopped else None
            rows=[r for r in self.navigation_rows if start_sample is not None
                  and stop_sample is not None and start_sample<=r["sample_ns"]<=stop_sample]
            positions=np.asarray([r["position"] for r in rows])
            path_length=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()) if len(rows)>=2 else None
            displacement=float(np.linalg.norm(positions[-1]-positions[0])) if len(rows)>=2 else None
            max_sample_gap=max(((b["sample_ns"]-a["sample_ns"])*1e-9 for a,b in zip(rows,rows[1:])),default=None)
            wall_latency=(first["receipt_wall"]-injection["request_started_wall"]) if first and injection else None
            ros_latency=((first["receipt_ros_ns"]-start_sample)*1e-9) if first and first["receipt_ros_ns"] is not None and start_sample is not None else None
            observed_latency_within_budget=(wall_latency is not None and ros_latency is not None
                and 0<=wall_latency<=self.reaction_budget and 0<=ros_latency<=self.reaction_budget)
            body_min=np.array([-self.motion_parameters["motion_body_radius_m"],
                               -self.motion_parameters["motion_body_radius_m"],
                               self.motion_parameters["motion_body_z_min_m"]])
            body_max=np.array([self.motion_parameters["motion_body_radius_m"],
                              self.motion_parameters["motion_body_radius_m"],
                              self.motion_parameters["motion_body_z_max_m"]])
            # A stop box bounds the whole swept physical volume. Containment of
            # the navigation reference alone is insufficient: fit the audited
            # body bound, with position uncertainty, inside each fixed box.
            def body_containment(snapshot, selected):
                box=snapshot.get("motion_stopping_envelope_enu") if snapshot else None
                if (not snapshot or not snapshot.get("motion_stop_certificate_valid")
                        or not isinstance(box,dict) or not selected):
                    return {"sampled_body_containment_confirmed":False,"reason":"missing_valid_box_or_navigation"}
                lower,upper=np.asarray(box.get("minimum_m",[])),np.asarray(box.get("maximum_m",[]))
                if (lower.shape!=(3,) or upper.shape!=(3,) or not np.isfinite(lower).all()
                        or not np.isfinite(upper).all() or np.any(lower>upper)):
                    return {"sampled_body_containment_confirmed":False,"reason":"invalid_stop_box"}
                margins=[]
                invalid=0
                for row in selected:
                    if not row["covariance_valid"]:
                        invalid+=1
                        continue
                    q=np.asarray(row["orientation_wxyz"])
                    measured_tilt=math.acos(float(np.clip(rotation(q)[2,2],-1,1)))
                    if measured_tilt>float(self.motion_parameters["initial_scan_max_tilt_rad"]):
                        invalid+=1
                        continue
                    uncertainty=self.sigma_multiplier*np.asarray(row["position_axis_sigma_certificate_m"])
                    p=np.asarray(row["position"])
                    physical_min=p+body_min-uncertainty
                    physical_max=p+body_max+uncertainty
                    margins.append(np.minimum(physical_min-lower,upper-physical_max))
                minimum=np.min(margins,axis=0).tolist() if margins else None
                return {"sampled_body_containment_confirmed":bool(margins and invalid==0 and min(minimum)>=0),
                    "checked_navigation_samples":len(margins),"invalid_navigation_or_tilt_samples":invalid,
                    "minimum_axis_margin_m":minimum,"fixed_stop_box_enu":box,
                    "body_reference":"navigation_model_origin; full audited cylinder AABB plus position uncertainty",
                    "includes_map_margin_in_physical_body":False,
                    "continuous_between_samples_confirmed":False,
                    "motion_report_time_association_confirmed":False}
            pre=injection.get("pre_request_motion_report") if injection else None
            pre_containment=body_containment(pre,rows)
            after_first=[r for r in rows if first and first.get("odom_sample_stamp_ns") is not None
                         and r["sample_ns"]>=first["odom_sample_stamp_ns"]]
            braking_containment=body_containment(first,after_first)
            observed_stop=bool(injection and injection.get("service_accepted") and first and stopped
                and rows and np.linalg.norm(rows[-1]["velocity"])<=.12)
            summary={"service_request":injection,"first_braking_report":first,"stopped_report":stopped,
                "navigation_samples_in_observed_window":len(rows),
                "observed_navigation_path_length_m":path_length,
                "observed_navigation_start_end_displacement_m":displacement,
                "observed_navigation_start_enu_m":rows[0]["position"] if rows else None,
                "observed_navigation_end_enu_m":rows[-1]["position"] if rows else None,
                "observed_start_velocity_enu_mps":rows[0]["velocity"] if rows else None,
                "observed_end_velocity_enu_mps":rows[-1]["velocity"] if rows else None,
                "observed_duration_sim_s":(rows[-1]["sample_ns"]-rows[0]["sample_ns"])*1e-9 if len(rows)>=2 else None,
                "maximum_navigation_sample_gap_s":max_sample_gap,
                "request_to_first_braking_receipt_wall_s":wall_latency,
                "request_to_first_braking_report_clock_s":ros_latency,
                "observed_request_to_braking_within_reaction_budget":observed_latency_within_budget,
                "observed_braking_then_measured_stop":observed_stop,
                "injection_preceded_first_braking_response":bool(injection and first
                    and injection.get("response_receipt_wall") is not None
                    and injection["response_receipt_wall"]<=first["receipt_wall"]),
                "pre_request_fixed_box_sampled_body_containment":pre_containment,
                "first_braking_fixed_box_sampled_body_containment":braking_containment,
                "physical_braking_acceptance":"UNCONFIRMED",
                "obstacle_to_brake_causal_attribution_confirmed":False,
                "continuous_stopping_envelope_confirmed":False,
                "physical_braking_acceptance_confirmed":False,
                "limitations":["Obstacle insertion has only a client request/response window, not a Gazebo application timestamp.",
                    "Motion status has no exact executor input/sample identifier; odom stamp and report receipt are retained but not treated as that identifier.",
                    "No obstacle-map-integration/braking-trigger identifier proves causality.",
                    "Body containment is checked at navigation samples only; it does not prove motion or tilt bounds between samples."],
                "evidence_file":str(args.output_dir/"braking.json")}
            artifact={**summary,"navigation_samples":rows,"motion_status_snapshots":list(self.braking_status_rows)}
            (args.output_dir/"braking.json").write_text(json_dumps(clean_json(artifact),indent=2,allow_nan=False))
            return summary

        def result(self, reason):
            chain=self.yolo_chain_evidence() if args.detection_mode=="yolo" else None
            gcs = self.gcs_evidence()
            startup = takeoff_startup_evidence(self.motion_parameters, gcs.get("read_only_px4_parameters", {}))
            if args.image_chain_only:
                return {"passed":chain["passed"],"exit_reason":reason,
                    "acceptance_scope":"yolo_image_chain_only","flight_acceptance_executed":False,
                    "start_requested":False,"yolo_image_chain_checks":chain["checks"],
                    "yolo_image_chain":chain,"detection_mode":args.detection_mode,
                    "synthetic_detection_replay":False,"observed_sensors":self.sensor_evidence(),
                    "gcs_evidence":gcs,
                    "takeoff_startup_validation":startup,
                    "phases":self.phases,"last_status":self.latest}
            evidence=self.execution_evidence()
            pva = self.setpoints
            gaps = np.diff([r["wall"] for r in pva]) if len(pva) > 1 else np.array([])
            rate = ((len(pva)-1)/(pva[-1]["wall"]-pva[0]["wall"])) if len(pva)>1 else 0
            finite = [r for r in pva if all(math.isfinite(x) for x in r["velocity"]+r["acceleration"])]
            max_vxy = max((math.hypot(*r["velocity"][:2]) for r in finite), default=math.inf)
            max_vz = max((abs(r["velocity"][2]) for r in finite), default=math.inf)
            max_axy = max((math.hypot(*r["acceleration"][:2]) for r in finite), default=math.inf)
            max_az = max((abs(r["acceleration"][2]) for r in finite), default=math.inf)
            ordered = [p for p in self.phases if p in EXPECTED_PHASES]
            required = EXPECTED_PHASES if args.detection_mode != "none" else [p for p in EXPECTED_PHASES if p != "TARGETING_CYCLE"]
            positions = [ordered.index(p) for p in required if p in ordered]
            truth_p95 = float(np.percentile(self.depth_truth_residuals,95)) if self.depth_truth_residuals else math.inf
            nav_p95 = float(np.percentile(self.depth_navigation_residuals,95)) if self.depth_navigation_residuals else math.inf
            reference_p95 = float(np.percentile(self.reference_errors,95)) if self.reference_errors else math.inf
            model_mean = float(np.mean(self.reference_errors)) if self.reference_errors else math.inf
            body_mean = float(np.mean(self.body_reference_errors)) if self.body_reference_errors else math.inf
            braking = self.braking_evidence()
            checks = {
                "actual_gcs_heartbeat_link_healthy": bool(gcs.get("link_healthy") and gcs["evidence_fresh_at_report"]),
                "px4_datalink_checks_retained": gcs["px4_datalink_policy_verified"],
                "takeoff_bootstrap_covers_actual_px4_startup": startup["passed"],
                "start_service_accepted":self.started,
                "sampled_tracking_certificates_available":evidence["tracking_certificates"]>=100 and evidence["unmatched_navigation_samples"]==0,
                "measured_tracking_within_uncertainty_bounds":evidence["tracking_certificates"]>=100 and evidence["tracking_violations"]==0,
                "measured_tracking_within_axis_uncertainty_bounds":evidence["tracking_certificates"]>=100 and evidence["axis_tracking_violations"]==0,
                "navigation_intervals_within_reaction_budget":all(evidence[k]["count"]>=99 and 0<=evidence[k]["min"] and evidence[k]["max"]<=self.reaction_budget for k in ("navigation_wall_interval_s","navigation_sample_interval_s")),
                "clock_receipt_within_reaction_budget":all(evidence[k]["count"]>=99 and -.05<=evidence[k]["min"] and evidence[k]["max"]<=self.reaction_budget for k in ("clock_wall_interval_s","clock_sample_interval_s","navigation_clock_sample_age_s","navigation_clock_receipt_age_wall_s","clock_wall_interval_difference_s")),
                "publication_max_gap_within_reaction_budget":all(evidence[k]["count"]>=99 and 0<=evidence[k]["min"] and evidence[k]["max"]<=self.reaction_budget for k in ("publication_wall_interval_s","publication_sample_interval_s")),
                "sampled_navigation_matches_model_origin": len(self.reference_errors)>=10 and reference_p95<=.20 and model_mean+.05<body_mean,
                "truth_depth_mount_wall_projection": len(self.depth_truth_residuals)>=200 and truth_p95<=.08,
                "navigation_depth_wall_projection": len(self.depth_navigation_residuals)>=200 and nav_p95<=.25,
                "all_required_phases": all(p in ordered for p in required),
                "phase_order": len(positions)==len(required) and positions==sorted(positions),
                "landed_and_disarmed_completion": self.latest.get("phase")=="COMPLETE",
                "finite_position_velocity_acceleration_yaw": len(pva)>=250 and self.bad_pva==0,
                "command_velocity_limits": max_vxy<=.61 and max_vz<=.31,
                "command_acceleration_limits": max_axy<=.51 and max_az<=.31,
                "publication_rate_at_least_40_hz": rate>=40,
                "p95_publication_gap_at_most_0_1_s": bool(len(gaps) and np.percentile(gaps,95)<=.1),
                "p99_publication_gap_at_most_0_1_s": bool(len(gaps) and np.percentile(gaps,99)<=.1),
                "no_mission_fault": "FAULT" not in self.phases,
                "topology_built": self.max_regions>=1,
            }
            if args.detection_mode != "none":
                checks.update({
                    "inspection_result_explicit": self.latest.get("inspection_result") in ("PARTIAL","REACHABLE_MODELED_COMPLETE"),
                    "actual_photo_and_surface_evidence": self.photo_count>0 and self.latest.get("inspection_covered_area_m2",0)>0,
                    "coverage_scope_explicit": self.latest.get("inspection_coverage_scope")=="reachable_modeled_surface",
                })
            if args.inject_obstacle:
                checks["injected_obstacle_braked_to_measured_stop"] = braking["observed_braking_then_measured_stop"]
                checks["physical_braking_acceptance_confirmed"] = braking["physical_braking_acceptance_confirmed"]
            if args.scenario == "two_rooms":
                checks["multiple_regions_and_known_portal"] = self.max_regions>=2 and self.max_portals>=1
            if args.detection_mode=="yolo":
                checks["real_yolo_image_chain"]=chain["passed"]
            return {"passed": all(checks.values()), "exit_reason": reason, "checks": checks,
                    "acceptance_scope":"full_mission","flight_acceptance_executed":self.started,
                    "yolo_image_chain_checks":chain["checks"] if chain else None,
                    "yolo_image_chain":chain,
                    "gcs_evidence":gcs,
                    "takeoff_startup_validation":startup,
                    "sensor_profile":args.sensor_profile,
                    "performance_scope":"native_sensor_load" if args.sensor_profile=="native" else "cpu_reduced_sensor_load",
                    "observed_sensors":self.sensor_evidence(),
                    "phases": self.phases, "detection_mode": args.detection_mode,
                    "synthetic_detection_replay": args.detection_mode=="replay",
                    "replayed_frames": self.replay_count, "saved_photos": self.photo_count,
                    "publication_rate_hz": rate, "setpoint_count": len(pva),
                    "maximum_command_horizontal_velocity_mps": max_vxy,
                    "maximum_command_vertical_velocity_mps": max_vz,
                    "maximum_command_horizontal_acceleration_mps2": max_axy,
                    "maximum_command_vertical_acceleration_mps2": max_az,
                    "p95_publication_gap_s": float(np.percentile(gaps,95)) if len(gaps) else None,
                    "p99_publication_gap_s": float(np.percentile(gaps,99)) if len(gaps) else None,
                    "maximum_publication_gap_s":float(gaps.max()) if len(gaps) else None,
                    "execution_evidence":evidence,
                    "braking_evidence":braking,
                    "max_regions": self.max_regions, "max_portals": self.max_portals,
                    "maximum_measured_horizontal_velocity_mps":self.measured_vxy,
                    "maximum_measured_vertical_velocity_mps":self.measured_vz,
                    "maximum_observed_tracking_error_m":max(self.tracking_errors,default=None),
                    "maximum_observed_velocity_error_mps":max(self.velocity_errors,default=None),
                    "latest_position_sigma_m":self.latest_position_sigma,
                    "latest_velocity_sigma_mps":self.latest_velocity_sigma,
                    "truth_source":"gazebo_pose_v_named_model_origin",
                    "truth_model_name":"x500_depth_0","truth_entity_id":self.truth_entity_id,
                    "truth_callback_messages":self.truth_callback_messages,
                    "truth_named_messages":self.truth_named_messages,
                    "calibration_frames":self.calibration_frames,
                    "known_wall_depth_sample_count":len(self.depth_truth_residuals),
                    "p95_truth_depth_wall_residual_m":truth_p95,
                    "p95_navigation_depth_wall_residual_m":nav_p95,
                    "p95_navigation_model_origin_error_m":reference_p95,
                    "mean_navigation_model_origin_error_m":model_mean,
                    "mean_incorrect_base_link_reference_error_m":body_mean,
                    "mean_navigation_minus_model_origin_m":np.mean(self.reference_offsets,axis=0).tolist() if self.reference_offsets else None,
                    "last_input_health":self.input_health,
                    "last_status": self.latest}

        def close(self):
            if self.obstacle_process is not None:
                if self.obstacle_process.poll() is None:
                    self.obstacle_process.kill()
                self.obstacle_process.communicate(timeout=1)
                self.obstacle_process = None
            self.gz_truth_node.unsubscribe("/world/powerplant/dynamic_pose/info")
            self.consume_truth()
            self.truth_file.close()
            self.yolo_file.close()
            self.health_file.close()
            self.event_file.close(); self.state_file.close(); self.pva_file.close(); self.pose_file.close(); self.calibration_file.close(); self.tracking_file.close()

    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    reason = "timeout"
    try:
        while time.monotonic()-started < args.timeout+args.startup_timeout:
            failures = processes.failed()
            if failures: raise RuntimeError("; ".join(failures))
            rclpy.spin_once(node, timeout_sec=.02)
            node.consume_truth()
            if not args.image_chain_only: node.calibrate()
            node.start_if_ready()
            node.inject()
            node.replay()
            if args.image_chain_only and node.yolo_success_count>=3:
                reason="yolo_image_chain_complete"; break
            if not args.image_chain_only and node.latest.get("phase") == "FAULT":
                reason = "mission_fault"; break
            if not args.image_chain_only and node.latest.get("phase") == "COMPLETE":
                reason = "complete"; break
            if not args.image_chain_only and not node.started and time.monotonic()-started > args.startup_timeout:
                reason = "startup_or_health_timeout"; break
        report = node.result(reason)
    except BaseException as error:
        report = node.result(type(error).__name__ + ": " + str(error))
        report["passed"] = False
    finally:
        if node.started and node.latest.get("phase") not in ("COMPLETE", "LANDING") and node.land_client.service_is_ready():
            future = node.land_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return report


def clean_json(value):
    # NumPy scalars/arrays can occur in diagnostics even when a run exits early.
    if type(value).__module__.startswith("numpy"):
        if hasattr(value,"tolist"): return clean_json(value.tolist())
        if hasattr(value,"item"): return clean_json(value.item())
    if isinstance(value,float) and not math.isfinite(value): return None
    if isinstance(value,dict): return {k:clean_json(v) for k,v in value.items()}
    if isinstance(value,list): return [clean_json(v) for v in value]
    return value


def main():
    args = parse_args()
    env, commands, manifest = prepare(args)
    report_path = args.output_dir / "report.json"
    if args.prepare_only:
        report = {"passed": False, "prepared_only": True, "missing_dependencies": manifest["missing_dependencies"], "sensor_profile":args.sensor_profile, "performance_scope":manifest["sensor_profile"]["performance_scope"]}
        report_path.write_text(json_dumps(report, indent=2))
        print(json_dumps({"prepared": True, "manifest": str(args.output_dir/"manifest.json"), **report}))
        return int(bool(manifest["missing_dependencies"]))
    if manifest["missing_dependencies"]:
        raise SystemExit("Missing dependencies: " + ", ".join(manifest["missing_dependencies"]))
    # Refuse rather than attach to or stop another PX4 instance.
    existing = subprocess.run(["pgrep", "-x", "px4"], capture_output=True, text=True)
    if existing.returncode == 0:
        raise SystemExit("A PX4 process is already running; stop that test before this isolated run")
    for port in (args.agent_port, 14550, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", port))
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "ROS2CLI_ENABLE_DAEMON")})
    processes = Processes(args.output_dir, env)
    report = {"passed": False, "exit_reason": "not_started"}
    try:
        for name, argv, cwd in commands:
            if name == "sensors":
                deadline=time.monotonic()+args.startup_timeout
                topics=[]
                while time.monotonic()<deadline:
                    query=subprocess.run(["gz","topic","-l"],env=env,capture_output=True,text=True,timeout=5)
                    topics=query.stdout.splitlines()
                    images=[t.strip() for t in topics if "/sensor/IMX214/image" in t or t.strip()=="/camera"]
                    infos=[t.strip() for t in topics if "/sensor/IMX214/camera_info" in t or t.strip()=="/camera/camera_info"]
                    depth_infos=[t.strip() for t in topics if "/sensor/StereoOV7251/camera_info" in t or t.strip() in ("/depth_camera/camera_info","/camera_info")]
                    if images and infos and depth_infos:
                        argv=argv+[f"rgb_gz_topic:={images[0]}",f"rgb_info_gz_topic:={infos[0]}",
                                   f"depth_info_gz_topic:={depth_infos[0]}"]
                        manifest["resolved_sensor_topics"]={"rgb":images[0],"rgb_info":infos[0],
                                                           "depth_info":depth_infos[0]}
                        manifest["discovered_gazebo_topics"]=topics
                        (args.output_dir/"manifest.json").write_text(json_dumps(manifest,ensure_ascii=False,indent=2))
                        break
                    if processes.failed(): raise RuntimeError("; ".join(processes.failed()))
                    time.sleep(.2)
                else:
                    raise RuntimeError("Gazebo RGB/CameraInfo topics unavailable: "+str(topics))
            processes.start(name, argv, cwd)
            time.sleep(.3)
        report = monitor(args, processes)
    except BaseException as error:
        report = {"passed": False, "exit_reason": type(error).__name__+": "+str(error)}
    finally:
        processes.close()
        report["sensor_profile"]=args.sensor_profile
        report["acceptance_scope"]="yolo_image_chain_only" if args.image_chain_only else "full_mission"
        report["performance_scope"]=manifest["sensor_profile"]["performance_scope"]
        manifest["sensor_profile"]["observed_sensors"]=report.get("observed_sensors",{})
        manifest["takeoff_startup_validation"]=report.get("takeoff_startup_validation")
        (args.output_dir / "manifest.json").write_text(json_dumps(clean_json(manifest),ensure_ascii=False,indent=2,allow_nan=False))
        report["manifest"] = str(args.output_dir / "manifest.json")
        report_path.write_text(json_dumps(clean_json(report), ensure_ascii=False, indent=2, allow_nan=False))
    print(json_dumps({"passed": report["passed"], "report": str(report_path),
                      "exit_reason": report.get("exit_reason")}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
