#!/usr/bin/env python3
"""Isolated PX4/Gazebo P2 acceptance run; never connects to an existing flight.

Source ROS, the workspace and tools/env/ros_env.sh before running. --prepare-only
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


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE = REPO_ROOT / "src/px4_powerplant_mission_cpp"
VALIDATION_ROOT = REPO_ROOT / "validation"
WORKSPACE = REPO_ROOT
_contact_spec=importlib.util.spec_from_file_location("powerplant_contact_monitor", VALIDATION_ROOT/"model/contact_monitor.py")
contact_support=importlib.util.module_from_spec(_contact_spec)
_contact_spec.loader.exec_module(contact_support)
_controller_spec=importlib.util.spec_from_file_location("controller_profile", Path(__file__).with_name("controller_profile.py"))
controller_support=importlib.util.module_from_spec(_controller_spec)
_controller_spec.loader.exec_module(controller_support)
_flight_spec=importlib.util.spec_from_file_location("flight_evidence", Path(__file__).with_name("flight_evidence.py"))
flight_support=importlib.util.module_from_spec(_flight_spec)
_flight_spec.loader.exec_module(flight_support)

EXPECTED_PHASES = ["START", "SAFE_TAKEOFF", "HOVER_SCAN", "EXPLORE",
                   "TARGETING_CYCLE", "RETURN_HOME", "LANDING", "COMPLETE"]

def controller_profile_evidence(args, observed):
    profile=getattr(args,"resolved_controller_profile",None)
    if profile is not None: return controller_support.readback_evidence(profile,observed)
    requested = {name:value for name,value in (
        ("MPC_THR_HOVER", getattr(args,"px4_hover_thrust",None)),
        ("MPC_Z_VEL_I_ACC", getattr(args,"px4_z_velocity_integral",None))) if value is not None}
    actual = {name:observed.get(name,{}).get("value") for name in requested}
    matched = all(isinstance(actual[name],(int,float)) and math.isfinite(actual[name])
                  and abs(actual[name]-value)<=1e-5 for name,value in requested.items())
    return {"passed":matched,"requested":requested,"actual_read_only_values":actual,
            "mode":"diagnostic_overrides" if requested else "stock_defaults",
            "formal_calibrated_acceptance":False,
            "scope":"isolated SITL stock/diagnostic comparison; frozen profile required for response-sync formal acceptance"}


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


def scenario_world(args):
    if args.scenario == "powerplant":
        return args.px4_root / "Tools/simulation/gz/worlds/powerplant.sdf"
    return Path(__file__).resolve().parent / "worlds" / f"{args.scenario}.sdf"


def capture_runtime_identity(args,config_path,local_models,asset_audit):
    """Pin actual executable/code/resource bytes, including uncommitted work."""
    def git(*arguments):
        return subprocess.run(['git','-C',str(WORKSPACE),*arguments],capture_output=True,check=True).stdout
    identity={'repository_head':git('rev-parse','HEAD').decode().strip(),
              'tracked_diff_binary_sha256':hashlib.sha256(git('diff','--binary','HEAD')).hexdigest(),
              'dirty_paths':git('status','--porcelain=v1','-z').decode().split('\0')}
    identity['dirty_paths']=[p for p in identity['dirty_paths'] if p]
    source_paths=[]
    for folder in (PACKAGE/'src',PACKAGE/'include',VALIDATION_ROOT,
                   WORKSPACE/'src/px4_powerplant_mission/px4_powerplant_mission',
                   WORKSPACE/'src/px4_powerplant_mission/launch',
                   WORKSPACE/'src/px4_powerplant_mission/config'):
        source_paths.extend(p for p in folder.rglob('*') if p.is_file()
                            and p.suffix in ('.cpp','.hpp','.py','.yaml'))
    source_paths.extend((PACKAGE/'CMakeLists.txt',Path(config_path)))
    if getattr(args,'resolved_motion_profile_path',None) is not None:
        source_paths.append(Path(args.resolved_motion_profile_path))
    if getattr(args,'controller_profile',None) is not None:
        source_paths.append(Path(args.controller_profile))
    try:
        from ament_index_python.packages import get_package_prefix
        prefix=Path(get_package_prefix('px4_powerplant_mission_cpp'))
        identity['cpp_package_resolution']='ament_index'
    except (ImportError,LookupError):
        prefix=WORKSPACE/'install/px4_powerplant_mission_cpp'
        identity['cpp_package_resolution']='workspace_fallback_unconfirmed'
    binaries=[prefix/'lib/px4_powerplant_mission_cpp'/name
              for name in ('offboard_mission_node','voxel_mapper_node')]
    identity['actual_cpp_executables']=contact_support.file_lock(binaries)
    identity['code_file_lock']=contact_support.file_lock(source_paths+binaries)
    resource_paths=[scenario_world(args),args.powerplant_asset_lock]
    if asset_audit:
        resource_paths.extend(Path(asset_audit['asset_root'])/r['path'] for r in asset_audit['verified_files'])
    else:
        resource_paths.remove(args.powerplant_asset_lock)
    if local_models:
        resource_paths.extend(p for p in Path(local_models).rglob('*') if p.is_file())
    for model in ('x500_depth','x500','x500_base','OakD-Lite'):
        resource_paths.extend(p for p in (args.px4_root/'Tools/simulation/gz/models'/model).rglob('*')
                              if p.is_file())
    resource_paths.append(args.px4_root/'build/px4_sitl_default/bin/px4')
    identity['resource_file_lock']=contact_support.file_lock(resource_paths)
    identity['scope']='actual source, resolved executables, isolated configuration and resources; logs/documents excluded from stability lock'
    return identity


def acceptance_scope(args):
    if args.image_chain_only: return "yolo_image_chain_only"
    if getattr(args,'governor_ablation',False):return "powerplant_takeoff_scan_ablation"
    if args.scenario == "powerplant":
        return {"scan":"powerplant_takeoff_scan", "roi-access":"powerplant_roi_access",
                "mission":"powerplant_local_roi_mission"}[args.stop_after]
    return "full_mission"


def resolve_render_context(requested, environment):
    if requested not in ("auto", "display", "egl"):
        raise ValueError("unknown Gazebo render context")
    selected = ("display" if environment.get("DISPLAY") else "egl") if requested=="auto" else requested
    if selected=="display" and not environment.get("DISPLAY"):
        raise RuntimeError("display rendering requires DISPLAY; use egl on a headless machine")
    return selected


def limited_stage_progress(stop_after, phases, events, state,
                           navigation_distance=0.0, truth_distance=0.0):
    required = ["START", "SAFE_TAKEOFF", "HOVER_SCAN", "EXPLORE"]
    positions = [phases.index(p) for p in required if p in phases]
    checks = {"initial_flight_phases_in_order":len(positions)==len(required) and positions==sorted(positions),
              "initial_scan_completed_with_observation_evidence":any(e.get("event")=="initial_scan_complete" for e in events),
              "currently_exploring":state.get("phase")=="EXPLORE",
              "navigation_healthy":bool(state.get("localization_ready"))}
    if stop_after == "roi-access":
        checks.update({"roi_access_mode_active":state.get("explore_roi_mode")=="ROI_ACCESS",
            "executing_tracking_reference":state.get("motion_mode")=="TRACKING",
            "measured_navigation_moved_0_3m":math.isfinite(navigation_distance) and navigation_distance>=.3,
            "gazebo_model_origin_moved_0_3m":math.isfinite(truth_distance) and truth_distance>=.3})
    return {"passed":all(checks.values()),"checks":checks,
            "navigation_displacement_m":navigation_distance,"truth_displacement_m":truth_distance,
            "scope":"takeoff_scan" if stop_after=="scan" else "roi_access_progress",
            "task_roi_complete":False,"full_flight_acceptance":False}


def landing_completion_evidence(state, status, landed, clock_ns, clock_wall,
                                now_wall, max_age_s, disarmed_state=1):
    """Require independent PX4 measurements, not only the task phase string."""
    def fresh(row):
        if not isinstance(row, dict) or clock_ns is None: return False
        sample,receipt=row.get("sample_ns"),row.get("receipt_wall")
        return (type(sample) is int and sample>0 and isinstance(receipt,(int,float))
                and math.isfinite(receipt) and 0<=now_wall-receipt<=max_age_s
                and -.05<=(clock_ns-sample)*1e-9<=max_age_s)
    clock_fresh=(isinstance(clock_wall,(int,float)) and math.isfinite(clock_wall)
                 and 0<=now_wall-clock_wall<=max_age_s)
    checks={"task_reports_complete":state.get("phase")=="COMPLETE",
            "fresh_simulation_clock":bool(clock_fresh),
            "fresh_px4_vehicle_status":fresh(status),
            "fresh_px4_landing_measurement":fresh(landed),
            "px4_disarmed":bool(status and status.get("arming_state")==disarmed_state),
            "px4_landed":bool(landed and landed.get("landed") is True)}
    return {"passed":all(checks.values()),"checks":checks,
            "vehicle_status":status,"landing_measurement":landed,
            "scope":"independent PX4 landed/disarmed sample and receipt evidence"}


def access_motion_progress(anchor, sample, identity, active):
    """Accumulate only paired measurements within one continuous task/epoch."""
    if (not active or sample is None or len(identity)!=2
            or any(type(value) is not int or value<=0 for value in identity)):
        return None,0.0,0.0
    if (type(sample.get("sample_ns")) is not int or sample["sample_ns"]<=0
            or any(len(sample.get(key,[]))!=3 or not all(math.isfinite(x) for x in sample[key])
                   for key in ("navigation","truth"))):
        return None,0.0,0.0
    if (anchor is None or anchor["identity"]!=identity
            or sample["sample_ns"]<anchor["sample_ns"]):
        return {**sample,"identity":identity},0.0,0.0
    if sample["sample_ns"]==anchor["sample_ns"]: return anchor,0.0,0.0
    return anchor,math.dist(sample["navigation"],anchor["navigation"]),math.dist(sample["truth"],anchor["truth"])


def read_asset_lock(path, asset_root, world):
    """Check the real scene's recursively referenced local assets before flight."""
    asset_root, world = Path(asset_root).resolve(), Path(world).resolve()
    payload = json.loads(Path(path).read_text())
    if payload.get("schema_version") != 1:
        raise RuntimeError("unsupported powerplant asset lock schema")
    if payload.get("asset_commit_pending"):
        raise RuntimeError("powerplant assets have not been committed to a reproducible baseline")
    commit = payload.get("model_git_commit")
    if commit:
        resolved = subprocess.run(["git", "-C", str(asset_root), "rev-parse", "HEAD"],
                                  capture_output=True, text=True, check=True).stdout.strip()
        if resolved != commit:
            raise RuntimeError("powerplant model repository differs from the pinned asset commit")
    entries = payload.get("files")
    if isinstance(entries, dict):
        entries = [{"path": key, "sha256": value} for key, value in entries.items()]
    if not isinstance(entries, list) or not entries:
        raise RuntimeError("powerplant asset lock must contain nonempty files")
    locked = {}
    for entry in entries:
        relative, expected = entry.get("path"), entry.get("sha256")
        if (not isinstance(relative, str) or Path(relative).is_absolute()
                or not isinstance(expected, str) or len(expected) != 64
                or any(c not in "0123456789abcdef" for c in expected)):
            raise RuntimeError("invalid powerplant asset lock entry")
        source = (asset_root / relative).resolve()
        if not source.is_relative_to(asset_root) or source in locked:
            raise RuntimeError("duplicate or escaping powerplant asset path")
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f"powerplant asset changed: {source}")
        locked[source] = actual

    def require(source):
        source = source.resolve()
        if source not in locked:
            raise RuntimeError(f"powerplant asset is not locked: {source}")
        return source

    checked = set()
    def walk(source):
        source = require(source)
        if source in checked: return
        checked.add(source)
        root = ET.parse(source).getroot()
        for include in root.findall(".//include"):
            uri = include.findtext("uri", "")
            if not uri.startswith("model://"):
                raise RuntimeError(f"unsupported included asset URI: {uri}")
            folder = asset_root / "models" / uri[len("model://"):]
            config = require(folder / "model.config")
            filename = ET.parse(config).getroot().findtext("sdf")
            if not filename: raise RuntimeError(f"missing model SDF declaration: {config}")
            walk(folder / filename)
        for uri_element in root.findall(".//geometry/mesh/uri"):
            uri = uri_element.text or ""
            if uri.startswith("model://"):
                mesh = asset_root / "models" / uri[len("model://"):]
            elif uri.startswith("file://"):
                mesh = source.parent / uri[len("file://"):]
            else:
                mesh = source.parent / uri
            require(mesh)
    walk(world)
    return {"passed": True, "manifest": str(Path(path).resolve()),
            "manifest_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
            "world_sha256": locked[world], "asset_root": str(asset_root),
            "verified_files": [{"path": str(p.relative_to(asset_root)), "sha256": digest}
                               for p, digest in sorted(locked.items())],
            "recursive_scene_references_verified": True}


class VisualSceneEvaluator:
    """Offline visual ray truth. Its mesh/geometry never reaches navigation."""
    def __init__(self, world, models):
        import numpy as np
        self.np, self.models = np, Path(models)
        self.geometries, self.visual_meshes = [], []
        root = ET.parse(world).getroot()
        scene = root.find("world")
        if scene is None: raise RuntimeError("evaluation world is missing")
        self._walk(scene, np.eye(4), Path(world).parent, "")

    def _pose(self, element):
        np = self.np
        if element is None: return np.eye(4)
        if element.get("relative_to"):
            raise RuntimeError("offline evaluator requires resolved relative_to poses")
        values = [float(x) for x in (element.text or "").split()]
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            raise RuntimeError("invalid visual pose")
        x,y,z,r,p,w = values
        cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
        transform = np.eye(4)
        transform[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                            [-sp,cp*sr,cp*cr]]
        transform[:3,3] = [x,y,z]
        return transform

    def _walk(self, element, parent, folder, name):
        np = self.np
        for include in element.findall("include"):
            uri = include.findtext("uri", "")
            if not uri.startswith("model://"):
                raise RuntimeError(f"unsupported visual include URI: {uri}")
            model_dir = self.models / uri[len("model://"):]
            config = model_dir / "model.config"
            sdf_name = ET.parse(config).getroot().findtext("sdf")
            model = ET.parse(model_dir / sdf_name).getroot().find("model")
            if model is None: raise RuntimeError(f"included model is missing: {model_dir}")
            self._model(model, parent @ self._pose(include.find("pose")), model_dir,
                        include.findtext("name", model.get("name", "")),
                        override_pose=include.find("pose") is not None)
        for model in element.findall("model"):
            self._model(model, parent, folder, model.get("name", ""))

    def _model(self, model, parent, folder, name, override_pose=False):
        np = self.np
        transform = parent if override_pose else parent @ self._pose(model.find("pose"))
        for link in model.findall("link"):
            body = transform @ self._pose(link.find("pose"))
            for visual in link.findall("visual"):
                pose = body @ self._pose(visual.find("pose"))
                geometry = visual.find("geometry")
                if geometry is None: continue
                calibration = name.startswith(("floor", "ground", "ceiling", "wall", "divider", "partition", "indoor_enclosure", "power_plant"))
                if geometry.find("box") is not None:
                    size = np.asarray([float(x) for x in geometry.findtext("box/size").split()])
                    self.geometries.append(("box", np.linalg.inv(pose), size/2, calibration, name))
                elif geometry.find("cylinder") is not None:
                    values = (float(geometry.findtext("cylinder/radius")), float(geometry.findtext("cylinder/length"))/2)
                    self.geometries.append(("cylinder", np.linalg.inv(pose), values, calibration, name))
                elif geometry.find("plane") is not None:
                    size = np.asarray([float(x) for x in geometry.findtext("plane/size").split()])
                    normal = np.asarray([float(x) for x in geometry.findtext("plane/normal", "0 0 1").split()])
                    if not np.allclose(normal, [0,0,1]): raise RuntimeError("unsupported visual plane normal")
                    self.geometries.append(("plane", np.linalg.inv(pose), size/2, calibration, name))
                elif geometry.find("mesh") is not None:
                    uri = geometry.findtext("mesh/uri", "")
                    source = (self.models / uri[len("model://"):]) if uri.startswith("model://") else folder / uri.removeprefix("file://")
                    scale = np.asarray([float(x) for x in geometry.findtext("mesh/scale", "1 1 1").split()])
                    if not np.isfinite(scale).all() or np.any(scale <= 0): raise RuntimeError("invalid visual mesh scale")
                    pose[:3,:3] = pose[:3,:3] @ np.diag(scale)
                    from vtkmodules.vtkIOGeometry import vtkSTLReader
                    from vtkmodules.vtkCommonDataModel import vtkStaticCellLocator
                    reader = vtkSTLReader();reader.SetFileName(str(source));reader.Update()
                    poly = reader.GetOutput()
                    if poly.GetNumberOfCells() == 0: raise RuntimeError(f"empty visual STL: {source}")
                    locator = vtkStaticCellLocator();locator.SetDataSet(poly);locator.BuildLocator()
                    self.geometries.append(("mesh", np.linalg.inv(pose), locator, calibration, name))
                    self.visual_meshes.append({"path": str(source.resolve()), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                                               "model_name": name, "pose_matrix": pose.tolist(), "triangles": poly.GetNumberOfCells()})
        self._walk(model, transform, folder, name)

    def first_hit(self, origin, ray, maximum_parameter=1000.0):
        np = self.np
        origin,ray = np.asarray(origin,dtype=float),np.asarray(ray,dtype=float)
        if not np.isfinite(origin).all() or not np.isfinite(ray).all() or np.linalg.norm(ray) <= 1e-12:
            return None
        closest = None
        for kind,inverse,data,calibration,name in self.geometries:
            p = inverse[:3,:3]@origin + inverse[:3,3]
            d = inverse[:3,:3]@ray
            hits = []
            if kind == "box":
                near,far = -math.inf,math.inf
                for axis in range(3):
                    if abs(d[axis]) < 1e-12:
                        if not -data[axis] <= p[axis] <= data[axis]: far=-math.inf;break
                    else:
                        a,b=(-data[axis]-p[axis])/d[axis],(data[axis]-p[axis])/d[axis]
                        near,far=max(near,min(a,b)),min(far,max(a,b))
                t=near if near>1e-8 else far
                if far>=max(0,near) and t>1e-8:hits.append(t)
            elif kind == "plane":
                if abs(d[2])>1e-12:
                    t=-p[2]/d[2]
                    if t>1e-8 and np.all(np.abs((p+t*d)[:2])<=data):hits.append(t)
            elif kind == "cylinder":
                radius,height=data
                a,b=float(d[:2]@d[:2]),float(2*p[:2]@d[:2]);disc=b*b-4*a*(p[:2]@p[:2]-radius*radius)
                if a>1e-12 and disc>=0:
                    for t in ((-b-math.sqrt(disc))/(2*a),(-b+math.sqrt(disc))/(2*a)):
                        if t>1e-8 and abs(p[2]+t*d[2])<=height:hits.append(t)
                if abs(d[2])>1e-12:
                    for z in (-height,height):
                        t=(z-p[2])/d[2]
                        if t>1e-8 and np.linalg.norm((p+t*d)[:2])<=radius:hits.append(t)
            else:
                from vtkmodules.vtkCommonCore import mutable
                t,sub,cell=mutable(0.0),mutable(0),mutable(0)
                if data.IntersectWithLine(p,p+d*maximum_parameter,1e-8,t,[0.,0.,0.],[0.,0.,0.],sub,cell):
                    if float(t)*maximum_parameter>1e-8:hits.append(float(t)*maximum_parameter)
            if hits and min(hits)<=maximum_parameter and (closest is None or min(hits)<closest["parameter"]):
                closest={"parameter":float(min(hits)),"model_name":name,"kind":kind,"calibration_surface":calibration}
        return closest


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--px4-root", type=Path, default=Path("/home/pawn/drone-simulation"))
    p.add_argument("--scenario", choices=("room", "two_rooms", "powerplant"), default="room")
    p.add_argument("--detection-mode", choices=("replay", "yolo", "none"), default=None)
    p.add_argument("--powerplant-asset-lock", type=Path, default=PACKAGE/"config/powerplant_assets.json")
    p.add_argument("--roi-access-test", action="store_true", help="powerplant only: overlay a task ROI outside the launch point")
    p.add_argument("--stop-after", choices=("scan", "roi-access", "mission"), default=None,
                   help="powerplant defaults to scan; regression rooms default to complete mission")
    p.add_argument("--render-context", choices=("auto", "display", "egl"), default="auto",
                   help="auto uses DISPLAY when available; egl explicitly requests headless rendering")
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
    p.add_argument("--explore-duration", type=float, default=60.0,
                   help="isolated exploration budget, independent from overall acceptance timeout")
    p.add_argument("--px4-hover-thrust", type=float, default=None,
                   help="explicit isolated MPC_THR_HOVER calibration; default retains the airframe value")
    p.add_argument("--px4-z-velocity-integral", type=float, default=None,
                   help="explicit isolated MPC_Z_VEL_I_ACC calibration; default retains the airframe value")
    p.add_argument("--controller-profile", type=Path, default=None,
                   help="explicit isolated controller YAML; frozen profile required for formal mission")
    p.add_argument("--takeoff-policy", choices=("legacy","A","B","C","D"),default=None)
    p.add_argument("--governor-ablation",action="store_true",
                   help="frozen-profile takeoff/scan comparison with reference governor disabled")
    p.add_argument("--controller-calibration-candidate", type=float, choices=controller_support.CANDIDATES, default=None,
                   help="formal nonzero integral calibration candidate; requires profile and stop-after scan")
    p.add_argument("--startup-timeout", type=float, default=90)
    p.add_argument("--ros-domain", type=int, default=25)
    p.add_argument("--agent-port", type=int, default=8889)
    p.add_argument("--inject-obstacle", action="store_true")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--prepare-only", action="store_true")
    args = p.parse_args()
    if args.detection_mode is None: args.detection_mode = "none" if args.scenario=="powerplant" else "replay"
    if args.stop_after is None: args.stop_after = "roi-access" if args.roi_access_test else ("scan" if args.scenario=="powerplant" else "mission")
    if args.roi_access_test and args.scenario != "powerplant": p.error("--roi-access-test requires --scenario powerplant")
    if args.scenario=="powerplant" and args.detection_mode=="replay":
        p.error("powerplant replay requires a verified annotation fixture from the actual STL; the regression cylinder replay is forbidden (use none or yolo)")
    if args.scenario!="powerplant" and args.stop_after!="mission": p.error("limited-stage acceptance requires --scenario powerplant")
    if not 0 <= args.ros_domain <= 232 or not 1024 <= args.agent_port <= 65535:
        p.error("choose a ROS domain in [0,232] and an unprivileged UDP port")
    if args.agent_port in (14550, 18570):
        p.error("the Agent port must not overlap the isolated SITL MAVLink/GCS ports")
    if (args.timeout <= 0 or args.startup_timeout <= 0
            or not math.isfinite(args.explore_duration) or args.explore_duration<=0):
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
    if args.px4_hover_thrust is not None and not (math.isfinite(args.px4_hover_thrust) and 0<args.px4_hover_thrust<1):
        p.error("hover thrust must be finite and between zero and one")
    if args.px4_z_velocity_integral is not None and not (math.isfinite(args.px4_z_velocity_integral) and args.px4_z_velocity_integral>=0):
        p.error("vertical velocity integral must be finite and nonnegative")
    if args.controller_profile is not None:
        if args.px4_hover_thrust is not None or args.px4_z_velocity_integral is not None:
            p.error("controller profiles cannot be combined with diagnostic PX4 parameter overrides")
        if args.scenario!="powerplant" or args.image_chain_only:
            p.error("controller calibration/profile is restricted to the actual powerplant flight")
        if args.controller_calibration_candidate is not None and args.stop_after!="scan":
            p.error("calibration candidates require --stop-after scan")
        inherited=[name for name in os.environ if name.startswith("PX4_PARAM_")]
        if inherited: p.error("profile rejects inherited controller parameter overrides: "+", ".join(inherited))
    elif args.controller_calibration_candidate is not None:
        p.error("calibration candidate requires --controller-profile")
    if args.governor_ablation and (args.controller_profile is None or
            args.controller_calibration_candidate is not None or args.stop_after!='scan'):
        p.error('governor ablation requires a frozen profile and scan scope')
    if args.image_chain_only and args.inject_obstacle:
        p.error("--image-chain-only does not execute a flight or inject obstacles")
    if args.output_dir is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        args.output_dir = WORKSPACE / "log" / f"p2_sitl_{args.scenario}_{stamp}"
    args.output_dir = args.output_dir.expanduser().resolve()
    args.px4_root = args.px4_root.expanduser().resolve()
    args.powerplant_asset_lock = args.powerplant_asset_lock.expanduser().resolve()
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


def configure_resource_paths(env, local_models, models, world_directory):
    """Use the same overlay precedence for Gazebo and nested SDF includes."""
    paths=[str(local_models) if local_models else "",str(models)]
    env["GZ_SIM_RESOURCE_PATH"]=os.pathsep.join(x for x in
        (*paths,str(world_directory),env.get("GZ_SIM_RESOURCE_PATH","")) if x)
    # libsdformat consults SDF_PATH for nested model:// includes. Omitting the
    # overlay here silently loads the original camera despite spawning the
    # copied CPU-profile vehicle and setting GZ_SIM_RESOURCE_PATH correctly.
    env["SDF_PATH"]=os.pathsep.join(x for x in (*paths,env.get("SDF_PATH","")) if x)


def sensor_profile_evidence(metadata, observed):
    """Bind reported load to actual image geometry and received exposure rates.

    This checks the observed ROS stream; it does not prove renderer adapter or
    distinguish every dropped intermediate Gazebo exposure. At least five
    distinct exposures and a rate within 20% are required for each sensor.
    """
    configured=metadata.get("configured_sensors",{})
    checks={};rates={}
    for name in ("rgb","depth","lidar"):
        expected=configured.get(name,{})
        actual=observed.get(name,{})
        count=actual.get("unique_exposures",0)
        checks[name+"_enough_exposures"]=(isinstance(count,int) and count>=5)
        if name=="lidar":
            samples=expected.get("horizontal",{}).get("samples")
            checks[name+"_dimensions_match"]=(samples is not None
                and actual.get("range_samples")==int(samples))
        else:
            checks[name+"_dimensions_match"]=all(expected.get(key) is not None
                and expected.get(key)==actual.get(key) for key in ("width","height"))
        rate=actual.get("observed_unique_exposure_rate_sim_hz")
        target=expected.get("configured_update_rate_hz")
        finite=lambda value:isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
        checks[name+"_exposure_rate_matches"]=(finite(rate) and finite(target)
            and target>0 and abs(rate-target)<=.20*target)
        rates[name]={"expected_sim_hz":target,"observed_sim_hz":rate}
    return {"passed":all(checks.values()),"checks":checks,"rates":rates,
        "scope":"observed ROS dimensions and unique-exposure rates; no renderer or real-time claim",
        "rate_relative_tolerance":.20,"minimum_unique_exposures":5}


def verified_body_envelope(models, parameters, audit_path):
    audit=json.loads(audit_path.read_text())
    expected={"x500_depth","x500","x500_base","OakD-Lite"}
    hashes=audit.get("source_sdf_sha256",{})
    if {Path(path).parent.name for path in hashes} != expected:
        raise RuntimeError("incomplete SDF body audit; rerun validation/model/body_envelope.py")
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
    world = scenario_world(args)
    ET.parse(world)
    asset_audit = read_asset_lock(args.powerplant_asset_lock,
        args.px4_root/"Tools/simulation/gz",world) if args.scenario=="powerplant" else None
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    runtime = output / "runtime" / f"run-{time.time_ns()}"
    runtime.mkdir(parents=True, exist_ok=True)
    local_models,sensor_metadata = sensor_profile(args,runtime)
    build = args.px4_root / "build/px4_sitl_default"
    binary = build / "bin/px4"
    config = yaml.safe_load((WORKSPACE / "src/px4_powerplant_mission/config/powerplant.yaml").read_text())
    mission = config["powerplant_offboard_mission"]["ros__parameters"]
    if getattr(args,"controller_profile",None) is not None:
        selected=controller_support.load_profile(args.controller_profile,args.controller_calibration_candidate)
        stored=selected.get('selected_transition_policy','legacy')
        policy=(args.takeoff_policy or 'A') if args.controller_calibration_candidate is not None else stored
        if args.controller_calibration_candidate is None and args.takeoff_policy not in (None,stored):
            raise ValueError('frozen transition policy cannot be overridden')
        selected['transition_policy']=policy
        model=controller_support.derive_model(args.px4_root/"Tools/simulation/gz/models",world,selected["expected_readback"])
        args.resolved_controller_profile=controller_support.resolve_profile(selected,model)
        mission["sitl_controller_profile_id"]=args.resolved_controller_profile["profile_id"]
        (output/"sitl_controller_derivation.json").write_text(json_dumps(args.resolved_controller_profile,indent=2,allow_nan=False))
    motion_profile = yaml.safe_load((WORKSPACE/"src/px4_powerplant_mission/config/p2_simulation.yaml").read_text())
    profile = motion_profile["powerplant_offboard_mission"]["ros__parameters"]
    body_audit=verified_body_envelope(args.px4_root/"Tools/simulation/gz/models",{**mission,**profile},
                                      PACKAGE/"research_sources/sitl_body_envelope.json")
    mission.update({
        "auto_start": False, "takeoff_region_confirmed": args.scenario!="powerplant",
        "takeoff_region_radius_m": 1.5,
        "takeoff_region_height_m": 6.2 if args.scenario=="powerplant" else 4.2,
        "takeoff_height_m": 5.0 if args.scenario=="powerplant" else 3.0,
        "return_home_height_m": 5.0 if args.scenario=="powerplant" else 3.0,
        "inspection_altitude_m": 5.0 if args.scenario=="powerplant" else 3.0,
        "inspection_photo_dir": str(output / "photos"),
        "explore_task_duration_s": args.explore_duration, "initial_scan_timeout_s": 100.0,
        "px4_timestamp_domain": "ros",
    })
    if args.scenario != "powerplant":
        mission.update({"explore_min_x_m": -5.5,
            "explore_max_x_m": 9.5 if args.scenario == "two_rooms" else 5.5,
            "explore_min_y_m": -5.5, "explore_max_y_m": 5.5,
            "explore_min_z_m": 1.0, "explore_max_z_m": 6.0})
    elif args.roi_access_test:
        mission.update({"explore_min_x_m":12.0,"explore_max_x_m":24.0,
            "explore_min_y_m":-9.0,"explore_max_y_m":12.0,
            "explore_min_z_m":3.5,"explore_max_z_m":7.0})
    mapper = config["powerplant_voxel_mapper"]["ros__parameters"]
    # This isolated lockstep simulator and PX4 share the Gazebo boot clock.
    # Hardware retains system↔boot synchronization in the shared defaults.
    config["powerplant_localization"]["ros__parameters"]["px4_timestamp_domain"] = "ros"
    config["powerplant_external_vision_bridge"]["ros__parameters"]["px4_timestamp_domain"] = "ros"
    if args.scenario != "powerplant":
        mapper.update({"grid_min_x_m": -6.0, "grid_max_x_m": 10.0,
                       "grid_min_y_m": -6.0, "grid_max_y_m": 6.0,
                       "grid_z_min_m": 1.0, "grid_z_max_m": 6.0})
    takeoff_audit = None
    ground_support_model = None
    contact_instrumentation = None
    takeoff_audit_path = output / "takeoff_domain_audit.json"
    if args.scenario=="powerplant":
        spec=importlib.util.spec_from_file_location("powerplant_takeoff_audit",
            VALIDATION_ROOT/"model/takeoff_audit.py")
        takeoff_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(takeoff_module)
        takeoff_audit=takeoff_module.audit_takeoff_domain(world,
            args.px4_root/"Tools/simulation/gz/models",{**mission,**profile},home=(0,0,.1))
        if not takeoff_audit["passed"]:
            takeoff_audit_path.write_text(json_dumps(takeoff_audit,indent=2,allow_nan=False))
            failed=[name for name,passed in takeoff_audit["checks"].items() if not passed]
            raise RuntimeError("real powerplant takeoff domain is unproven: "+", ".join(failed))
        # The boolean is granted only after the actual source/collision scene
        # has a sufficient separation proof for this operating column.
        mission["takeoff_region_confirmed"]=True
        ground_support_model=takeoff_module.generate_ground_support_model(world,
            args.px4_root/"Tools/simulation/gz/models",PACKAGE/"research_sources/sitl_body_envelope.json",
            {**mission,**profile},home=(0,0,.1),floor_uncertainty_m=.01)
        (output/"ground_support_model.json").write_text(json_dumps(ground_support_model,indent=2,allow_nan=False))
        if not (ground_support_model.get("passed") and ground_support_model.get("verified")):
            raise RuntimeError("actual world/body ground support model is unproven")
        mission.update(ground_support_model["ground_support_parameters"])
        if not args.image_chain_only:
            local_models=local_models or runtime/"models"
            contact_instrumentation=contact_support.instrument_vehicle_contacts(
                args.px4_root/"Tools/simulation/gz/models",local_models,output,world,
                ground_support_model["support_collision_names"],
                support_surfaces=ground_support_model["support_surfaces"],
                support_region=ground_support_model["support_region"])
            effective_camera=local_models/"OakD-Lite/model.sdf"
            effective_vehicle=local_models/"x500_depth/model.sdf"
            sensor_metadata.update(parameter_source=str(effective_camera),
                copied_model_directory=str(effective_camera.parent),
                effective_model_sdf_sha256=hashlib.sha256(effective_camera.read_bytes()).hexdigest())
            sensor_metadata["vehicle_model"].update(effective_directory=str(effective_vehicle.parent),
                effective_model_sdf_sha256=hashlib.sha256(effective_vehicle.read_bytes()).hexdigest())
    # The ordinary simulation launch has no independently audited support
    # model. Grant response mode only in this derived, hash-bound profile.
    audited_support = bool(ground_support_model and ground_support_model.get("verified")
                           and ground_support_model.get("passed"))
    profile["takeoff_response_sync_enabled"] = audited_support
    profile["takeoff_transition_budget_enabled"] = audited_support
    policy=getattr(args,'resolved_controller_profile',{}).get('transition_policy',getattr(args,'takeoff_policy',None) or 'legacy')
    if policy!='legacy':
        if not audited_support:raise ValueError('transition policy requires actual audited powerplant support')
        profile.update(takeoff_transition_policy_id=policy,takeoff_transition_hard_limit_s=45.,
            motion_rounded_envelope_enabled=True,
            takeoff_reference_governor_enabled=not getattr(args,'governor_ablation',False))
        if policy=='D':
            verified_body_envelope(args.px4_root/'Tools/simulation/gz/models',
                {**mission,**profile,'motion_body_radius_m':.48},PACKAGE/'research_sources/sitl_body_envelope.json')
    motion_profile_path = runtime / "p2_sitl_motion.yaml"
    motion_profile_path.write_text(yaml.safe_dump(motion_profile, sort_keys=False))
    args.resolved_motion_profile_path = motion_profile_path
    if getattr(args,"resolved_controller_profile",None) is not None:
        args.resolved_controller_profile["acceptance_policy"]=controller_support.validate_acceptance_policy({**mission,**profile})
    config_path = runtime / "powerplant_p2_sitl.yaml"
    config_path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    if takeoff_audit is not None:
        takeoff_module.bind_actual_configuration(takeoff_audit,config_path,{**mission,**profile})
        takeoff_audit["motion_profile_config_sha256"]=hashlib.sha256(
            (WORKSPACE/"src/px4_powerplant_mission/config/p2_simulation.yaml").read_bytes()).hexdigest()
        takeoff_audit["resolved_motion_profile_config"] = str(motion_profile_path)
        takeoff_audit["resolved_motion_profile_sha256"] = hashlib.sha256(motion_profile_path.read_bytes()).hexdigest()
        takeoff_audit_path.write_text(json_dumps(takeoff_audit,indent=2,allow_nan=False))
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
        "POWERPLANT_SITL_EFFECTIVE_CONFIG": str(config_path),
        "POWERPLANT_SITL_MOTION_PROFILE": str(motion_profile_path),
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
    configure_resource_paths(env,local_models,args.px4_root/"Tools/simulation/gz/models",world.parent)
    plugins = str(build / "src/modules/simulation/gz_plugins")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        x for x in (plugins, env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")) if x)
    render_context=resolve_render_context(args.render_context,env)
    gz = ["gz", "sim", "-r", "-s"]
    if render_context=="egl": gz.append("--headless-rendering")
    gz.append(str(world))
    for name,value in (getattr(args,"resolved_controller_profile",{}).get("requested_parameters",{})).items():
        env["PX4_PARAM_"+name]=str(value)
    for name,value in (("MPC_THR_HOVER",args.px4_hover_thrust),
                       ("MPC_Z_VEL_I_ACC",args.px4_z_velocity_integral)):
        if value is not None: env["PX4_PARAM_"+name]=str(value)
    commands = [
        ("agent", ["MicroXRCEAgent", "udp4", "-p", str(args.agent_port)], runtime),
        ("gazebo", gz, runtime),
        ("px4", [str(binary), "-d", "-i", "0", "-w", str(rootfs), str(build / "etc")], rootfs),
        ("gcs", [sys.executable, str(Path(__file__).resolve().parent / "gcs_heartbeat.py"),
                 "--output-dir", str(output), "--startup-timeout", str(args.startup_timeout)], runtime),
        ("sensors", ["ros2", "launch", "px4_powerplant_mission", "gz_sensor_bridge.launch.py"], runtime),
        ("mission", ["ros2", "launch", "px4_powerplant_mission", "powerplant_system.launch.py",
                     f"config_file:={config_path}", f"motion_config_file:={motion_profile_path}", "localization_profile:=simulation",
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
    if args.scenario=="powerplant" and importlib.util.find_spec("vtkmodules") is None:
        missing.append("Python module vtkmodules (offline mesh evaluator only)")
    manifest = {
        "scenario": args.scenario, "detection_mode": args.detection_mode,
        "acceptance_scope":acceptance_scope(args),
        "full_scene_exploration_acceptance":False,
        "stop_after":args.stop_after,"roi_access_test_overlay":args.roi_access_test,
        "powerplant_asset_audit":asset_audit,
        "render_context":{"requested":args.render_context,"selected":render_context,
            "display":env.get("DISPLAY"),"gpu_device_present":Path("/dev/dxg").exists(),
            "hardware_acceleration_confirmed":False,
            "scope":"render context selection only; actual adapter must be confirmed from Gazebo/Ogre logs"},
        "sensor_profile":sensor_metadata,
        "body_envelope_audit":body_audit,
        "takeoff_geometry_audit":None if takeoff_audit is None else {
            "path":str(takeoff_audit_path),"passed":True,
            "sha256":hashlib.sha256(takeoff_audit_path.read_bytes()).hexdigest(),
            "world_sha256":takeoff_audit["world_sha256"],
            "actual_config_sha256":takeoff_audit["actual_config_sha256"],
            "evidence_prism":takeoff_audit["evidence_prism"],
            "operating_column":takeoff_audit["operating_column"],
            "navigation_error_envelope_proven":False,"actual_startup_response_proven":False},
        "synthetic_detection_replay": args.detection_mode == "replay",
        "world": str(world), "world_sha256":hashlib.sha256(world.read_bytes()).hexdigest(),
        "config": str(config_path), "motion_profile_config":str(motion_profile_path), "rootfs": str(rootfs),
        "task_roi":{axis:[mission[f"explore_min_{axis}_m"],mission[f"explore_max_{axis}_m"]]
                    for axis in "xyz"},
        "ros_domain": args.ros_domain, "agent_port": args.agent_port,
        "gcs_link": {"listen_endpoint": ["127.0.0.1", 14550], "expected_px4_endpoint": ["127.0.0.1", 18570],
                     "verified_peer_required_before_transmit": True, "heartbeat_period_wall_s": 1.0,
                     "read_only_heartbeat_polling": {"command":"MAV_CMD_REQUEST_MESSAGE",
                         "message":"HEARTBEAT", "period_wall_s":1.0,
                         "changes_stream_interval":False,"ack_establishes_health":False,
                         "scope":"request the actual PX4 stream during slow lockstep; telemetry only"},
                     "expected_px4_NAV_DLL_ACT": 2,
                     "px4_datalink_checks_disabled": False, "sends_arming_mode_or_trajectory_commands": False,
                     "health_file": str(output / "gcs_health.json"),
                     "heartbeat_evidence_file": str(output / "gcs_heartbeats.jsonl")},
        "gz_partition": env["GZ_PARTITION"], "missing_dependencies": missing,
        "commands": [{"name": name, "argv": argv, "cwd": str(cwd)} for name, argv, cwd in commands],
        "ground_support_model":ground_support_model,
        "contact_instrumentation":contact_instrumentation,
        "isolated_exploration_budget_s":args.explore_duration,
        "isolated_controller_profile":controller_profile_evidence(args,{}),
        "resolved_controller_profile":getattr(args,"resolved_controller_profile",None),
        "runtime_identity":capture_runtime_identity(args,config_path,local_models,asset_audit),
        "test_only_confirmation": {"radius_m": mission["takeoff_region_radius_m"],
            "height_m":mission["takeoff_region_height_m"],"relative_takeoff_m":mission["takeoff_height_m"]},
        "takeoff_startup_configuration": takeoff_startup_evidence({**mission, **profile}, {}),
    }
    (output / "manifest.json").write_text(json_dumps(manifest, ensure_ascii=False, indent=2))
    return env, commands, manifest


class RendererLogCapture:
    """Attribute the process-global Ogre log to this run's live writer."""
    def __init__(self, source, output, env):
        self.source, self.output = Path(source), Path(output)
        self.started_ns = time.time_ns()
        self.requested_adapter = {key: env[key] for key in
            ("GALLIUM_DRIVER", "MESA_D3D12_DEFAULT_ADAPTER_NAME") if key in env}

    def finish(self, expected_pgid):
        result = {"status": "UNCONFIRMED", "log_provenance_confirmed": False,
                  "hardware_acceleration_confirmed": False,
                  "requested_adapter_environment": self.requested_adapter,
                  "log_writers": [], "renderer": None, "vendor": None,
                  "active_context_api": None}
        try:
            stat = self.source.stat()
            content = self.source.read_text(errors="replace")
        except OSError:
            return result
        # A requested adapter, a stale global file, or a different GUI's log
        # cannot establish which backend rendered this isolated server.
        if stat.st_mtime_ns < self.started_ns or expected_pgid is None:
            return result
        identity = (stat.st_dev, stat.st_ino)
        for process in Path("/proc").iterdir():
            if not process.name.isdigit():
                continue
            try:
                for fd in (process / "fd").iterdir():
                    try:
                        observed = fd.stat()
                        if (observed.st_dev, observed.st_ino) != identity:
                            continue
                        info = (process / "fdinfo" / fd.name).read_text()
                        flags = next(int(line.split()[1], 8) for line in info.splitlines()
                                     if line.startswith("flags:"))
                        if flags & os.O_ACCMODE not in (os.O_WRONLY, os.O_RDWR):
                            continue
                        result["log_writers"].append({"pid": int(process.name),
                            "process_group": os.getpgid(int(process.name))})
                    except (OSError, StopIteration, ValueError):
                        continue
            except OSError:
                continue
        writers = result["log_writers"]
        if not writers or any(row["process_group"] != expected_pgid for row in writers):
            return result
        for line in content.splitlines():
            for marker, field in (("GL_RENDERER =", "renderer"), ("GL_VENDOR =", "vendor")):
                if marker in line:
                    result[field] = line.split(marker, 1)[1].strip()
            if "Starting GLX Subsystem" in line:
                result["active_context_api"] = "GLX"
            elif "Starting EGL Subsystem" in line:
                result["active_context_api"] = "EGL"
        if not result["renderer"]:
            return result
        lower = result["renderer"].lower()
        software = any(word in lower for word in
            ("llvmpipe", "softpipe", "swrast", "software", "swiftshader", "warp", "basic render"))
        hardware = not software and any(word in lower for word in
            ("nvidia", "geforce", "radeon", "intel", "quadro"))
        self.output.mkdir(parents=True, exist_ok=True)
        destination = self.output / "ogre2.log"
        destination.write_bytes(self.source.read_bytes())
        result.update(status="HARDWARE" if hardware else "SOFTWARE" if software else "IDENTIFIED",
                      log_provenance_confirmed=True,
                      hardware_acceleration_confirmed=hardware,
                      log_copy=str(destination),
                      log_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                      scope="live server process-group log identity; not throughput or flight acceptance")
        return result


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
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import TrajectorySetpoint, VehicleStatus, VehicleLandDetected
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
            self.status_history=[]
            self.contact_lock=threading.Lock()
            self.contact_pending=deque()
            self.contact_pending_dropped=0
            self.contact_callback_counts={}
            self.world_step_rows=[]
            self.contact_file=(args.output_dir/"contacts.jsonl").open("w")
            instrumentation_path=args.output_dir/"contact_instrumentation.json"
            self.contact_evidence=None
            self.ground_support_model=None
            self.contact_subscriptions=[]
            if instrumentation_path.exists():
                instrumentation=json.loads(instrumentation_path.read_text())
                support_model=json.loads((args.output_dir/"ground_support_model.json").read_text())
                self.ground_support_model=support_model
                self.contact_evidence=contact_support.ContactEvidence(instrumentation,
                    float(support_model["plane_z_enu_m"]),float(support_model["plane_uncertainty_m"]))
                manifest=json.loads((args.output_dir/"manifest.json").read_text())
                self.contact_evidence.advertised=set(manifest.get("discovered_gazebo_topics",[]))
            self.pending_explore_pairs=deque()
            self.exploration_pairs=[]
            self.exploration_pair_drops=0
            self.explore_depth_baselines={}
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
            profile_path = Path(processes.env.get("POWERPLANT_SITL_MOTION_PROFILE", ""))
            if not profile_path.is_file():
                raise RuntimeError("resolved SITL motion profile is missing")
            profile = yaml.safe_load(profile_path.read_text())["powerplant_offboard_mission"]["ros__parameters"]
            shared = yaml.safe_load((WORKSPACE / "src/px4_powerplant_mission/config/powerplant.yaml").read_text())["powerplant_offboard_mission"]["ros__parameters"]
            effective_config=processes.env.get("POWERPLANT_SITL_EFFECTIVE_CONFIG")
            effective=yaml.safe_load(Path(effective_config).read_text())["powerplant_offboard_mission"]["ros__parameters"] if effective_config else shared
            self.motion_parameters = {**effective,**profile}
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
            # Offline truth is resolved from exactly the same world as Gazebo.
            # The original visual STL (not the simplified collision mesh) is
            # projected for sensor/extrinsic checks and never fed to the mapper.
            self.visual_scene = VisualSceneEvaluator(
                scenario_world(args), args.px4_root / "Tools/simulation/gz/models")
            self.limited_stage_reached = False
            self.limited_stage_evidence = None
            self.stage_access_anchor = None
            self.stage_land_future = None
            self.stage_landing_accepted = False
            self.raw_vehicle_status = self.raw_landed = None
            self.raw_landed_history=[]
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
            status_topic = self.motion_parameters["vehicle_status_topic"]
            if status_topic=="/fmu/out/vehicle_status" and getattr(VehicleStatus,"MESSAGE_VERSION",0)>0:
                status_topic+=f"_v{VehicleStatus.MESSAGE_VERSION}"
            land_topic = self.motion_parameters["vehicle_land_detected_topic"]
            if land_topic=="/fmu/out/vehicle_land_detected" and getattr(VehicleLandDetected,"MESSAGE_VERSION",0)>0:
                land_topic+=f"_v{VehicleLandDetected.MESSAGE_VERSION}"
            self.create_subscription(VehicleStatus,status_topic,self.raw_status,qos_profile_sensor_data)
            self.create_subscription(VehicleLandDetected,land_topic,self.raw_land,qos_profile_sensor_data)
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
            if self.contact_evidence:
                if not self.gz_truth_node.subscribe(WorldStatistics,"/world/powerplant/stats",self.world_statistics_sample):
                    raise RuntimeError("cannot subscribe to actual world iteration evidence")
                for sensor in self.contact_evidence.instrumentation["sensors"]:
                    topic=sensor["topic"]
                    if not self.gz_truth_node.subscribe(Contacts,topic,partial(self.contact_sample,topic)):
                        raise RuntimeError("cannot subscribe to evaluation contact sensor: "+topic)
                    self.contact_subscriptions.append(topic)
                    self.contact_evidence.subscribed.add(topic)
            if args.detection_mode == "replay":
                self.source_pub = self.create_publisher(Image, "/powerplant/perception/yolo_source_image", reliable)
                self.detection_pub = self.create_publisher(String, "/powerplant/perception/yolo_detections", reliable)
            self.start_client = self.create_client(Trigger, "/start_powerplant_mission")
            self.land_client = self.create_client(Trigger, "/land_powerplant_mission")

        def world_statistics_sample(self,message):
            row={"sample_ns":int(message.sim_time.sec)*1_000_000_000+int(message.sim_time.nsec),
                 "iterations":int(message.iterations),"paused":bool(message.paused),"receipt_wall":time.monotonic()}
            with self.contact_lock:self.world_step_rows.append(row)

        def contact_sample(self,topic,message):
            rows=contact_support.contact_packet(message,topic,time.monotonic(),self.clock_ns)
            if self.clock_ns is None and not self.started:
                for row in rows:row["before_monitor_clock"]=True
            with self.contact_lock:
                self.contact_callback_counts[topic]=self.contact_callback_counts.get(topic,0)+1
                for row in rows:
                    if len(self.contact_pending)>=65536:self.contact_pending_dropped+=1
                    else:self.contact_pending.append(row)

        def consume_contacts(self):
            with self.contact_lock:
                pending=list(self.contact_pending);self.contact_pending.clear()
                counts=dict(self.contact_callback_counts)
                steps=list(self.world_step_rows)
            if self.contact_evidence:
                self.contact_evidence.observe(pending)
                self.contact_evidence.callback_counts=counts
                self.contact_evidence.world_steps=steps
            for row in pending:self.contact_file.write(json_dumps(row)+"\n")
            if pending:self.contact_file.flush()

        def consume_exploration_pairs(self):
            # A delayed truth sample is allowed to arrive before judging the
            # pair. Neither a new phase nor a new epoch may lend it progress.
            remaining=deque()
            for row in self.pending_explore_pairs:
                truth=self.pose_at(row["sample_ns"],self.truth_poses)
                if truth is None:
                    if self.truth_poses and row["sample_ns"]<self.truth_poses[0][0]:
                        self.exploration_pair_drops+=1
                    else:remaining.append(row)
                    continue
                if (row["covariance_valid"] and row.get("clock_sample_age_s") is not None
                        and -.05<=row["clock_sample_age_s"]<=self.reaction_budget
                        and row.get("clock_receipt_age_wall_s") is not None
                        and 0<=row["clock_receipt_age_wall_s"]<=self.reaction_budget):
                    self.exploration_pairs.append({"sample_ns":row["sample_ns"],
                        "navigation":row["position"],"truth":truth[0].tolist(),
                        "identity":row["task_identity"],"depth_base":row["explore_depth_base"]})
                else:self.exploration_pair_drops+=1
            self.pending_explore_pairs=remaining

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
            self.status_history.append(state)
            identity=(state.get("localization_epoch"),state.get("task_roi_revision"))
            if state.get("phase")=="EXPLORE" and identity not in self.explore_depth_baselines:
                self.explore_depth_baselines[identity]=state.get("map_fusion_depth_count",0)
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
                covariance_valid=bool(frame_valid and valid_covariance(pc)
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
                row["task_identity"]=(self.latest.get("localization_epoch"),self.latest.get("task_roi_revision"))
                row["explore_depth_base"]=self.explore_depth_baselines.get(tuple(row["task_identity"]),0)
                self.navigation_rows.append(row)
                if row["execution_active"] and row["phase"]=="EXPLORE":
                    if len(self.pending_explore_pairs)>=360:
                        self.pending_explore_pairs.popleft();self.exploration_pair_drops+=1
                    self.pending_explore_pairs.append(row)
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
            policy_schedule=controller_support.tracking_policy_schedule(self.status_history,self.motion_parameters)
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
                position_tolerance,policy_identity=controller_support.tracking_policy_at(
                    policy_schedule,sample,row['task_identity'][0],self.motion_parameters)
                pb=(position_tolerance+self.sigma_multiplier*row["position_sigma_certificate_m"]) if row["covariance_valid"] else None
                vb=(self.velocity_tolerance+self.sigma_multiplier*row["velocity_sigma_certificate_mps"]) if row["covariance_valid"] else None
                position_axis_bound=(position_tolerance+self.sigma_multiplier*np.asarray(row["position_axis_sigma_certificate_m"])) if row["covariance_valid"] else None
                velocity_axis_bound=(self.velocity_tolerance+self.sigma_multiplier*np.asarray(row["velocity_axis_sigma_certificate_mps"])) if row["covariance_valid"] else None
                position_axes_accepted=(position_axis_bound is not None and bool(np.all(np.abs(position_error)<=position_axis_bound)))
                velocity_axes_accepted=(velocity_axis_bound is not None and bool(np.all(np.abs(velocity_error)<=velocity_axis_bound)))
                certificate={"sample_ns":sample,"wall":row["wall"],"phase":row["phase"],
                    "position_tolerance_m":position_tolerance,"tracking_policy_id":policy_identity,
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
                        and gcs["px4_datalink_policy_verified"] and startup["passed"]
                        and controller_profile_evidence(args,gcs.get("read_only_px4_parameters",{}))["passed"]):
                    return
                self.event_file.write(json_dumps({"event":"test_start_readiness_dwell","wall":now,
                    "localization_epoch":self.start_ready_epoch,
                    "takeoff_startup_validation":startup,
                    "ready_sim_s":(self.clock_ns-self.start_ready_clock_ns)*1e-9,
                    "ready_wall_s":now-self.start_ready_wall})+"\n")
                self.event_file.flush()
                self.start_future = self.start_client.call_async(Trigger.Request())

        def stop_at_requested_stage(self):
            if (args.image_chain_only or args.scenario!="powerplant"
                    or args.stop_after=="mission" or not self.started):
                return
            nav_distance = truth_distance = 0.0
            active_access = (self.latest.get("phase")=="EXPLORE"
                and self.latest.get("explore_roi_mode")=="ROI_ACCESS"
                and self.latest.get("motion_mode")=="TRACKING"
                and self.latest.get("localization_ready") and self.execution_active())
            paired_sample = None
            if args.stop_after=="roi-access" and active_access:
                now=time.monotonic()
                status_sample=self.latest.get("odom_sample_stamp_ns")
                for row in reversed(self.navigation_rows):
                    if now-row["wall"]>self.reaction_budget: break
                    if (not row["covariance_valid"] or self.clock_ns is None
                            or type(status_sample) is not int or row["sample_ns"]<status_sample
                            or not -.05<=(self.clock_ns-row["sample_ns"])*1e-9<=self.reaction_budget):
                        continue
                    truth=self.pose_at(row["sample_ns"],self.truth_poses)
                    if truth is not None:
                        paired_sample={"sample_ns":row["sample_ns"],"navigation":np.asarray(row["position"]),
                                       "truth":truth[0],"wall":now}
                        break
                identity=(self.latest.get("localization_epoch"),self.latest.get("task_roi_revision"))
                self.stage_access_anchor,nav_distance,truth_distance=access_motion_progress(
                    self.stage_access_anchor,paired_sample,identity,active_access)
            elif not self.limited_stage_reached:
                # Motion in another phase, epoch or task must not satisfy this
                # access experiment when the mode later becomes active again.
                self.stage_access_anchor=None
            if not self.limited_stage_reached:
                evidence=limited_stage_progress(args.stop_after,self.phases,self.events,
                    self.latest,nav_distance,truth_distance)
                if evidence["passed"]:
                    self.limited_stage_reached=True
                    self.limited_stage_evidence={**evidence,"wall":time.monotonic(),
                        "phase":self.latest.get("phase"),"roi_mode":self.latest.get("explore_roi_mode"),
                        "paired_navigation_truth_sample_ns":paired_sample["sample_ns"] if paired_sample else None,
                        "access_anchor_sample_ns":self.stage_access_anchor["sample_ns"] if self.stage_access_anchor else None}
                    self.event_file.write(json_dumps({"event":"test_limited_stage_reached",**self.limited_stage_evidence})+"\n")
                    self.event_file.flush()
            if self.limited_stage_reached and not self.stage_landing_accepted:
                if self.stage_land_future is not None:
                    if self.stage_land_future.done():
                        response=self.stage_land_future.result()
                        if response is None or not response.success:
                            raise RuntimeError("limited-stage landing request was rejected")
                        self.stage_landing_accepted=True
                elif self.land_client.service_is_ready():
                    self.stage_land_future=self.land_client.call_async(Trigger.Request())

        def raw_status(self, message):
            sample=int(message.timestamp)*1000
            if sample>0 and (self.raw_vehicle_status is None or sample>self.raw_vehicle_status["sample_ns"]):
                self.raw_vehicle_status={"sample_ns":sample,"receipt_wall":time.monotonic(),
                                         "arming_state":int(message.arming_state)}

        def raw_land(self, message):
            sample=int(message.timestamp)*1000
            if sample>0 and (self.raw_landed is None or sample>self.raw_landed["sample_ns"]):
                self.raw_landed={"sample_ns":sample,"receipt_wall":time.monotonic(),"landed":bool(message.landed)}
                self.raw_landed_history.append(self.raw_landed)

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
            self.consume_exploration_pairs()

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
            hit = self.visual_scene.first_hit(origin, ray)
            # Non-wall regression fixtures remain occluders. Real electric
            # plant surfaces are eligible calibration geometry in their world.
            return hit["parameter"] if hit and hit["calibration_surface"] else None

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
            self.consume_contacts()
            self.consume_exploration_pairs()
            contacts=self.contact_evidence.result(self.status_history,self.events) if self.contact_evidence else None
            handoff=None
            if contacts is not None:
                contacts["pending_queue_overflow"]=self.contact_pending_dropped
                contacts["passed"]=bool(contacts["passed"] and self.contact_pending_dropped==0)
                (args.output_dir/"world_steps.json").write_text(json_dumps(self.contact_evidence.world_steps,indent=2,allow_nan=False))
                (args.output_dir/"physical_contacts.json").write_text(json_dumps(contacts,indent=2,allow_nan=False))
                handoff=contact_support.takeoff_handoff_evidence(self.status_history,self.events,
                    self.raw_landed_history,self.setpoints,self.ground_support_model,self.motion_parameters,self.navigation_rows)
                (args.output_dir/"takeoff_ground_handoff.json").write_text(json_dumps(handoff,indent=2,allow_nan=False))
            self.truth_file.flush()
            truth_rows=[json.loads(line) for line in (args.output_dir/"truth.jsonl").read_text().splitlines() if line.strip()]
            flight=flight_support.physical_flight_evidence(self.navigation_rows,truth_rows,self.status_history,self.motion_parameters)
            phase_policy=controller_support.phase_policy_evidence(self.status_history,self.motion_parameters)
            (args.output_dir/"physical_flight_evidence.json").write_text(json_dumps(flight,indent=2,allow_nan=False))
            progress=contact_support.exploration_progress_evidence(self.exploration_pairs,self.events,
                (self.latest.get("localization_epoch"),self.latest.get("task_roi_revision")))
            progress["unpaired_or_invalid_samples"]=self.exploration_pair_drops
            (args.output_dir/"exploration_progress.json").write_text(json_dumps(progress,indent=2,allow_nan=False))
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
            limited = args.scenario == "powerplant" and args.stop_after != "mission"
            if limited:
                required = ["START", "SAFE_TAKEOFF", "HOVER_SCAN", "EXPLORE", "LANDING", "COMPLETE"]
            positions = [ordered.index(p) for p in required if p in ordered]
            truth_p95 = float(np.percentile(self.depth_truth_residuals,95)) if self.depth_truth_residuals else math.inf
            nav_p95 = float(np.percentile(self.depth_navigation_residuals,95)) if self.depth_navigation_residuals else math.inf
            reference_p95 = float(np.percentile(self.reference_errors,95)) if self.reference_errors else math.inf
            model_mean = float(np.mean(self.reference_errors)) if self.reference_errors else math.inf
            body_mean = float(np.mean(self.body_reference_errors)) if self.body_reference_errors else math.inf
            braking = self.braking_evidence()
            landing = landing_completion_evidence(self.latest,self.raw_vehicle_status,
                self.raw_landed,self.clock_ns,self.clock_wall,time.monotonic(),
                float(self.motion_parameters["max_land_detection_age_s"]),
                VehicleStatus.ARMING_STATE_DISARMED)
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
                "landed_and_disarmed_completion": landing["passed"],
                "finite_position_velocity_acceleration_yaw": len(pva)>=250 and self.bad_pva==0,
                "command_velocity_limits": max_vxy<=.61 and max_vz<=.31,
                "command_acceleration_limits": max_axy<=.51 and max_az<=.31,
                "publication_rate_at_least_40_hz": rate>=40,
                "p95_publication_gap_at_most_0_1_s": bool(len(gaps) and np.percentile(gaps,95)<=.1),
                "p99_publication_gap_at_most_0_1_s": bool(len(gaps) and np.percentile(gaps,99)<=.1),
                "no_mission_fault": "FAULT" not in self.phases,
                "topology_built": self.max_regions>=1,
            }
            if args.scenario=="powerplant":
                checks['actual_phase_policy_matches']=phase_policy['passed']
                checks["actual_physical_contacts_verified"]=bool(contacts and contacts["passed"])
                checks["actual_takeoff_and_two_second_stable_hover"]=flight["actual_takeoff_and_stable_hover"]["passed"]
                checks["ground_to_strict_runtime_handoff_verified"]=bool(handoff and handoff["passed"])
                if not limited:
                    checks["actual_autonomous_viewpoint_and_movement"]=progress["passed"]
                    checks["actual_return_home_stopped_before_landing"]=flight["actual_return_before_landing"]["passed"]
            if limited:
                checks["requested_stage_observed"] = bool(self.limited_stage_reached and self.limited_stage_evidence and self.limited_stage_evidence["passed"])
                checks["stage_landing_service_accepted"] = self.stage_landing_accepted
                if args.stop_after=="scan":
                    # This scope ends immediately after scan completion. A
                    # topology worker need not finish before requesting land.
                    checks.pop("topology_built",None)
            if args.detection_mode != "none" and not limited:
                checks.update({
                    "inspection_result_explicit": self.latest.get("inspection_result") in ("PARTIAL","REACHABLE_MODELED_COMPLETE"),
                    "actual_photo_and_surface_evidence": self.photo_count>0 and self.latest.get("inspection_covered_area_m2",0)>0,
                    "coverage_scope_explicit": self.latest.get("inspection_coverage_scope") in ("reachable_modeled_surface", "reachable_modeled_surface_in_task_roi"),
                })
            if args.inject_obstacle:
                checks["injected_obstacle_braked_to_measured_stop"] = braking["observed_braking_then_measured_stop"]
                checks["physical_braking_acceptance_confirmed"] = braking["physical_braking_acceptance_confirmed"]
            if args.scenario == "two_rooms":
                checks["multiple_regions_and_known_portal"] = self.max_regions>=2 and self.max_portals>=1
            if args.detection_mode=="yolo":
                checks["real_yolo_image_chain"]=chain["passed"]
            return {"passed": all(checks.values()), "exit_reason": reason, "checks": checks,
                    "acceptance_scope":acceptance_scope(args),"flight_acceptance_executed":self.started,
                    "full_mission_acceptance":bool(not limited and all(checks.values())),
                    "full_scene_exploration_acceptance":False,
                    "inspection_acceptance_executed":bool(args.detection_mode!="none" and not limited),
                    "physical_contact_evidence":contacts,
                    "physical_flight_evidence":flight,
                    "phase_policy_evidence":phase_policy,
                    "takeoff_ground_handoff_evidence":handoff,
                    "exploration_progress_evidence":progress,
                    "limited_stage_evidence":self.limited_stage_evidence if limited else None,
                    "landing_evidence":landing,
                    "evaluation_geometry":{"world":str(scenario_world(args)),
                        "visual_meshes":self.visual_scene.visual_meshes,
                        "navigation_receives_scene_truth":False},
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
            for topic in self.contact_subscriptions:self.gz_truth_node.unsubscribe(topic)
            if self.contact_evidence:self.gz_truth_node.unsubscribe("/world/powerplant/stats")
            self.consume_contacts()
            self.contact_file.close()
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
    fault_recovery_started = None
    fault_land_future = None
    fault_land_accepted = False
    try:
        while time.monotonic()-started < args.timeout+args.startup_timeout:
            failures = processes.failed()
            if failures: raise RuntimeError("; ".join(failures))
            rclpy.spin_once(node, timeout_sec=.02)
            node.consume_truth()
            node.consume_contacts()
            if not args.image_chain_only: node.calibrate()
            node.start_if_ready()
            node.stop_at_requested_stage()
            node.inject()
            node.replay()
            if args.image_chain_only and node.yolo_success_count>=3:
                reason="yolo_image_chain_complete"; break
            if fault_land_future is not None and fault_land_future.done():
                response = fault_land_future.result()
                fault_land_accepted = bool(response and response.success)
            if (fault_recovery_started is not None and time.monotonic()-fault_recovery_started > 60
                    and node.latest.get("phase") != "COMPLETE"):
                reason = "mission_fault_landing_unconfirmed"; break
            if not args.image_chain_only and node.latest.get("phase") == "FAULT":
                if fault_recovery_started is None:
                    fault_recovery_started = time.monotonic()
                    reason = "mission_fault"
                if fault_land_future is None and node.land_client.service_is_ready():
                    fault_land_future = node.land_client.call_async(Trigger.Request())
                elif fault_land_future is not None and fault_land_future.done() and not fault_land_accepted:
                    break
            if not args.image_chain_only and node.latest.get("phase") == "COMPLETE":
                reason = "mission_fault_recovered_land" if fault_recovery_started is not None else "complete"
                break
            if not args.image_chain_only and not node.started and time.monotonic()-started > args.startup_timeout:
                reason = "startup_or_health_timeout"; break
        report = node.result(reason)
        report["failure_recovery"] = {"requested":fault_recovery_started is not None,
            "landing_service_accepted":fault_land_accepted,
            "landed_and_disarmed":bool(fault_recovery_started is not None and
                report.get("landing_evidence",{}).get("passed")),
            "main_mission_pass_restored":False}
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
        report = {"passed": False, "prepared_only": True, "missing_dependencies": manifest["missing_dependencies"], "sensor_profile":args.sensor_profile, "performance_scope":manifest["sensor_profile"]["performance_scope"],
            "acceptance_scope":acceptance_scope(args),"full_mission_acceptance":False,
            "full_scene_exploration_acceptance":False,"render_context":manifest["render_context"]}
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
    renderer_capture = RendererLogCapture(Path.home()/".gz/rendering/ogre2.log", args.output_dir, env)
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
                    expected_contacts={x["topic"] for x in (manifest.get("contact_instrumentation") or {}).get("sensors",[])}
                    if images and infos and depth_infos and expected_contacts<=set(topics):
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
        server = next((child for name, child, _ in processes.children if name == "gazebo"), None)
        actual_renderer = renderer_capture.finish(server.pid if server is not None else None)
        manifest["render_context"].update(actual_renderer)
        processes.close()
        identity=manifest['runtime_identity']
        code_stability=contact_support.verify_file_lock(identity['code_file_lock'])
        resource_stability=contact_support.verify_file_lock(identity['resource_file_lock'])
        report['runtime_identity_validation']={'code':code_stability,'resources':resource_stability,
            'actual_cpp_executables':identity['actual_cpp_executables'],'repository_head':identity['repository_head'],
            'tracked_diff_binary_sha256':identity['tracked_diff_binary_sha256']}
        identity_valid=(code_stability['passed'] and resource_stability['passed']
                        and identity['cpp_package_resolution']=='ament_index')
        report.setdefault('checks',{})['code_and_resources_unchanged_during_run']=identity_valid
        report['passed']=bool(report['passed'] and identity_valid)
        if 'full_mission_acceptance' in report:
            report['full_mission_acceptance']=bool(report['full_mission_acceptance'] and identity_valid)
        report["sensor_profile"]=args.sensor_profile
        report["acceptance_scope"]=acceptance_scope(args)
        report["render_context"]=manifest["render_context"]
        profile_evidence=sensor_profile_evidence(manifest["sensor_profile"],report.get("observed_sensors",{}))
        report["sensor_profile_validation"]=profile_evidence
        report.setdefault("checks",{})["actual_sensor_profile_matches_requested"]=profile_evidence["passed"]
        report["passed"]=bool(report["passed"] and profile_evidence["passed"])
        if "full_mission_acceptance" in report:
            report["full_mission_acceptance"]=bool(report["full_mission_acceptance"] and profile_evidence["passed"])
        report["requested_performance_scope"]=manifest["sensor_profile"]["performance_scope"]
        controller=controller_profile_evidence(args,report.get("gcs_evidence",{}).get("read_only_px4_parameters",{}))
        report["isolated_controller_profile"]=controller
        report["checks"]["actual_controller_profile_matches_requested"]=controller["passed"]
        report["passed"]=bool(report["passed"] and controller["passed"])
        if "full_mission_acceptance" in report:
            report["full_mission_acceptance"]=bool(report["full_mission_acceptance"] and controller["passed"])
        report["performance_scope"]=(report["requested_performance_scope"]
            if profile_evidence["passed"] else "unverified_sensor_load")
        manifest["sensor_profile"]["observed_sensors"]=report.get("observed_sensors",{})
        manifest["sensor_profile"]["validation"]=profile_evidence
        manifest["takeoff_startup_validation"]=report.get("takeoff_startup_validation")
        (args.output_dir / "manifest.json").write_text(json_dumps(clean_json(manifest),ensure_ascii=False,indent=2,allow_nan=False))
        report["manifest"] = str(args.output_dir / "manifest.json")
        report_path.write_text(json_dumps(clean_json(report), ensure_ascii=False, indent=2, allow_nan=False))
    print(json_dumps({"passed": report["passed"], "report": str(report_path),
                      "exit_reason": report.get("exit_reason")}), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
