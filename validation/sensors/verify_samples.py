#!/usr/bin/env python3
"""Offline native-Gazebo sensor/visual/collision comparison for powerplant.

Reads saved gz.msgs10 protobufs. Never creates a ROS/Gazebo node, publishes a
map/waypoint or commands flight. GPU samples validate visual geometry only;
DART contact and simplification-distance evidence remain separate requirements.
"""
from __future__ import annotations

import argparse
import copy
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def native_message(path, message_type):
    message = message_type()
    data = Path(path).read_bytes()
    if not data:
        raise ValueError(f"empty native sample: {path}")
    message.ParseFromString(data)
    return message


def metadata(message):
    header = message.header
    return {"stamp_sec": int(header.stamp.sec), "stamp_nsec": int(header.stamp.nsec),
            "header_data": {entry.key: list(entry.value) for entry in header.data}}


def quantiles(values):
    import numpy as np
    if not values:
        return {"count": 0, "p50_m": None, "p95_m": None, "max_m": None}
    values = np.asarray(values, dtype=float)
    if not np.isfinite(values).all():
        raise ValueError("non-finite value entered geometry error statistics")
    return {"count": len(values), "p50_m": float(np.quantile(values, .50)),
            "p95_m": float(np.quantile(values, .95)), "max_m": float(values.max())}


def quaternion_matrix(quaternion):
    import numpy as np
    q = np.asarray([quaternion.w, quaternion.x, quaternion.y, quaternion.z], dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("invalid native pose quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def message_pose(pose):
    import numpy as np
    result = np.eye(4)
    result[:3,:3] = quaternion_matrix(pose.orientation)
    result[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(result).all():
        raise ValueError("non-finite native pose")
    return result


def verified_runtime_sensor_libraries(folder):
    """Bind explicit local-pose handling to the actually captured 8.2.2 build."""
    path = folder / "report.json"
    report = json.loads(path.read_text())
    entries = report.get("runtime_sensor_libraries", [])
    if not entries:
        raise ValueError("sensor-local convention requires captured runtime_sensor_libraries in report.json")
    verified = []
    for entry in entries:
        library = Path(entry["path"])
        if not library.is_file() or digest(library) != entry.get("sha256"):
            raise ValueError("captured runtime sensor library hash is unavailable or changed")
        verified.append({"path": str(library), "sha256": entry["sha256"]})
    names = [Path(x["path"]).name for x in verified]
    if not any("libgz-sensors8-gpu_lidar" in x and x.endswith(".8.2.2") for x in names):
        raise ValueError("sensor-local support is source-verified for gz-sensors 8.2.2 only")
    return {"report_path": str(path), "report_sha256": digest(path), "libraries": verified,
            "source_verified_version": "gz-sensors 8.2.2"}


def verified_scene_rig(snapshot, rig, evaluator):
    """Require runtime model/link/sensor IDs and poses, not only planned SDF."""
    import numpy as np
    from gz.msgs10.scene_pb2 import Scene
    scene = native_message(snapshot, Scene)
    leaf = lambda name: name.rsplit("::", 1)[-1]
    matches = [model for model in scene.model if leaf(model.name) == rig.get("name")]
    if len(matches) != 1:
        raise ValueError("Scene snapshot must contain one top-level saved sensor rig")
    model = matches[0]
    xml_link = rig.find("link")
    links = [link for link in model.link if leaf(link.name) == xml_link.get("name")]
    if len(links) != 1 or model.id == 0 or model.deleted:
        raise ValueError("Scene snapshot rig/link identity is invalid")
    link = links[0]
    if link.id == 0 or link.id == model.id:
        raise ValueError("Scene rig link ID is missing or not distinct")
    if rig.findtext("static", "false").strip() != "true":
        raise ValueError("Scene snapshot composition is only supported for this saved static rig")
    model_pose = message_pose(model.pose)
    link_pose = message_pose(link.pose)
    expected_model_pose = evaluator._pose(rig.find("pose"))
    expected_link_pose = evaluator._pose(xml_link.find("pose"))
    if not np.allclose(model_pose, expected_model_pose, atol=1e-6, rtol=0) or not np.allclose(link_pose, expected_link_pose, atol=1e-6, rtol=0):
        raise ValueError("runtime Scene parent pose differs from saved sensor rig")
    mounts, sensors = {}, []
    seen_ids = {model.id, link.id}
    for xml_sensor in xml_link.findall("sensor"):
        matches = [sensor for sensor in link.sensor if leaf(sensor.name) == xml_sensor.get("name")]
        if len(matches) != 1:
            raise ValueError("runtime Scene sensor is missing or ambiguous")
        sensor = matches[0]
        if sensor.id == 0 or sensor.id in seen_ids or sensor.parent_id != link.id:
            raise ValueError("runtime Scene sensor ID/parent link mismatch")
        seen_ids.add(sensor.id)
        if sensor.type != xml_sensor.get("type"):
            raise ValueError("runtime Scene sensor type differs from saved rig")
        mount = message_pose(sensor.pose)
        if not np.allclose(mount, evaluator._pose(xml_sensor.find("pose")), atol=1e-6, rtol=0):
            raise ValueError("runtime Scene sensor mount differs from saved rig")
        mounts[xml_sensor.get("name")] = mount
        sensors.append({"name": sensor.name, "id": sensor.id, "parent_id": sensor.parent_id,
                        "type": sensor.type, "local_pose_matrix": mount.tolist()})
    evidence = {"path": str(snapshot), "sha256": digest(snapshot),
                "model": {"name": model.name, "id": model.id, "world_pose_matrix": model_pose.tolist()},
                "link": {"name": link.name, "id": link.id, "model_relative_pose_matrix": link_pose.tolist()},
                "sensors": sensors,
                "static_assumption": "saved factory rig is static; Scene model.is_static is not populated by gz-sim 8.11 SceneBroadcaster and is not used as a false proof"}
    return model_pose @ link_pose, mounts, evidence


def intersects_box(origin, ray, limit, low, high):
    # Python min/max can silently retain their first operand when the other is
    # NaN. Reject invalid geometry before the slab test can call it a smoke hit.
    values = [*origin, *ray, limit, *low, *high]
    if not all(math.isfinite(float(x)) for x in values):
        raise ValueError("particle intersection requires finite geometry")
    if limit < 0 or not any(abs(x) > 1e-12 for x in ray) or any(a > b for a, b in zip(low, high)):
        raise ValueError("particle intersection requires a nonzero ray and ordered bounds")
    near, far = 0., float(limit)
    for axis in range(3):
        if abs(ray[axis]) < 1e-12:
            if origin[axis] < low[axis] or origin[axis] > high[axis]:
                return False
        else:
            a, b = (low[axis]-origin[axis])/ray[axis], (high[axis]-origin[axis])/ray[axis]
            near, far = max(near, min(a, b)), min(far, max(a, b))
            if far < near:
                return False
    return far >= max(near, 0.)


def lidar_samples(scan, sensor_pose, world_rays=None):
    """Decode native angles, including undefined singleton-axis increments.

    gz-sensors 8.2.2 emits NaN for a one-layer vertical angle step. There is
    no increment to apply in that case: 0 * NaN must never enter a world ray.
    For multiple samples the increment must be finite and match the endpoints.
    """
    import numpy as np

    def angles(count, low, high, step, label):
        if count < 1 or not all(math.isfinite(float(x)) for x in (low, high)):
            raise ValueError(f"invalid {label} LaserScan endpoints/count")
        if count == 1:
            if abs(high-low) > 1e-5:
                raise ValueError(f"singleton {label} LaserScan endpoints disagree")
            return [float(low)]
        if not math.isfinite(step) or step < 0 or abs(low+(count-1)*step-high) > 1e-5:
            raise ValueError(f"invalid {label} LaserScan angular increment")
        return [low+i*step for i in range(count)]

    rows = int(scan.vertical_count)
    columns = int(scan.count)
    if len(scan.ranges) != columns*rows or not all(math.isfinite(x) for x in (scan.range_min, scan.range_max)) or not 0 < scan.range_min < scan.range_max:
        raise ValueError("invalid native LaserScan shape or range limits")
    horizontal = angles(columns, scan.angle_min, scan.angle_max, scan.angle_step, "horizontal")
    vertical = angles(rows, scan.vertical_angle_min, scan.vertical_angle_max, scan.vertical_angle_step, "vertical")
    pose = np.asarray(sensor_pose, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("invalid finite LaserScan sensor pose")
    if world_rays is not None:
        world_rays = np.asarray(world_rays, dtype=float)
        if world_rays.shape != (columns*rows, 3) or not np.isfinite(world_rays).all():
            raise ValueError("invalid finite explicit LaserScan world rays")
    origin, rotation = pose[:3, 3], pose[:3, :3]
    result = []
    for j, elevation in enumerate(vertical):
        for i, angle in enumerate(horizontal):
            value = float(scan.ranges[j*columns+i])
            if not math.isfinite(value):
                result.append({"excluded": "nonfinite_lidar"})
                continue
            if not scan.range_min+1e-6 < value < scan.range_max-1e-5:
                result.append({"excluded": "outside_or_saturated_lidar_clip"})
                continue
            ray = (world_rays[j*columns+i] if world_rays is not None else
                   rotation @ np.asarray([math.cos(elevation)*math.cos(angle),
                                          math.cos(elevation)*math.sin(angle), math.sin(elevation)]))
            if not np.isfinite(ray).all() or np.linalg.norm(ray) <= 1e-12:
                raise ValueError("invalid finite LaserScan world ray")
            result.append({"id": [i, j], "origin": origin, "ray": ray, "value": value,
                           "minimum": scan.range_min, "maximum": scan.range_max})
    return result


def particle_bounds(world, evaluator):
    """Conservative emitter+life-displacement bounds, not a smoke sensor model."""
    import numpy as np
    result = []
    for model in ET.parse(world).getroot().findall("world/model"):
        for link in model.findall("link"):
            for emitter in link.findall("particle_emitter"):
                transform = evaluator._pose(model.find("pose")) @ evaluator._pose(link.find("pose")) @ evaluator._pose(emitter.find("pose"))
                size = np.asarray([float(x) for x in emitter.findtext("size", "1 1 1").split()])
                particle = np.asarray([float(x) for x in emitter.findtext("particle_size", "1 1 1").split()])
                lifetime = float(emitter.findtext("lifetime", "5"))
                speed = max(abs(float(emitter.findtext("min_velocity", "0"))), abs(float(emitter.findtext("max_velocity", "0"))))
                growth = max(0., float(emitter.findtext("scale_rate", "0"))) * lifetime
                if len(size) != 3 or len(particle) != 3 or not np.isfinite(size).all() or np.any(size < 0):
                    raise ValueError("unsupported particle-emitter dimensions")
                corners = np.asarray([[x, y, z] for x in (-size[0]/2, size[0]/2)
                                      for y in (-size[1]/2, size[1]/2) for z in (-size[2]/2, size[2]/2)])
                points = corners @ transform[:3, :3].T + transform[:3, 3]
                margin = speed*lifetime + float(particle.max())/2 + growth
                result.append({"name": emitter.get("name", ""), "minimum_m": points.min(axis=0)-margin,
                               "maximum_m": points.max(axis=0)+margin, "drift_margin_m": margin})
    return result


def scenes(px4_root):
    runner_path = Path(__file__).resolve().parents[1] / "sitl/run_acceptance.py"
    spec = importlib.util.spec_from_file_location("run_acceptance", runner_path)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    VisualSceneEvaluator = runner.VisualSceneEvaluator
    from vtkmodules.vtkIOGeometry import vtkSTLReader
    from vtkmodules.vtkCommonDataModel import vtkStaticCellLocator
    asset_root = px4_root / "Tools/simulation/gz"
    world = asset_root / "worlds/powerplant.sdf"
    models = asset_root / "models"
    visual = VisualSceneEvaluator(world, models)
    collision = copy.copy(visual)
    collision.geometries = []
    collision_mesh = models / "power-plant/meshes/power-plant-collision.stl"
    reader = vtkSTLReader(); reader.SetFileName(str(collision_mesh)); reader.Update()
    if reader.GetOutput().GetNumberOfCells() == 0:
        raise ValueError("empty collision STL")
    physical_locator = vtkStaticCellLocator(); physical_locator.SetDataSet(reader.GetOutput()); physical_locator.BuildLocator()
    replaced, removed_markers = 0, []
    for kind, inverse, data, calibration, name in visual.geometries:
        if name.startswith("land_zone"):
            removed_markers.append(name)  # SDF landing sticker is visual-only.
            continue
        if name == "power_plant_env" and kind == "mesh":
            data = physical_locator
            replaced += 1
        collision.geometries.append((kind, inverse, data, calibration, name))
    if replaced != 1:
        raise ValueError("expected one power_plant_env visual mesh to replace")
    # Primitive visual/collision equality is a precondition for reuse above.
    for model in ET.parse(world).getroot().findall("world/model"):
        if model.get("name", "") != "ground_plane" and not model.get("name", "").startswith("indoor_enclosure_"):
            continue
        for link in model.findall("link"):
            a, b = link.find("visual"), link.find("collision")
            if a is None or b is None:
                raise ValueError("missing paired enclosure visual/collision")
            canonical = lambda x: (x.tag, (x.text or "").strip(), tuple(canonical(c) for c in x))
            if canonical(a.find("geometry")) != canonical(b.find("geometry")):
                raise ValueError("enclosure primitive visual/collision mismatch")
            if (a.findtext("pose", "0 0 0 0 0 0") != b.findtext("pose", "0 0 0 0 0 0")):
                raise ValueError("enclosure primitive relative pose mismatch")
    return visual, collision, world, collision_mesh, removed_markers


def compare_samples(samples, visual, collision, clouds, minimum_samples, visual_tolerance, collision_tolerance):
    import numpy as np
    excluded, records, failures = Counter(), [], []
    for sample in samples:
        if "excluded" in sample:
            excluded[sample["excluded"]] += 1
            continue
        origin, ray, measured = sample["origin"], sample["ray"], sample["value"]
        first = visual.first_hit(origin, ray, sample["maximum"])
        # Reject potentially particle-affected rays *without* looking at error.
        # Exclusion counts and minimum clean coverage are part of acceptance.
        limit = min(sample["maximum"], first["parameter"] if first else sample["maximum"])
        if any(intersects_box(origin, ray, limit, cloud["minimum_m"], cloud["maximum_m"]) for cloud in clouds):
            excluded["potential_particle_volume"] += 1
            continue
        if first is None or first["parameter"] < sample["minimum"]:
            failures.append({"sample": sample["id"], "reason": "finite_sensor_return_without_in_range_visual_surface", "value_m": measured})
            continue
        physical = collision.first_hit(origin, ray, sample["maximum"])
        if physical is None:
            failures.append({"sample": sample["id"], "reason": "visual_surface_missing_from_collision", "visual_model": first["model_name"]})
            continue
        observed_visual = abs(measured-first["parameter"])
        observed_collision = abs(measured-physical["parameter"])
        visual_collision = abs(first["parameter"]-physical["parameter"])
        records.append({"sample": sample["id"], "measured_m": measured, "visual_m": first["parameter"],
                        "collision_m": physical["parameter"], "sensor_to_visual_error_m": observed_visual,
                        "sensor_to_collision_error_m": observed_collision, "visual_to_collision_error_m": visual_collision,
                        "visual_model": first["model_name"], "collision_model": physical["model_name"]})
    sv = quantiles([x["sensor_to_visual_error_m"] for x in records])
    sc = quantiles([x["sensor_to_collision_error_m"] for x in records])
    vc = quantiles([x["visual_to_collision_error_m"] for x in records])
    enough = len(records) >= minimum_samples
    measured_bad = bool(records) and (sv["p95_m"] > visual_tolerance or vc["max_m"] > collision_tolerance)
    status = "FAIL" if failures or measured_bad else "PASS" if enough else "UNCONFIRMED"
    return {"status": status, "minimum_clean_samples": minimum_samples, "clean_samples": len(records),
            "excluded": dict(excluded), "structural_failure_count": len(failures), "structural_failures": failures[:30],
            "sensor_to_visual": sv, "sensor_to_collision": sc, "visual_to_collision": vc,
            "criteria": {"sensor_to_visual_p95_m": visual_tolerance, "visual_to_collision_max_m": collision_tolerance,
                         "sensor_to_collision": "reported; no additional hard 5cm limit"}, "samples": records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples-dir", type=Path, required=True)
    parser.add_argument("--px4-root", type=Path, default=Path("/home/pawn/drone-simulation"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--lidar-pose-convention", choices=("world", "sensor-local"), default="world",
                        help="sensor-local is explicit gz-sensors8.2.2 handling and requires runtime Scene/library evidence")
    parser.add_argument("--scene-snapshot", type=Path,
                        help="native gz.msgs.Scene snapshot; defaults to samples-dir/scene_snapshot.pb for sensor-local")
    parser.add_argument("--lidar-ray-convention", choices=("nominal", "gz-rendering8-cubemap-nearest"), default="nominal",
                        help="explicit source-derived GPU texel directions; requires supported captured renderer and shader hashes")
    args = parser.parse_args()
    folder = args.samples_dir.resolve()
    output = args.output or folder / "sensor_geometry_report.json"
    from lidar_rays import RendererEvidenceUnavailable, predict_scan, verify_renderer_evidence
    try:
        import numpy as np
        from gz.msgs10.image_pb2 import Image, R_FLOAT32, RGB_INT8, BGR_INT8, RGBA_INT8, BGRA_INT8
        from gz.msgs10.camera_info_pb2 import CameraInfo
        from gz.msgs10.laserscan_pb2 import LaserScan
        visual, collision, world, collision_mesh, removed_markers = scenes(args.px4_root.resolve())
        files = {name: folder/name for name in ("rgb_first.pb", "depth_first.pb", "lidar_first.pb", "camera_info_640_480.pb", "sensor_rig.sdf")}
        rgb = native_message(files["rgb_first.pb"], Image)
        depth = native_message(files["depth_first.pb"], Image)
        info = native_message(files["camera_info_640_480.pb"], CameraInfo)
        scan = native_message(files["lidar_first.pb"], LaserScan)
        channels = {RGB_INT8: 3, BGR_INT8: 3, RGBA_INT8: 4, BGRA_INT8: 4}
        if rgb.pixel_format_type not in channels or rgb.width == 0 or rgb.height == 0 or rgb.step < rgb.width*channels[rgb.pixel_format_type] or len(rgb.data) != rgb.step*rgb.height:
            raise ValueError("invalid RGB dimensions, stride, format or payload")
        if depth.pixel_format_type != R_FLOAT32 or depth.step < depth.width*4 or len(depth.data) != depth.step*depth.height:
            raise ValueError("expected complete native R_FLOAT32 depth sample")
        if depth.width != info.width or depth.height != info.height or len(info.intrinsics.k) != 9:
            raise ValueError("depth/CameraInfo dimensions or K do not match")
        if any(abs(x) > 1e-12 for x in info.distortion.k):
            raise ValueError("nonzero distortion requires an explicit rectification evaluator")
        K = np.asarray(info.intrinsics.k, dtype=float).reshape(3, 3)
        if not np.isfinite(K).all() or K[0,0] <= 0 or K[1,1] <= 0 or not np.allclose(K[2], [0,0,1]):
            raise ValueError("invalid CameraInfo intrinsic matrix")
        inv_k = np.linalg.inv(K)
        rig = ET.parse(files["sensor_rig.sdf"]).getroot().find("model")
        link = rig.find("link")
        body_pose = visual._pose(rig.find("pose")) @ visual._pose(link.find("pose"))
        depth_sensor = link.find("sensor[@type='depth_camera']")
        lidar_sensor = link.find("sensor[@type='gpu_lidar']")
        if depth_sensor is None or lidar_sensor is None:
            raise ValueError("expected saved depth/lidar rig sensors")
        camera_mount = visual._pose(depth_sensor.find("pose"))
        lidar_mount = visual._pose(lidar_sensor.find("pose"))
        scene_evidence, library_evidence = None, None
        if args.scene_snapshot is not None or args.lidar_pose_convention == "sensor-local":
            snapshot = (args.scene_snapshot or folder/"scene_snapshot.pb").resolve()
            body_pose, mounts, scene_evidence = verified_scene_rig(snapshot, rig, visual)
            camera_mount = mounts[depth_sensor.get("name")]
            lidar_mount = mounts[lidar_sensor.get("name")]
            files["scene_snapshot.pb"] = snapshot
        camera_pose = body_pose @ camera_mount
        lidar_pose = body_pose @ lidar_mount
        raw_lidar_pose = message_pose(scan.world_pose)
        if args.lidar_pose_convention == "sensor-local":
            library_evidence = verified_runtime_sensor_libraries(folder)
            if not np.allclose(raw_lidar_pose, lidar_mount, atol=1e-6, rtol=0):
                raise ValueError("LaserScan local pose differs from runtime Scene sensor mount")
            sampled_lidar_pose = body_pose @ raw_lidar_pose
        else:
            sampled_lidar_pose = raw_lidar_pose
        if not np.allclose(sampled_lidar_pose, lidar_pose, atol=1e-5, rtol=0):
            raise ValueError("LaserScan pose convention differs from independently verified rig geometry")
        ray_prediction, renderer_evidence = None, None
        if args.lidar_ray_convention == "gz-rendering8-cubemap-nearest":
            if scene_evidence is None:
                raise RendererEvidenceUnavailable("explicit GPU ray convention requires a verified runtime Scene snapshot")
            renderer_evidence = verify_renderer_evidence(json.loads((folder/"report.json").read_text()), folder/"ogre2.log")
            ray_prediction = predict_scan(scan, sampled_lidar_pose)
        near = float(depth_sensor.findtext("camera/clip/near")); far = float(depth_sensor.findtext("camera/clip/far"))
        array = np.ndarray((depth.height, depth.width), dtype="<f4", buffer=depth.data, strides=(depth.step, 4))
        clouds = particle_bounds(world, visual)
        depth_samples = []
        # A fixed, error-independent grid; dense enough to retain >=100 clean
        # samples even when much of the camera sees the emitter volume.
        xs = np.unique(np.linspace(2, depth.width-3, min(64, depth.width-4)).round().astype(int))
        ys = np.unique(np.linspace(2, depth.height-3, min(48, depth.height-4)).round().astype(int))
        for y in ys:
            for x in xs:
                value = float(array[y,x])
                if not math.isfinite(value): depth_samples.append({"excluded": "nonfinite_depth"}); continue
                if not near+1e-6 < value < far-1e-5: depth_samples.append({"excluded": "outside_or_saturated_depth_clip"}); continue
                neighborhood = array[y-1:y+2, x-1:x+2]
                if not np.isfinite(neighborhood).all() or float(neighborhood.max()-neighborhood.min()) > max(.08, .01*value):
                    depth_samples.append({"excluded": "depth_discontinuity_or_occlusion_edge"}); continue
                optical = inv_k @ np.asarray([x,y,1.])
                optical /= optical[2]
                ray = camera_pose[:3,:3] @ np.asarray([1., -optical[0], -optical[1]])
                depth_samples.append({"id": [int(x),int(y)], "origin": camera_pose[:3,3], "ray": ray,
                                      "value": value, "minimum": near, "maximum": far})
        depth_result = compare_samples(depth_samples, visual, collision, clouds, 100, .08, .05)
        lidar_result=compare_samples(lidar_samples(scan, sampled_lidar_pose,
                                     ray_prediction["world_rays"] if ray_prediction else None),visual,collision,clouds,30,.08,.05)
        statuses=[depth_result["status"],lidar_result["status"]]
        status="FAIL" if "FAIL" in statuses else "UNCONFIRMED" if "UNCONFIRMED" in statuses else "PASS"
        source=args.px4_root.resolve()/"Tools/simulation/gz/models/power-plant/meshes/power-plant.stl"
        report={"status":status,"passed":status=="PASS","evaluation_only":True,"publishes_map_or_waypoints":False,"sends_flight_commands":False,
                "scope":"Static visual and finalized simplified-collision ray comparison against native GPU sensor samples; not DART-contact, YOLO or photo-quality acceptance.",
                "world":{"path":str(world),"sha256":digest(world)},"visual_stl_sha256":digest(source),"collision_stl_sha256":digest(collision_mesh),
                "samples":{name:{"path":str(path),"sha256":digest(path)} for name,path in files.items()},
                "rgb":{"width":rgb.width,"height":rgb.height,"step":rgb.step,"pixel_format":rgb.pixel_format_type,"metadata":metadata(rgb),"payload_verified":True},
                "depth_calibration":{"K":K.tolist(),"width":info.width,"height":info.height,"metadata":metadata(info),"image_metadata":metadata(depth),
                                     "camera_world_pose":camera_pose.tolist(),"ray_convention":"K^-1*[u,v,1] optical ray -> FLU [1,-x,-y]; parameter is forward/optical depth, not Euclidean slant range"},
                "lidar_metadata":{"header":metadata(scan),"frame":scan.frame,"pose_convention":args.lidar_pose_convention,
                                  "ray_convention_name":args.lidar_ray_convention,
                                  "raw_message_world_pose_matrix":raw_lidar_pose.tolist(),"sensor_local_pose_matrix":lidar_mount.tolist(),
                                  "verified_world_pose_matrix":sampled_lidar_pose.tolist(),"count":scan.count,"vertical_count":scan.vertical_count,
                                  "angle_min":scan.angle_min,"angle_max":scan.angle_max,"angle_step":scan.angle_step,
                                  "vertical_angle_min":scan.vertical_angle_min,"vertical_angle_max":scan.vertical_angle_max,
                                  "vertical_angle_step":float(scan.vertical_angle_step) if math.isfinite(scan.vertical_angle_step) else None,
                                  "singleton_vertical_increment_unused":scan.vertical_count==1,
                                  "nonfinite_vertical_increment_in_native_message":not math.isfinite(scan.vertical_angle_step),
                                  "ray_convention":("source-derived verified Ogre2 nearest cubemap texel center, transformed by verified runtime pose; parameter is range" if ray_prediction else
                                                    "verified runtime pose * nominal unit spherical FLU ray from native angular metadata; parameter is range")},
                "renderer_sampling_evidence":renderer_evidence,
                "lidar_ray_prediction":({k:v for k,v in ray_prediction.items() if k!="world_rays"} if ray_prediction else None),
                "runtime_scene_evidence":scene_evidence,"runtime_sensor_library_evidence":library_evidence,
                "pose_semantics_primary_sources":[
                    "https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/Lidar.cc#L251",
                    "https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/GpuLidarSensor.cc#L201",
                    "https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/Sensor.cc#L183",
                    "https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.11.0/src/rendering/RenderUtil.cc#L1217"],
                "potential_particle_regions":[{k:(v.tolist() if hasattr(v,'tolist') else v) for k,v in cloud.items()} for cloud in clouds],
                "particle_exclusion_policy":"Conservative emitter bounds expanded by maximum lifetime displacement and particle radius; independent of measured/reference error. Excluded rays never count toward minimum clean coverage.",
                "collision_visual_only_markers_removed":removed_markers,"depth":depth_result,"lidar":lidar_result,
                "limitations":["Inf/NaN returns are excluded and do not prove empty geometry.","Depth discontinuity pixels are excluded as occlusion-edge samples; counts are reported.",
                               "RGB/depth first samples need not share exposure; RGB payload is evidence only, not RGB-D registration.",
                               "The static-rig samples do not establish dynamic tracking accuracy or physical contact correctness."]}
        benchmark=folder/"report.json"
        if benchmark.is_file():report["benchmark_report"]={"path":str(benchmark),"sha256":digest(benchmark)}
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
        print(json.dumps({"status":status,"report":str(output),"depth_clean":depth_result["clean_samples"],"lidar_clean":lidar_result["clean_samples"],
                          "depth_visual_p95_m":depth_result["sensor_to_visual"]["p95_m"],"lidar_visual_p95_m":lidar_result["sensor_to_visual"]["p95_m"]},indent=2))
        return 0 if status=="PASS" else 2 if status=="UNCONFIRMED" else 1
    except Exception as error:
        unconfirmed = isinstance(error, RendererEvidenceUnavailable)
        report={"status":"UNCONFIRMED" if unconfirmed else "ERROR","passed":False,"evaluation_only":True,"error":str(error),"samples_dir":str(folder),
                "lidar_ray_convention":args.lidar_ray_convention}
        if output.parent.is_dir():output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
        print(json.dumps(report,ensure_ascii=False,indent=2),file=sys.stderr)
        return 2 if unconfirmed else 1


if __name__=="__main__":
    raise SystemExit(main())
