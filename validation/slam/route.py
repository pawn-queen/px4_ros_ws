"""Ground truth geometry and prescribed observation windows for native powerplant SITL."""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from validation.model.takeoff_audit import audit_takeoff_domain, triangle_aabb_audit
from validation.sitl.run_acceptance import VisualSceneEvaluator, read_asset_lock

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "src/px4_powerplant_mission/config"
A = (0., 4., 3.)
B = (7., 6., 3.)
C = (16., 5., 3.)
CAMERA = np.array([.13233, 0., .26078])
# The reference is a fixed, predeclared patch on the visual STL, with its back excluded by first-hit rays.
B_LOW = np.array([9.9, 9.5, 2.7])
B_HIGH = np.array([10.6, 10.55, 3.9])


def itinerary():
    # angle = ENU yaw, 'mask' describes what is delivered to SLAM throughout the segment.
    # At C turn via south so the camera cannot look back at the B surface on the return leg.
    return [
        ("takeoff", (0., 0., 3.), 90., 2., False),
        ("first_a", A, 90., 5., False),
        ("b", B, 50., 4., False),
        ("occlusion", (10., 5.5, 3.), -12., .6, True),
        ("negative", C, 42., 4., False),
        ("turn_south", C, -90., 0., False),
        ("turn_west", C, -180., 0., False),
        ("return_lane", (0., 3., 3.), -180., 0., False),
        ("revisit", A, 90., 7., False),
        ("land_column", (0., 0., 3.), 90., .5, False),
    ]


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_json(path, payload):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def reference_surface(scene, origin_body, degrees, region, u_range, v_range):
    yaw = math.radians(degrees)
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.])
    left = np.array([-forward[1], forward[0], 0.])
    origin = origin_body + forward * CAMERA[0] + [0., 0., CAMERA[2]]
    points = []
    for u in np.linspace(*u_range, 41):
        for v in np.linspace(*v_range, 37):
            ray = forward + u * left + [0., 0., v]
            hit = scene.first_hit(origin, ray, 19.1)
            if hit and hit["model_name"] == "power_plant_env":
                point = origin + hit["parameter"] * ray
                if (point >= region[0]).all() and (point <= region[1]).all():
                    points.append(point)
    if len(points) < 80:
        raise ValueError(f"too few first-hit visual samples on the predeclared patch {region}")
    return np.asarray(points)


def camera_profile(sensors):
    result = {}
    for kind in ("camera", "depth_camera"):
        matches = [s for s in sensors if s.get("type") == kind]
        if len(matches) != 1:
            raise ValueError(f"expected one native {kind}")
        sensor = matches[0]
        result["/" + kind] = {"sensor": sensor.get("name"), "hz": float(sensor.findtext("update_rate")),
                              "width": int(sensor.findtext("camera/image/width")),
                              "height": int(sensor.findtext("camera/image/height")),
                              "horizontal_fov": float(sensor.findtext("camera/horizontal_fov")),
                              "near": float(sensor.findtext("camera/clip/near")),
                              "far": float(sensor.findtext("camera/clip/far"))}
    return result


def check_loaded_model(xml, plan):
    world = ET.fromstring(xml).find("world")
    if world is None or world.get("name") != plan["world"]:
        raise ValueError("Gazebo returned a different world")
    vehicle = world.find(f"model[@name='{plan['model']}']")
    if vehicle is None:
        raise ValueError("the requested x500_depth instance is absent from the live world")
    camera_link = vehicle.find("link[@name='camera_link']")
    sensors = camera_link.findall("sensor") if camera_link is not None else []
    profile = camera_profile(sensors)
    if profile != plan["native_camera_profile"]:
        raise ValueError(f"loaded camera profile differs from the native OakD source: {profile}")
    pose = np.fromstring(camera_link.findtext("pose"), sep=" ")
    for sensor in sensors:
        optical = np.fromstring(sensor.findtext("pose"), sep=" ")
        if (not np.allclose(pose[3:], 0., atol=1e-5) or not np.allclose(optical[3:], 0., atol=1e-5)
                or not np.allclose(pose[:3] + optical[:3], CAMERA, atol=1e-5)):
            raise ValueError("loaded camera extrinsics differ from the calibrated optical transform")
    plant = world.find("model[@name='power_plant_env']")
    if plant is None or not np.allclose(np.fromstring(plant.findtext("pose"), sep=" "), [10, 10, 0, 0, 0, 0]):
        raise ValueError("loaded plant transform differs from audited geometry")
    from urllib.parse import unquote, urlparse
    for mesh in plan["cruise_checks"]:
        uri = plant.findtext(f".//{mesh['role']}/geometry/mesh/uri")
        parsed = urlparse(uri or "")
        if parsed.scheme != "file" or sha(Path(unquote(parsed.path))) != mesh["sha256"]:
            raise ValueError(f"loaded {mesh['role']} mesh differs from the audited asset")
    imu = vehicle.find(".//sensor[@name='imu_sensor']")
    if imu is None or float(imu.findtext("update_rate")) != plan["imu_hz"]:
        raise ValueError("live raw IMU profile differs from native x500_base")
    return profile


def prepare(px4_root, split, output):
    assets = px4_root / "Tools/simulation/gz"
    models = assets / "models"
    world = assets / "worlds/powerplant.sdf"
    lock = read_asset_lock(ROOT / "src/px4_powerplant_mission_cpp/config/powerplant_assets.json",
                           assets, world)
    vehicle = models / "x500_depth/model.sdf"
    if not vehicle.is_file():
        raise ValueError("x500_depth is unavailable")
    base = yaml.safe_load((CONFIG / "powerplant.yaml").read_text())["powerplant_offboard_mission"]["ros__parameters"]
    profile = yaml.safe_load((CONFIG / "p2_simulation.yaml").read_text())["powerplant_offboard_mission"]["ros__parameters"]
    parameters = {**base, **profile, "takeoff_height_m": 3., "takeoff_region_height_m": 4.}
    audit = audit_takeoff_domain(world, models, parameters)
    if not audit["passed"]:
        raise ValueError("takeoff column has unproven clearance")
    # Native locked model transform, checked independently by reading the visual and collision SDF above.
    # Prove one connected cruise box free; takeoff/landing columns overlap it at the spawn.
    low = np.min([p for _, p, _, _, _ in itinerary()], axis=0) - [1.1, 1.1, .7]
    high = np.max([p for _, p, _, _, _ in itinerary()], axis=0) + [1.1, 1.1, 1.]
    if low[2] <= 0:
        raise ValueError("cruise box enters the ground")
    for box in audit["boxes"]:
        if np.all(np.asarray(box["world_maximum"]) >= low) and np.all(np.asarray(box["world_minimum"]) <= high):
            raise ValueError("cruise box overlaps " + box["model"])
    geometry_checks = []
    for mesh in audit["meshes"]:
        if mesh["model"] != "power_plant_env":
            raise ValueError("unsupported additional route mesh")
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        vertices = np.fromfile(mesh["path"], dtype, offset=84)["vertices"] + [10., 10., 0.]
        check = triangle_aabb_audit(vertices, low, high)
        if not check["passed"]:
            raise ValueError("cruise box has collision candidates in " + mesh["role"])
        geometry_checks.append({**check, "role": mesh["role"], "sha256": mesh["sha256"]})
    scene = VisualSceneEvaluator(world, models)
    reference = reference_surface(scene, B, 50., (B_LOW, B_HIGH), (-.12,.12), (-.13,.13))
    # Two different vertical sections of cylindrical equipment in the original monochrome STL.
    # Narrow angular crops choose their front surface before any capture or place recognition.
    for name, position, yaw, region, u, v in [
        ("a", A, 90., ([1.43, 10.65, 1.25], [1.85, 11.2, 2.43]), (-.277,-.196), (-.31,-.119)),
        ("c", C, 42., ([19.12, 10.95, 1.25], [19.65, 11.5, 2.43]), (.307,.407), (-.319,-.115)),
    ]:
        np.save(output / f"{name}_reference.npy", reference_surface(scene, position, yaw, region, u, v))
    camera_sdf = models / "OakD-Lite/model.sdf"
    np.save(output / "b_reference.npy", reference, allow_pickle=False)
    plan = {
        "world": "powerplant", "model": "x500_depth_0", "split": split,
        "cruise_speed_mps": .72, "approach_speed_mps": .60 if split == "calibration" else .45,
        "segments": [{"name": n, "position_enu_m": p, "yaw_deg": a, "dwell_s": t, "occluded": mask}
                     for n, p, a, t, mask in itinerary()],
        "bounds_min_enu_m": low.tolist(), "bounds_max_enu_m": high.tolist(),
        "cruise_checks": geometry_checks, "takeoff_checks": audit["checks"],
        "world_sha256": lock["world_sha256"], "assets_sha256": lock["manifest_sha256"],
        "model_sha256": sha(vehicle), "camera_sha256": sha(camera_sdf),
        "imu_hz": float(ET.parse(models/"x500_base/model.sdf").findtext(".//sensor[@name='imu_sensor']/update_rate")),
        "native_camera_profile": camera_profile(ET.parse(camera_sdf).findall(".//sensor")),
        "a_reference_sha256": sha(output/"a_reference.npy"), "c_reference_sha256": sha(output/"c_reference.npy"), "reference_patch_count": len(reference),
        "reference_sha256": sha(output / "b_reference.npy"),
    }
    save_json(output / "route.json", plan)
    return plan


def visible_points(points, position, rotation, intrinsics):
    """Conservative optical frustum of the actual recorded depth frame. No occlusion culling."""
    fx, fy, cx, cy, width, height = intrinsics
    camera_frame = (points - position) @ rotation - CAMERA
    forward, left, up = camera_frame.T
    return ((forward >= .2) & (forward <= 19.1) &
            (fx * -left >= -cx * forward) & (fx * -left <= (width-cx) * forward) &
            (fy * -up >= -cy * forward) & (fy * -up <= (height-cy) * forward))
