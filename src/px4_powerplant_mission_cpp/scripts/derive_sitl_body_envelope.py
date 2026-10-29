#!/usr/bin/env python3
"""Analytic body envelope of the local SDF collision model, without simulation.

Expand includes with libsdformat, resolve its pose graph, and bound every box
collision over arbitrary body yaw, total body tilt <= alpha, and complete yaw
rotation of every vertical revolute joint. Unsupported shapes/kinematics fail
closed; render meshes and hardware geometry are deliberately outside this claim.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def pose(element):
    values = [float(v) for v in (element.text or "").split()] if element is not None else [0.] * 6
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("requires finite six-value Euler SDF poses")
    if element is not None and (element.get("rotation_format", "euler_rpy") != "euler_rpy" or element.get("degrees", "false") != "false"):
        raise ValueError("unsupported SDF pose representation")
    x, y, z, r, p, yaw = values
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, x],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, y],
            [-sp, cp*sr, cp*cr, z], [0., 0., 0., 1.]]


IDENTITY = pose(None)


def point(transform, p):
    return [sum(transform[i][k] * p[k] for k in range(3)) + transform[i][3] for i in range(3)]


def point_bounds(r, z, alpha):
    """Exact extrema for a point's radius/height over the complete tilt cone."""
    norm = math.hypot(r, z)
    phi = math.atan2(r, z)
    low_angles = [0., alpha]
    high_angles = [0., alpha]
    if 0 <= math.pi-phi <= alpha:
        low_angles.append(math.pi-phi)
    if 0 <= phi <= alpha:
        high_angles.append(phi)
    low = min(z*math.cos(t)-r*math.sin(t) for t in low_angles)
    high = max(z*math.cos(t)+r*math.sin(t) for t in high_angles)
    nearest_zero = 0. if low <= 0 <= high else min(abs(low), abs(high))
    radius = math.sqrt(max(0., norm*norm-nearest_zero*nearest_zero))
    return low, high, radius


def derive(expanded, alpha, include_geometry=False):
    sdf = ET.fromstring(expanded)
    model = sdf.find("model")
    if model is None or model.findall("model") or model.findall("include"):
        raise ValueError("expected one completely expanded, merged model")
    if model.find("pose") is not None:
        root_pose = pose(model.find("pose"))
        if any(abs(root_pose[i][j]-IDENTITY[i][j]) > 1e-10 for i in range(4) for j in range(4)):
            raise ValueError("navigation reference is the unplaced model origin")
    frames = {"__model__": (None, IDENTITY)}
    for tag in ("frame", "link", "joint"):
        for element in model.findall(tag):
            name, p = element.get("name"), element.find("pose")
            default = element.findtext("child") if tag == "joint" else element.get("attached_to", "__model__")
            relative = p.get("relative_to", default) if p is not None else default
            if not name or name in frames:
                raise ValueError("missing or duplicate frame")
            frames[name] = (relative, pose(p))
    resolved, resolving = {"__model__": IDENTITY}, set()

    def resolve(name):
        if name in resolved:
            return resolved[name]
        if name not in frames or name in resolving:
            raise ValueError(f"unresolved or cyclic SDF frame: {name}")
        resolving.add(name)
        parent, local = frames[name]
        result = matmul(resolve(parent), local)
        resolving.remove(name)
        resolved[name] = result
        return result

    child_joints = {}
    sweeps = {}
    for joint in model.findall("joint"):
        kind, name, child = joint.get("type"), joint.get("name"), joint.findtext("child")
        if child in child_joints:
            raise ValueError("multiple parent joints are not supported")
        child_joints[child] = joint
        if kind == "fixed":
            continue
        if kind not in ("revolute", "continuous"):
            raise ValueError(f"unsupported articulation: {name}/{kind}")
        transform = resolve(name)
        axis_element = joint.find("axis/xyz")
        if axis_element is None:
            raise ValueError("missing articulated axis")
        axis = [float(v) for v in axis_element.text.split()]
        axis_frame = axis_element.get("expressed_in", name)
        ar = resolve(axis_frame)
        axis = [sum(ar[i][j]*axis[j] for j in range(3)) for i in range(3)]
        length = math.sqrt(sum(v*v for v in axis))
        if length < 1e-10 or math.hypot(axis[0], axis[1])/length > 1e-10:
            raise ValueError("only model-vertical full-yaw rotor sweeps are supported")
        parent = joint.findtext("parent")
        if parent in child_joints and child_joints[parent].get("type") != "fixed":
            raise ValueError("nested articulated chains are unsupported")
        sweeps[child] = [transform[i][3] for i in range(3)]

    rows = []
    for link in model.findall("link"):
        link_name = link.get("name")
        # A fixed child of an articulated rotor also moves. Refuse to silently
        # treat its collision as fixed until that kinematic case is implemented.
        ancestor, seen = link_name, set()
        while ancestor in child_joints:
            if ancestor in seen:
                raise ValueError("cyclic joint chain")
            seen.add(ancestor)
            j = child_joints[ancestor]
            if j.get("type") != "fixed" and ancestor != link_name:
                raise ValueError("articulated descendants need a complete sweep model")
            ancestor = j.findtext("parent")
        for collision in link.findall("collision"):
            shape = collision.find("geometry")
            box = shape.find("box") if shape is not None else None
            if shape is None or box is None or len(shape) != 1:
                raise ValueError(f"unsupported collision geometry {link_name}/{collision.get('name')}")
            sizes = [float(v) for v in box.findtext("size", "").split()]
            if len(sizes) != 3 or not all(math.isfinite(v) and v > 0 for v in sizes):
                raise ValueError("invalid box collision")
            p = collision.find("pose")
            transform = matmul(resolve(p.get("relative_to", link_name) if p is not None else link_name), pose(p))
            corners = [point(transform, [signs[i]*sizes[i]/2 for i in range(3)])
                       for signs in itertools.product((-1, 1), repeat=3)]
            bounds = []
            for corner in corners:
                if link_name in sweeps:
                    pivot = sweeps[link_name]
                    radius = math.hypot(pivot[0], pivot[1]) + math.hypot(corner[0]-pivot[0], corner[1]-pivot[1])
                else:
                    radius = math.hypot(corner[0], corner[1])
                bounds.append(point_bounds(radius, corner[2], alpha))
            rows.append({"link": link_name, "collision": collision.get("name"),
                         "full_rotor_yaw_sweep": link_name in sweeps,
                         "zero_tilt_min_z_m": min(p[2] for p in corners),
                         "zero_tilt_max_z_m": max(p[2] for p in corners),
                         "min_z_m": min(b[0] for b in bounds),
                         "max_z_m": max(b[1] for b in bounds),
                         "max_horizontal_radius_m": max(b[2] for b in bounds)})
            if include_geometry:
                rows[-1]["corners_model_origin_flu"] = corners
                rows[-1]["box_size_m"] = sizes
    if not rows:
        raise ValueError("no collision geometry found")
    return {"sdf_version": sdf.get("version"), "model": model.get("name"),
            "reference": "model_origin/PX4 retained SITL navigation reference",
            "total_tilt_bound_rad": alpha,
            "collision_count": len(rows), "full_yaw_sweep_links": sorted(sweeps),
            "minimum_z_m": min(r["min_z_m"] for r in rows),
            "maximum_z_m": max(r["max_z_m"] for r in rows),
            "maximum_horizontal_radius_m": max(r["max_horizontal_radius_m"] for r in rows),
            "collisions": rows,
            "proof": "Convex box vertices suffice for linear height and convex radial extrema; complete vertical-joint yaw is enclosed analytically by pivot radius plus vertex orbit radius. The whole total-tilt cone uses exact scalar trigonometric extrema, not sampled attitudes.",
            "scope": "All collision boxes in this expanded SDF, all rotor yaw angles; not render mesh certification, hardware geometry, controller response or contact dynamics."}


def derive_ground_support(expanded, alpha=.35):
    """Extract audited supporting geometry in the retained navigation frame.

    The conservative whole-body z_min is deliberately not a contact offset.
    These fixed landing-gear box vertices support an attitude-dependent minimum
    height calculation without altering the measured navigation pose.
    """
    if not math.isfinite(alpha) or not 0 <= alpha < math.pi/2:
        raise ValueError("invalid total tilt for ground support")
    body = derive(expanded, alpha, include_geometry=True)
    if body["model"] != "x500_depth":
        raise ValueError("ground-support audit requires x500_depth model origin")
    names = tuple(f"base_link_collision_{i}" for i in range(1, 5))
    rails = ("base_link_collision_3", "base_link_collision_4")
    selected = []
    for name in names:
        matches = [r for r in body["collisions"]
                   if r["link"] == "base_link" and r["collision"] == name]
        if len(matches) != 1 or matches[0]["full_rotor_yaw_sweep"]:
            raise ValueError(f"missing, duplicate or moving support collision: {name}")
        selected.append(matches[0])
    rail_rows = [r for r in selected if r["collision"] in rails]
    contact_offset = min(r["zero_tilt_min_z_m"] for r in selected)
    if any(abs(r["zero_tilt_min_z_m"]-contact_offset) > 1e-9 for r in rail_rows):
        raise ValueError("audited bottom rails do not define the nominal support plane")
    non_gear = [r for r in body["collisions"]
                if not (r["link"] == "base_link" and r["collision"] in names)]
    if not non_gear or min(r["zero_tilt_min_z_m"] for r in non_gear) <= contact_offset:
        raise ValueError("a non-gear collision reaches the nominal support plane")
    # A deliberately conservative all-tilt proof: every non-gear point is
    # higher than every point of the two bottom rails over the admitted cone.
    # Thus a supported body cannot first strike its camera, fuselage or rotors.
    non_gear_lower = min(r["min_z_m"] for r in non_gear)
    rail_upper = max(r["max_z_m"] for r in rail_rows)
    if non_gear_lower <= rail_upper:
        raise ValueError("support-first contact is unproven over the admitted tilt cone")
    points = [p for r in selected for p in r["corners_model_origin_flu"]]
    bottom_points = [p for r in rail_rows for p in r["corners_model_origin_flu"]
                     if abs(p[2]-contact_offset) <= 1e-9]
    geometry = {"collisions": body["collisions"], "total_tilt_bound_rad": alpha,
                "reference": body["reference"]}
    digest = hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()
    return {"verified": True, "reference": body["reference"],
            "support_collision_names": list(names), "lowest_rail_collision_names": list(rails),
            "support_points_flu": points, "lowest_rail_points_flu": bottom_points,
            "nominal_contact_offset_m": contact_offset,
            "total_tilt_bound_rad": alpha, "support_geometry_sha256": digest,
            "non_gear_minimum_height_over_tilt_m": non_gear_lower,
            "bottom_rails_maximum_height_over_tilt_m": rail_upper,
            "support_first_height_gap_m": non_gear_lower-rail_upper,
            "support_point_semantics": "complete fixed landing-gear box vertices; minimum after measured orientation gives contact offset",
            "body_envelope": {k: body[k] for k in
                ("collision_count", "minimum_z_m", "maximum_z_m", "maximum_horizontal_radius_m")},
            "scope": "audited static SITL support geometry; not a new navigation measurement, runtime contact observation or hardware calibration"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models-root", type=Path, default=Path("/home/pawn/drone-simulation/Tools/simulation/gz/models"))
    parser.add_argument("--tilt-rad", type=float, default=.35)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--check-profile", type=Path,
                        help="optionally check the explicit SITL motion body parameters (requires PyYAML)")
    args = parser.parse_args()
    if not math.isfinite(args.tilt_rad) or not 0 <= args.tilt_rad < math.pi/2:
        parser.error("total tilt must lie in [0, pi/2)")
    models = args.models_root.resolve()
    source = models / "x500_depth/model.sdf"
    env = os.environ.copy()
    env["SDF_PATH"] = os.pathsep.join(filter(None, (str(models), env.get("SDF_PATH", ""))))
    result = subprocess.run(["gz", "sdf", "-p", str(source)], env=env, capture_output=True, text=True, timeout=30, check=True)
    report = derive(result.stdout, args.tilt_rad)
    report["expanded_sdf_sha256"] = hashlib.sha256(result.stdout.encode()).hexdigest()
    report["source_sdf_sha256"] = {str(models/name/"model.sdf"): hashlib.sha256((models/name/"model.sdf").read_bytes()).hexdigest()
                                    for name in ("x500_depth", "x500", "x500_base", "OakD-Lite")}
    report["sdformat_warnings"] = re.sub(r"\x1b\[[0-9;]*m", "", result.stderr).splitlines()
    valid = True
    if args.check_profile:
        import yaml
        profile = yaml.safe_load(args.check_profile.read_text())["powerplant_offboard_mission"]["ros__parameters"]
        body = {key:float(profile["motion_"+key]) for key in ("body_radius_m", "body_z_min_m", "body_z_max_m")}
        valid = (all(math.isfinite(v) for v in body.values())
                 and body["body_radius_m"] >= report["maximum_horizontal_radius_m"]
                 and body["body_z_min_m"] <= report["minimum_z_m"]
                 and body["body_z_max_m"] >= report["maximum_z_m"])
        report["checked_profile"] = str(args.check_profile.resolve())
        report["configured_body"] = body
        report["configured_body_contains_collision_envelope"] = valid
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({key:report[key] for key in ("collision_count", "minimum_z_m", "maximum_z_m", "maximum_horizontal_radius_m")}, ensure_ascii=False))
    return 0 if valid else 1


if __name__ == "__main__":
    sys.exit(main())
