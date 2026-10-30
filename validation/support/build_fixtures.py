#!/usr/bin/env python3
"""Build evaluation-only high-level, changed-surface and equipment-gap probes.

The output belongs in a run directory. This does not alter the committed model,
and none of these coordinates may be used by online flight or perception.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np


class DeadlineReached(RuntimeError):
    pass


def sphere_first_contact(nearest, start, end, radius, tolerance=1e-7):
    """First contact of a finite sphere, using the surface distance bound.

    Distance to a closed surface is 1-Lipschitz. Advancing by less than
    distance-radius cannot skip an earlier contact, including off-axis edges.
    The nearest query is independent of any Gazebo contact observation.
    """
    start, end = np.asarray(start, float), np.asarray(end, float)
    length = float(np.linalg.norm(end-start))
    if length <= 0 or radius <= 0 or not np.isfinite([*start, *end, radius]).all():
        raise ValueError("finite nonzero sweep and positive radius required")
    direction, travel = (end-start)/length, 0.
    if nearest(start)[1] <= radius:
        raise ValueError("sphere initial point is already touching the surface")
    for _ in range(2000):
        center = start+direction*travel
        surface, distance = nearest(center)[:2]
        gap = float(distance)-radius
        if gap <= tolerance:
            return {"center": center, "surface": np.asarray(surface), "travel_m": travel}
        if gap > length-travel:
            return None
        travel += .9*gap
    raise ValueError("sphere sweep failed to converge; fixture remains unconfirmed")


def source_translation(world: Path, model: Path) -> np.ndarray:
    """The actual asset has a translation-only include and unit mesh scale."""
    matches = [x for x in ET.parse(world).getroot().findall("world/include")
               if x.findtext("uri", "").rstrip("/") == "model://power-plant"]
    if len(matches) != 1:
        raise ValueError("Expected one actual power-plant include")
    model_xml = ET.parse(model / "model.sdf").getroot()
    pose = matches[0].findtext("pose")
    if pose is None:
        pose = model_xml.findtext("model/pose", "0 0 0 0 0 0")
    values = np.asarray([float(x) for x in pose.split()])
    if len(values) != 6 or not np.isfinite(values).all() or np.any(values[3:] != 0):
        raise ValueError("Fixture generator requires the actual translation-only model pose")
    for mesh in model_xml.findall(".//mesh"):
        scale = np.asarray([float(x) for x in mesh.findtext("scale", "1 1 1").split()])
        if scale.shape != (3,) or not np.array_equal(scale, np.ones(3)):
            raise ValueError("Fixture generator requires unit mesh scale")
    return values[:3]


def generate(args):
    # Import the exact committed cleaning/locator implementation without running
    # its builder. The generator and imported helper hashes are both recorded.
    model = args.px4_root / "Tools/simulation/gz/models/power-plant"
    world = args.px4_root / "Tools/simulation/gz/worlds/powerplant.sdf"
    helper_path = model / "scripts/build_collision.py"
    spec = importlib.util.spec_from_file_location("issue3_collision_helpers", helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from vtkmodules.vtkCommonCore import reference
    from vtkmodules.vtkFiltersGeneral import vtkOBBTree

    start_wall = time.monotonic()
    deadline = start_wall + args.max_duration

    def check():
        if time.monotonic() >= deadline:
            raise DeadlineReached("bounded fixture-generation deadline reached")

    shift = source_translation(world, model)
    source = model / "meshes/power-plant.stl"
    collision = model / "meshes/power-plant-collision.stl"
    vertices, faces, _ = helper.read_clean(source)
    cv, cf, _ = helper.read_clean(collision)
    source_loc, collision_loc = helper.locator(vertices, faces), helper.locator(cv, cf)
    check()
    edges, counts = helper.topology(vertices, faces)
    graph = coo_matrix((np.ones(len(edges), dtype=np.uint8), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    _, labels = connected_components(graph, directed=False)
    face_labels = labels[faces[:, 0]]
    nonclosed = set(map(int, labels[edges[counts != 2, 0]]))
    component_faces, bounds, trees = {}, {}, {}
    for component in sorted(set(map(int, face_labels))):
        ids = np.flatnonzero(face_labels == component)
        component_faces[component] = ids
        points = vertices[np.unique(faces[ids])]
        bounds[component] = points.min(0), points.max(0)
    check()

    def outside_all(point):
        check()
        for component, (low, high) in bounds.items():
            if np.all(point >= low) and np.all(point <= high):
                # Surface distance alone cannot prove an initial point outside
                # a solid. Reject ambiguous open-component AABB interiors.
                if component in nonclosed:
                    return False
                if component not in trees:
                    tree = vtkOBBTree()
                    tree.SetDataSet(helper.poly(vertices, faces[component_faces[component]]))
                    tree.BuildLocator()
                    trees[component] = tree
                if trees[component].InsideOrOutside(point) != 1:
                    return False
        return True

    def first_hit(loc, start, end):
        check()
        t = reference(0.)
        position, coords = [0., 0., 0.], [0., 0., 0.]
        sub, cell = reference(0), reference(0)
        hit = loc.IntersectWithLine(start, end, 1e-8, t, position, coords, sub, cell)
        return (np.asarray(position), int(cell)) if hit else None

    def clearances(points):
        check()
        return helper.distances(points, source_loc), helper.distances(points, collision_loc)

    def nearest_surface(loc, point):
        check()
        closest, cell, sub, squared = [0., 0., 0.], reference(0), reference(0), reference(0.)
        loc.FindClosestPoint(point, closest, cell, sub, squared)
        return np.asarray(closest), float(squared)**.5, int(cell)

    tri = vertices[faces]
    centers = tri.mean(1)
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area2 = np.linalg.norm(cross, axis=1)
    normals = cross / area2[:, None]
    # Large column side triangles often span the entire height, with centroids
    # below the inspection band. For such faces sample the middle of the actual
    # triangle/height-plane segment, rather than rejecting a valid high surface.
    world_low, world_high = tri.min(1) + shift, tri.max(1) + shift
    crossing_band = ((world_high[:, 0] >= -24) & (world_low[:, 0] <= 34)
                     & (world_high[:, 1] >= -9) & (world_low[:, 1] <= 49)
                     & (world_high[:, 2] >= args.height_min)
                     & (world_low[:, 2] <= args.height_max))
    centroid_height = centers[:, 2] + shift[2]
    for face in np.flatnonzero(crossing_band & ((centroid_height < args.height_min)
                                               | (centroid_height > args.height_max))):
        z = .5 * (max(world_low[face, 2], args.height_min)
                  + min(world_high[face, 2], args.height_max)) - shift[2]
        intersections = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            first, second = tri[face, a], tri[face, b]
            if first[2] == second[2]:
                if first[2] == z:
                    intersections.extend((first, second))
            elif min(first[2], second[2]) <= z <= max(first[2], second[2]):
                intersections.append(first + (second - first) * ((z - first[2]) / (second[2] - first[2])))
        if intersections:
            centers[face] = np.mean(intersections, axis=0)
    world_centers = centers + shift
    mask = ((world_centers[:, 0] >= -24) & (world_centers[:, 0] <= 34)
            & (world_centers[:, 1] >= -9) & (world_centers[:, 1] <= 49)
            & (world_centers[:, 2] >= args.height_min)
            & (world_centers[:, 2] <= args.height_max) & (area2 > 2e-6))
    # Height/spatial/normal strata prevent large low plates consuming every
    # candidate, unlike selecting only the largest few faces per component.
    groups = {}
    for face in np.flatnonzero(mask):
        point = world_centers[face]
        bucket = (int(face_labels[face]), int(np.argmax(np.abs(normals[face]))),
                  int(np.floor(point[2])), int(np.floor(point[0] / 3)),
                  int(np.floor(point[1] / 3)))
        groups.setdefault(bucket, []).append(int(face))
    for bucket in groups:
        groups[bucket].sort(key=lambda face: (-area2[face], face))
    candidates = []
    for depth in range(12):
        for bucket in sorted(groups):
            if depth < len(groups[bucket]):
                candidates.append(groups[bucket][depth])
                if len(candidates) >= args.max_candidates:
                    break
        if len(candidates) >= args.max_candidates:
            break
    changed_distance = helper.distances(centers[candidates], collision_loc)
    changed = dict(zip(candidates, changed_distance))
    check()
    cases, paths = [], []
    attempted = 0
    rejected = {"no_two_verified_sides": 0, "changed_first_hit_not_confirmed": 0}

    def surface_case(face, category):
        p, normal = centers[face], normals[face]
        component = int(face_labels[face])
        directions = []
        for sign in (1, -1):
            found = None
            for offset in (.2, .3, .5, .8, 1.2, 2., 3.):
                begin, end = p + sign * normal * offset, p - sign * normal * .1
                da, db = clearances([begin])
                if min(da[0], db[0]) < .1 or not outside_all(begin):
                    continue
                a, b = first_hit(source_loc, begin, end), first_hit(collision_loc, begin, end)
                if a is None or b is None:
                    continue
                if int(face_labels[a[1]]) != component or np.linalg.norm(a[0] - b[0]) > .025:
                    continue
                if not all(args.height_min <= x[2] + shift[2] <= args.height_max
                           for x in (a[0], b[0])):
                    continue
                distance = np.linalg.norm(a[0] - begin)
                if distance <= .1:
                    continue
                sa = sphere_first_contact(lambda q: nearest_surface(source_loc, q), begin, end, .05)
                sb = sphere_first_contact(lambda q: nearest_surface(collision_loc, q), begin, end, .05)
                if sa is None or sb is None or np.linalg.norm(sa['surface']-sb['surface']) > .025:
                    continue
                if not all(args.height_min <= item['surface'][2]+shift[2] <= args.height_max
                           for item in (sa, sb)):
                    continue
                if int(face_labels[nearest_surface(source_loc, sa['center'])[2]]) != component:
                    continue
                found = {
                    "side": sign, "start_world_m": (begin + shift).tolist(),
                    "end_world_m": (end + shift).tolist(),
                    "velocity_world_mps": (-sign * normal * .05).tolist(),
                    "expected_source_surface_world_m": (sa['surface'] + shift).tolist(),
                    "expected_collision_surface_world_m": (sb['surface'] + shift).tolist(),
                    "source_centerline_hit_world_m": (a[0] + shift).tolist(),
                    "collision_centerline_hit_world_m": (b[0] + shift).tolist(),
                    "source_sphere_contact_center_world_m": (sa['center']+shift).tolist(),
                    "source_sphere_contact_time_s": sa['travel_m']/.05,
                    "source_collision_sphere_contact_difference_m": float(np.linalg.norm(sa['surface']-sb['surface'])),
                    "contact_prediction": "independent finite-radius sphere trace against both complete surfaces",
                    "source_start_clearance_m": float(da[0]),
                    "collision_start_clearance_m": float(db[0]),
                    "outside_closed_source_components": True, "probe_radius_m": .05,
                    "source_collision_first_hit_difference_m": float(np.linalg.norm(a[0] - b[0])),
                    "seed_to_first_hit_distance_m": float(np.linalg.norm(a[0] - p)),
                    "source_first_hit_face": a[1], "collision_first_hit_face": b[1],
                    "category": category,
                }
                break
            if found is None:
                rejected["no_two_verified_sides"] += 1
                return None
            directions.append(found)
        if category == "changed_high_surface" and not any(
                x["source_collision_sphere_contact_difference_m"] >= .001 for x in directions):
            rejected["changed_first_hit_not_confirmed"] += 1
            return None
        lo, hi = bounds[component]
        dimensions = hi - lo
        vertical = abs(normal[2]) < .3
        slender = dimensions[2] >= 1.5 and max(dimensions[:2]) <= 1.5
        return {
            "id": f"supplemental_{len(cases):02d}", "category": category,
            "component": component, "source_face": face,
            "surface_world_m": (p + shift).tolist(), "normal": normal.tolist(),
            "component_extent_m": dimensions.tolist(),
            "geometric_feature": "vertical_slender_component" if vertical and slender
                                 else "vertical_equipment_surface" if vertical else "equipment_surface",
            "source_sample_collision_distance_m": float(changed[face]),
            "directions": directions,
        }

    def separated(face, gap=.6):
        return all(np.linalg.norm(world_centers[face] - np.asarray(c["surface_world_m"])) >= gap
                   for c in cases)

    timed_out = False
    try:
        changed_candidates = sorted((x for x in candidates if changed[x] >= .001),
                                    key=lambda x: (-changed[x], x))
        for category, sequence, required in (
                ("changed_high_surface", changed_candidates, args.changed_cases),
                ("high_column_surface", candidates, args.column_cases),
                ("high_equipment_surface", candidates, args.high_cases)):
            selected = 0
            for face in sequence:
                check()
                if not separated(face):
                    continue
                # Prioritize upright sides in the equipment group; changed
                # geometry keeps arbitrary normal directions.
                if category != "changed_high_surface" and abs(normals[face, 2]) >= .3:
                    continue
                if category == "high_column_surface":
                    low, high = bounds[int(face_labels[face])]
                    dimensions = high - low
                    if dimensions[2] < 1.5 or max(dimensions[:2]) > 1.5:
                        continue
                attempted += 1
                case = surface_case(face, category)
                if case:
                    cases.append(case)
                    selected += 1
                if selected >= required:
                    break
        # Gaps are discovered from actual surfaces, never hand-selected routes.
        # Require source/collision agreement for both facing side obstacles at
        # each of three cross-sections, and different connected components.
        gap_attempts = 0
        for face in candidates:
            check()
            normal = normals[face].copy()
            normal[2] = 0
            norm = np.linalg.norm(normal)
            if norm < .9:
                continue
            normal /= norm
            tangent = np.array([-normal[1], normal[0], 0.])
            for offset in (.8, 1.2, 1.8, 2.5):
                check()
                gap_attempts += 1
                if gap_attempts > args.max_gap_candidates:
                    break
                midpoint = centers[face] + normal * offset
                if any(np.linalg.norm(midpoint + shift - .5 * (
                        np.asarray(p["start_world_m"]) + np.asarray(p["end_world_m"]))) < 2.
                       for p in paths):
                    continue
                begin, end = midpoint - tangent, midpoint + tangent
                if not outside_all(begin):
                    continue
                length = float(np.linalg.norm(end - begin))
                count = int(np.ceil(length / .025)) + 1
                samples = np.linspace(begin, end, count)
                da, db = clearances(samples)
                # Distance to a triangle surface union is 1-Lipschitz. Since
                # the initial point is outside every source solid and no part
                # of the line can touch either mesh, it cannot enter a solid.
                lower = float(min(da.min(), db.min()) - length / (count - 1) / 2)
                if lower < .55:
                    continue
                sections = []
                for center in (begin, midpoint, end):
                    walls = []
                    for sign in (-1, 1):
                        a = first_hit(source_loc, center, center + normal * sign * 5.)
                        b = first_hit(collision_loc, center, center + normal * sign * 5.)
                        if a is None or b is None or np.linalg.norm(a[0] - b[0]) > .025:
                            break
                        walls.append({"component": int(face_labels[a[1]]),
                                      "source_hit_world_m": (a[0] + shift).tolist(),
                                      "collision_hit_world_m": (b[0] + shift).tolist(),
                                      "ray_difference_m": float(np.linalg.norm(a[0] - b[0]))})
                    if len(walls) != 2 or walls[0]["component"] == walls[1]["component"]:
                        break
                    sections.append(walls)
                if len(sections) != 3:
                    continue
                paths.append({"id": f"equipment_gap_{len(paths):02d}",
                              "start_world_m": (begin + shift).tolist(),
                              "end_world_m": (end + shift).tolist(), "probe_radius_m": .05,
                              "sample_count": count, "sample_spacing_m": length / (count - 1),
                              "continuous_surface_clearance_lower_bound_m": lower,
                              "initial_point_outside_source_components": True,
                              "facing_equipment_cross_sections": sections,
                              "scope": "physical probe passage; not a flight or braking certificate"})
                break
            if len(paths) >= args.paths or gap_attempts > args.max_gap_candidates:
                break
    except DeadlineReached:
        timed_out = True
    category_counts = {name: sum(c["category"] == name for c in cases)
                       for name in ("changed_high_surface", "high_column_surface", "high_equipment_surface")}
    qualification = {
        "changed_surfaces": category_counts["changed_high_surface"] >= args.changed_cases,
        "high_equipment_surfaces": category_counts["high_equipment_surface"] >= args.high_cases,
        "high_column_surfaces": category_counts["high_column_surface"] >= args.column_cases,
        "verified_equipment_passages": len(paths) >= args.paths,
        "completed_within_generation_budget": not timed_out,
    }
    result = {
        "evaluation_only": True, "status": "PASS" if all(qualification.values()) else "UNCONFIRMED",
        "fixture_qualification": {"passed": all(qualification.values()), "checks": qualification},
        "source_sha256": helper.sha(source), "collision_sha256": helper.sha(collision),
        "world_sha256": helper.sha(world), "generator_sha256": helper.sha(Path(__file__)),
        "imported_helper_sha256": helper.sha(helper_path), "world_translation_m": shift.tolist(),
        "surface_cases": cases, "free_paths": paths, "probe_radius_m": .05, "speed_mps": .05,
        "height_band_world_m": [args.height_min, args.height_max],
        "generation": {"candidate_limit": args.max_candidates, "eligible_faces": int(mask.sum()),
                       "stratified_candidates": len(candidates), "surface_attempts": attempted,
                       "changed_centroid_candidates": len(changed_candidates), "rejections": rejected,
                       "duration_wall_s": time.monotonic() - start_wall, "deadline_s": args.max_duration},
        "free_start_method": "Both mesh surface clearances; source closed-component OBB parity; "
                             "reject ambiguous open-component bounding boxes; first-hit agreement.",
        "notice": "Finite independent evaluation fixtures, not exhaustive geometry or flight clearance. "
                  "These coordinates must never become online flight/perception inputs.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-root", type=Path, default=Path("/home/pawn/drone-simulation"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--height-min", type=float, default=3.5)
    parser.add_argument("--height-max", type=float, default=7.)
    parser.add_argument("--high-cases", type=int, default=4)
    parser.add_argument("--column-cases", type=int, default=1)
    parser.add_argument("--changed-cases", type=int, default=2)
    parser.add_argument("--paths", type=int, default=1)
    parser.add_argument("--max-candidates", type=int, default=6000)
    parser.add_argument("--max-gap-candidates", type=int, default=1500)
    parser.add_argument("--max-duration", type=float, default=180.)
    args = parser.parse_args()
    if (not np.isfinite([args.height_min, args.height_max, args.max_duration]).all()
            or args.height_min >= args.height_max or args.max_duration <= 0
            or min(args.high_cases, args.column_cases, args.changed_cases, args.paths, args.max_candidates,
                   args.max_gap_candidates) < 1):
        parser.error("Expected finite increasing height band and positive limits")
    result = generate(args)
    print(json.dumps({"status": result["status"], "qualification": result["fixture_qualification"],
                      "surface_cases": len(result["surface_cases"]), "free_paths": len(result["free_paths"]),
                      "generation": result["generation"]}, indent=2), flush=True)
    return 0 if result["fixture_qualification"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
