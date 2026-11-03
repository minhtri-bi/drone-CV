"""Frozen target geometry is grader-only, never imported by the evidence gate."""

import xml.etree.ElementTree as ET

import numpy as np


def build_world(base_xml, fixture):
    root = ET.fromstring(base_xml.lstrip("\ufeff"))
    world = root.find("world")
    if world is None or world.get("name") != fixture["world_name"]:
        raise ValueError("fixture world name mismatch")
    for box in fixture["boxes"]:
        lo, hi = np.array(box["lo"]), np.array(box["hi"])
        model = ET.SubElement(world, "model", name=box["name"])
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(map(str, [*((lo+hi)/2), 0, 0, 0]))
        link = ET.SubElement(model, "link", name="target")
        for kind in ("collision", "visual"):
            part = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(part, "geometry")
            shape = ET.SubElement(geometry, "box")
            ET.SubElement(shape, "size").text = " ".join(map(str, hi-lo))
    return ET.tostring(root, encoding="unicode")


def aligned_camera(camera, pose, world_from_odom):
    alignment = np.asarray(world_from_odom, dtype=np.float64)
    if (alignment.shape != (4, 4) or not np.isfinite(alignment).all()
            or not np.allclose(alignment[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(alignment[:3, :3].T @ alignment[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(alignment[:3, :3]), 1, atol=1e-8)):
        raise ValueError("invalid world-from-odometry rigid alignment")
    rotation, origin = camera.world_from_optical(pose)
    return alignment[:3, :3] @ rotation, alignment[:3, :3] @ origin + alignment[:3, 3]


def grade_frame(depth, camera, pose, world_from_odom, fixture):
    rotation, origin = aligned_camera(camera, pose, world_from_odom)
    k = camera.intrinsics
    if depth.shape != (k.height, k.width):
        raise ValueError("depth/calibration shape mismatch")
    v, u = np.mgrid[:k.height, :k.width]
    rays = np.stack(((u-k.cx)/k.fx, (v-k.cy)/k.fy, np.ones_like(u)), axis=-1) @ rotation.T
    targets = {}
    for target in fixture["targets"]:
        axis = target["axis"]
        with np.errstate(divide="ignore", invalid="ignore"):
            expected_z = (target["coordinate"]-origin[axis])/rays[..., axis]
            intersections = origin + rays*expected_z[..., None]
        lo, hi = np.array(target["lo"]), np.array(target["hi"])
        other_axes = [index for index in range(3) if index != axis]
        roi = np.isfinite(expected_z) & (expected_z > camera.near_m) & (expected_z < camera.far_m)
        for index in other_axes:
            roi &= (intersections[..., index] >= lo[index]) & (intersections[..., index] <= hi[index])
        edge = fixture["edge_pixels"]
        padded = np.pad(roi, edge, constant_values=False)
        eroded = np.ones_like(roi)
        for dy in range(2*edge+1):
            for dx in range(2*edge+1):
                eroded &= padded[dy:dy+k.height, dx:dx+k.width]
        total = int(eroded.sum())
        valid = eroded & np.isfinite(depth) & (depth >= camera.near_m) & (depth <= camera.far_m)
        points = origin + rays[valid]*depth[valid, None]
        # ROI chooses rays; physical surface bounds determine point-to-surface error.
        # In particular, an interior ground ROI is not the edge of the ground plane.
        surface_lo = np.array(target.get("surface_lo", target["lo"]))
        surface_hi = np.array(target.get("surface_hi", target["hi"]))
        distances = np.linalg.norm(points-np.clip(points, surface_lo, surface_hi), axis=1)
        support = float(valid.sum()/total) if total else 0.0
        p95 = float(np.quantile(distances, .95)) if distances.size else None
        maximum = float(distances.max()) if distances.size else None
        passed = bool(total and support >= fixture["minimum_target_support"]
                      and p95 is not None and p95 <= fixture["p95_error_m"]
                      and maximum <= fixture["maximum_error_m"])
        targets[target["name"]] = {"roi_pixels": total, "valid_pixels": int(valid.sum()),
                                  "support": support, "p95_m": p95,
                                  "maximum_m": maximum, "pass": passed}
    return {"targets": targets, "all_pass": all(row["pass"] for row in targets.values())}
