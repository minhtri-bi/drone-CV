"""Pinned-model collision proxy and optical mount; not a complete flight oracle."""

import argparse
import hashlib
from itertools import product
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from evidence import Envelope


MODELS = ("x500_depth", "x500", "x500_base", "OakD-Lite")


def rigid(element) -> np.ndarray:
    pose = element.find("pose")
    if pose is not None and pose.get("relative_to"):
        raise ValueError("UNSUPPORTED_RELATIVE_POSE")
    values = np.array([float(v) for v in (element.findtext("pose") or "0 0 0 0 0 0").split()])
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("INVALID_SDF_POSE")
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = [x, y, z]
    return transform


def model_profile(directory: Path) -> dict:
    paths = {name: directory / f"{name}.sdf" for name in MODELS}
    roots = {name: ET.fromstring(path.read_bytes()).find("model") for name, path in paths.items()}
    if any(root is None for root in roots.values()):
        raise ValueError("MISSING_MODEL")
    # This is deliberately a bounded adapter for the pinned merged chain, not a generic SDF resolver.
    expected = {"x500_depth": {"x500", "model://OakD-Lite"}, "x500": {"model://x500_base"},
                "x500_base": set(), "OakD-Lite": set()}
    for name, root in roots.items():
        includes = root.findall("include")
        if (len(includes) != len(expected[name])
                or {item.findtext("uri") for item in includes} != expected[name]
                or any(item.get("merge") != "true" for item in includes)):
            raise ValueError("UNSUPPORTED_MODEL_INCLUDE_CHAIN")
    base = roots["x500_base"]
    base_link = base.find("link[@name='base_link']")
    if base_link is None:
        raise ValueError("BASE_LINK_MISSING")
    x500_include = roots["x500_depth"].find("include[uri='x500']")
    base_include = roots["x500"].find("include[uri='model://x500_base']")
    world_base = rigid(x500_include) @ rigid(roots["x500"]) @ rigid(base_include) @ rigid(base) @ rigid(base_link)
    camera_include = roots["x500_depth"].find("include[uri='model://OakD-Lite']")
    body_camera_model = np.linalg.inv(world_base) @ rigid(camera_include) @ rigid(roots["OakD-Lite"])
    bounds, rotors, meshes = [], [], []
    for model, body_model in ((base, np.linalg.inv(rigid(base_link))), (roots["OakD-Lite"], body_camera_model)):
        for link in model.findall("link"):
            body_link = body_model @ rigid(link)
            rotor = model is base and link.get("name").startswith("rotor_")
            if rotor:
                joint = base.find(f"joint[child='{link.get('name')}']")
                axis = np.array([float(v) for v in joint.findtext("axis/xyz").split()]) if joint is not None else None
                if axis is None or not np.allclose(abs(axis), [0, 0, 1]) or not np.allclose(abs(body_link[:3, 2]), [0, 0, 1]):
                    raise ValueError("UNSUPPORTED_ROTOR_AXIS")
            for collision in link.findall("collision"):
                size_text = collision.findtext("geometry/box/size")
                if size_text is None:
                    raise ValueError("UNSUPPORTED_COLLISION_GEOMETRY")
                size = np.array([float(v) for v in size_text.split()])
                if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
                    raise ValueError("INVALID_COLLISION_BOX")
                local = np.array(list(product((-1., 1.), repeat=3)))*size/2
                transform = rigid(collision)
                in_link = local @ transform[:3, :3].T + transform[:3, 3]
                in_body = in_link @ body_link[:3, :3].T + body_link[:3, 3]
                lo, hi = in_body.min(axis=0), in_body.max(axis=0)
                radius = float(np.linalg.norm(in_body[:, :2], axis=1).max())
                if rotor:
                    disc_radius = float(np.linalg.norm(in_link[:, :2], axis=1).max())
                    hub = body_link[:3, 3]
                    radius = float(np.linalg.norm(hub[:2])+disc_radius)
                    lo[:2], hi[:2] = hub[:2]-disc_radius, hub[:2]+disc_radius
                    rotors.append({"link": link.get("name"), "hub_body": hub.tolist(), "sweep_radius_m": disc_radius})
                bounds.append({"link": link.get("name"), "collision": collision.get("name"),
                               "lo": lo.tolist(), "hi": hi.tolist(), "radial_bound_m": radius})
            meshes.extend(visual.findtext("geometry/mesh/uri") for visual in link.findall("visual")
                          if visual.find("geometry/mesh") is not None)
    if len(rotors) != 4 or not bounds:
        raise ValueError("INCOMPLETE_COLLISION_PROFILE")
    camera_link = roots["OakD-Lite"].find("link[@name='camera_link']")
    sensor = camera_link.find("sensor[@name='StereoOV7251']") if camera_link is not None else None
    if sensor is None:
        raise ValueError("DEPTH_SENSOR_MISSING")
    body_sensor = body_camera_model @ rigid(camera_link) @ rigid(sensor)
    if not np.allclose(body_sensor[:3, :3], np.eye(3), atol=1e-12):
        raise ValueError("UNSUPPORTED_SENSOR_RPY")
    return {"schema": 1, "frame": "base_link FLU", "physical_geometry_source": "collision boxes plus full rotor collision sweep",
            "z_lo_m": min(row["lo"][2] for row in bounds), "z_hi_m": max(row["hi"][2] for row in bounds),
            "radius_m": max(row["radial_bound_m"] for row in bounds),
            "optical_translation_flu": body_sensor[:3, 3].tolist(), "collision_bounds": bounds,
            "rotors": rotors, "visual_meshes_not_bounded": sorted(set(meshes)),
            "source_sha256": {name+"/model.sdf": hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()},
            "flight_admission": False,
            "limitations": ["Visual meshes are identified but not bounded; collision completeness is not certified.",
                            "Yaw/level swept proxy only; tilted-body sweep and PX4 tracking are not modeled.",
                            "This profile is not a contact ledger, phase allowlist, calibrated uncertainty or continuous coverage proof."]}


def diagnostic_envelope(profile: dict, pose, distance_m: float, *, clearance_m: float,
                        pose_xy_m: float, pose_z_m: float) -> Envelope:
    values = (distance_m, clearance_m, pose_xy_m, pose_z_m)
    if any(not np.isfinite(value) or value < 0 for value in values):
        raise ValueError("INVALID_DIAGNOSTIC_INFLATION")
    if not np.allclose(pose.world_from_body()[:, 2], [0, 0, 1], atol=1e-10, rtol=0):
        raise ValueError("UNSUPPORTED_TILTED_BODY_ENVELOPE")
    return Envelope((pose.x, pose.y), (pose.x+distance_m*np.cos(pose.yaw_rad), pose.y+distance_m*np.sin(pose.yaw_rad)),
                    profile["radius_m"]+clearance_m+pose_xy_m,
                    pose.z+profile["z_lo_m"]-clearance_m-pose_z_m,
                    pose.z+profile["z_hi_m"]+clearance_m+pose_z_m)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = model_profile(args.models)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(profile, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: profile[key] for key in ("z_lo_m", "z_hi_m", "radius_m", "optical_translation_flu", "flight_admission")}))


if __name__ == "__main__":
    main()
