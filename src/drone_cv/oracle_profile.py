"""Pinned SDFormat-resolved physical bounds for the evaluator, never motion admission."""

import hashlib
from importlib import import_module
from itertools import product
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from drone_cv.frames import rotation_wxyz
from drone_cv.oracle_mesh import mesh_vertices


PINNED_MODELS = ("x500_depth", "x500", "x500_base", "OakD-Lite")
GEAR_NODES = frozenset(("LandingFoam", "LandingRubber", "LandingPlastic"))
GEAR_COLLISIONS = frozenset(f"base_link_collision_{index}" for index in range(1, 5))


def _vector(value) -> NDArray[np.float64]:
    return np.array([value.x(), value.y(), value.z()], dtype=np.float64)


def _pose_matrix(pose) -> NDArray[np.float64]:
    rotation = pose.rot()
    result = np.eye(4)
    result[:3, :3] = rotation_wxyz([rotation.w(), rotation.x(), rotation.y(), rotation.z()])
    result[:3, 3] = _vector(pose.pos())
    return result


def rotor_bounds(points: NDArray, hub: NDArray, axis: NDArray) -> tuple[NDArray, NDArray, float]:
    """Enclose all rotations of points around a joint line, not the mesh origin."""
    points, hub, axis = (np.asarray(value, dtype=np.float64) for value in (points, hub, axis))
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) == 0
            or hub.shape != (3,) or axis.shape != (3,)
            or not all(np.isfinite(value).all() for value in (points, hub, axis))
            or not np.isclose(np.linalg.norm(axis), 1., atol=1e-10, rtol=0)):
        raise ValueError("INVALID_ROTOR_GEOMETRY")
    offsets = points-hub
    axial = offsets @ axis
    radial = offsets-axial[:, None]*axis
    radius = float(np.linalg.norm(radial, axis=1).max())
    # An arbitrary-axis full circular sweep has coordinate radius r*sqrt(1-a_i^2).
    extent = radius*np.sqrt(np.maximum(0., 1.-axis*axis))
    axial_ends = np.stack((hub+axial.min()*axis, hub+axial.max()*axis))
    return axial_ends.min(axis=0)-extent, axial_ends.max(axis=0)+extent, radius


def _mesh_path(models: Path, uri: str) -> Path:
    if not uri.startswith("model://") or "\\" in uri:
        raise ValueError("UNSUPPORTED_MESH_URI")
    relative = Path(uri.removeprefix("model://"))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("MESH_PATH_ESCAPE")
    target = (models/relative).resolve(strict=True)
    if not target.is_relative_to(models.resolve(strict=True)) or not target.is_file():
        raise ValueError("MESH_PATH_ESCAPE")
    return target


def _primitive_points(geometry) -> NDArray[np.float64]:
    box = geometry.box_shape()
    if box is not None:
        size = _vector(box.size())
        if not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError("INVALID_PHYSICAL_BOX")
        return np.array(tuple(product((-1., 1.), repeat=3)))*size/2
    plane = geometry.plane_shape()
    if plane is not None:
        normal = _vector(plane.normal())
        size = np.array([plane.size().x(), plane.size().y()])
        if (not np.isfinite(normal).all() or not np.isfinite(size).all()
                or np.linalg.norm(normal) == 0 or np.any(size <= 0)):
            raise ValueError("INVALID_PHYSICAL_PLANE")
        if not np.allclose(normal/np.linalg.norm(normal), [0., 0., 1.], atol=1e-12, rtol=0):
            raise ValueError("UNSUPPORTED_PHYSICAL_PLANE_NORMAL")
        return np.array([[a*size[0]/2, b*size[1]/2, 0.]
                         for a, b in product((-1., 1.), repeat=2)])
    raise ValueError("UNSUPPORTED_PHYSICAL_PRIMITIVE")


def _part(points: NDArray, transform: NDArray, part_id: str, role: str,
          source: str, rotor: tuple | None) -> dict:
    body = points @ transform[:3, :3].T+transform[:3, 3]
    if body.ndim != 2 or body.shape[1] != 3 or len(body) == 0 or not np.isfinite(body).all():
        raise ValueError("INVALID_PHYSICAL_POINTS")
    lower, upper = body.min(axis=0), body.max(axis=0)
    extra = {}
    if rotor is not None:
        lower, upper, radius = rotor_bounds(body, *rotor)
        extra = {"joint_origin_body": rotor[0].tolist(), "joint_axis_body": rotor[1].tolist(),
                 "sweep_radius_m": radius}
    return {"id": part_id, "role": role, "lo": lower.tolist(), "hi": upper.tolist(),
            "source": source, "point_count": len(points), **extra}


def build_profile(models: Path) -> dict:
    """Requires the existing pinned image's sdformat14 binding; never installs one."""
    import os
    sdf = import_module("sdformat14")

    models = models.resolve(strict=True)
    os.environ["SDF_PATH"] = str(models)
    root = sdf.Root()
    root.load(str(models/"x500_depth/model.sdf"))
    model = root.model()
    if model is None or model.name() != "x500_depth" or model.model_count() != 0:
        raise ValueError("UNSUPPORTED_RESOLVED_MODEL")
    hashes = {f"{name}/model.sdf": hashlib.sha256((models/name/"model.sdf").read_bytes()).hexdigest()
              for name in PINNED_MODELS}
    joints = {model.joint_by_index(index).child_name(): model.joint_by_index(index)
              for index in range(model.joint_count())}
    parts, mesh_cache, resolved_links = [], {}, {}
    for index in range(model.link_count()):
        link = model.link_by_index(index)
        link_name = link.name()
        resolved_links[link_name] = _pose_matrix(link.semantic_pose().resolve("base_link")).tolist()
        rotor = None
        if link_name.startswith("rotor_"):
            joint = joints.get(link_name)
            if joint is None or joint.axis(0) is None:
                raise ValueError("ROTOR_JOINT_MISSING")
            joint_pose = _pose_matrix(joint.semantic_pose().resolve("base_link"))
            axis = _vector(joint.axis(0).resolve_xyz("base_link"))
            rotor = (joint_pose[:3, 3], axis)
        for category, count, getter in (("collision", link.collision_count(), link.collision_by_index),
                                         ("visual", link.visual_count(), link.visual_by_index)):
            for item_index in range(count):
                item = getter(item_index)
                transform = _pose_matrix(item.semantic_pose().resolve("base_link"))
                geometry = item.geometry()
                mesh = geometry.mesh_shape()
                base_id = f"{category}/{link_name}/{item.name()}"
                role = "rotor" if rotor is not None else "body"
                if link_name == "base_link" and category == "collision" and item.name() in GEAR_COLLISIONS:
                    role = "gear"
                if mesh is None:
                    points = _primitive_points(geometry)
                    parts.append(_part(points, transform, base_id, role, "resolved SDF primitive", rotor))
                    continue
                if mesh.submesh() or mesh.center_submesh():
                    raise ValueError("UNSUPPORTED_MESH_SUBMESH")
                path = _mesh_path(models, mesh.uri())
                relative = path.relative_to(models).as_posix()
                hashes[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
                if relative not in mesh_cache:
                    mesh_cache[relative] = mesh_vertices(path)
                scale = _vector(mesh.scale())
                if not np.isfinite(scale).all() or np.any(scale <= 0):
                    raise ValueError("INVALID_MESH_SCALE")
                for node_name, vertices in mesh_cache[relative]:
                    node_role = role
                    if (link_name == "base_link" and item.name() == "base_link_visual"
                            and node_name.split("/")[-1].split("[", 1)[0] in GEAR_NODES):
                        node_role = "gear"
                    parts.append(_part(vertices*scale, transform, f"{base_id}/{node_name}",
                                       node_role, relative, rotor))
    if len(parts) == 0 or len({part["id"] for part in parts}) != len(parts):
        raise ValueError("EMPTY_OR_AMBIGUOUS_PHYSICAL_PROFILE")
    optical = None
    camera_link = model.link_by_name("camera_link")
    if camera_link is not None:
        sensor = camera_link.sensor_by_name("StereoOV7251")
        if sensor is not None:
            optical = _pose_matrix(sensor.semantic_pose().resolve("base_link"))
    if optical is None:
        raise ValueError("PINNED_OPTICAL_SENSOR_MISSING")
    if not np.allclose(optical[:3, 3], [.13233, 0., .02078], atol=1e-9, rtol=0):
        raise ValueError("RESOLVED_OPTICAL_MOUNT_CHANGED")
    return {"schema": 1, "frame": "base_link FLU", "parts": parts, "source_sha256": hashes,
            "resolved_links": resolved_links, "optical_mount_body": optical.tolist(),
            "model_from_body": _pose_matrix(model.link_by_name("base_link").semantic_pose().resolve("__model__")).tolist(),
            "resolver": "pinned sdformat14 semantic_pose().resolve(base_link)",
            "geometry_scope": "Per-instance conservative AABBs of collision and visual geometry; full rotor sweeps",
            "z_lo_m": min(part["lo"][2] for part in parts),
            "z_hi_m": max(part["hi"][2] for part in parts),
            "flight_admission": False, "physics_contact_sensor_added": False}
