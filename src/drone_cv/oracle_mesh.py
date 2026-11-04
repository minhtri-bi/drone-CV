"""Bounded offline STL/COLLADA point extraction, not a general asset importer.

COLLADA text matrices contain rows; algebra uses column vectors, so a node's
ordered transforms postmultiply and nested nodes compose parent @ local. Asset
units and the Y-up to Z-up basis change apply after the complete scene transform.
Only the active visual scene contributes physical instances. Names retain the
scene/node/instance path; no name or material implies a contact exemption.
"""

from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np


_MAX_BYTES = 64 * 1024 * 1024
_MAX_NUMBERS = 8_000_000
_MAX_POINTS = 2_000_000
_MAX_ELEMENTS = 200_000
_MAX_INSTANCES = 10_000
_MAX_DEPTH = 128
_INTEGER = re.compile(r"[0-9]+\Z", re.ASCII)
_FLOAT = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?\Z", re.ASCII)
_TRANSFORMS = {"matrix", "translate", "rotate", "scale"}


def _integer(text: str | None, name: str, *, default: int | None = None) -> int:
    if text is None and default is not None:
        return default
    if text is None or len(text) > 10 or _INTEGER.fullmatch(text) is None:
        raise ValueError(f"{name} must be a bounded nonnegative integer")
    value = int(text)
    if value > _MAX_NUMBERS:
        raise ValueError(f"{name} exceeds mesh numeric budget")
    return value


def _numbers(text: str | None, count: int | None = None) -> np.ndarray:
    tokens = (text or "").split()
    if not tokens or len(tokens) > _MAX_NUMBERS or (count is not None and len(tokens) != count):
        raise ValueError("mesh numeric length mismatch or budget exceeded")
    if any(_FLOAT.fullmatch(token) is None for token in tokens):
        raise ValueError("invalid mesh floating-point syntax")
    try:
        values = np.array([float(token) for token in tokens], dtype=np.float64)
    except (ValueError, OverflowError) as error:
        raise ValueError("invalid mesh number") from error
    if not np.isfinite(values).all():
        raise ValueError("mesh numbers must be finite")
    return values


def _readonly(points: np.ndarray) -> np.ndarray:
    if points.ndim != 2 or points.shape[1] != 3 or not 0 < len(points) <= _MAX_POINTS:
        raise ValueError("mesh must contain bounded nonempty 3D points")
    if not np.isfinite(points).all():
        raise ValueError("transformed mesh positions must be finite")
    result = np.array(points, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def _stl(data: bytes, name: str) -> tuple[tuple[str, np.ndarray], ...]:
    # Length, not the ambiguous 'solid' prefix, identifies binary STL.
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            if not 0 < count <= _MAX_POINTS // 3:
                raise ValueError("binary STL facet budget exceeded or empty mesh")
            records = np.frombuffer(data, dtype=np.dtype([
                ("numbers", "<f4", (12,)), ("attribute", "<u2")]), offset=84, count=count)
            if not np.isfinite(records["numbers"]).all():
                raise ValueError("binary STL normals and vertices must be finite")
            points = records["numbers"][:, 3:].reshape(-1, 3)
            return ((name, _readonly(points)),)
    try:
        lines = [line.strip() for line in data.decode("ascii").splitlines() if line.strip()]
    except UnicodeDecodeError as error:
        raise ValueError("invalid or truncated binary STL") from error
    if len(lines) < 2 or lines[0].split(maxsplit=1)[0] != "solid":
        raise ValueError("invalid STL header or binary length")
    solid_name = lines[0][len("solid"):].strip()
    if lines[-1] != ("endsolid " + solid_name).strip():
        raise ValueError("ASCII STL solid terminator mismatch")
    facets = lines[1:-1]
    if not facets or len(facets) % 7 or len(facets) // 7 > _MAX_POINTS // 3:
        raise ValueError("ASCII STL facet length or budget mismatch")
    points = np.empty((len(facets) // 7 * 3, 3), dtype=np.float64)
    for offset in range(0, len(facets), 7):
        facet = facets[offset:offset + 7]
        normal = facet[0].split()
        if normal[:2] != ["facet", "normal"] or len(normal) != 5:
            raise ValueError("invalid ASCII STL facet normal")
        _numbers(" ".join(normal[2:]), 3)
        if facet[1] != "outer loop" or facet[5:] != ["endloop", "endfacet"]:
            raise ValueError("invalid ASCII STL facet grammar")
        for index in range(3):
            vertex = facet[index + 2].split()
            if len(vertex) != 4 or vertex[0] != "vertex":
                raise ValueError("invalid ASCII STL vertex")
            points[offset // 7 * 3 + index] = _numbers(" ".join(vertex[1:]), 3)
    return ((name, _readonly(points)),)


def _one(element: ET.Element, tag: str) -> ET.Element:
    children = element.findall(tag)
    if len(children) != 1:
        raise ValueError(f"expected exactly one COLLADA {tag}")
    return children[0]


def _resolve(reference: str | None, ids: dict[str, ET.Element], tag: str | None = None) -> ET.Element:
    if not reference or not reference.startswith("#") or reference.count("#") != 1:
        raise ValueError("COLLADA references must be local fragments")
    target = ids.get(reference[1:])
    if target is None or (tag is not None and target.tag != tag):
        raise ValueError(f"unresolved COLLADA {tag or 'element'} reference: {reference}")
    return target


@dataclass(frozen=True)
class _Source:
    values: np.ndarray
    names: tuple[str | None, ...]


def _source(element: ET.Element, ids: dict[str, ET.Element]) -> _Source:
    array = _one(element, "float_array")
    count = _integer(array.get("count"), "float array count")
    values = _numbers(array.text, count)
    accessor = _one(_one(element, "technique_common"), "accessor")
    if _resolve(accessor.get("source"), ids, "float_array") is not array:
        raise ValueError("COLLADA accessor must reference its source float array")
    rows = _integer(accessor.get("count"), "accessor count")
    offset = _integer(accessor.get("offset"), "accessor offset", default=0)
    stride = _integer(accessor.get("stride"), "accessor stride", default=1)
    params = accessor.findall("param")
    if not rows or not params or stride < len(params):
        raise ValueError("invalid COLLADA accessor dimensions")
    if any(param.get("type") != "float" for param in params):
        raise ValueError("unsupported COLLADA accessor parameter type")
    if offset + (rows - 1) * stride + len(params) > len(values):
        raise ValueError("COLLADA accessor exceeds float array")
    names = tuple(param.get("name") for param in params)
    named = [name for name in names if name is not None]
    if len(set(named)) != len(named):
        raise ValueError("duplicate COLLADA accessor parameter")
    indices = offset + np.arange(rows)[:, None] * stride + np.arange(len(params))
    return _Source(values[indices], names)


def _geometry(element: ET.Element, ids: dict[str, ET.Element]) -> np.ndarray:
    mesh = _one(element, "mesh")
    if len(element) != 1:
        raise ValueError("unsupported COLLADA geometry children")
    sources: dict[str, _Source] = {}
    vertices: dict[str, dict[str, str]] = {}
    for source in mesh.findall("source"):
        source_id = source.get("id")
        if not source_id:
            raise ValueError("COLLADA source requires identity")
        sources[source_id] = _source(source, ids)
    for vertex in mesh.findall("vertices"):
        vertex_id = vertex.get("id")
        vertex_inputs: dict[str, str] = {}
        for item in vertex:
            semantic = item.get("semantic")
            if item.tag != "input" or not semantic or semantic in vertex_inputs:
                raise ValueError("unsupported COLLADA vertices input")
            source = _resolve(item.get("source"), ids, "source")
            if source.get("id") not in sources:
                raise ValueError("COLLADA vertices source must belong to its mesh")
            vertex_inputs[semantic] = source.attrib["id"]
        if not vertex_id or "POSITION" not in vertex_inputs:
            raise ValueError("COLLADA vertices require a POSITION source")
        vertices[vertex_id] = vertex_inputs
    used: dict[str, list[np.ndarray]] = {}
    for primitive in mesh:
        if primitive.tag in {"source", "vertices"}:
            continue
        if primitive.tag not in {"triangles", "lines"}:
            raise ValueError(f"unsupported COLLADA primitive: {primitive.tag}")
        inputs = primitive.findall("input")
        if not inputs or any(child.tag not in {"input", "p"} for child in primitive):
            raise ValueError("unsupported COLLADA primitive children")
        offsets = [_integer(item.get("offset"), "input offset", default=0) for item in inputs]
        stride = max(offsets) + 1
        count = _integer(primitive.get("count"), "primitive count")
        expected = count * (3 if primitive.tag == "triangles" else 2) * stride
        if not count or expected > _MAX_NUMBERS:
            raise ValueError("COLLADA primitive count exceeds budget or is empty")
        tokens = (_one(primitive, "p").text or "").split()
        if len(tokens) != expected:
            raise ValueError("COLLADA primitive index length mismatch")
        indices = np.array([_integer(token, "vertex index") for token in tokens], dtype=np.int64)
        indices = indices.reshape(-1, stride)
        position_inputs = 0
        for item, offset in zip(inputs, offsets):
            target = _resolve(item.get("source"), ids)
            semantic = item.get("semantic")
            if semantic == "VERTEX" and target.tag == "vertices" and target.get("id") in vertices:
                bindings = vertices[target.attrib["id"]]
            elif target.tag == "source" and target.get("id") in sources and semantic:
                bindings = {semantic: target.attrib["id"]}
            else:
                raise ValueError("unsupported COLLADA primitive input reference")
            for meaning, source_id in bindings.items():
                source = sources[source_id]
                selected = indices[:, offset]
                if np.max(selected) >= len(source.values):
                    raise ValueError("COLLADA primitive index exceeds accessor count")
                if meaning == "POSITION":
                    if not all(axis in source.names for axis in ("X", "Y", "Z")):
                        raise ValueError("COLLADA POSITION accessor requires X Y Z")
                    position_inputs += 1
                    used.setdefault(source_id, []).append(selected)
        if position_inputs != 1:
            raise ValueError("COLLADA primitive requires exactly one POSITION input")
    points = []
    for source_id, index_groups in used.items():
        source = sources[source_id]
        indices = np.unique(np.concatenate(index_groups))
        points.append(source.values[indices][:, [source.names.index(axis) for axis in ("X", "Y", "Z")]])
    if not points or sum(len(group) for group in points) > _MAX_POINTS:
        raise ValueError("COLLADA geometry has no bounded supported positions")
    return np.concatenate(points)


def _transform(element: ET.Element) -> np.ndarray:
    matrix = np.eye(4)
    if element.tag == "matrix":
        # Serialized rows, column-vector algebra; translation is text indices 3/7/11.
        matrix = _numbers(element.text, 16).reshape(4, 4)
        if not np.array_equal(matrix[3], [0, 0, 0, 1]):
            raise ValueError("COLLADA matrix must be affine")
    elif element.tag == "translate":
        matrix[:3, 3] = _numbers(element.text, 3)
    elif element.tag == "scale":
        matrix[np.arange(3), np.arange(3)] = _numbers(element.text, 3)
    elif element.tag == "rotate":
        x, y, z, angle = _numbers(element.text, 4)
        axis = np.array([x, y, z])
        length = np.linalg.norm(axis)
        if not np.isfinite(length) or length == 0:
            raise ValueError("COLLADA rotation requires a finite nonzero axis")
        axis /= length
        x, y, z = axis
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        angle = radians(float(angle))
        matrix[:3, :3] = cos(angle) * np.eye(3) + (1 - cos(angle)) * np.outer(axis, axis) + sin(angle) * skew
    return matrix


def _collada(data: bytes) -> tuple[tuple[str, np.ndarray], ...]:
    # No DTD/entity expansion, alternate encodings hiding declarations, or network I/O.
    try:
        text = data.decode("utf-8-sig")
    except UnicodeDecodeError as error:
        raise ValueError("COLLADA must use UTF-8") from error
    if "\x00" in text or "<!DOCTYPE" in text.upper() or "<!ENTITY" in text.upper():
        raise ValueError("COLLADA DTDs and entities are unsupported")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as error:
        raise ValueError("malformed or truncated COLLADA XML") from error
    namespace = root.tag.partition("}")[0] + "}" if root.tag.startswith("{") else ""
    if namespace not in {"{http://www.collada.org/2005/11/COLLADASchema}",
                         "{http://www.collada.org/2008/03/COLLADASchema}"}:
        raise ValueError("unsupported COLLADA namespace")
    ids: dict[str, ET.Element] = {}
    pending = [(root, 0)]
    elements = 0
    while pending:
        element, depth = pending.pop()
        elements += 1
        if depth > _MAX_DEPTH or elements > _MAX_ELEMENTS:
            raise ValueError("COLLADA XML complexity budget exceeded")
        if not element.tag.startswith(namespace):
            raise ValueError("unsupported foreign COLLADA element")
        element.tag = element.tag[len(namespace):]
        if "{http://www.w3.org/XML/1998/namespace}base" in element.attrib:
            raise ValueError("COLLADA reference base overrides are unsupported")
        identity = element.get("id")
        if identity is not None:
            if not identity or identity in ids:
                raise ValueError("COLLADA IDs must be nonempty and unique")
            ids[identity] = element
        for attribute in ("url", "source", "target"):
            if attribute in element.attrib:
                reference = element.attrib[attribute]
                if not reference.startswith("#") or reference.count("#") != 1:
                    raise ValueError("COLLADA external references are unsupported")
        if element.tag in {"library_controllers", "instance_controller", "controller",
                           "library_animations", "library_animation_clips"}:
            raise ValueError("COLLADA controllers and animations are unsupported")
        pending.extend((child, depth + 1) for child in element)
    if root.tag != "COLLADA" or root.get("version") not in {"1.4.0", "1.4.1", "1.5.0"}:
        raise ValueError("unsupported COLLADA document")
    asset = _one(root, "asset")
    if root.findall(".//asset") != [asset]:
        raise ValueError("nested COLLADA asset overrides are unsupported")
    units = asset.findall("unit")
    axes = asset.findall("up_axis")
    if len(units) > 1 or len(axes) > 1:
        raise ValueError("ambiguous COLLADA asset metadata")
    meter = _numbers(units[0].get("meter"), 1)[0] if units else 1.0
    axis = (axes[0].text or "").strip() if axes else "Y_UP"
    if meter <= 0 or axis not in {"Y_UP", "Z_UP"}:
        raise ValueError("unsupported COLLADA unit or up axis")
    basis = np.eye(4)
    if axis == "Y_UP":
        basis[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    basis[:3] *= meter
    scene_element = _one(root, "scene")
    if any(child.tag not in {"instance_visual_scene", "extra"} for child in scene_element):
        raise ValueError("unsupported COLLADA scene instance")
    scene = _resolve(_one(scene_element, "instance_visual_scene").get("url"), ids, "visual_scene")
    geometries: dict[str, np.ndarray] = {}
    instances: list[tuple[str, np.ndarray]] = []
    visits = 0
    total_points = 0

    def visit(node: ET.Element, parent: np.ndarray, path: str, ancestry: tuple[int, ...]) -> None:
        nonlocal visits, total_points
        visits += 1
        if visits > _MAX_ELEMENTS or len(ancestry) > _MAX_DEPTH or id(node) in ancestry:
            raise ValueError("COLLADA node cycle or traversal budget exceeded")
        if node.get("type", "NODE") != "NODE":
            raise ValueError("unsupported COLLADA joint node")
        if node.findall("asset"):
            raise ValueError("nested COLLADA asset overrides are unsupported")
        local = np.eye(4)
        for child in node:
            if child.tag in _TRANSFORMS:
                local = local @ _transform(child)
            elif child.tag not in {"node", "instance_node", "instance_geometry", "instance_camera",
                                    "instance_light", "extra"}:
                raise ValueError(f"unsupported COLLADA node child: {child.tag}")
        world = parent @ local
        if not np.isfinite(world).all():
            raise ValueError("COLLADA transform overflow")
        ancestry = (*ancestry, id(node))
        for index, child in enumerate(node):
            label = child.get("name") or child.get("id") or child.tag
            child_path = f"{path}/{label}[{index}]"
            if child.tag == "node":
                visit(child, world, child_path, ancestry)
            elif child.tag == "instance_node":
                if any(item.tag != "extra" for item in child):
                    raise ValueError("unsupported COLLADA node instance children")
                target = _resolve(child.get("url"), ids, "node")
                target_name = target.get("name") or target.get("id") or "node"
                visit(target, world, f"{child_path}/{target_name}", ancestry)
            elif child.tag == "instance_geometry":
                if any(item.tag not in {"bind_material", "extra"} for item in child):
                    raise ValueError("unsupported COLLADA geometry instance children")
                geometry = _resolve(child.get("url"), ids, "geometry")
                geometry_id = geometry.attrib["id"]
                if geometry_id not in geometries:
                    geometries[geometry_id] = _geometry(geometry, ids)
                points = geometries[geometry_id]
                total_points += len(points)
                if len(instances) >= _MAX_INSTANCES or total_points > _MAX_POINTS:
                    raise ValueError("COLLADA instance point budget exceeded")
                transformed = points @ world[:3, :3].T + world[:3, 3]
                instances.append((f"{child_path}#{geometry_id}", _readonly(transformed)))
            elif child.tag in {"instance_camera", "instance_light"}:
                _resolve(child.get("url"), ids, child.tag.removeprefix("instance_"))

    for index, child in enumerate(scene):
        if child.tag == "node":
            label = child.get("name") or child.get("id") or "node"
            visit(child, basis, f"{scene.get('id')}/{label}[{index}]", ())
        elif child.tag != "extra":
            raise ValueError(f"unsupported COLLADA visual scene child: {child.tag}")
    if not instances:
        raise ValueError("active COLLADA scene contains no geometry instances")
    return tuple(instances)


def mesh_vertices(path: Path) -> tuple[tuple[str, np.ndarray], ...]:
    """Return named, independent read-only float64 point clouds in meters/Z-up.

    Only local-fragment COLLADA references and static triangles/lines are accepted.
    STL coordinates are interpreted as meters/Z-up (STL has no unit metadata).
    File/XML/numeric/point budgets bound work and reject incomplete asset coverage.
    Malformed, unsupported, missing and oversized inputs raise ValueError.
    """
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix not in {".stl", ".dae"}:
        raise ValueError("oracle mesh supports only STL and COLLADA DAE")
    try:
        with path.open("rb") as stream:
            if path.stat().st_size > _MAX_BYTES:
                raise ValueError("mesh file exceeds 64 MiB budget")
            data = stream.read(_MAX_BYTES + 1)
    except OSError as error:
        raise ValueError(f"cannot read mesh: {path}") from error
    if not data or len(data) > _MAX_BYTES:
        raise ValueError("mesh file is empty or exceeds 64 MiB budget")
    return _stl(data, path.stem) if suffix == ".stl" else _collada(data)
