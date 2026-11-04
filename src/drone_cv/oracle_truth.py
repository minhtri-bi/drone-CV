"""Evaluator-only complete Pose_V protobuf content in a standard ROS byte array."""

from dataclasses import dataclass
from importlib import import_module

import numpy as np

from drone_cv.frames import rotation_wxyz


TRUTH_TOPIC = "/drone_cv_g1/evaluator/truth_pose_v"
WIRE_LABEL = "gz.msgs.Pose_V;oracle_schema=1"
MAX_PAYLOAD_BYTES = 4*1024*1024


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nsec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("INVALID_TRUTH_STAMP")
    return sec*1_000_000_000+nanosec


def wire_bytes(message) -> bytes:
    layout = message.layout
    if (layout.data_offset != 0 or len(layout.dim) != 1
            or layout.dim[0].label != WIRE_LABEL
            or layout.dim[0].size != len(message.data)
            or layout.dim[0].stride != len(message.data)
            or not 0 < len(message.data) <= MAX_PAYLOAD_BYTES):
        raise ValueError("INVALID_TRUTH_WIRE_LAYOUT")
    try:
        return bytes(message.data)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError("INVALID_TRUTH_WIRE_BYTES") from error


def decode_wire(message):
    """Use the installed generated protobuf decoder, not a second CDR parser."""
    payload = wire_bytes(message)
    pose_type = import_module("gz.msgs10.pose_v_pb2").Pose_V
    decode_error = import_module("google.protobuf.message").DecodeError
    result = pose_type()
    try:
        result.ParseFromString(payload)
    except decode_error as error:
        raise ValueError("INVALID_TRUTH_PROTOBUF") from error
    stamp_ns(result)
    return result


def _matrix(pose) -> np.ndarray:
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = rotation_wxyz([q.w, q.x, q.y, q.z])
    result[:3, 3] = [p.x, p.y, p.z]
    if not np.isfinite(result).all():
        raise ValueError("INVALID_TRUTH_POSE")
    return result


@dataclass(frozen=True)
class TruthSample:
    source_ns: int
    world_from_body: tuple[tuple[float, ...], ...]
    model_id: int
    body_id: int


def body_truth(message, entities: dict) -> TruthSample:
    """Compose the declared model-relative body link without depending on array order."""
    if (entities.get("link_pose_convention") != "model_relative"
            or not all(type(entities.get(key)) is int and entities[key] > 0
                       for key in ("model_id", "body_id"))
            or entities["model_id"] == entities["body_id"]
            or not all(isinstance(entities.get(key), str) and entities[key]
                       for key in ("model_name", "body_name"))):
        raise ValueError("INVALID_TRUTH_ENTITY_CONTRACT")
    poses = list(message.pose)
    ids = [int(pose.id) for pose in poses]
    if not poses or any(identity <= 0 for identity in ids) or len(set(ids)) != len(ids):
        raise ValueError("EMPTY_OR_AMBIGUOUS_TRUTH_ENTITIES")
    selected = []
    for prefix in ("model", "body"):
        candidates = [pose for pose in poses if pose.name == entities[f"{prefix}_name"]]
        if len(candidates) != 1 or candidates[0].id != entities[f"{prefix}_id"]:
            raise ValueError(f"TRUTH_{prefix.upper()}_IDENTITY_CHANGED")
        selected.append(candidates[0])
    transform = _matrix(selected[0]) @ _matrix(selected[1])
    return TruthSample(stamp_ns(message), tuple(tuple(float(v) for v in row) for row in transform),
                       entities["model_id"], entities["body_id"])
