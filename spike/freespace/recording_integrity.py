"""Acquisition provenance and progress checks, not flight admission or calibrated bounds."""

from bisect import bisect_left
from math import isfinite

import numpy as np

from capture_pose import OdomSample, interpolate_pose


SCHEMA = 3
ORACLE_SCHEMA = 4
TRUTH_TOPIC = "/drone_cv_g1/evaluator/truth_pose_v"
REQUIRED_ORACLE_STREAMS = frozenset({
    "/drone_cv_g1/camera/depth/image_raw", "/drone_cv_g1/camera/depth/camera_info",
    "/drone_cv_g1/fmu/out/vehicle_odometry", "/drone_cv_g1/fmu/out/vehicle_status_v4",
    TRUTH_TOPIC,
})


def validate_policy(policy: dict) -> None:
    values = [policy["startup_wall_s"], policy["alignment_wait_wall_s"]]
    if policy["schema"] == 2:
        # Immutable historical bags retain their declared wall-progress contract.
        limits = policy["stream_progress_wall_s"]
    elif policy["schema"] in (SCHEMA, ORACLE_SCHEMA):
        limits = policy["stream_age_sim_s"]
        values += [policy["clock_progress_wall_s"]]
        if (type(policy["max_source_clock_lead_ns"]) is not int
                or policy["max_source_clock_lead_ns"] < 0
                or type(policy["alignment_buffer_samples"]) is not int
                or policy["alignment_buffer_samples"] < 2):
            raise ValueError("INVALID_ACQUISITION_POLICY")
        if "writer_queue_items" in policy or "writer_shutdown_wall_s" in policy:
            if (type(policy["writer_queue_items"]) is not int or policy["writer_queue_items"] < 1):
                raise ValueError("INVALID_WRITER_QUEUE_POLICY")
            values += [policy["writer_shutdown_wall_s"]]
        if policy["schema"] == ORACLE_SCHEMA:
            if set(limits) != REQUIRED_ORACLE_STREAMS:
                raise ValueError("POLICY_REQUIRED_STREAMS_MISMATCH")
            if "writer_queue_items" not in policy or "writer_shutdown_wall_s" not in policy:
                raise ValueError("INVALID_WRITER_QUEUE_POLICY")
    else:
        raise ValueError("UNSUPPORTED_ACQUISITION_POLICY")
    if (not limits or any(not isfinite(value) or value <= 0 for value in [*values, *limits.values()])
            or type(policy["max_pose_gap_ns"]) is not int or policy["max_pose_gap_ns"] <= 0):
        raise ValueError("INVALID_ACQUISITION_POLICY")


def source_stamp_ns(topic: str, message) -> int | None:
    if topic == TRUTH_TOPIC:
        from drone_cv.oracle_truth import decode_wire, stamp_ns
        return stamp_ns(decode_wire(message))
    if topic == "/tf_static":
        return None
    if topic == "/clock":
        return int(message.clock.sec)*1_000_000_000 + int(message.clock.nanosec)
    if topic.endswith("vehicle_odometry"):
        return int(message.timestamp_sample)*1000
    if topic.endswith("vehicle_status_v4"):
        return int(message.timestamp)*1000
    return int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)


def advance_progress(progress: dict, topic: str, source_ns: int, wall_ns: int) -> dict:
    previous = progress.get(topic)
    if previous is not None and source_ns < previous["source_ns"]:
        raise ValueError(f"STREAM_STAMP_REGRESSED:{topic}")
    if previous is not None and source_ns == previous["source_ns"]:
        return progress
    return {**progress, topic: {"source_ns": source_ns, "wall_ns": wall_ns}}


def require_fresh(progress: dict, policy: dict, now_ns: int, *, sim_now_ns: int | None = None,
                  allow_startup_missing: bool = False) -> None:
    if policy["schema"] == 2:
        for topic, limit_s in policy["stream_progress_wall_s"].items():
            if topic not in progress:
                raise ValueError(f"REQUIRED_STREAM_MISSING:{topic}")
            age_ns = now_ns-progress[topic]["wall_ns"]
            if age_ns < 0 or age_ns > limit_s*1e9:
                raise ValueError(f"STREAM_PROGRESS_STALLED:{topic}")
        return
    if "/clock" not in progress:
        if not allow_startup_missing:
            raise ValueError("REQUIRED_STREAM_MISSING:/clock")
    else:
        clock = progress["/clock"]
        clock_age = now_ns-clock["wall_ns"]
        if clock_age < 0 or clock_age > policy["clock_progress_wall_s"]*1e9:
            raise ValueError("STREAM_PROGRESS_STALLED:/clock")
    if sim_now_ns is None:
        raise ValueError("MISSING_NODE_CLOCK_SNAPSHOT")
    if allow_startup_missing and ("/clock" not in progress or sim_now_ns == 0):
        # Preserve pre-clock raw preflight observations, but never call them fresh.
        # Startup admission still requires a live /clock and all required streams.
        return
    for topic, limit_s in policy["stream_age_sim_s"].items():
        if topic not in progress:
            if allow_startup_missing:
                continue
            raise ValueError(f"REQUIRED_STREAM_MISSING:{topic}")
        age_ns = sim_now_ns-progress[topic]["source_ns"]
        if age_ns < -policy["max_source_clock_lead_ns"]:
            raise ValueError(f"SOURCE_AHEAD_OF_CLOCK:{topic}")
        if age_ns > limit_s*1e9:
            raise ValueError(f"STREAM_SIM_AGE_EXCEEDED:{topic}")


def odom_sample(message) -> OdomSample:
    return OdomSample(int(message.timestamp_sample)*1000, tuple(message.position),
                      tuple(message.q), int(message.reset_counter), int(message.pose_frame))


def make_alignment(world_body: np.ndarray, capture_ns: int, samples: list,
                   max_gap_ns: int) -> tuple[np.ndarray, dict]:
    world_body = np.asarray(world_body, dtype=np.float64)
    if (world_body.shape != (4, 4) or not np.isfinite(world_body).all()
            or not np.allclose(world_body[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(world_body[:3, :3].T @ world_body[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(world_body[:3, :3]), 1, atol=1e-8)):
        raise ValueError("INVALID_ALIGNMENT_TRUTH")
    pose = interpolate_pose(samples, capture_ns, max_gap_ns)
    index = bisect_left([sample.timestamp_ns for sample in samples], capture_ns)
    bracket = ([samples[index]] if samples[index].timestamp_ns == capture_ns
               else samples[index-1:index+1])
    odom_body = np.eye(4)
    odom_body[:3, :3], odom_body[:3, 3] = pose.world_from_body(), pose.position
    contract = {"clock_epoch": 0, "reset_counter": bracket[0].reset_counter,
                "capture_ns": capture_ns, "max_pose_gap_ns": max_gap_ns,
                "world_body": world_body.tolist(), "pose_samples": [
                    {"timestamp_ns": sample.timestamp_ns, "position_ned": sample.position_ned,
                     "quaternion_wxyz": sample.quaternion_wxyz,
                     "reset_counter": sample.reset_counter, "pose_frame": sample.pose_frame}
                    for sample in bracket],
                "bracket_endpoint_delta_ns": [sample.timestamp_ns-capture_ns for sample in bracket],
                "bracket_gap_ns": bracket[-1].timestamp_ns-bracket[0].timestamp_ns,
                "interpolation_fraction": (0. if len(bracket) == 1 else
                    (capture_ns-bracket[0].timestamp_ns)/(bracket[-1].timestamp_ns-bracket[0].timestamp_ns)),
                "timing_note": "Shared simulation epoch assumed; bracket deltas include sampling cadence and do not independently establish synchronization."}
    return world_body @ np.linalg.inv(odom_body), contract


def validate_alignment(manifest: dict, maximum_gap_ns: int) -> dict:
    if manifest.get("recording_schema") not in (2, SCHEMA, ORACLE_SCHEMA) or "alignment" not in manifest:
        raise ValueError("LEGACY_UNBOUND_ALIGNMENT")
    contract = manifest["alignment"]
    if (contract["clock_epoch"] != 0 or type(contract["reset_counter"]) is not int
            or not 0 <= contract["reset_counter"] <= 255
            or type(contract["max_pose_gap_ns"]) is not int
            or not 0 < contract["max_pose_gap_ns"] <= maximum_gap_ns):
        raise ValueError("INVALID_ALIGNMENT_CONTRACT")
    samples = [OdomSample(**sample) for sample in contract["pose_samples"]]
    alignment, rebuilt = make_alignment(np.array(contract["world_body"]),
                                        contract["capture_ns"], samples,
                                        contract["max_pose_gap_ns"])
    if (rebuilt["reset_counter"] != contract["reset_counter"]
            or not np.allclose(alignment, manifest["world_from_odom"], atol=1e-9, rtol=0)):
        raise ValueError("ALIGNMENT_PROVENANCE_MISMATCH")
    if manifest["recording_schema"] == ORACLE_SCHEMA:
        from oracle_recording import decode_packet
        packet = manifest["alignment_source"]["truth_packet"]
        sample = decode_packet(packet, manifest["truth_entities"])
        if (sample.source_ns != contract["capture_ns"]
                or not np.allclose(sample.world_from_body, contract["world_body"], atol=1e-12, rtol=0)
                or any(contract.get(key) != rebuilt[key] for key in
                       ("bracket_endpoint_delta_ns", "bracket_gap_ns", "interpolation_fraction"))):
            raise ValueError("ALIGNMENT_TRUTH_PACKET_MISMATCH")
    return contract


def require_alignment_epoch(contract: dict, clock_epoch: int, samples: list) -> None:
    if clock_epoch != contract["clock_epoch"]:
        raise ValueError("ALIGNMENT_CLOCK_EPOCH_CHANGED")
    if not samples or any(sample.reset_counter != contract["reset_counter"] for sample in samples):
        raise ValueError("ALIGNMENT_RESET_CHANGED")
