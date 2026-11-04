"""Replay a recorded unarmed bag. ROS imports stay inside the bag decoder."""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from camera import Mount
from capture_pose import (OdomSample, camera_from_info, decode_depth, interpolate_pose,
                          quaternion_rotation)
from evidence import EvidenceStore, Frame, SensorProfile, decide, swept_envelope
from replay_fixture import aligned_camera, grade_frame
from bag_diagnostics import diagnostics, geometry_summary, validate_receive_log
from recording_integrity import require_alignment_epoch, validate_alignment


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def read_bag(path, *, serialized_hashes: list | None = None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    # Keep tuple3 records compatible; optionally retain hashes of ORIGINAL stored CDR.
    # Deserializing then serializing may change padding without changing ROS fields.
    if serialized_hashes:
        raise ValueError("RAW_HASH_DESTINATION_NOT_EMPTY")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    records = []
    while reader.has_next():
        topic, payload, recorded_ns = reader.read_next()
        if serialized_hashes is not None:
            serialized_hashes.append((topic, recorded_ns, hashlib.sha256(payload).hexdigest()))
        records.append((topic, deserialize_message(payload, types[topic]), recorded_ns))
    return records


def validate_tf(message, expected_translation):
    for transform in message.transforms:
        if (transform.header.frame_id == "drone_cv_g1/base_link"
                and transform.child_frame_id == "drone_cv_g1/camera_optical"):
            p, q = transform.transform.translation, transform.transform.rotation
            mount = Mount((float(p.x), float(p.y), float(p.z)))
            actual = quaternion_rotation([q.w, q.x, q.y, q.z])
            if (not np.allclose(mount.translation_flu, expected_translation, atol=1e-8, rtol=0)
                    or not np.allclose(actual, mount.body_from_optical(), atol=1e-8, rtol=0)):
                raise ValueError("STATIC_TF_CHANGED")
            return mount
    raise ValueError("OPTICAL_TF_MISSING")


def collect(records, fixture, fault=None):
    """Decode events into independent epochs; never interpolate or fuse across reset."""
    clock_epoch = 0
    alignment_invalid = False
    epochs = defaultdict(lambda: {"odom": [], "infos": {}, "images": [], "clocks": [],
                                  "clock_epoch": clock_epoch, "alignment_invalid": alignment_invalid})
    rejected = Counter()
    epoch, last_clock, reset, last_odom = 0, None, None, None
    mount, calibration = None, None
    last_image = None
    odom_seen, image_seen = 0, 0
    odom_total = sum(topic.endswith("vehicle_odometry") for topic, _, _ in records)
    for topic, message, recorded_ns in records:
        if topic == "/tf_static" and fault != "missing_tf":
            candidate = validate_tf(message, fixture["camera_translation_flu"])
            if mount is not None and candidate != mount:
                epoch += 1
            mount = candidate
        elif topic == "/clock":
            value = stamp_ns(message.clock)
            if last_clock is not None and value < last_clock:
                clock_epoch += 1
                alignment_invalid = True
                epoch, last_odom, reset, last_image = epoch+1, None, None, None
            last_clock = value
            epochs[epoch]["clocks"].append(value)
        elif topic.endswith("vehicle_odometry"):
            odom_seen += 1
            if fault == "pose_gap" and odom_seen not in (1, odom_total):
                continue
            counter = int(message.reset_counter)
            if fault == "reset_counter" and odom_seen >= odom_total//2:
                counter = (counter+1) % 256
            if reset is not None and counter != reset:
                alignment_invalid = True
                epoch, last_odom, last_image = epoch+1, None, None
            reset = counter
            try:
                sample = OdomSample(int(message.timestamp_sample)*1000, tuple(message.position),
                                    tuple(message.q), counter, int(message.pose_frame))
                if last_odom is not None and sample.timestamp_ns <= last_odom:
                    raise ValueError("NONMONOTONIC_ODOMETRY")
                epochs[epoch]["odom"].append(sample)
                last_odom = sample.timestamp_ns
            except ValueError as error:
                epoch, last_odom, reset, last_image = epoch+1, None, None, None
                rejected[str(error)] += 1
        elif topic.endswith("camera_info") and fault != "missing_info":
            signature = (message.width, message.height, tuple(message.k), tuple(message.d),
                         message.header.frame_id, message.distortion_model,
                         message.binning_x, message.binning_y, message.roi.x_offset,
                         message.roi.y_offset, message.roi.width, message.roi.height)
            if calibration is not None and signature != calibration:
                epoch, last_odom, last_image = epoch+1, None, None
            calibration = signature
            epochs[epoch]["infos"][stamp_ns(message.header.stamp)] = message
        elif topic.endswith("image_raw"):
            image_seen += 1
            value = stamp_ns(message.header.stamp)
            if fault == "future_stamps":
                value += 1_000_000_000_000
            elif fault == "backward_stamps":
                value = 1 if image_seen > 1 else value
            if last_image is not None and value <= last_image:
                epoch, last_odom, reset = epoch+1, None, None
            last_image = value
            epochs[epoch]["images"].append((value, message, recorded_ns))
    return epochs, mount, rejected


def evaluate(records, manifest, fixture, output=None, fault=None, *, legacy_analysis=False):
    epochs, mount, rejected = collect(records, fixture, fault)
    if legacy_analysis:
        first_sample = next(sample for group in epochs.values() for sample in group["odom"])
        contract = {"clock_epoch": 0, "reset_counter": first_sample.reset_counter}
    else:
        contract = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    reports, gates = [], []
    accepted, considered, previous_stamp = 0, 0, None
    preview = None
    warmup = manifest["warmup_end_sim_ns"]
    for epoch_id, group in sorted(epochs.items()):
        store, last_pose = None, None
        previous_stamp = None
        max_clock = max(group["clocks"]) if group["clocks"] else None
        min_clock = min(group["clocks"]) if group["clocks"] else None
        try:
            if group["alignment_invalid"]:
                raise ValueError("ALIGNMENT_EPOCH_INVALIDATED")
            require_alignment_epoch(contract, group["clock_epoch"], group["odom"])
            alignment_error = None
        except ValueError as error:
            alignment_error = str(error)
        for capture_ns, image, _ in group["images"]:
            if capture_ns < warmup and fault != "backward_stamps":
                continue
            considered += 1
            try:
                if mount is None:
                    raise ValueError("OPTICAL_TF_MISSING")
                if previous_stamp is not None and capture_ns <= previous_stamp:
                    raise ValueError("NONMONOTONIC_IMAGE")
                previous_stamp = capture_ns
                if max_clock is None or not min_clock <= capture_ns <= max_clock:
                    raise ValueError("CAPTURE_OUTSIDE_CLOCK_EPOCH")
                info = group["infos"].get(capture_ns)
                if info is None:
                    raise ValueError("CAMERA_INFO_MISSING")
                if alignment_error:
                    raise ValueError(alignment_error)
                camera = camera_from_info(info, mount)
                depth = decode_depth(image)
                if depth.shape != (camera.intrinsics.height, camera.intrinsics.width):
                    raise ValueError("DEPTH_CALIBRATION_MISMATCH")
                pose = interpolate_pose(group["odom"], capture_ns, fixture["max_pose_gap_ns"])
                if store is None:
                    store = EvidenceStore(camera, SensorProfile(no_return_certified=False))
                store = store.add(Frame(depth, pose, f"bag-epoch-{epoch_id}", capture_ns/1e9, accepted))
                geometry = grade_frame(depth, camera, pose, manifest["world_from_odom"], fixture)
                reports.append({"stamp_ns": capture_ns, "epoch": epoch_id, **geometry})
                accepted += 1
                last_pose = pose
                preview = (depth, camera, pose)
            except ValueError as error:
                rejected[str(error)] += 1
        if store is not None:
            # A stationary bag cannot certify the rear swept envelope; report STOP honestly.
            envelope = swept_envelope(last_pose, 1.0)
            decision, samples, status = decide(store, envelope, epoch=f"bag-epoch-{epoch_id}",
                                               now_s=max_clock/1e9, ttl_s=10.0)
            stale, _, _ = decide(store, envelope, epoch=f"bag-epoch-{epoch_id}",
                                  now_s=max_clock/1e9+11.0, ttl_s=10.0)
            gates.append({"epoch": epoch_id, "allowed": decision.allowed, "reason": decision.reason,
                          "counts": decision.counts, "frames_used": decision.frames_used,
                          "epoch_frames": len(store.frames), "stale_reason": stale.reason})
            if output is not None and preview is not None:
                depth, camera, pose = preview
                rotation, origin = aligned_camera(camera, pose, manifest["world_from_odom"])
                k = camera.intrinsics
                v, u = np.mgrid[:k.height, :k.width]
                valid = np.isfinite(depth) & (depth >= camera.near_m) & (depth <= camera.far_m)
                rays = np.stack(((u-k.cx)/k.fx, (v-k.cy)/k.fy, np.ones_like(u)), axis=-1)
                points = origin + (rays[valid]*depth[valid, None]) @ rotation.T
                np.savez_compressed(output / "preview.npz", depth=depth, points_world=points[::10],
                                    samples=samples, status=status)
    pair_fraction = accepted/considered if considered else 0.0
    geometry_pass = bool(reports) and all(row["all_pass"] for row in reports)
    stationary_stop = bool(gates) and all(not row["allowed"] for row in gates)
    checks = {"minimum_frames": accepted >= fixture["minimum_frames"],
              "paired_fraction": pair_fraction >= fixture["minimum_pair_fraction"],
              "all_target_frames_pass": geometry_pass, "stationary_query_blocked": stationary_stop,
              "stale_evidence_blocked": bool(gates) and all(row["stale_reason"] == "NO_FRESH_FRAMES" for row in gates),
              "no_cross_epoch_history": all(row["frames_used"] <= row["epoch_frames"] for row in gates)}
    if legacy_analysis:
        checks["verified_alignment_provenance"] = False
    return {"fault": fault, "checks": checks, "all_checks_pass": all(checks.values()),
            "accepted_frames": accepted, "considered_frames": considered,
            "pair_fraction": pair_fraction, "rejected": dict(rejected), "geometry": reports,
            "gates": gates, "no_return_certified": False, "scope": "unarmed stationary geometry only",
            "acceptance_eligible": not legacy_analysis,
            "alignment_provenance": "unbound historical analysis" if legacy_analysis else "verified epoch contract"}


def fault_passes(name, variant):
    gates, rejected = variant["gates"], variant["rejected"]
    if any(row["allowed"] for row in gates):
        return False
    if name == "reset_counter":
        return (bool(gates) and variant["accepted_frames"] > 0
                and (variant["rejected"].get("ALIGNMENT_RESET_CHANGED", 0)
                     + variant["rejected"].get("ALIGNMENT_EPOCH_INVALIDATED", 0)) > 0
                and variant["checks"]["no_cross_epoch_history"])
    expected = {"missing_info": "CAMERA_INFO_MISSING",
                "missing_tf": "OPTICAL_TF_MISSING",
                "future_stamps": "CAPTURE_OUTSIDE_CLOCK_EPOCH"}
    if name in expected:
        return variant["accepted_frames"] == 0 and rejected.get(expected[name], 0) > 0
    if name == "pose_gap":
        return variant["accepted_frames"] <= 2 and sum(rejected.values()) > 0
    if name == "backward_stamps":
        return variant["accepted_frames"] <= 1 and sum(rejected.values()) > 0
    raise ValueError("UNKNOWN_FAULT_VARIANT")


def replay_exit_code(report: dict, legacy_analysis: bool) -> int:
    required = {key: value for key, value in report["checks"].items()
                if not (legacy_analysis and key == "verified_alignment_provenance")}
    return 0 if required and all(required.values()) else 2


def load_recording(recording: Path, *, legacy_analysis: bool = False):
    manifest = json.loads((recording / "manifest.json").read_text())
    fixture_bytes = (recording / "fixture.json").read_bytes()
    if not manifest["complete"] or not manifest["all_observed_disarmed"]:
        raise ValueError("RECORDING_INCOMPLETE_OR_ARMED")
    if hashlib.sha256(fixture_bytes).hexdigest() != manifest["fixture_sha256"]:
        raise ValueError("FIXTURE_HASH_MISMATCH")
    if manifest.get("recording_schema") == 4:
        actual_files = {path.relative_to(recording).as_posix() for path in (recording / "bag").rglob("*")
                        if path.is_file()}
        if not actual_files or set(manifest["bag_files"]) != actual_files:
            raise ValueError("SCHEMA4_BAG_INVENTORY_MISMATCH")
    for name, identity in manifest["bag_files"].items():
        path = (recording / name).resolve()
        if not path.is_relative_to(recording.resolve()) or path.is_symlink():
            raise ValueError("UNSAFE_BAG_PATH")
        if manifest.get("recording_schema") == 4 and path.stat().st_size != identity["bytes"]:
            raise ValueError("SCHEMA4_BAG_SIZE_MISMATCH")
        if hashlib.sha256(path.read_bytes()).hexdigest() != identity["sha256"]:
            raise ValueError("BAG_HASH_MISMATCH")
    fixture = json.loads(fixture_bytes)
    if manifest.get("recording_schema") == 4:
        from oracle_recording import validate_binding
        validate_binding((recording / "oracle_profile.json").read_bytes(), fixture, manifest)
        if manifest.get("oracle", {}).get("max_gap_ns") != fixture["oracle"]["fixture"]["max_truth_gap_ns"]:
            raise ValueError("ORACLE_COVERAGE_POLICY_BINDING_MISMATCH")
        if legacy_analysis:
            raise ValueError("SCHEMA4_REQUIRES_CONTINUOUS_TRUTH_VALIDATION")
    if not legacy_analysis:
        validate_alignment(manifest, fixture["max_pose_gap_ns"])
    serialized_hashes = [] if manifest.get("recording_schema") == 4 else None
    records = (read_bag(recording / "bag", serialized_hashes=serialized_hashes)
               if serialized_hashes is not None else read_bag(recording / "bag"))
    events = None
    if not legacy_analysis:
        policy_bytes = (recording / "recording_policy.json").read_bytes()
        event_bytes = (recording / "receive_progress.jsonl").read_bytes()
        if (hashlib.sha256(policy_bytes).hexdigest() != manifest["policy_sha256"]
                or hashlib.sha256(event_bytes).hexdigest() != manifest["receive_progress_sha256"]):
            raise ValueError("RECEIVE_PROVENANCE_HASH_MISMATCH")
        events = [json.loads(line) for line in event_bytes.splitlines()]
        if manifest.get("recording_schema") == 4 and json.loads(policy_bytes).get("schema") != 4:
            raise ValueError("SCHEMA4_POLICY_BINDING_MISMATCH")
        validate_receive_log(records, manifest, json.loads(policy_bytes), events,
                             serialized_hashes=serialized_hashes)
    return records, manifest, fixture, events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--legacy-analysis", action="store_true",
                        help="Analyze historical unbound provenance; never report new-profile acceptance")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    records, manifest, fixture, events = load_recording(args.recording, legacy_analysis=args.legacy_analysis)
    report = evaluate(records, manifest, fixture, args.output, legacy_analysis=args.legacy_analysis)
    report["diagnostics"] = diagnostics(records, manifest, events)
    report["geometry_summary"] = geometry_summary(report["geometry"])
    faults = {}
    for name in ("missing_info", "missing_tf", "pose_gap", "backward_stamps", "future_stamps", "reset_counter"):
        variant = evaluate(records, manifest, fixture, fault=name, legacy_analysis=args.legacy_analysis)
        faults[name] = {key: variant[key] for key in ("accepted_frames", "considered_frames", "rejected", "gates", "checks")}
        faults[name]["fault_pass"] = fault_passes(name, variant)
    report["fault_variants"] = faults
    report["fault_variants_note"] = "In-memory derived event variants; original bag remains unchanged. Not new sensor recordings."
    report["checks"]["fault_variants_pass"] = all(row["fault_pass"] for row in faults.values())
    report["all_checks_pass"] = all(report["checks"].values())
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in ("all_checks_pass", "checks", "accepted_frames", "pair_fraction", "rejected", "gates")}))
    return replay_exit_code(report, args.legacy_analysis)


if __name__ == "__main__":
    raise SystemExit(main())
