"""Freeze exact retained membership, then evaluate using pinned decoders only.

No nodes, publishers, synthesized ROS messages, new recordings, or runtime authority.
"""
import argparse
from bisect import bisect_left
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "spike/freespace")]

from drone_cv.empirical_profile import (
    Binding, Identity, ProfileProtocol, Sample, derive_profile, digest, evaluate_profile,
)


def file_hash(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            value.update(chunk)
    return value.hexdigest()


def write_exclusive(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def freeze(recordings, output):
    """Read identities only, not estimator/truth values or evaluation outcomes."""
    bindings, derivation, heldout, inputs = [], [], [], []
    for path in recordings:
        manifest_hash = file_hash(path / "manifest.json")
        manifest = json.loads((path / "manifest.json").read_text())
        alignment = manifest["alignment"]
        binding = Binding(manifest_hash, alignment["clock_epoch"], alignment["reset_counter"],
            digest({"alignment": alignment, "world_from_odom": manifest["world_from_odom"],
                    "truth_entities": manifest.get("truth_entities")}),
            "base_link_origin", "local_ned", "gazebo_sim",
            ("PX4_timestamp_sample_NED", "receive_monotonic_wall",
             "Gazebo_Pose_V_world_ENU" if manifest.get("recording_schema") == 4
             else "Gazebo_alignment_snapshot_only_NO_continuous_truth"))
        bindings.append(binding)
        # Entire post-warmup domain, deterministic midpoint, no outcome-dependent trimming.
        start, end = manifest["warmup_end_sim_ns"], manifest["record_end_sim_ns"]
        midpoint = (start + end) // 2
        events = [json.loads(line) for line in (path / "receive_progress.jsonl").read_text().splitlines()]
        for event in events:
            if not event["topic"].endswith("vehicle_odometry") or not start <= event["source_ns"] <= end:
                continue
            identity = Identity(manifest_hash, event["topic_sequence"], event["source_ns"], event.get("serialized_sha256"))
            (derivation if event["source_ns"] <= midpoint else heldout).append(identity)
        names = ["manifest.json", "fixture.json", "recording_policy.json", "receive_progress.jsonl", *manifest["bag_files"]]
        if manifest.get("recording_schema") == 4:
            names.append("oracle_profile.json")
        location = path.resolve().relative_to(ROOT).as_posix() if path.resolve().is_relative_to(ROOT) else str(path.resolve())
        inputs.append({"path": location, "manifest_sha256": manifest_hash,
                       "files": {name: file_hash(path / name) for name in names},
                       "window_ns": [start, end], "derivation_end_ns": midpoint})
    protocol = ProfileProtocol("retained-second-round-N2-no-fit-to-test", tuple(bindings),
        tuple(derivation), tuple(heldout), "retrospective_derivation_disjoint", "",
        (.01, .01, .01), (.01, .01, .01), (.01, .01, .01), (.01, .01, .01),
        (0., 0., 0.), (0., 0., 0.),
        "Zero added margin: descriptive derivation maxima, no invented positive reserve and no heldout fitting",
        "Unchanged historical synthetic 0.01 per-axis variance ceilings; diagnostic comparison, not justified runtime health",
        "Exact retained inputs and entire post-warmup windows; historical full traces previously reviewed; not unseen or blinded",
        "instantaneous", (50., 95., 99., 100.))
    payload = {"schema": 1, "protocol": protocol.to_dict(), "protocol_sha256": protocol.sha256,
               "inputs": inputs, "evaluation_performed": False,
               "freeze_scope": "membership and protocol frozen before this evaluation; historical data not untouched",
               "truth_velocity_policy": "Prefer matching independent twist if present; pinned Pose_V has pose only, so bracket differences are interval averages",
               "position_scope": "absolute local NED base_link origin; not capture-to-query relative error",
               "missing_evidence": ["untouched/blinded heldout", "instantaneous independent truth velocity",
                                    "justified runtime health/uncertainty/response bounds", "independent tracking bound"]}
    write_exclusive(output, {**payload, "frozen_sha256": digest(payload)})


def recording_samples(path, binding, membership):
    import numpy as np
    from drone_cv.oracle_truth import TRUTH_TOPIC, body_truth, decode_wire
    from drone_cv.px4_bridge import decode_odometry
    from capture_pose import ENU_FROM_NED
    from replay_bag import collect, load_recording
    from recording_integrity import require_alignment_epoch, validate_alignment
    from truth_error import interpolate_truth

    manifest = json.loads((path / "manifest.json").read_text())
    records, manifest, fixture, events = load_recording(path)
    groups, _, _ = collect(records, fixture)
    contract = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    epoch_failure = None
    try:
        if len(groups) != 1:
            raise ValueError("ALIGNMENT_MULTIPLE_OR_RESET_EPOCHS")
        group = next(iter(groups.values()))
        if group["alignment_invalid"]:
            raise ValueError("ALIGNMENT_EPOCH_INVALIDATED")
        require_alignment_epoch(contract, group["clock_epoch"], group["odom"])
    except ValueError as error:
        epoch_failure = str(error)
    if manifest.get("recording_schema") != 4:
        indexed = {}
        for topic, message, _ in records:
            if topic.endswith("vehicle_odometry"):
                indexed.setdefault(int(message.timestamp_sample) * 1000, []).append(message)
        original_events = {(e["topic_sequence"], e["source_ns"]): e for e in events
                           if e["topic"].endswith("vehicle_odometry")}
        rows = []
        for identity in membership:
            reasons = ["CONTINUOUS_TRUTH_MISSING_SCHEMA_2"]
            position_health = velocity_health = None
            try:
                event = original_events[(identity.sequence, identity.source_ns)]
                candidates = indexed[identity.source_ns]
                if len(candidates) != 1:
                    raise ValueError("DUPLICATE_ODOMETRY_STAMP")
                odom = decode_odometry(candidates[0], str(binding.clock_epoch), event["received_wall_ns"] / 1e9)
                if odom.reset_counter != binding.reset_counter:
                    raise ValueError("RESET_BINDING_MISMATCH")
                p, v = odom.position_variance, odom.velocity_variance
                position_health = float(p[0]), float(p[1]), float(p[2])
                velocity_health = float(v[0]), float(v[1]), float(v[2])
                if epoch_failure is not None:
                    reasons.append(epoch_failure)
                row = Sample(identity, binding, None, None, position_health, velocity_health,
                             "missing", tuple(reasons), None)
            except (ValueError, KeyError, TypeError) as error:
                row = Sample(identity, binding, None, None, None, None, "missing",
                             tuple([*reasons, str(error)]), None)
            rows.append(row)
        return rows
    truth, truth_failures = [], []
    for topic, message, _ in records:
        if topic == TRUTH_TOPIC:
            try:
                sample = body_truth(decode_wire(message), manifest["truth_entities"])
                truth.append((sample.source_ns, np.asarray(sample.world_from_body)))
            except (ValueError, KeyError, TypeError) as error:
                truth_failures.append(str(error))
    stamps = [row[0] for row in truth]
    maximum_gap = fixture["oracle"]["fixture"]["max_truth_gap_ns"]
    indexed = {}
    for topic, message, _ in records:
        if topic.endswith("vehicle_odometry"):
            indexed.setdefault(int(message.timestamp_sample) * 1000, []).append(message)
    by_identity = {(e["topic_sequence"], e["source_ns"], e["serialized_sha256"]): e for e in events
                   if e["topic"].endswith("vehicle_odometry")}
    world_from_odom = np.asarray(manifest["world_from_odom"], dtype=float)
    result = []
    def ned(value):
        return float(value[0]), float(value[1]), float(value[2])
    for identity in membership:
        position_error = position_health = velocity_health = None
        position_bracket = None
        try:
            if epoch_failure is not None:
                raise ValueError(epoch_failure)
            if truth_failures:
                raise ValueError("TRUTH_DECODE_FAILURE:" + ";".join(truth_failures))
            event = by_identity[(identity.sequence, identity.source_ns, identity.serialized_sha256)]
            messages = indexed[identity.source_ns]
            if len(messages) != 1:
                raise ValueError("DUPLICATE_ODOMETRY_STAMP")
            odom = decode_odometry(messages[0], str(binding.clock_epoch), event["received_wall_ns"] / 1e9)
            if odom.reset_counter != binding.reset_counter:
                raise ValueError("RESET_BINDING_MISMATCH")
            position_health, velocity_health = ned(odom.position_variance), ned(odom.velocity_variance)
            actual, position_delta = interpolate_truth(truth, identity.source_ns, maximum_gap)
            position_bracket = (identity.source_ns + position_delta[0], identity.source_ns + position_delta[1])
            # Frozen alignment maps odometry ENU into truth world ENU. Invert once,
            # then ENU->NED: this keeps variance/error components in the PX4 basis.
            true_position = ENU_FROM_NED.T @ (world_from_odom[:3, :3].T @
                (actual[:3, 3] - world_from_odom[:3, 3]))
            position_error = ned(np.asarray(odom.position_ned) - true_position)
            index = bisect_left(stamps, identity.source_ns)
            left = max(0, index - 1)
            right = index if index < len(stamps) and stamps[index] > identity.source_ns else index + 1
            if right >= len(stamps) or left == right or stamps[right] - stamps[left] > maximum_gap:
                raise ValueError("VELOCITY_TRUTH_BRACKET_MISSING")
            dt = (stamps[right] - stamps[left]) / 1e9
            # Pose_V has NO twist. This comparison is explicitly interval-average,
            # never an instantaneous velocity bound, even for stationary samples.
            world_velocity = (truth[right][1][:3, 3] - truth[left][1][:3, 3]) / dt
            true_velocity = ENU_FROM_NED.T @ world_from_odom[:3, :3].T @ world_velocity
            result.append(Sample(identity, binding,
                position_error,
                ned(np.asarray(odom.velocity_ned) - true_velocity),
                position_health, velocity_health, "interval_average", (),
                (stamps[left], stamps[right])))
        except (ValueError, KeyError, TypeError, IndexError) as error:
            result.append(Sample(identity, binding, position_error, None, position_health,
                                 velocity_health, "missing", (str(error),), position_bracket))
    return result


def evaluate(frozen_path, output):
    frozen = json.loads(frozen_path.read_text())
    if digest({k: v for k, v in frozen.items() if k != "frozen_sha256"}) != frozen["frozen_sha256"]:
        raise ValueError("FROZEN_PROTOCOL_HASH_MISMATCH")
    protocol = ProfileProtocol.from_dict(frozen["protocol"])
    if protocol.sha256 != frozen["protocol_sha256"]:
        raise ValueError("PROTOCOL_HASH_MISMATCH")
    import rclpy
    from rclpy.node import Node
    def forbidden(*args, **kwargs):
        raise RuntimeError("OFFLINE_GUARD: no ROS runtime or publisher")
    rclpy.init = rclpy.create_node = Node.create_publisher = forbidden
    rows = []
    for item in frozen["inputs"]:
        path = ROOT / item["path"]
        for name, expected in item["files"].items():
            if file_hash(path / name) != expected:
                raise ValueError("INPUT_HASH_MISMATCH:" + name)
        binding = next(b for b in protocol.bindings if b.manifest_sha256 == item["manifest_sha256"])
        members = tuple(dict.fromkeys(i for i in (*protocol.derivation, *protocol.heldout)
                                     if i.manifest_sha256 == binding.manifest_sha256))
        rows.extend(recording_samples(path, binding, members))
    profile = derive_profile(protocol, [r for r in rows if r.identity in set(protocol.derivation)])
    report = evaluate_profile(profile, [r for r in rows if r.identity in set(protocol.heldout)])
    write_exclusive(output, {"frozen_sha256": frozen["frozen_sha256"],
                             "profile": profile.to_dict(), "evaluation": report})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    freeze_parser = sub.add_parser("freeze", help="hash inputs and freeze identities only, before evaluation")
    freeze_parser.add_argument("--recording", type=Path, action="append", required=True)
    freeze_parser.add_argument("--output", type=Path, required=True)
    evaluate_parser = sub.add_parser("evaluate")
    evaluate_parser.add_argument("--protocol", type=Path, required=True)
    evaluate_parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "freeze":
        freeze(args.recording, args.output)
    else:
        evaluate(args.protocol, args.output)


if __name__ == "__main__":
    main()
