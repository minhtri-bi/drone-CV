"""Freeze exact retrospective source-disjoint membership, then evaluate offline.

Freeze reads identity/provenance metadata only. Evaluate uses existing strict,
pinned recording decoders and capture/truth interpolation, never ROS runtime.
No new recordings, instantaneous twist, calibration, or motion admission.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "spike/freespace")]

from drone_cv.empirical_profile import Binding, Identity, digest
from drone_cv.relative_profile import (
    PoseObservation, RelativeProtocol, evaluate_relative_profile, relative_profile_payload,
    SEEN_RETAINED_SOURCES,
)
from sim_profile_consistency import file_hash, write_exclusive


PINNED = {
    "firmware": "61907962c9220437ec01a98fa26bfbf5cbb254b5",
    "messages": "8fb8277cf95c4ba87094146bc8922e76c4ac0799",
    "image": "sha256:061a39905d2d84d40859bb46b67048241b6dacf2ed3983889da7f5fd3e718285",
}


def freeze(aliases_path, output, *, max_age_s=.5, rho_m=30., translation_limit_m=.05,
           rotation_limit_rad=.01, derivation_sources=None, heldout_sources=None,
           holdout_kind="retrospective_source_disjoint", blinding_evidence_sha256="",
           conditional_model_scope_sha256=""):
    """Freeze full post-warmup identity sets, without reading any pose outcomes.

    Default current aliases: whole support06 derivation, whole r39 heldout.
    The source-disjoint heldout lacks continuous truth and remains uncovered.
    Separate same-source temporal diagnostics are explicitly not independent.
    Future freezes accept explicit manifest hashes for both source partitions.
    """
    aliases = json.loads(aliases_path.read_text(encoding="utf-8"))
    if digest({k: v for k, v in aliases.items() if k != "frozen_sha256"}) != aliases["frozen_sha256"]:
        raise ValueError("INPUT_ALIAS_PROTOCOL_HASH_MISMATCH")
    bindings, captures, queries, inputs = [], [], [], []
    calibration = []
    for item in aliases["inputs"]:
        path = ROOT / item["path"]
        for name, expected in item["files"].items():
            if file_hash(path / name) != expected:
                raise ValueError("INPUT_ALIAS_HASH_MISMATCH:" + name)
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
        fixture = json.loads((path / "fixture.json").read_text(encoding="utf-8"))
        alignment = manifest["alignment"]
        binding = Binding(item["manifest_sha256"], alignment["clock_epoch"], alignment["reset_counter"],
            digest({"alignment": alignment, "world_from_odom": manifest["world_from_odom"],
                    "truth_entities": manifest.get("truth_entities")}),
            "base_link_origin", "local_ned", "gazebo_sim",
            ("PX4_timestamp_sample_NED", "receive_monotonic_wall",
             "Gazebo_Pose_V_world_ENU" if manifest.get("recording_schema") == 4
             else "Gazebo_alignment_snapshot_only_NO_continuous_truth"))
        bindings.append(binding)
        start, end = manifest["warmup_end_sim_ns"], manifest["record_end_sim_ns"]
        for line in (path / "receive_progress.jsonl").read_text(encoding="utf-8").splitlines():
            event = json.loads(line)
            role = ("capture" if event["topic"].endswith("image_raw") else
                    "query" if event["topic"].endswith("vehicle_odometry") else None)
            if role is None or event["source_ns"] is None or not start <= event["source_ns"] <= end:
                continue
            identity = Identity(item["manifest_sha256"], event["topic_sequence"],
                                event["source_ns"], event.get("serialized_sha256"))
            (captures if role == "capture" else queries).append(identity)
        inputs.append({**item, "window_ns": [start, end], "recording_schema": manifest.get("recording_schema")})
        calibration.append({"camera_translation_flu": fixture["camera_translation_flu"],
                            "body_point": "base_link_origin", "clock_source": "gazebo_sim",
                            "pose_frame": "local_ned", "body_axes": "FLU",
                            "camera_frame": "drone_cv_g1/camera_optical"})
    source_ids = tuple(b.manifest_sha256 for b in bindings)
    if derivation_sources is None and heldout_sources is None:
        derivation_sources, heldout_sources = source_ids[:1], source_ids[1:]
    elif derivation_sources is None or heldout_sources is None:
        raise ValueError("BOTH_SOURCE_PARTITIONS_REQUIRED")
    if max_age_s > .5:
        raise ValueError("TTL_EXCEEDS_FROZEN_SYNTHETIC_COMMAND_OBSERVATION_MAXIMUM")
    if holdout_kind == "untouched_source_disjoint" and set(heldout_sources) & SEEN_RETAINED_SOURCES:
        raise ValueError("PREVIOUSLY_SEEN_RETAINED_SOURCE_CANNOT_BE_UNTOUCHED")
    # Model/calibration identities exclude per-run world alignment, entity IDs,
    # reset counters, source-message CDR hashes, and historical record epochs.
    model_sha256 = digest({**PINNED, "model": "x500_depth", "body_point": "base_link_origin"})
    provenance_sha256 = digest({"pinned": PINNED, "calibration_contracts":
                               sorted({digest(value): value for value in calibration}.values(), key=digest),
                               "clock_policy": "Gazebo sim; PX4 UXRCE_DDS_SYNCT=0; capture timestamp preserved"})
    protocol = RelativeProtocol("retained-round-three-F5-relative-v3", tuple(bindings), tuple(captures),
        tuple(queries), tuple(derivation_sources), tuple(heldout_sources), max_age_s, rho_m,
        translation_limit_m, rotation_limit_rad, holdout_kind, blinding_evidence_sha256, model_sha256,
        "TTL is bounded by the unchanged synthetic 0.5 s observation/command maximum, not inferred from the old 10 s report. "
        "rho is a declared full capture-body point ball (30 m default covers the pinned 19.1 m optical-Z "
        "outer cone plus mount; no empirical fitting or physical calibration). Translation/rotation limits "
        "are frozen hypothetical diagnostic claims, not justified operational error limits. Entire post-warmup "
        "identity sets are frozen without outcomes. Holdout history is declared explicitly; already reviewed retained "
        "sources can never become untouched by relabelling. No cross-source or cross-split pairs.",
        provenance_sha256, conditional_model_scope_sha256)
    payload = {"schema": 3, "protocol": protocol.to_dict(), "protocol_sha256": protocol.sha256,
        "inputs": inputs, "alias_protocol_sha256": file_hash(aliases_path), "pinned": PINNED,
        "evaluation_performed": False,
        "freeze_scope": "metadata/source/reset/epoch/body-point membership before this evaluation; previously seen retained sources",
        "missing_evidence": ["untouched independent source-disjoint heldout", "continuous truth for schema-two r39",
                             "independent timing/calibration/continuous-time transform bounds",
                             "independent instantaneous truth twist", "operational dynamics and tracking bounds"],
        "source_sha256": protocol.source_sha256, "model_sha256": model_sha256,
        "provenance_sha256": provenance_sha256}
    write_exclusive(output, {**payload, "frozen_sha256": digest(payload)})


def recording_observations(path, binding, captures, queries):
    """Use exact validated recording events; missing/rejected identities survive."""
    import numpy as np
    from drone_cv.oracle_truth import TRUTH_TOPIC, body_truth, decode_wire
    from drone_cv.px4_bridge import decode_odometry
    from capture_pose import ENU_FROM_NED, decode_depth, interpolate_pose
    from replay_bag import collect, load_recording, stamp_ns
    from recording_integrity import require_alignment_epoch, validate_alignment
    from truth_error import interpolate_truth, rigid

    records, manifest, fixture, events = load_recording(path)
    groups, _, rejected = collect(records, fixture)
    alignment = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    failures = []
    group = None
    try:
        if len(groups) != 1:
            raise ValueError("ALIGNMENT_MULTIPLE_OR_RESET_EPOCHS")
        group = next(iter(groups.values()))
        if group["alignment_invalid"]:
            raise ValueError("ALIGNMENT_EPOCH_INVALIDATED")
        require_alignment_epoch(alignment, group["clock_epoch"], group["odom"])
        if rejected:
            raise ValueError("RECORDING_DECODE_REJECTIONS:" + json.dumps(dict(rejected), sort_keys=True))
    except ValueError as error:
        failures.append(str(error))
    if manifest.get("recording_schema") != 4:
        failures.append("CONTINUOUS_TRUTH_MISSING_SCHEMA_2")
    truth, truth_failures = [], []
    if manifest.get("recording_schema") == 4:
        for topic, message, _ in records:
            if topic == TRUTH_TOPIC:
                try:
                    actual = body_truth(decode_wire(message), manifest["truth_entities"])
                    truth.append((actual.source_ns, np.asarray(actual.world_from_body)))
                except (ValueError, KeyError, TypeError) as error:
                    truth_failures.append("TRUTH_DECODE_FAILURE:" + str(error))
    failures.extend(truth_failures)
    odometry, images = {}, {}
    for topic, message, _ in records:
        if topic.endswith("vehicle_odometry"):
            odometry.setdefault(int(message.timestamp_sample) * 1000, []).append(message)
        elif topic.endswith("image_raw"):
            images.setdefault(stamp_ns(message.header.stamp), []).append(message)
    by_event = {}
    for event in events:
        role = ("capture" if event["topic"].endswith("image_raw") else
                "query" if event["topic"].endswith("vehicle_odometry") else None)
        if role is not None:
            key = (role, event["topic_sequence"], event["source_ns"], event.get("serialized_sha256"))
            if key in by_event:
                raise ValueError("DUPLICATE_RECEIVE_IDENTITY")
            by_event[key] = event
    world_from_odom = np.asarray(manifest["world_from_odom"])
    ned_from_world = tuple(tuple(float(v) for v in row) for row in ENU_FROM_NED.T @ world_from_odom[:3, :3].T)
    def frozen_matrix(value):
        return tuple(tuple(float(v) for v in row) for row in value)
    result = []
    for role, identities in (("capture", captures), ("query", queries)):
        for identity in identities:
            estimate = actual = None
            reasons = list(failures)
            try:
                event = by_event[(role, identity.sequence, identity.source_ns, identity.serialized_sha256)]
                if event.get("validation_error"):
                    raise ValueError("RECEIVE_VALIDATION_REJECTED:" + str(event["validation_error"]))
                if role == "capture":
                    if len(images.get(identity.source_ns, [])) != 1:
                        raise ValueError("MISSING_OR_DUPLICATE_CAPTURE_IMAGE")
                    decode_depth(images[identity.source_ns][0])
                else:
                    if len(odometry.get(identity.source_ns, [])) != 1:
                        raise ValueError("MISSING_OR_DUPLICATE_QUERY_ODOMETRY")
                    odom = decode_odometry(odometry[identity.source_ns][0], str(binding.clock_epoch),
                                           event["received_wall_ns"] / 1e9)
                    if odom.reset_counter != binding.reset_counter:
                        raise ValueError("RESET_BINDING_MISMATCH")
                if group is not None:
                    pose = interpolate_pose(group["odom"], identity.source_ns, fixture["max_pose_gap_ns"])
                    estimate = frozen_matrix(world_from_odom @ rigid(pose))
                if not reasons:
                    actual, _ = interpolate_truth(truth, identity.source_ns,
                                                  fixture["oracle"]["fixture"]["max_truth_gap_ns"])
                    actual = frozen_matrix(actual)
            except (ValueError, KeyError, TypeError, IndexError) as error:
                reasons.append(str(error))
            result.append(PoseObservation(role, identity, binding, estimate, actual,
                                           tuple(dict.fromkeys(reasons)), ned_from_world))
    return result


def evaluate(frozen_path, output):
    frozen = json.loads(frozen_path.read_text(encoding="utf-8"))
    if frozen.get("schema") != 3 or digest({k: v for k, v in frozen.items() if k != "frozen_sha256"}) != frozen["frozen_sha256"]:
        raise ValueError("FROZEN_RELATIVE_PROTOCOL_HASH_MISMATCH")
    protocol = RelativeProtocol.from_dict(frozen["protocol"])
    if protocol.sha256 != frozen["protocol_sha256"]:
        raise ValueError("RELATIVE_PROTOCOL_HASH_MISMATCH")
    if frozen["pinned"] != PINNED:
        raise ValueError("PINNED_IMPLEMENTATION_MISMATCH")
    import rclpy
    from rclpy.node import Node
    def forbidden(*args, **kwargs):
        raise RuntimeError("OFFLINE_GUARD: no ROS init/node/publisher")
    rclpy.init = rclpy.create_node = Node.__init__ = Node.create_publisher = forbidden
    rows = []
    for item in frozen["inputs"]:
        path = ROOT / item["path"]
        for name, expected in item["files"].items():
            if file_hash(path / name) != expected:
                raise ValueError("INPUT_HASH_MISMATCH:" + name)
        binding = next(b for b in protocol.bindings if b.manifest_sha256 == item["manifest_sha256"])
        captures = tuple(i for i in protocol.captures if i.manifest_sha256 == binding.manifest_sha256)
        queries = tuple(i for i in protocol.queries if i.manifest_sha256 == binding.manifest_sha256)
        rows.extend(recording_observations(path, binding, captures, queries))
    profile = relative_profile_payload(protocol, rows)
    report = evaluate_relative_profile(profile)
    # Existing pinned full-SE(3) evaluator independently checks the complete usable
    # per-source population. It is descriptive only; rejected rows stay above.
    import numpy as np
    from truth_error import all_pairs, relative_error, transform_error
    crosscheck = {}
    for binding in protocol.bindings:
        source_rows = [row for row in rows if row.binding == binding and not row.rejection_reasons
                       and row.estimated_world_from_body is not None and row.truth_world_from_body is not None]
        captures = sorted([(r.identity.source_ns, np.asarray(r.estimated_world_from_body),
                            np.asarray(r.truth_world_from_body)) for r in source_rows if r.role == "capture"], key=lambda row: row[0])
        queries = sorted([(r.identity.source_ns, np.asarray(r.estimated_world_from_body),
                           np.asarray(r.truth_world_from_body)) for r in source_rows if r.role == "query"], key=lambda row: row[0])
        if not captures or not queries:
            crosscheck[binding.manifest_sha256] = {"covered": False, "reason": "NO_TRUTH_COVERED_CAPTURE_QUERY_POPULATION"}
            continue
        try:
            measured = all_pairs(captures, queries, int(protocol.max_age_s * 1e9), protocol.rho_m)
        except ValueError as error:
            crosscheck[binding.manifest_sha256] = {"covered": False, "reason": str(error)}
            continue
        metrics = report["per_source"][binding.manifest_sha256]
        if measured["pair_count"] != metrics["covered_pair_count"]:
            raise ValueError("RELATIVE_ALL_PAIRS_CROSSCHECK_COUNT_MISMATCH")
        for old, new in (("maximum_translation_m", "translation_m"),
                         ("maximum_rotation_rad", "rotation_rad"), ("maximum_bound_m", "displacement_m")):
            if not np.isclose(measured[old], metrics["metrics"][new]["maximum"], atol=1e-10, rtol=1e-10):
                raise ValueError("RELATIVE_ALL_PAIRS_CROSSCHECK_METRIC_MISMATCH:" + new)
        # Preserve the original formula and worst-pair values in the artifact.
        worst = measured["worst_pairs"]["maximum_bound_m"]
        c = next(row for row in captures if row[0] == worst["capture_ns"])
        q = next(row for row in queries if row[0] == worst["query_ns"])
        measured["worst_recomputed"] = transform_error(relative_error(c[1], q[1], c[2], q[2]), protocol.rho_m)
        crosscheck[binding.manifest_sha256] = {"covered": True, **measured}
    write_exclusive(output, {"schema": 3, "frozen_sha256": frozen["frozen_sha256"],
        "profile": profile, "evaluation": report, "existing_truth_error_crosscheck": crosscheck,
        "runtime_admission": False})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    freeze_parser = sub.add_parser("freeze", help="freeze identities/hashes only; never read outcomes")
    freeze_parser.add_argument("--input-aliases", type=Path, required=True)
    freeze_parser.add_argument("--output", type=Path, required=True)
    freeze_parser.add_argument("--max-age-s", type=float, default=.5)
    freeze_parser.add_argument("--rho-m", type=float, default=30.)
    freeze_parser.add_argument("--translation-limit-m", type=float, default=.05)
    freeze_parser.add_argument("--rotation-limit-rad", type=float, default=.01)
    freeze_parser.add_argument("--derivation-source", action="append")
    freeze_parser.add_argument("--heldout-source", action="append")
    freeze_parser.add_argument("--holdout-kind", choices=("retrospective_source_disjoint", "untouched_source_disjoint"),
                               default="retrospective_source_disjoint")
    freeze_parser.add_argument("--blinding-evidence-sha256", default="")
    freeze_parser.add_argument("--conditional-model-scope-sha256", default="")
    evaluator = sub.add_parser("evaluate")
    evaluator.add_argument("--protocol", type=Path, required=True)
    evaluator.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "freeze":
        freeze(args.input_aliases, args.output, max_age_s=args.max_age_s, rho_m=args.rho_m,
               translation_limit_m=args.translation_limit_m, rotation_limit_rad=args.rotation_limit_rad,
               derivation_sources=args.derivation_source, heldout_sources=args.heldout_source,
               holdout_kind=args.holdout_kind, blinding_evidence_sha256=args.blinding_evidence_sha256,
               conditional_model_scope_sha256=args.conditional_model_scope_sha256)
    else:
        evaluate(args.protocol, args.output)


if __name__ == "__main__":
    main()
