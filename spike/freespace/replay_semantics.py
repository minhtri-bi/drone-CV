"""Pinned-decoder stationary truth/TTL diagnostics; no ROS node or motion authority."""

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import asdict
import hashlib
from itertools import product
import json
from pathlib import Path

import numpy as np

from capture_pose import camera_from_info, decode_depth, interpolate_pose
from drone_cv.oracle_truth import TRUTH_TOPIC, body_truth, decode_wire
from evidence import EvidenceSource, Envelope, EvidenceStore, Frame, SensorProfile, decide, swept_envelope
from evidence_variants import early_obstacle_variant, gap_variant
from plane_diagnostics import plane_crosscheck
from recording_integrity import require_alignment_epoch, validate_alignment
from replay_bag import collect, load_recording
from truth_error import all_pairs, interpolate_truth, inverse, ray_radius, rigid, transform_error


def load_frames(recording):
    records, manifest, fixture, events = load_recording(recording)
    groups, mount, rejected = collect(records, fixture)
    contract = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    if mount is None or len(groups) != 1:
        raise ValueError("SEMANTICS_REQUIRES_ONE_VALID_EPOCH")
    epoch_id, group = next(iter(groups.items()))
    require_alignment_epoch(contract, group["clock_epoch"], group["odom"])
    if group["alignment_invalid"]:
        raise ValueError("ALIGNMENT_EPOCH_INVALIDATED")
    if not group["clocks"]:
        raise ValueError("SEMANTICS_CLOCK_COVERAGE_MISSING")
    first_clock, last_clock = min(group["clocks"]), max(group["clocks"])
    frames, camera, considered = [], None, 0
    for sequence, (stamp, image, _) in enumerate(group["images"]):
        if stamp < manifest["warmup_end_sim_ns"]:
            continue
        considered += 1
        try:
            if not first_clock <= stamp <= last_clock:
                raise ValueError("CAPTURE_OUTSIDE_CLOCK_EPOCH")
            if stamp not in group["infos"]:
                raise ValueError("CAMERA_INFO_MISSING")
            camera = camera_from_info(group["infos"][stamp], mount)
            pose = interpolate_pose(group["odom"], stamp, fixture["max_pose_gap_ns"])
            depth = decode_depth(image)
            if depth.shape != (camera.intrinsics.height, camera.intrinsics.width):
                raise ValueError("DEPTH_CALIBRATION_MISMATCH")
            frames.append(Frame(depth, pose, f"semantics-epoch-{epoch_id}", stamp/1e9, sequence))
        except ValueError as error:
            rejected[str(error)] += 1
    if (len(frames) < fixture["minimum_frames"]
            or len(frames)/max(1, considered) < fixture["minimum_pair_fraction"]):
        raise ValueError("SEMANTICS_CAPTURE_PAIRING_INSUFFICIENT")
    return records, manifest, fixture, group, tuple(frames), camera, dict(rejected)


def truth_diagnostic(recording, records, manifest, fixture, group, frames, camera, ttl_ns):
    """Consume one strictly validated epoch; freeze the discrete query population."""
    if manifest["recording_schema"] != 4:
        raise ValueError("CONTINUOUS_TRUTH_SCHEMA_REQUIRED")
    if not frames or len({frame.epoch for frame in frames}) != 1 or group["alignment_invalid"]:
        raise ValueError("SEMANTICS_REQUIRES_ONE_VALID_EPOCH")
    contract = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    require_alignment_epoch(contract, group["clock_epoch"], group["odom"])
    truth = []
    for topic, message, _ in records:
        if topic == TRUTH_TOPIC:
            sample = body_truth(decode_wire(message), manifest["truth_entities"])
            truth.append((sample.source_ns, np.asarray(sample.world_from_body)))
    maximum_gap = fixture["oracle"]["fixture"]["max_truth_gap_ns"]
    alignment = np.asarray(manifest["world_from_odom"])
    sensor = SensorProfile(0., 0., EvidenceSource.HISTORICAL_ANALYSIS,
                           "Historical semantics ray radius; not calibrated sensing evidence", False)
    radius = ray_radius(camera, sensor.far_trusted_m)
    captures, queries, skipped, bracket_deltas, absolute = [], [], Counter(), [], []
    capture_coverage, query_coverage, missing = [], [], []
    odom_stamps = [sample.timestamp_ns for sample in group["odom"]]
    for frame in frames:
        stamp = round(frame.time_s*1e9)
        try:
            actual, delta = interpolate_truth(truth, stamp, maximum_gap)
            estimate = alignment @ rigid(frame.pose)
            index = bisect_left(odom_stamps, stamp)
            left = right = odom_stamps[index]
            if right != stamp:
                left = odom_stamps[index-1]
            gap = right-left
            captures.append((stamp, estimate, actual))
            bracket_deltas.append(delta)
            capture_coverage.append({"stamp_ns": stamp, "sequence": frame.sequence, "epoch": frame.epoch,
                                     "truth_endpoint_delta_ns": delta, "truth_bracket_gap_ns": delta[1]-delta[0],
                                     "truth_interpolation_fraction": -delta[0]/(delta[1]-delta[0]) if delta[1] != delta[0] else 0.,
                                     "px4_endpoint_delta_ns": [left-stamp, right-stamp],
                                     "px4_bracket_gap_ns": gap,
                                     "px4_interpolation_fraction": (stamp-left)/gap if gap else 0.})
            error = inverse(estimate) @ actual
            absolute.append({"stamp_ns": stamp, "sequence": frame.sequence, "epoch": frame.epoch,
                             **transform_error(error, radius["rho_m"]),
                             "world_translation_error_m": (actual[:3, 3]-estimate[:3, 3]).tolist()})
        except ValueError as error:
            skipped["capture:"+str(error)] += 1
            missing.append({"population": "capture", "stamp_ns": stamp,
                            "sequence": frame.sequence, "reason": str(error)})
    first, last = round(frames[0].time_s*1e9), round(frames[-1].time_s*1e9)
    available_queries = [sample for sample in group["odom"] if first <= sample.timestamp_ns <= last]
    for sample in available_queries:
        stamp = sample.timestamp_ns
        try:
            actual, delta = interpolate_truth(truth, stamp, maximum_gap)
            estimate = alignment @ rigid(interpolate_pose([sample], stamp, fixture["max_pose_gap_ns"]))
            queries.append((stamp, estimate, actual))
            bracket_deltas.append(delta)
            query_coverage.append({"stamp_ns": stamp, "epoch": frames[0].epoch,
                                   "truth_endpoint_delta_ns": delta, "truth_bracket_gap_ns": delta[1]-delta[0],
                                   "truth_interpolation_fraction": -delta[0]/(delta[1]-delta[0]) if delta[1] != delta[0] else 0.})
        except ValueError as error:
            skipped["query:"+str(error)] += 1
            missing.append({"population": "query", "stamp_ns": stamp, "reason": str(error)})
    if not captures or not queries:
        raise ValueError("NO_TRUTH_COVERED_POSE_PAIRS")
    relative = all_pairs(captures, queries, ttl_ns, radius["rho_m"])
    identities = {round(frame.time_s*1e9): frame.sequence for frame in frames}
    for pair in relative["worst_pairs"].values():
        pair["capture_sequence"], pair["epoch"] = identities[pair["capture_ns"]], frames[0].epoch
    errors = np.array([row["world_translation_error_m"] for row in absolute])
    return {"scope": "STATIONARY_TRUTH_DIAGNOSTIC_NOT_CALIBRATION", "motion_authority": False,
            "source": {"recording_manifest_sha256": hashlib.sha256((recording/"manifest.json").read_bytes()).hexdigest()},
            "relative_error": relative,
            "absolute_error": {"samples": absolute, "maximum_axis_translation_m": np.abs(errors).max(axis=0).tolist(),
                               "maximum_translation_m": max(row["translation_m"] for row in absolute),
                               "maximum_rotation_rad": max(row["rotation_rad"] for row in absolute)},
            "sensing_caps": {"near_trusted_m": sensor.near_trusted_m,
                             "far_trusted_m": sensor.far_trusted_m, **radius,
                             "renderer_far": ray_radius(camera, camera.far_m)},
            "coverage": {"epoch": frames[0].epoch, "clock_epoch": group["clock_epoch"],
                         "available_capture_count": len(frames), "covered_capture_count": len(captures),
                         "available_query_count": len(available_queries), "covered_query_count": len(queries),
                         "query_population_rule": "Every recorded PX4 timestamp_sample between first and last accepted image stamps, inclusive",
                         "query_population_ns": [sample.timestamp_ns for sample in available_queries],
                         "query_maximum_gap_ns": max(np.diff([row[0] for row in queries]).tolist(), default=0),
                         "observed_capture_span_ns": last-first,
                         "full_ttl_duration_observed": last-first >= ttl_ns,
                         "capture_brackets": capture_coverage, "query_brackets": query_coverage,
                         "missing_samples": missing, "rejected": dict(skipped),
                         "truth_samples": len(truth), "max_truth_bracket_ns": maximum_gap,
                         "bracket_endpoint_delta_ns_min": np.min(bracket_deltas, axis=0).tolist(),
                         "bracket_endpoint_delta_ns_max": np.max(bracket_deltas, axis=0).tolist(),
                         "complete_observed_grid": not skipped, "continuous_time_certified": False},
            "clock_note": "Shared Gazebo/PX4 simulation epoch is the pinned recording contract, not independently measured synchronization. Bracket deltas include cadence.",
            "roi_note": "Original grading ROI and thresholds unchanged; this direct truth comparison does not fit plane residuals.",
            "limitations": ["All retained capture/query pairs on the declared observed grid are evaluated, not continuous query times.",
                            "Stationary measured errors are uncalibrated, non-admitting diagnostics, not future-motion pose bounds."]}


def full_profile_envelope(profile, pose, observed_bound_m):
    from model_liveness import physical_fixture
    physical_fixture(profile)
    if not np.isfinite(observed_bound_m) or observed_bound_m < 0:
        raise ValueError("INVALID_FULL_PROFILE_DIAGNOSTIC")
    corners = np.concatenate([np.array(list(product(*zip(part["lo"], part["hi"]))))
                              for part in profile["parts"]])
    rotated = corners @ pose.world_from_body().T
    inflation = .5+observed_bound_m
    radius = float(np.linalg.norm(rotated[:, :2], axis=1).max())+inflation
    return Envelope((pose.x, pose.y), (pose.x+np.cos(pose.yaw_rad), pose.y+np.sin(pose.yaw_rad)),
                    radius, pose.z+float(rotated[:, 2].min())-inflation,
                    pose.z+float(rotated[:, 2].max())+inflation)


def envelope_comparison(frames, camera, profile, diagnostic, ttl_ns):
    pose = frames[-1].pose
    now = frames[-1].time_s
    sensor = SensorProfile(
        near_translation_uncertainty_m=diagnostic["relative_error"]["maximum_translation_m"],
        near_rotation_uncertainty_rad=diagnostic["relative_error"]["maximum_rotation_rad"],
        source=EvidenceSource.STATIONARY_DIAGNOSTIC,
        provenance="Same-bag all-pairs stationary truth-relative observed maxima; not calibration",
        no_return_certified=False)
    store = EvidenceStore(camera, sensor, frames)
    observed_bound = diagnostic["relative_error"]["maximum_bound_m"]
    envelopes = {"historical_stationary": swept_envelope(pose, 1.),
                 "full_model_observed_error_diagnostic": full_profile_envelope(profile, pose, observed_bound)}
    rows = {}
    for name, envelope in envelopes.items():
        result, _, _ = decide(store, envelope, epoch=frames[-1].epoch, now_s=now, ttl_s=ttl_ns/1e9)
        rows[name] = {"envelope": asdict(envelope), "decision": asdict(result)}
    return {"scope": "REAL_REPLAY_DIAGNOSTIC_NOT_ADMISSION", "motion_authority": False,
            "full_profile_source_sha256": profile["source_sha256"], "observed_bound_m": observed_bound,
            "navigation_margin_m": .5, "tracking_bound_m": None, "calibrated_pose_bound_m": None,
            "sensor_profile": asdict(sensor),
            "uncertainty_note": "Observed stationary maxima inflate both near hazards and the model query; conservative diagnostic only, not calibrated admission.",
            "query_ns": round(now*1e9), "results": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--mode", choices=("truth", "temporal", "planes"), required=True)
    parser.add_argument("--ttl-s", type=float, default=10.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.ttl_s) or args.ttl_s <= 0:
        raise ValueError("INVALID_TTL")
    ttl_ns = round(args.ttl_s*1e9)
    records, manifest, fixture, group, frames, camera, rejected = load_frames(args.recording)
    if args.mode == "truth":
        if args.profile is None:
            raise ValueError("FULL_PROFILE_REQUIRED")
        report = truth_diagnostic(args.recording, records, manifest, fixture, group, frames, camera, ttl_ns)
        profile_bytes = args.profile.read_bytes()
        if hashlib.sha256(profile_bytes).hexdigest() != manifest["oracle_profile_sha256"]:
            raise ValueError("DIAGNOSTIC_PROFILE_BINDING_MISMATCH")
        report["envelopes"] = envelope_comparison(frames, camera, json.loads(profile_bytes), report, ttl_ns)
        report["capture_pairing_rejected"] = rejected
        report["all_checks_pass"] = report["coverage"]["complete_observed_grid"] and not rejected
    elif args.mode == "planes":
        report = plane_crosscheck(records, manifest, fixture, frames, camera)
        report["source_manifest_sha256"] = hashlib.sha256((args.recording/"manifest.json").read_bytes()).hexdigest()
        report["capture_pairing_rejected"] = rejected
        report["all_checks_pass"] = report["diagnostic_complete"] and not rejected
    else:
        gap = gap_variant(frames, camera, ttl_ns, swept_envelope(frames[-1].pose, 1.))
        early = early_obstacle_variant(frames, camera, ttl_ns)
        start, end = gap["gap_ns"]
        clock_stamps = [stamp for stamp in group["clocks"] if start <= stamp < end]
        odom_stamps = [sample.timestamp_ns for sample in group["odom"] if start <= sample.timestamp_ns < end]
        progress = {"clock_sample_count": len(clock_stamps), "odom_sample_count": len(odom_stamps),
                    "clock_source_span_ns": max(clock_stamps)-min(clock_stamps) if clock_stamps else 0,
                    "odom_source_span_ns": max(odom_stamps)-min(odom_stamps) if odom_stamps else 0}
        progress["both_sources_progressed"] = (progress["clock_source_span_ns"] > 0
                                               and progress["odom_source_span_ns"] > 0)
        gap["unmodified_stream_progress_during_image_gap"] = progress
        report = {"scope": "OFFLINE_DERIVATIVES_OF_STRICT_REAL_RECORDING", "motion_authority": False,
                  "source_manifest_sha256": hashlib.sha256((args.recording/"manifest.json").read_bytes()).hexdigest(),
                  "accepted_frames": len(frames), "rejected": rejected, "ttl_ns": ttl_ns,
                  "gap": gap, "early_obstacle": early,
                  "recording_schema": manifest["recording_schema"],
                  "all_checks_pass": (gap["all_checks_pass"] and early["all_checks_pass"]
                                     and progress["both_sources_progressed"] and not rejected)}
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in ("scope", "motion_authority", "all_checks_pass")}))
    return 0 if report["all_checks_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
