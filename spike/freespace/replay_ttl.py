"""Observe rolling expiry on an immutable longer stationary bag; never authorize motion."""

import argparse
from collections import Counter
import json
from pathlib import Path

from bag_diagnostics import diagnostics
from capture_pose import camera_from_info, decode_depth, interpolate_pose
from evidence import EvidenceSource, EvidenceStore, Frame, SensorProfile, decide, swept_envelope
from recording_integrity import require_alignment_epoch, validate_alignment
from replay_bag import collect, load_recording


def run(recording: Path) -> dict:
    records, manifest, fixture, events = load_recording(recording)
    span = (manifest["record_end_sim_ns"]-manifest["record_start_sim_ns"])/1e9
    ttl = fixture["evidence_ttl_sim_s"]
    if span < fixture["minimum_record_sim_s"] or span <= ttl:
        raise ValueError("RECORDING_TOO_SHORT_FOR_ROLLING_TTL")
    groups, mount, rejected = collect(records, fixture)
    contract = validate_alignment(manifest, fixture["max_pose_gap_ns"])
    if mount is None or len(groups) != 1:
        raise ValueError("TTL_REQUIRES_ONE_VALID_ALIGNMENT_EPOCH")
    epoch_id, group = next(iter(groups.items()))
    require_alignment_epoch(contract, group["clock_epoch"], group["odom"])
    if group["alignment_invalid"]:
        raise ValueError("ALIGNMENT_EPOCH_INVALIDATED")
    observations = []
    for stamp, image, _ in group["images"]:
        if stamp < manifest["warmup_end_sim_ns"]:
            continue
        try:
            if stamp not in group["infos"]:
                raise ValueError("CAMERA_INFO_MISSING")
            camera = camera_from_info(group["infos"][stamp], mount)
            pose = interpolate_pose(group["odom"], stamp, fixture["max_pose_gap_ns"])
            observations.append((stamp/1e9, image, pose, camera))
        except ValueError as error:
            rejected[str(error)] += 1
    considered = len(observations)+sum(rejected.values())
    if (len(observations) < fixture["minimum_frames"]
            or len(observations)/max(1, considered) < fixture["minimum_pair_fraction"]):
        raise ValueError("TTL_CAPTURE_PAIRING_INSUFFICIENT")
    first, last = observations[0][0], observations[-1][0]
    query_times = [first+ttl/2, first+ttl+1., last]
    if query_times[-2] >= last:
        raise ValueError("NOT_ENOUGH_POST_WARMUP_HISTORY")
    store = EvidenceStore(observations[0][3], SensorProfile(
        0., 0., EvidenceSource.HISTORICAL_ANALYSIS,
        "Historical TTL comparison; zero is an explicit uncalibrated analysis assumption", False))
    cursor, trace, latest_pose = 0, [], None
    epoch = f"ttl-epoch-{epoch_id}"
    for query_time in query_times:
        while cursor < len(observations) and observations[cursor][0] <= query_time:
            stamp, image, latest_pose, camera = observations[cursor]
            depth = decode_depth(image)
            if depth.shape != (camera.intrinsics.height, camera.intrinsics.width):
                raise ValueError("DEPTH_CALIBRATION_MISMATCH")
            store = EvidenceStore(camera, store.profile, store.retained(epoch, stamp, ttl))
            store = store.add(Frame(depth, latest_pose, epoch, stamp, cursor))
            cursor += 1
        retained = store.retained(epoch, query_time, ttl)
        decision, _, _ = decide(store, swept_envelope(latest_pose, 1.), epoch=epoch,
                                 now_s=query_time, ttl_s=ttl)
        trace.append({"query_sim_s": query_time, "observed_prefix_frames": cursor,
                      "retained_sequences": [frame.sequence for frame in retained],
                      "retained_capture_sim_s": [frame.time_s for frame in retained],
                      "expired_count": cursor-len(retained), "future_frames_used": False,
                      "allowed": decision.allowed, "reason": decision.reason,
                      "frames_used": decision.frames_used, "counts": decision.counts})
    stale_time = last+ttl+1.
    stale, _, _ = decide(store, swept_envelope(latest_pose, 1.), epoch=epoch,
                          now_s=stale_time, ttl_s=ttl)
    checks = {"longer_than_ttl": span > ttl,
              "old_and_new_frames_separated": trace[-1]["expired_count"] > 0 and trace[-1]["frames_used"] > 0,
              "first_frame_expires": 0 not in trace[-1]["retained_sequences"],
              "fresh_frames_remain": all(0 <= row["query_sim_s"]-stamp <= ttl+1e-9
                                          for row in trace for stamp in row["retained_capture_sim_s"]),
              "stationary_queries_stop": all(not row["allowed"] for row in trace),
              "stale_all_stops": stale.reason == "NO_FRESH_FRAMES" and not stale.allowed}
    return {"scope": "Real stationary temporal retention only", "motion_authority": False,
            "recording_sim_s": span, "ttl_sim_s": ttl, "accepted_frames": len(observations),
            "pair_fraction": len(observations)/max(1, considered), "rejected": dict(rejected),
            "queries": trace, "stale_query": {"query_sim_s": stale_time, "reason": stale.reason,
                                               "allowed": stale.allowed, "frames_used": stale.frames_used},
            "envelope_note": "Historical spike query solely for temporal diagnostics; not model-profile navigation clearance or uncertainty admission.",
            "no_return_certified": False, "checks": checks, "all_checks_pass": all(checks.values()),
            "diagnostics": diagnostics(records, manifest, events)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.recording)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in ("recording_sim_s", "ttl_sim_s", "accepted_frames", "checks", "all_checks_pass")}))
    return 0 if report["all_checks_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
