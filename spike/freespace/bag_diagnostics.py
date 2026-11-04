"""Diagnostics from decoded messages and actual acquisition timestamps; never a pose bound."""

from collections import Counter

import numpy as np

from recording_integrity import (ORACLE_SCHEMA, REQUIRED_ORACLE_STREAMS, TRUTH_TOPIC,
                                 advance_progress, require_fresh, source_stamp_ns, validate_policy)


def validate_receive_log(records: list, manifest: dict, policy: dict, events: list, *,
                         serialized_hashes: list | None = None) -> None:
    validate_policy(policy)
    progress, sequences, logged = {}, Counter(), Counter()
    topics = set(manifest["topics"])
    schema4 = policy["schema"] == ORACLE_SCHEMA
    if schema4:
        from oracle_recording import validate_entities
        validate_entities(manifest.get("truth_entities"))
        if (manifest.get("recording_schema") != ORACLE_SCHEMA
                or not REQUIRED_ORACLE_STREAMS | {"/clock", "/tf_static"} <= topics
                or manifest["topics"].get(TRUTH_TOPIC) != "std_msgs/msg/UInt8MultiArray"):
            raise ValueError("SCHEMA4_REQUIRED_TRUTH_STREAM_MISMATCH")
    previous_wall = None
    for event in events:
        topic = event["topic"]
        if topic not in topics:
            raise ValueError("RECEIVE_LOG_UNKNOWN_TOPIC")
        if schema4:
            if (type(event["received_wall_ns"]) is not int
                    or previous_wall is not None and event["received_wall_ns"] <= previous_wall
                    or event.get("phase") not in ("preflight", "collection")
                    or event.get("validation_error") is not None):
                raise ValueError("INVALID_SCHEMA4_RECEIVE_EVIDENCE")
            previous_wall = event["received_wall_ns"]
            if (event["seeded"] or event["phase"] != (
                    "preflight" if event["received_wall_ns"] < manifest.get(
                        "record_start_wall_monotonic_ns", float("inf")) else "collection")):
                raise ValueError("SCHEMA4_RECEIVE_PHASE_MISMATCH")
        sequences[topic] += 1
        if event["topic_sequence"] != sequences[topic]:
            raise ValueError("RECEIVE_LOG_SEQUENCE_GAP")
        if event["source_ns"] is not None:
            progress = advance_progress(progress, topic, event["source_ns"], event["received_wall_ns"])
        logged[topic] += 1
        if not event["seeded"]:
            require_fresh(progress, policy, event["received_wall_ns"],
                          sim_now_ns=event["received_sim_ns"],
                          allow_startup_missing=schema4 and event["phase"] == "preflight")
    require_fresh(progress, policy, manifest["record_end_wall_monotonic_ns"],
                  sim_now_ns=manifest.get("record_end_sim_ns"))
    actual = Counter(topic for topic, _, _ in records)
    if dict(actual) != manifest["topic_counts"] or actual != logged:
        raise ValueError("RECEIVE_LOG_BAG_COUNTS_MISMATCH")
    if schema4 and set(actual) != topics:
        raise ValueError("SCHEMA4_REQUIRED_STREAM_NOT_RECORDED")
    by_topic = {topic: [] for topic in topics}
    for topic, message, storage_ns in records:
        by_topic[topic].append((source_stamp_ns(topic, message), storage_ns))
    if schema4:
        from drone_cv.oracle_truth import body_truth, decode_wire
        if serialized_hashes is None or len(serialized_hashes) != len(records):
            raise ValueError("RECEIVE_LOG_RAW_SERIALIZED_PROVENANCE_REQUIRED")
        topic_events = {topic: iter([event for event in events if event["topic"] == topic])
                        for topic in topics}
        previous_truth = None
        truth_count = 0
        alignment_packet = manifest["alignment_source"]["truth_packet"]
        alignment_seen = False
        for (topic, message, stored_ns), (raw_topic, raw_stored_ns, digest) in zip(records, serialized_hashes):
            if (raw_topic, raw_stored_ns) != (topic, stored_ns):
                raise ValueError("RECEIVE_LOG_RAW_SERIALIZED_ORDER_MISMATCH")
            event = next(topic_events[topic])
            if digest != event.get("serialized_sha256"):
                raise ValueError("RECEIVE_LOG_SERIALIZED_HASH_MISMATCH")
            if topic == TRUTH_TOPIC:
                sample = body_truth(decode_wire(message), manifest["truth_entities"])
                from oracle_recording import packet_evidence
                packet = packet_evidence(message, event["received_wall_ns"], event["received_sim_ns"])
                if packet["payload_sha256"] != event.get("truth_payload_sha256"):
                    raise ValueError("RECEIVE_LOG_TRUTH_PAYLOAD_HASH_MISMATCH")
                if (all(packet[key] == alignment_packet.get(key) for key in packet)
                        and sample.source_ns == alignment_packet.get("source_ns")
                        and manifest["truth_entities"] == alignment_packet.get("entities")):
                    alignment_seen = True
                if (previous_truth is not None and sample.source_ns <= previous_truth
                        or sample.source_ns != event["source_ns"]):
                    raise ValueError("TRUTH_STAMP_NOT_ADVANCING")
                if (previous_truth is not None
                        and sample.source_ns-previous_truth > manifest["oracle"]["max_gap_ns"]):
                    raise ValueError("TRUTH_COVERAGE_GAP")
                age_ns = event["received_sim_ns"]-sample.source_ns
                if (not (event["phase"] == "preflight" and event["received_sim_ns"] == 0)
                        and not -policy["max_source_clock_lead_ns"] <= age_ns <= policy["stream_age_sim_s"][topic]*1e9):
                    raise ValueError("TRUTH_SIM_AGE_EXCEEDED")
                previous_truth = sample.source_ns
                truth_count += 1
        if not truth_count:
            raise ValueError("REQUIRED_TRUTH_STREAM_MISSING")
        if not alignment_seen:
            raise ValueError("ALIGNMENT_TRUTH_NOT_RECORDED")
        if not manifest.get("complete"):
            raise ValueError("SCHEMA4_COLLECTION_INCOMPLETE")
        if not manifest.get("oracle", {}).get("coverage_complete"):
            raise ValueError("SCHEMA4_ORACLE_COVERAGE_INCOMPLETE")
    event_rows = {topic: [] for topic in topics}
    for event in events:
        event_rows[event["topic"]].append((event["source_ns"], event["received_sim_ns"]))
    if by_topic != event_rows:
        raise ValueError("RECEIVE_LOG_MESSAGE_MISMATCH")
    statuses = [message for topic, message, _ in records if topic.endswith("vehicle_status_v4")]
    if not statuses or any(message.arming_state != message.ARMING_STATE_DISARMED for message in statuses):
        raise ValueError("STATUS_EVIDENCE_NOT_DISARMED")


def quantiles(values: list, scale: float = 1.) -> dict | None:
    if not len(values):
        return None
    return dict(zip(("p50", "p95", "maximum"), (np.quantile(values, [.5, .95, 1])*scale).tolist()))


def diagnostics(records: list, manifest: dict, events: list | None = None) -> dict:
    images = [(source_stamp_ns(topic, msg), stored) for topic, msg, stored in records if topic.endswith("image_raw")]
    infos = [source_stamp_ns(topic, msg) for topic, msg, _ in records if topic.endswith("camera_info")]
    odometry = [(msg, stored) for topic, msg, stored in records if topic.endswith("vehicle_odometry")]
    clocks = [source_stamp_ns(topic, msg) for topic, msg, _ in records if topic == "/clock"]
    image_stamps = [stamp for stamp, _ in images]
    span = (max(clocks)-min(clocks))/1e9 if clocks else None
    wall = manifest.get("recorded_wall_s")
    result = {"clock_span_sim_s": span, "recorded_wall_s": wall,
              "mean_rtf": span/wall if span is not None and wall and wall > 0 else None,
              "warmup_sim_s": (manifest["warmup_end_sim_ns"]-manifest["record_start_sim_ns"])/1e9,
              "image_count": len(images), "camera_info_count": len(infos),
              "camera_info_without_image_stamps": len(set(infos)-set(image_stamps)),
              "image_without_camera_info_stamps": len(set(image_stamps)-set(infos)),
              "image_gap_sim_ms": quantiles(np.diff(image_stamps), 1e-6),
              "availability_note": "Observed gaps and unmatched stamps are not a proven renderer/transport loss fraction.",
              "storage_minus_capture_sim_ms": {
                  "depth": quantiles([stored-stamp for stamp, stored in images], 1e-6),
                  "odometry": quantiles([stored-int(msg.timestamp_sample)*1000 for msg, stored in odometry], 1e-6)},
              "source_minus_received_clock_sim_ms": {
                  "depth": quantiles([stamp-stored for stamp, stored in images], 1e-6),
                  "odometry": quantiles([int(msg.timestamp_sample)*1000-stored for msg, stored in odometry], 1e-6)},
              "clock_offset_note": "Difference to the recorder's latest delivered ROS clock, including clock transport lag; not independent timebase calibration.",
              "receive_wall_diagnostics": None,
              "variance_note": "Reported estimator variance is not calibrated relative drift or a motion safety bound."}
    if odometry:
        messages = [msg for msg, _ in odometry]
        position = np.array([msg.position for msg in messages], dtype=float)
        result["odometry"] = {"reset_counters": sorted(set(int(msg.reset_counter) for msg in messages)),
                               "pose_frames": sorted(set(int(msg.pose_frame) for msg in messages)),
                               "position_ned_peak_to_peak_m": [
                                   float(np.ptp(column)) if np.isfinite(column).all() else None
                                   for column in position.T],
                               "quality_note": "Pinned VehicleOdometry quality is unused; zero is not an admission test."}
        for field, unit in (("position_variance", "m"), ("orientation_variance", "rad")):
            variance = np.array([getattr(msg, field) for msg in messages], dtype=float)
            valid = np.isfinite(variance) & (variance >= 0)
            result["odometry"][field] = {
                "invalid_components": int((~valid).sum()),
                "last_sigma_"+unit: [float(np.sqrt(value)) if np.isfinite(value) and value >= 0 else None
                                     for value in variance[-1]],
                "frame": "body" if field == "orientation_variance" else "pose reference"}
    if events is not None:
        clock_events = [event for event in events if event["topic"] == "/clock" and not event["seeded"]]
        if len(clock_events) >= 2:
            origin = clock_events[0]["received_wall_ns"]
            wall_times = np.array([event["received_wall_ns"]-origin for event in clock_events])/1e9
            sim_times = np.array([event["source_ns"] for event in clock_events])/1e9
            boundaries = np.arange(0., np.floor(wall_times[-1])+1, 1.)
            progress = np.diff(np.interp(boundaries, wall_times, sim_times))
            result["receive_wall_diagnostics"] = {
                "minimum_observed_rtf_in_nonoverlapping_1s_wall_windows": float(progress.min()) if progress.size else None,
                "complete_window_count": len(progress),
                "note": "Interpolated received clock progress; not a physical compute or flight deadline."}
    else:
        result["missing_wall_provenance"] = "Historical bag has no per-message monotonic receive trace; windowed RTF is unavailable."
    return result


def geometry_summary(rows: list) -> dict:
    if not rows:
        return {}
    return {name: {"minimum_support": min(row["targets"][name]["support"] for row in rows),
                   "first_frame_p95_m": rows[0]["targets"][name]["p95_m"],
                   "worst_frame_p95_m": max((row["targets"][name]["p95_m"] for row in rows
                                              if row["targets"][name]["p95_m"] is not None), default=None),
                   "maximum_error_m": max((row["targets"][name]["maximum_m"] for row in rows
                                             if row["targets"][name]["maximum_m"] is not None), default=None)}
            for name in rows[0]["targets"]}
