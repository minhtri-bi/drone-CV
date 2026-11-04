"""Explicit offline derivatives; no synthesized recording or navigation acceptance."""

from dataclasses import asdict
import heapq

import numpy as np

from evidence import EvidenceSource, Envelope, EvidenceStore, Frame, SensorProfile, decide, observed_surface_points


def scheduled_identities(rows, queries, ttl_ns, *, epoch):
    """Independent integer-stamp event oracle, keyed by (epoch, sequence).

    Rows are (source_ns, epoch, sequence). Expiry is a separate event one
    nanosecond after the inclusive TTL boundary, not a copy of retained().
    """
    if type(ttl_ns) is not int or ttl_ns <= 0:
        raise ValueError("INVALID_VARIANT_TTL")
    events, identities = [], set()
    for stamp, source_epoch, sequence in rows:
        identity = (source_epoch, sequence)
        if identity in identities:
            raise ValueError("DUPLICATE_VARIANT_IDENTITY")
        identities.add(identity)
        heapq.heappush(events, (stamp, 1, identity))
        heapq.heappush(events, (stamp+ttl_ns+1, -1, identity))
    active, result = set(), {}
    for query in sorted(queries):
        while events and events[0][0] <= query:
            _, action, identity = heapq.heappop(events)
            if action == 1:
                active.add(identity)
            else:
                active.discard(identity)
        result[query] = sorted(sequence for source_epoch, sequence in active if source_epoch == epoch)
    return result


def _require_epoch(frames, ttl_ns):
    if (not frames or len({frame.epoch for frame in frames}) != 1
            or any(b.time_s <= a.time_s or b.sequence <= a.sequence
                   for a, b in zip(frames, frames[1:]))):
        raise ValueError("VARIANT_REQUIRES_ONE_ORDERED_EPOCH")
    if type(ttl_ns) is not int or ttl_ns <= 0:
        raise ValueError("INVALID_VARIANT_TTL")


def gap_variant(frames, camera, ttl_ns, query_envelope):
    _require_epoch(frames, ttl_ns)
    first, last = round(frames[0].time_s*1e9), round(frames[-1].time_s*1e9)
    gap_start, gap_end = first+1_000_000_000, first+ttl_ns+2_000_000_000
    if last <= gap_end:
        raise ValueError("TRACE_TOO_SHORT_FOR_MIDTRACE_TTL_GAP")
    kept = tuple(frame for frame in frames if not gap_start <= round(frame.time_s*1e9) < gap_end)
    recovery = next(frame for frame in kept if round(frame.time_s*1e9) >= gap_end)
    queries = [gap_start-1, gap_end-500_000_000, round(recovery.time_s*1e9)]
    expected = scheduled_identities([(round(f.time_s*1e9), f.epoch, f.sequence) for f in kept],
                                    queries, ttl_ns, epoch=frames[0].epoch)
    trace = []
    for query in queries:
        prefix = tuple(frame for frame in kept if round(frame.time_s*1e9) <= query)
        store = EvidenceStore(camera, SensorProfile(0., 0., EvidenceSource.HISTORICAL_ANALYSIS,
                              "Temporal omission comparison; original capture uncertainty uncalibrated", False), prefix)
        actual = sorted(frame.sequence for frame in store.retained(frames[0].epoch, query/1e9, ttl_ns/1e9))
        decision, _, _ = decide(store, query_envelope, epoch=frames[0].epoch, now_s=query/1e9, ttl_s=ttl_ns/1e9)
        trace.append({"query_ns": query, "epoch": frames[0].epoch, "expected_sequences": expected[query],
                      "retained_sequences": actual, "identities_match": actual == expected[query],
                      "decision": asdict(decision)})
    passed = (all(row["identities_match"] for row in trace)
              and trace[1]["retained_sequences"] == []
              and trace[1]["decision"]["reason"] == "NO_FRESH_FRAMES"
              and trace[2]["retained_sequences"] == [recovery.sequence])
    return {"scope": "OFFLINE_IMAGE_STREAM_DROPOUT_DERIVATIVE", "gap_ns": [gap_start, gap_end],
            "dropped_sequences": [f.sequence for f in frames if gap_start <= round(f.time_s*1e9) < gap_end],
            "clock_and_odometry_modified": False, "queries": trace, "all_checks_pass": passed,
            "identity_oracle": "Integer source-stamp activation/expiry events keyed by epoch and sequence"}


def early_obstacle_variant(frames, camera, ttl_ns):
    _require_epoch(frames, ttl_ns)
    first = frames[0]
    k = camera.intrinsics
    u, v = int(k.cx), int(k.cy)
    z = 1.0
    depth = first.depth.copy()
    depth[max(0, v-1):v+2, max(0, u-1):u+2] = z
    changed = Frame(depth, first.pose, first.epoch, first.time_s, first.sequence)
    variants = (changed, *frames[1:])
    rotation, origin = camera.world_from_optical(first.pose)
    point = origin+rotation @ np.array([(u-k.cx)/k.fx*z, (v-k.cy)/k.fy*z, z])
    probe = Envelope(tuple(point[:2]), tuple(point[:2]), .03, float(point[2]-.03), float(point[2]+.03))
    expiry = round(first.time_s*1e9)+ttl_ns
    queries = [expiry-1_000_000, expiry+1_000_000]
    if round(frames[-1].time_s*1e9) < queries[-1]:
        raise ValueError("TRACE_TOO_SHORT_FOR_EARLY_OBSTACLE_EXPIRY")
    expected = scheduled_identities([(round(f.time_s*1e9), f.epoch, f.sequence) for f in variants],
                                    queries, ttl_ns, epoch=first.epoch)
    trace = []
    for query in queries:
        prefix = tuple(f for f in variants if round(f.time_s*1e9) <= query)
        store = EvidenceStore(camera, SensorProfile(0., 0., EvidenceSource.HISTORICAL_ANALYSIS,
                              "Early hazard retention comparison; not calibrated acquisition evidence", False), prefix)
        retained = store.retained(first.epoch, query/1e9, ttl_ns/1e9)
        contributors = [f.sequence for f in retained if np.any(probe.contains(
            observed_surface_points(f, camera, store.profile), inflate_m=store.profile.fixed_margin_m))]
        decision, _, _ = decide(store, probe, epoch=first.epoch, now_s=query/1e9, ttl_s=ttl_ns/1e9)
        actual = sorted(f.sequence for f in retained)
        trace.append({"query_ns": query, "epoch": first.epoch, "retained_sequences": actual,
                      "expected_sequences": expected[query], "identities_match": actual == expected[query],
                      "probe_surface_contributors": contributors, "decision": asdict(decision)})
    passed = (all(row["identities_match"] for row in trace)
              and trace[0]["probe_surface_contributors"] == [first.sequence]
              and not trace[0]["decision"]["allowed"]
              and trace[1]["probe_surface_contributors"] == []
              and bool(trace[1]["retained_sequences"]))
    return {"scope": "OFFLINE_EARLY_ONLY_PIXEL_PATCH_DERIVATIVE_NOT_CAPTURE", "source_sequence": first.sequence,
            "source_epoch": first.epoch, "expiry_ns_inclusive": expiry,
            "expected_patch_contributors": [[first.sequence], []],
            "identity_oracle": "Integer source-stamp activation/expiry events keyed by epoch and sequence",
            "pixel_patch": {"u_half_open": [max(0, u-1), min(k.width, u+2)],
                            "v_half_open": [max(0, v-1), min(k.height, v+2)], "optical_z_m": z},
            "probe": asdict(probe), "queries": trace, "all_checks_pass": passed,
            "note": "The tiny visibility probe is not a vehicle envelope. Expiry removes this contributor, not all other hazards."}
