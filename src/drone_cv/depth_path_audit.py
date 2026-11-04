"""Offline matched-stage accounting; missing authoritative taps stay UNKNOWN."""
from collections import Counter, defaultdict

import numpy as np

STAGES = ("source_publish", "bridge_receive", "bridge_publish", "recorder_callback",
          "enqueue", "storage_complete")
MAX_EVENTS = 1_000_000


def _integer(value, name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _gaps(values: list[int]) -> dict:
    if len(values) < 2:
        return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None}
    # Distinct capture/wall instants in chronological order; duplicates and
    # source reordering are reported separately, never silently called loss.
    gaps = np.diff(sorted(set(values)))/1e9
    if not len(gaps):
        return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None}
    return {"count": len(gaps), **dict(zip(("p50", "p95", "p99", "max"),
        (float(x) for x in np.quantile(gaps, [.5, .95, .99, 1.]))))}


def audit(contract: dict, events: list[dict]) -> dict:
    """Analyze one frozen stream/epoch/source window, not unsynchronized totals.

    Stage coverage must attest every identity in [source_start_ns,source_end_ns)
    and bounded drain. Source sequences must be preserved across all stages;
    recorder-local arrival ordinals cannot be relabelled as publisher sequences.
    Sequence=None uses capture stamp plus unchanged payload identity (multisets).
    """
    if (not isinstance(contract, dict) or type(contract.get("schema")) is not int
            or contract["schema"] != 1 or not isinstance(events, list) or len(events) > MAX_EVENTS):
        raise ValueError("unsupported or oversized depth audit")
    start = _integer(contract.get("source_start_ns"), "window start")
    end = _integer(contract.get("source_end_ns"), "window end")
    epoch, stream = contract.get("epoch"), contract.get("stream")
    if end <= start or not isinstance(epoch, str) or not epoch or not isinstance(stream, str) or not stream:
        raise ValueError("explicit nonempty epoch/stream/window required")
    coverage = contract.get("coverage")
    if not isinstance(coverage, dict) or any(stage not in STAGES for stage in coverage):
        raise ValueError("invalid stage coverage")
    observed, gaps, metadata, late = {}, {}, {}, Counter()
    grouped = defaultdict(list)
    for event in events:
        if not isinstance(event, dict):
            raise ValueError("audit event must be an object")
        stage = event.get("stage")
        if stage not in STAGES:
            raise ValueError("unknown audit stage")
        if event.get("epoch") != epoch or event.get("stream") != stream:
            raise ValueError("mixed stream/epoch is not a matched window")
        stamp = _integer(event.get("source_ns"), "source stamp")
        wall = _integer(event.get("wall_ns"), "stage wall stamp")
        sequence = event.get("source_sequence")
        if sequence is not None:
            _integer(sequence, "source sequence")
        payload = event.get("payload_identity")
        if not isinstance(payload, str) or not payload or len(payload) > 128:
            raise ValueError("preserved payload identity required")
        if start <= stamp < end:
            grouped[stage].append((stamp, sequence, payload, wall))
        else:
            late[stage] += 1
    for stage in STAGES:
        rows = grouped[stage]
        identities = Counter((stamp, sequence, payload) for stamp, sequence, payload, _ in rows)
        observed[stage] = identities
        stamps = [row[0] for row in rows]
        gaps[stage] = {"source_sim_gap_s": _gaps(stamps),
                       "receive_wall_gap_s": _gaps([row[3] for row in rows])}
        value = coverage.get(stage, {})
        if not isinstance(value, dict):
            raise ValueError("stage coverage must be an object")
        valid = (value.get("authoritative") is True and value.get("complete") is True
                 and value.get("drained") is True and type(value.get("probe_loss_count")) is int
                 and value["probe_loss_count"] == 0
                 and value.get("source_start_ns") == start and value.get("source_end_ns") == end
                 and isinstance(value.get("provenance"), str) and bool(value["provenance"]))
        metadata[stage] = {"coverage_complete": valid, "observed_count": sum(identities.values()),
            "duplicate_count": sum(max(0, count-1) for count in identities.values()),
            "source_reorder_count": sum(a>b for a,b in zip(stamps, stamps[1:])),
            "outside_window_count": late[stage], "coverage": value}
    comparisons = []
    for before, after in zip(STAGES, STAGES[1:]):
        valid = metadata[before]["coverage_complete"] and metadata[after]["coverage_complete"]
        deficit = observed[before]-observed[after]
        excess = observed[after]-observed[before]
        comparisons.append({"before": before, "after": after,
            "classification": ("UNKNOWN_COVERAGE" if not valid else "MATCHED" if not deficit and not excess
                               else "MATCHED_IDENTITY_DEFICIT" if deficit else "UNEXPECTED_DOWNSTREAM_IDENTITY"),
            "measured_deficit_count": sum(deficit.values()) if valid else None,
            "measured_excess_count": sum(excess.values()) if valid else None,
            "conditional_missing_identities": [list(key)+[count] for key,count in deficit.items()][:100],
            "identities_truncated": len(deficit)>100})
    schedule = contract.get("planned_source_ns")
    missing_source = None
    if schedule is not None:
        if (not isinstance(schedule, list) or len(schedule) > MAX_EVENTS
                or not contract.get("schedule_provenance")):
            raise ValueError("bounded frozen source schedule and provenance required")
        for value in schedule:
            _integer(value, "planned source stamp")
            if not start <= value < end:
                raise ValueError("planned source outside matched window")
        if len(set(schedule)) != len(schedule):
            raise ValueError("duplicate source schedule identity")
        if metadata["source_publish"]["coverage_complete"]:
            missing_source = sorted(set(schedule)-{row[0] for row in grouped["source_publish"]})
    complete = all(item["coverage_complete"] for item in metadata.values())
    return {"schema": 1, "scope": "conditional_matched_stage_audit_not_latency_or_motion_bound",
            "stream": stream, "epoch": epoch, "source_window_ns": [start,end],
            "stages": metadata, "comparisons": comparisons, "gaps": gaps,
            "all_stage_coverage_complete": complete,
            "no_unexplained_deficit": complete and all(row["classification"] == "MATCHED" for row in comparisons),
            "unpublished_requested_source_ns": missing_source,
            "source_skip_observable": metadata["source_publish"]["coverage_complete"] and schedule is not None,
            "note": "Source skip needs independent requested-frame schedule; CameraInfo is not a production count.",
            "end_to_end_latency_bound": None, "flight_admission": False}


def historical_recording(rows: list[dict], manifest: dict) -> dict:
    """Summarize immutable old stored receive trace without synthesizing lost taps."""
    streams = sorted({row["topic"] for row in rows if "/camera/depth/" in row["topic"]})
    summaries = {}
    for stream in streams:
        selected = [row for row in rows if row["topic"] == stream and not row.get("seeded")]
        stamps = [row["source_ns"] for row in selected if type(row.get("source_ns")) is int]
        summaries[stream] = {"stored_events": len(selected), "source_sim_gap_s": _gaps(stamps),
            "receive_wall_gap_s": _gaps([row["received_wall_ns"] for row in selected]),
            "stored_duplicate_stamps": len(stamps)-len(set(stamps)),
            "source_reorder_count": sum(a>b for a,b in zip(stamps,stamps[1:]))}
    return {"schema": 1, "scope": "historical_persisted_receive_trace_only",
        "recording_complete": manifest.get("complete"), "streams": summaries,
        "all_stage_coverage_complete": False, "loss_location_certified": False,
        "missing_authoritative_stages": ["source_publish","bridge_receive","bridge_publish"],
        "callback_enqueue_storage_identity_proof": manifest.get("storage_writer", {}).get("depth_audit_schema") == 1,
        "end_to_end_latency_bound": None, "flight_admission": False}
