from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from bag_diagnostics import diagnostics, validate_receive_log


STATUS = "/drone_cv_g1/fmu/out/vehicle_status_v4"


def trace_fixture():
    status = NS(timestamp=100_000, arming_state=1, ARMING_STATE_DISARMED=1)
    clock = NS(clock=NS(sec=0, nanosec=100_000_000))
    records = [(STATUS, status, 100_000_000), ("/clock", clock, 100_000_000)]
    policy = {"schema": 2, "startup_wall_s": 20., "alignment_wait_wall_s": 5.,
              "max_pose_gap_ns": 50_000_000,
              "stream_progress_wall_s": {STATUS: 2., "/clock": 2.}}
    manifest = {"topics": {STATUS: "status", "/clock": "clock"},
                "topic_counts": {STATUS: 1, "/clock": 1},
                "record_end_wall_monotonic_ns": 1_000_000_000}
    events = [{"topic": topic, "topic_sequence": 1, "source_ns": 100_000_000,
               "received_wall_ns": 0, "received_sim_ns": 100_000_000, "seeded": True}
              for topic in (STATUS, "/clock")]
    return records, manifest, policy, events


def test_receive_trace_detects_a_live_clock_after_status_stops():
    records, manifest, policy, events = trace_fixture()
    validate_receive_log(records, manifest, policy, events)
    later_clock = NS(clock=NS(sec=1, nanosec=0))
    clock_event = {**events[-1], "topic_sequence": 2, "source_ns": 1_000_000_000,
                   "received_sim_ns": 1_000_000_000, "received_wall_ns": 3_000_000_000,
                   "seeded": False}
    with pytest.raises(ValueError, match="STALLED:"+STATUS):
        validate_receive_log([*records, ("/clock", later_clock, 1_000_000_000)], manifest,
                             policy, [*events, clock_event])


def test_record_counts_cannot_hide_message_stamp_tampering_or_armed_status():
    records, manifest, policy, events = trace_fixture()
    changed = deepcopy(events)
    changed[0]["source_ns"] += 1
    with pytest.raises(ValueError, match="MESSAGE_MISMATCH"):
        validate_receive_log(records, manifest, policy, changed)
    armed = NS(timestamp=100_000, arming_state=2, ARMING_STATE_DISARMED=1)
    with pytest.raises(ValueError, match="NOT_DISARMED"):
        validate_receive_log([(STATUS, armed, records[0][2]), records[1]], manifest, policy, events)


def test_missing_wall_provenance_stays_unknown_and_invalid_variance_is_not_nan_json():
    odom = NS(timestamp_sample=100_000, position=[0, 0, 0], pose_frame=1, reset_counter=12,
              position_variance=[.01, np.nan, -.1], orientation_variance=[.04, 0, .09])
    record = [("/drone_cv_g1/fmu/out/vehicle_odometry", odom, 108_000_000)]
    manifest = {"warmup_end_sim_ns": 200, "record_start_sim_ns": 100,
                "recorded_wall_s": 30.}
    result = diagnostics(record, manifest)
    assert result["receive_wall_diagnostics"] is None
    assert result["mean_rtf"] is None
    assert result["odometry"]["position_variance"]["last_sigma_m"] == [.1, None, None]
    assert result["odometry"]["position_variance"]["invalid_components"] == 2


def test_windowed_rtf_uses_only_complete_observed_wall_windows():
    records = [("/clock", NS(clock=NS(sec=i, nanosec=0)), i*10**9) for i in (0, 1, 2)]
    events = [{"topic": "/clock", "source_ns": int(t*.25*1e9),
               "received_wall_ns": int(t*1e9), "seeded": False} for t in (0, 1, 2, 2.5)]
    result = diagnostics(records, {"warmup_end_sim_ns": 0, "record_start_sim_ns": 0,
                                   "recorded_wall_s": 8.}, events)
    assert result["mean_rtf"] == .25
    assert result["receive_wall_diagnostics"]["complete_window_count"] == 2
    assert result["receive_wall_diagnostics"]["minimum_observed_rtf_in_nonoverlapping_1s_wall_windows"] == .25
