from copy import deepcopy

import numpy as np
import pytest

from capture_pose import OdomSample
from recording_integrity import (advance_progress, make_alignment, require_alignment_epoch,
                                 require_fresh, validate_alignment)


def bracket(reset=7):
    q = (np.sqrt(.5), 0, 0, np.sqrt(.5))
    return [OdomSample(100_000_000, (0, 0, 0), q, reset),
            OdomSample(140_000_000, (0, 4, 0), q, reset)]


def test_alignment_interpolates_at_snapshot_not_at_cached_sample():
    truth = np.eye(4)
    truth[:3, 3] = [10, 20, 1]
    transform, contract = make_alignment(truth, 120_000_000, bracket(), 50_000_000)
    np.testing.assert_allclose(transform[:3, 3], [8, 20, 1], atol=1e-12)
    assert contract["reset_counter"] == 7
    manifest = {"recording_schema": 2, "alignment": contract, "world_from_odom": transform}
    validate_alignment(manifest, 50_000_000)
    with pytest.raises(ValueError, match="UNBRACKETED"):
        make_alignment(truth, 200_000_000, bracket(), 50_000_000)
    with pytest.raises(ValueError, match="POSE_GAP"):
        make_alignment(truth, 120_000_000, bracket(), 30_000_000)


def test_reset_during_alignment_and_after_alignment_invalidates_world_grading():
    samples = bracket()
    changed = [samples[0], OdomSample(140_000_000, (0, 4, 0), samples[1].quaternion_wxyz, 8)]
    with pytest.raises(ValueError, match="POSE_RESET"):
        make_alignment(np.eye(4), 120_000_000, changed, 50_000_000)
    _, contract = make_alignment(np.eye(4), 120_000_000, samples, 50_000_000)
    with pytest.raises(ValueError, match="ALIGNMENT_RESET_CHANGED"):
        require_alignment_epoch(contract, 0, [changed[1]])
    with pytest.raises(ValueError, match="ALIGNMENT_CLOCK_EPOCH_CHANGED"):
        require_alignment_epoch(contract, 1, samples)


def test_legacy_alignment_and_tampered_snapshot_provenance_are_not_admitted():
    with pytest.raises(ValueError, match="LEGACY_UNBOUND_ALIGNMENT"):
        validate_alignment({"world_from_odom": np.eye(4)}, 50_000_000)
    transform, contract = make_alignment(np.eye(4), 120_000_000, bracket(), 50_000_000)
    changed = deepcopy(contract)
    changed["reset_counter"] = 9
    with pytest.raises(ValueError, match="PROVENANCE_MISMATCH"):
        validate_alignment({"recording_schema": 2, "alignment": changed,
                            "world_from_odom": transform}, 50_000_000)


@pytest.mark.parametrize("lost", ["image", "info", "odom", "status", "clock"])
def test_a_live_clock_does_not_hide_another_required_stream_loss(lost):
    limits = {name: 2. for name in ("image", "info", "odom", "status", "clock")}
    progress = {name: {"source_ns": 100, "wall_ns": 0} for name in limits}
    refreshed = {name: value if name == lost else {"source_ns": 200, "wall_ns": 3_000_000_000}
                 for name, value in progress.items()}
    with pytest.raises(ValueError, match=f"STREAM_PROGRESS_STALLED:{lost}"):
        require_fresh(refreshed, {"schema": 2, "stream_progress_wall_s": limits}, 3_000_000_000)


def test_repeated_or_backward_source_stamps_cannot_refresh_stream_health():
    first = advance_progress({}, "status", 100, 0)
    repeated = advance_progress(first, "status", 100, 5_000_000_000)
    with pytest.raises(ValueError, match="STALLED:status"):
        require_fresh(repeated, {"schema": 2, "stream_progress_wall_s": {"status": 2.}}, 5_000_000_000)
    with pytest.raises(ValueError, match="REGRESSED:status"):
        advance_progress(first, "status", 99, 1)
    require_fresh(advance_progress(first, "status", 101, 1_000_000_000),
                  {"schema": 2, "stream_progress_wall_s": {"status": 2.}}, 2_000_000_000)


def mixed_policy():
    return {"schema": 3, "clock_progress_wall_s": 1.5,
            "max_source_clock_lead_ns": 100_000_000,
            "stream_age_sim_s": {"status": 1.1, "image": .5}}


def test_slow_sim_status_is_fresh_even_after_three_wall_seconds():
    progress = {"/clock": {"source_ns": 10_500_000_000, "wall_ns": 5_000_000_000},
                "status": {"source_ns": 10_000_000_000, "wall_ns": 0},
                "image": {"source_ns": 10_460_000_000, "wall_ns": 4_600_000_000}}
    require_fresh(progress, mixed_policy(), 5_000_000_000, sim_now_ns=10_500_000_000)
    with pytest.raises(ValueError, match="STALLED:/clock"):
        require_fresh(progress, mixed_policy(), 7_000_000_000, sim_now_ns=10_500_000_000)
    stale_status = {**progress, "status": {"source_ns": 9_000_000_000, "wall_ns": 5_000_000_000}}
    with pytest.raises(ValueError, match="SIM_AGE_EXCEEDED:status"):
        require_fresh(stale_status, mixed_policy(), 5_000_000_000, sim_now_ns=10_500_000_000)


def test_bounded_delivered_clock_lag_is_not_ignored_or_subtracted_as_wall_age():
    progress = {"/clock": {"source_ns": 10_000_000_000, "wall_ns": 0},
                "status": {"source_ns": 10_044_000_000, "wall_ns": 0},
                "image": {"source_ns": 10_000_000_000, "wall_ns": 0}}
    require_fresh(progress, mixed_policy(), 0, sim_now_ns=10_000_000_000)
    with pytest.raises(ValueError, match="SOURCE_AHEAD_OF_CLOCK:status"):
        require_fresh({**progress, "status": {"source_ns": 10_101_000_000, "wall_ns": 0}},
                      mixed_policy(), 0, sim_now_ns=10_000_000_000)


def test_queued_clock_callback_does_not_override_the_nodes_newer_ros_clock():
    progress = {"/clock": {"source_ns": 106_848_000_000, "wall_ns": 0},
                "status": {"source_ns": 106_900_000_000, "wall_ns": 0},
                "image": {"source_ns": 106_956_000_000, "wall_ns": 0}}
    require_fresh(progress, mixed_policy(), 1, sim_now_ns=106_980_000_000)
    with pytest.raises(ValueError, match="MISSING_NODE_CLOCK"):
        require_fresh(progress, mixed_policy(), 1)


def test_full_storage_queue_rejects_evidence_instead_of_silently_dropping_it():
    from queue import Queue
    from types import SimpleNamespace
    from bag_writer import BagWriter
    writer = object.__new__(BagWriter)
    writer.process = SimpleNamespace(is_alive=lambda: True)
    writer.incoming = Queue(maxsize=1)
    from collections import Counter
    import hashlib
    writer.enqueued = Counter()
    writer.identity_hashes = {"depth": hashlib.sha256()}
    writer.write("depth", b"first", {"topic_sequence": 1, "source_ns": 1})
    with pytest.raises(ValueError, match="BAG_WRITER_BACKPRESSURE"):
        writer.write("depth", b"second", {"topic_sequence": 2, "source_ns": 2})
    assert writer.enqueued == {"depth": 1}
    writer.process = SimpleNamespace(is_alive=lambda: False)
    with pytest.raises(ValueError, match="BAG_WRITER_EXITED"):
        writer.write("depth", b"third", {"topic_sequence": 3, "source_ns": 3})
