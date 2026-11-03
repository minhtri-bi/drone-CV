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
        require_fresh(refreshed, limits, 3_000_000_000)


def test_repeated_or_backward_source_stamps_cannot_refresh_stream_health():
    first = advance_progress({}, "status", 100, 0)
    repeated = advance_progress(first, "status", 100, 5_000_000_000)
    with pytest.raises(ValueError, match="STALLED:status"):
        require_fresh(repeated, {"status": 2.}, 5_000_000_000)
    with pytest.raises(ValueError, match="REGRESSED:status"):
        advance_progress(first, "status", 99, 1)
    require_fresh(advance_progress(first, "status", 101, 1_000_000_000),
                  {"status": 2.}, 2_000_000_000)
