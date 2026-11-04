import pytest

from drone_cv.depth_path_audit import STAGES, audit, historical_recording


def fixture():
    contract = {"schema": 1, "epoch": "test", "stream": "depth", "source_start_ns": 10,
                "source_end_ns": 50, "planned_source_ns": [10,20,30,40],
                "schedule_provenance": "synthetic requested-frame schedule",
                "coverage": {stage: {"authoritative": True, "complete": True,
                    "drained": True, "probe_loss_count": 0, "source_start_ns": 10,
                    "source_end_ns": 50, "provenance": "synthetic exact stage tap"} for stage in STAGES}}
    events = [{"stage": stage, "epoch": "test", "stream": "depth", "source_ns": stamp,
               "source_sequence": stamp, "payload_identity": str(stamp), "wall_ns": stamp+index}
              for index,stage in enumerate(STAGES) for stamp in (10,20,30,40)]
    return contract, events


def test_source_skip_is_not_transport_loss_and_camera_info_does_not_substitute():
    contract, events = fixture()
    report = audit(contract, [row for row in events if row["source_ns"] != 20])
    assert report["unpublished_requested_source_ns"] == [20]
    assert all(row["classification"] == "MATCHED" for row in report["comparisons"])
    assert report["end_to_end_latency_bound"] is None


@pytest.mark.parametrize("first_missing", [1,2,3,4,5])
def test_transport_enqueue_and_storage_deficits_are_distinct_boundaries(first_missing):
    contract, events = fixture()
    result = audit(contract, [row for row in events if not
                   (row["source_ns"] == 20 and STAGES.index(row["stage"]) >= first_missing)])
    broken = [row for row in result["comparisons"] if row["measured_deficit_count"]]
    assert [(row["before"],row["after"],row["measured_deficit_count"]) for row in broken] == [
        (STAGES[first_missing-1],STAGES[first_missing],1)]
    assert not result["no_unexplained_deficit"]


@pytest.mark.parametrize("change", [{"probe_loss_count":1}, {"drained":False},
                                   {"source_end_ns":49}, {"authoritative":False}])
def test_probe_or_window_failure_prevents_exact_localization(change):
    contract, events = fixture()
    contract["coverage"]["bridge_publish"].update(change)
    report = audit(contract, [row for row in events if row["stage"] != "bridge_publish"])
    comparison = report["comparisons"][2]
    assert comparison["classification"] == "UNKNOWN_COVERAGE"
    assert comparison["measured_deficit_count"] is None
    assert not report["all_stage_coverage_complete"]


def test_duplicate_reordering_and_outside_window_do_not_disappear():
    contract, events = fixture()
    callback = next(row for row in events if row["stage"] == "recorder_callback")
    report = audit(contract, [*events, callback, {**callback,"source_ns":99}])
    stage = report["stages"]["recorder_callback"]
    assert stage["duplicate_count"] == 1
    assert stage["source_reorder_count"] == 1
    assert stage["outside_window_count"] == 1
    assert report["comparisons"][2]["measured_excess_count"] == 1


def test_mixed_epoch_and_large_trace_are_rejected():
    contract, events = fixture()
    with pytest.raises(ValueError, match="mixed"):
        audit(contract, [{**events[0],"epoch":"other"}])
    with pytest.raises(ValueError, match="nonnegative"):
        audit(contract, [{**events[0],"source_ns":-1}])


def test_historical_receive_rows_stay_partial_even_when_manifest_says_complete():
    report = historical_recording([{"topic":"/camera/depth/image_raw", "source_ns":10,
        "received_wall_ns":20}], {"complete":True,"storage_writer":{"complete":True}})
    assert not report["all_stage_coverage_complete"]
    assert not report["callback_enqueue_storage_identity_proof"]
    assert not report["loss_location_certified"]
