"""CLI unit regressions; fake bag I/O is not pinned-decoder acceptance evidence."""

import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SPEC = importlib.util.spec_from_file_location(
    "g1_oracle_cli", Path(__file__).parents[1]/"scripts/g1_oracle.py")
assert SPEC is not None and SPEC.loader is not None
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)
TRUTH_TOPIC = "/drone_cv_g1/evaluator/truth_pose_v"
ENTITIES = {"model_name": "x500_depth", "model_id": 7, "body_name": "base_link",
            "body_id": 8, "link_pose_convention": "model_relative"}


def profile():
    return {"schema": 1, "frame": "base_link FLU", "source_sha256": {"model.sdf": "a"*64},
            "parts": [{"id": "body", "role": "body", "lo": [-1]*3, "hi": [1]*3,
                       "source": "model.sdf"}]}


def fixture():
    return {"truth_entities": ENTITIES, "phase": "rest", "max_truth_gap_ns": 100,
            "clearance_by_phase": {"rest": 0}, "support_allowlist": [],
            "surfaces": [{"id": "wall", "lo": [3, -5, -5], "hi": [4, 5, 5]}]}


def pose(x=0):
    return [[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def save(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def report(path):
    return json.loads((path/"report.json").read_text(encoding="utf-8"))


def offline(tmp_path, observations, scene=None):
    return ["evaluate", "--profile", str(save(tmp_path/"profile.json", profile())),
            "--fixture", str(save(tmp_path/"fixture.json", scene or fixture())),
            "--observations", str(save(tmp_path/"observations.json", observations)),
            "--output", str(tmp_path/"result")]


def test_offline_free_scene_is_explicitly_nonaccepting(tmp_path):
    args = offline(tmp_path, [{"world_from_body": pose(), "sim_ns": 10, "epoch": 0,
                               "wall_ns": 20}])
    assert CLI.main(args) == 0
    result = report(tmp_path/"result")
    assert result["safety_outcome"] == "NO_VIOLATION_OBSERVED"
    assert result["scope"] == "OFFLINE_TYPED_FIXTURE"
    assert result["live_acceptance_certified"] is False
    assert result["ledger"][0]["evaluation"]["pairs"][0]["part"] == "body"


def test_unsafe_then_invalid_observation_preserves_both_axes_and_exit_one(tmp_path):
    args = offline(tmp_path, [
        {"world_from_body": pose(3), "sim_ns": 10, "epoch": 0, "wall_ns": 20},
        {"world_from_body": [[0]], "sim_ns": 20, "epoch": 0, "wall_ns": 30},
    ])
    assert CLI.main(args) == 1
    result = report(tmp_path/"result")
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert result["coverage_complete"] is False
    assert result["sample_count"] == 1


@pytest.mark.parametrize("contact,expected", [("support", 0), ("body", 1), ("rotor", 1)])
def test_offline_exact_support_does_not_exempt_body_or_rotor(tmp_path, contact, expected):
    scene = fixture()
    scene["surfaces"] = [{"id": "floor", "lo": [-5, -5, -1], "hi": [5, 5, 0]}]
    scene["support_allowlist"] = [{"part": "gear", "surface": "floor", "phases": ["rest"],
                                  "zone_lo": [-.5, -.5, -.2], "zone_hi": [.5, .5, 0]}]
    args = offline(tmp_path, [{"world_from_body": pose(), "sim_ns": 10, "epoch": 0,
                               "wall_ns": 20}], scene)
    vehicle = profile()
    vehicle["parts"] = [
        {"id": "gear", "role": "gear", "lo": [-.2, -.2, -.1], "hi": [.2, .2, .1],
         "source": "model.sdf"},
        {"id": "body", "role": "body", "lo": [-.5, -.5, .2], "hi": [.5, .5, .8],
         "source": "model.sdf"},
        {"id": "rotor", "role": "rotor", "lo": [-1, -1, .9], "hi": [1, 1, 1],
         "source": "model.sdf"},
    ]
    if contact != "support":
        next(part for part in vehicle["parts"] if part["id"] == contact)["lo"][2] = -.1
    save(tmp_path/"profile.json", vehicle)
    assert CLI.main(args) == expected
    pairs = report(tmp_path/"result")["ledger"][0]["evaluation"]["pairs"]
    assert next(pair for pair in pairs if pair["part"] == "gear")["allowed_support"] is True
    if contact != "support":
        assert next(pair for pair in pairs if pair["part"] == contact)["unsafe"] is True


@pytest.mark.parametrize("observations", [[], [{"sim_ns": 10}],
    [{"world_from_body": pose(), "sim_ns": 10, "epoch": 0, "wall_ns": 20},
     {"world_from_body": pose(), "sim_ns": 111, "epoch": 0, "wall_ns": 21}],
    [{"world_from_body": pose(), "sim_ns": 10, "epoch": 0, "wall_ns": 20},
     {"world_from_body": pose(), "sim_ns": 11, "epoch": 1, "wall_ns": 21}],
])
def test_offline_missing_invalid_gap_or_reset_never_passes(tmp_path, observations):
    assert CLI.main(offline(tmp_path, observations)) == 2
    assert report(tmp_path/"result")["safety_outcome"] == "UNKNOWN"


def test_invalid_input_still_writes_infra_report(tmp_path):
    args = offline(tmp_path, [])
    (tmp_path/"fixture.json").write_text("not JSON", encoding="utf-8")
    assert CLI.main(args) == 2
    assert report(tmp_path/"result")["execution_outcome"] == "INFRA_ERROR"


def test_profile_produces_builder_result_and_refuses_overwrite(tmp_path, monkeypatch):
    import drone_cv.oracle_profile as builder
    expected = profile()
    monkeypatch.setattr(builder, "build_profile", lambda models: expected)
    target = tmp_path/"profile.json"
    args = ["profile", "--models", str(tmp_path/"models"), "--output", str(target)]
    assert CLI.main(args) == 0
    assert json.loads(target.read_text(encoding="utf-8")) == expected
    original = target.read_bytes()
    assert CLI.main(args) == 2
    assert target.read_bytes() == original


def install_recording(tmp_path, monkeypatch, *, stamps=(10, 90), xs=(0, 0), complete=True,
                      load_error=None, source_mismatch=False):
    recording = tmp_path/"recording"
    recording.mkdir()
    profile_path = save(tmp_path/"profile.json", profile())
    digest = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    (recording/"oracle_profile.json").write_bytes(profile_path.read_bytes())
    acquisition = {"truth_entities": ENTITIES, "oracle": {"profile_sha256": digest}}
    save(recording/"fixture.json", acquisition)
    save(recording/"recording_policy.json", {"schema": 4})
    records, events = [], []
    for sequence, (stamp, x) in enumerate(zip(stamps, xs), 1):
        message = SimpleNamespace(source_ns=stamp, world_from_body=pose(x),
                                  model_id=7, body_id=8)
        records.append((TRUTH_TOPIC, message, stamp))
        events.append({"topic": TRUTH_TOPIC, "topic_sequence": sequence, "source_ns": stamp,
                       "received_sim_ns": stamp, "received_wall_ns": 1000+sequence,
                       "seeded": False})
    if source_mismatch:
        events[-1]["source_ns"] += 1
    (recording/"receive_progress.jsonl").write_text(
        "".join(json.dumps(event)+"\n" for event in events), encoding="utf-8")
    (recording/"bag").mkdir()
    (recording/"bag"/"data.db3").write_bytes(b"unit-test-bag")
    manifest = {"recording_schema": 4, "complete": complete, "all_observed_disarmed": True,
                "oracle_profile_sha256": digest, "record_start_sim_ns": 0,
                "record_end_sim_ns": 100, "alignment": {"clock_epoch": 0, "reset_counter": 0},
                "topics": {TRUTH_TOPIC: "std_msgs/msg/UInt8MultiArray"},
                "topic_counts": {TRUTH_TOPIC: len(records)}, "bag_files": {
                    "bag/data.db3": {"sha256": hashlib.sha256(b"unit-test-bag").hexdigest()}},
                "fixture_sha256": hashlib.sha256((recording/"fixture.json").read_bytes()).hexdigest(),
                "policy_sha256": hashlib.sha256((recording/"recording_policy.json").read_bytes()).hexdigest(),
                "receive_progress_sha256": hashlib.sha256((recording/"receive_progress.jsonl").read_bytes()).hexdigest()}
    save(recording/"manifest.json", manifest)
    def load(path):
        if load_error:
            raise ValueError(load_error)
        return records, json.loads((path/"manifest.json").read_bytes()), acquisition, events
    api = SimpleNamespace(load_recording=load, read_bag=lambda path, **kwargs: records,
                          validate_receive_log=lambda *args, **kwargs: None)
    monkeypatch.setattr(CLI, "_replay_api", lambda: api)
    truth = SimpleNamespace(TRUTH_TOPIC=TRUTH_TOPIC, decode_wire=lambda msg: msg,
                            body_truth=lambda msg, entities: msg)
    monkeypatch.setattr(CLI, "_truth_api", lambda: truth)
    oracle_path = save(tmp_path/"oracle_fixture.json", fixture())
    args = ["replay", "--recording", str(recording), "--profile", str(profile_path),
            "--fixture", str(oracle_path), "--output", str(tmp_path/"result")]
    return args, recording


def test_replay_uses_receive_order_and_actual_truth_transform(tmp_path, monkeypatch):
    args, _ = install_recording(tmp_path, monkeypatch, xs=(0, 3))
    assert CLI.main(args) == 1
    result = report(tmp_path/"result")
    assert result["sample_count"] == 2
    assert [row["wall_ns"] for row in result["ledger"]] == [1001, 1002]
    assert result["ledger"][1]["evaluation"]["unsafe"] is True
    assert result["coverage_complete"] is True
    assert result["scope"] == "RECORDED_CONTINUOUS_TRUTH"


def test_schema_three_is_rejected_without_decoding_or_snapshot_upgrade(tmp_path, monkeypatch):
    args, recording = install_recording(tmp_path, monkeypatch)
    manifest = json.loads((recording/"manifest.json").read_text(encoding="utf-8"))
    manifest["recording_schema"] = 3
    manifest["alignment"]["world_body"] = pose()
    save(recording/"manifest.json", manifest)
    monkeypatch.setattr(CLI, "_replay_api", lambda: pytest.fail("must reject before decoder"))
    assert CLI.main(args) == 2
    assert "NOT_CONTINUOUS_TRUTH" in report(tmp_path/"result")["reasons"]


def test_profile_binding_mismatch_is_infra_not_free(tmp_path, monkeypatch):
    args, _ = install_recording(tmp_path, monkeypatch)
    (tmp_path/"profile.json").write_text(json.dumps(profile())+"\n", encoding="utf-8")
    assert CLI.main(args) == 2
    assert "ORACLE_PROFILE_HASH_MISMATCH" in report(tmp_path/"result")["reasons"]


def test_incomplete_collection_retains_decoded_unsafe(tmp_path, monkeypatch):
    args, _ = install_recording(tmp_path, monkeypatch, xs=(3, 0), complete=False,
                                load_error="RECORDING_INCOMPLETE_OR_ARMED")
    assert CLI.main(args) == 1
    result = report(tmp_path/"result")
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert "RECORDING_INCOMPLETE_OR_ARMED" in result["reasons"]


@pytest.mark.parametrize("stamps,expected", [((101, 102), "TRUTH_START_UNCOVERED"),
                                             ((0, 10), "TRUTH_END_UNCOVERED")])
def test_missing_recording_endpoint_coverage_fails(tmp_path, monkeypatch, stamps, expected):
    args, recording = install_recording(tmp_path, monkeypatch, stamps=stamps)
    manifest = json.loads((recording/"manifest.json").read_text(encoding="utf-8"))
    manifest["record_end_sim_ns"] = 200
    save(recording/"manifest.json", manifest)
    assert CLI.main(args) == 2
    assert expected in report(tmp_path/"result")["reasons"]


def test_receive_source_mismatch_after_unsafe_preserves_unsafe(tmp_path, monkeypatch):
    args, _ = install_recording(tmp_path, monkeypatch, xs=(3, 0), source_mismatch=True)
    assert CLI.main(args) == 1
    result = report(tmp_path/"result")
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert "TRUTH_RECEIVE_SOURCE_MISMATCH" in result["reasons"]


def test_hash_corruption_never_uses_untrusted_payload(tmp_path, monkeypatch):
    args, recording = install_recording(tmp_path, monkeypatch,
                                       load_error="BAG_HASH_MISMATCH", xs=(3, 3))
    (recording/"bag"/"data.db3").write_bytes(b"corrupt")
    assert CLI.main(args) == 2
    result = report(tmp_path/"result")
    assert result["sample_count"] == 0
    assert "BAG_HASH_MISMATCH" in result["reasons"]


def test_retained_raw_unsafe_survives_bag_storage_failure(tmp_path, monkeypatch):
    args, recording = install_recording(tmp_path, monkeypatch, load_error="STORAGE_FAILURE")
    manifest = json.loads((recording/"manifest.json").read_text(encoding="utf-8"))
    packet = {"received_wall_ns": 1005, "source_ns": 50, "payload_sha256": "b"*64}
    manifest["oracle"] = {"first_unsafe": packet}
    save(recording/"manifest.json", manifest)
    (recording/"bag"/"data.db3").write_bytes(b"truncated storage")
    monkeypatch.setattr(CLI, "_packet_sample", lambda packet, entities: SimpleNamespace(
        source_ns=50, world_from_body=pose(3), model_id=7, body_id=8))
    assert CLI.main(args) == 1
    result = report(tmp_path/"result")
    assert result["observed_unsafe"] is True
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert "BAG_HASH_MISMATCH" in result["reasons"]
    assert result["truth_samples"][0]["source"] == "manifest.oracle.first_unsafe"


def test_preflight_truth_before_start_does_not_hide_valid_start_coverage(tmp_path, monkeypatch):
    args, recording = install_recording(tmp_path, monkeypatch, stamps=(0, 90, 180, 270),
                                       xs=(0, 0, 0, 0))
    manifest = json.loads((recording/"manifest.json").read_text(encoding="utf-8"))
    manifest.update(record_start_sim_ns=180, record_end_sim_ns=280)
    save(recording/"manifest.json", manifest)
    assert CLI.main(args) == 0
    assert report(tmp_path/"result")["coverage_complete"] is True


def test_replay_reset_never_bridges_truth_coverage(tmp_path, monkeypatch):
    args, recording = install_recording(tmp_path, monkeypatch)
    api = CLI._replay_api()
    records, manifest, _, events = api.load_recording(recording)
    topic = "/drone_cv_g1/fmu/out/vehicle_odometry"
    records.insert(1, (topic, SimpleNamespace(reset_counter=1), 50))
    events.insert(1, {"topic": topic, "topic_sequence": 1, "source_ns": 50,
                      "received_sim_ns": 50, "received_wall_ns": 1002, "seeded": False})
    events[-1]["received_wall_ns"] = 1003
    manifest["topics"][topic] = "px4_msgs/msg/VehicleOdometry"
    manifest["topic_counts"][topic] = 1
    (recording/"receive_progress.jsonl").write_text(
        "".join(json.dumps(event)+"\n" for event in events), encoding="utf-8")
    manifest["receive_progress_sha256"] = hashlib.sha256(
        (recording/"receive_progress.jsonl").read_bytes()).hexdigest()
    save(recording/"manifest.json", manifest)
    assert CLI.main(args) == 2
    result = report(tmp_path/"result")
    assert "ODOMETRY_RESET_CHANGED" in result["reasons"]
    assert "EPOCH_CHANGED" in result["reasons"]
    assert result["coverage_intervals"] == []
    assert [row["epoch"] for row in result["ledger"]] == [0, 1]


@pytest.mark.parametrize("duplicate", [False, True])
def test_entity_discovery_waits_for_spawn_but_rejects_ambiguous_body(monkeypatch, duplicate):
    spec = importlib.util.spec_from_file_location(
        "g1_oracle_run", Path(__file__).parents[1]/"scripts/g1_oracle_run.py")
    assert spec is not None and spec.loader is not None
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    model = SimpleNamespace(name="x500_depth_0", id=7)
    body = SimpleNamespace(name="base_link", id=8)
    final = [model, body, body] if duplicate else [model, body]
    messages = iter([SimpleNamespace(pose=[]), SimpleNamespace(pose=[model]),
                     SimpleNamespace(pose=final)])
    callback = []
    node = SimpleNamespace(
        create_subscription=lambda kind, topic, receive, depth: callback.append(receive),
        destroy_node=lambda: None)
    ros = SimpleNamespace(init=lambda: None, shutdown=lambda: None,
                          create_node=lambda name: node,
                          spin_once=lambda *args, **kwargs: callback[0](next(messages)))
    monkeypatch.setattr(runner, "import_module", lambda name: ros if name == "rclpy"
                        else SimpleNamespace(UInt8MultiArray=object))
    monkeypatch.setattr(runner, "decode_wire", lambda message: message)
    if duplicate:
        with pytest.raises(ValueError, match="AMBIGUOUS_TRUTH_DISCOVERY"):
            runner.discover_entities(1.)
    else:
        result = runner.discover_entities(1.)
        assert (result["model_id"], result["body_id"]) == (7, 8)
