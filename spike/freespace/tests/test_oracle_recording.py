from copy import deepcopy
import base64
import hashlib
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from capture_pose import OdomSample
from recording_integrity import (ORACLE_SCHEMA, REQUIRED_ORACLE_STREAMS, TRUTH_TOPIC,
                                 make_alignment, require_fresh, source_stamp_ns,
                                 validate_alignment, validate_policy)
from oracle_recording import OracleRecording, validate_binding


def profile():
    return {"schema": 1, "frame": "base_link FLU", "source_sha256": {"model.sdf": "a"*64},
            "parts": [{"id": "body", "role": "body", "lo": [-.1]*3,
                       "hi": [.1]*3, "source": "model.sdf"}]}


def fixture():
    payload = json.dumps(profile()).encode()
    return {"model_name": "vehicle", "truth_entities": {
        "model_name": "vehicle", "model_id": 1, "body_name": "base_link", "body_id": 2,
        "link_pose_convention": "model_relative"}, "oracle": {
        "profile_sha256": hashlib.sha256(payload).hexdigest(), "phase": "rest",
        "fixture": {"surfaces": [{"id": "wall", "lo": [-1]*3, "hi": [1]*3}],
                    "support_allowlist": [], "clearance_by_phase": {"rest": 0.},
                    "max_truth_gap_ns": 50_000_000}}}


def policy():
    return {"schema": ORACLE_SCHEMA, "startup_wall_s": 20., "alignment_wait_wall_s": 5.,
            "max_pose_gap_ns": 50_000_000, "alignment_buffer_samples": 512,
            "writer_queue_items": 128, "writer_shutdown_wall_s": 10.,
            "clock_progress_wall_s": 1.5, "max_source_clock_lead_ns": 100_000_000,
            "stream_age_sim_s": dict.fromkeys(REQUIRED_ORACLE_STREAMS, .5)}


def wire(payload=b"raw"):
    return NS(data=payload, layout=NS(data_offset=0, dim=[NS(
        label="gz.msgs.Pose_V;oracle_schema=1", size=len(payload), stride=len(payload))]))


def decoded(stamp=120_000_000, model_id=1, duplicate=False):
    def pose(name, identity):
        return NS(name=name, id=identity, position=NS(x=0., y=0., z=0.),
                  orientation=NS(w=1., x=0., y=0., z=0.))
    poses = [pose("vehicle", model_id), pose("base_link", 2)]
    if duplicate:
        poses.append(pose("base_link", 3))
    return NS(header=NS(stamp=NS(sec=0, nsec=stamp)), pose=poses)


def test_policy4_requires_truth_but_schema3_defaults_remain_accepted():
    validate_policy(policy())
    missing = policy()
    del missing["stream_age_sim_s"][TRUTH_TOPIC]
    with pytest.raises(ValueError, match="POLICY_REQUIRED_STREAMS"):
        validate_policy(missing)
    legacy = json.loads((__import__("pathlib").Path(__file__).parents[1] / "recording_policy.json").read_text())
    validate_policy(legacy)
    assert legacy["schema"] == 3


def test_profile_fixture_hash_binding_rejects_mutation():
    raw = json.dumps(profile()).encode()
    validate_binding(raw, fixture())
    with pytest.raises(ValueError, match="PROFILE_BINDING"):
        validate_binding(raw+b" ", fixture())
    changed = fixture()
    changed["truth_entities"]["body_id"] = 1
    with pytest.raises(ValueError, match="ENTITY_CONTRACT"):
        validate_binding(raw, changed)


def test_truth_uses_lazy_wire_stamp_and_sim_age_not_wall_rtf(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    assert source_stamp_ns(TRUTH_TOPIC, wire()) == 120_000_000
    progress = {name: {"source_ns": 120_000_000, "wall_ns": 0}
                for name in REQUIRED_ORACLE_STREAMS}
    progress["/clock"] = {"source_ns": 130_000_000, "wall_ns": 5_000_000_000}
    require_fresh(progress, policy(), 5_000_000_000, sim_now_ns=130_000_000)
    with pytest.raises(ValueError, match="SIM_AGE_EXCEEDED"):
        require_fresh(progress, policy(), 5_000_000_000, sim_now_ns=700_000_000)
    del progress[TRUTH_TOPIC]
    with pytest.raises(ValueError, match="REQUIRED_STREAM_MISSING"):
        require_fresh(progress, policy(), 5_000_000_000, sim_now_ns=130_000_000)


@pytest.mark.parametrize("bad", [decoded(model_id=8), decoded(duplicate=True), NS(header=NS(stamp=NS(sec=-1, nsec=0)), pose=[])])
def test_missing_ambiguous_identity_or_invalid_time_is_raw_recorded_first(monkeypatch, bad):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: bad)
    recorder = OracleRecording(profile(), fixture())
    writes = []
    with pytest.raises(ValueError):
        recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: writes.append((stamp, error)))
    assert len(writes) == 1
    assert recorder.finish(False)["safety_outcome"] == "UNKNOWN"
    assert recorder.first_invalid is not None
    assert recorder.first_invalid["payload_base64"] == base64.b64encode(b"raw").decode()


def test_malformed_wire_is_not_lost_before_storage(monkeypatch):
    def reject(message):
        raise ValueError("INVALID_TRUTH_PROTOBUF")
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", reject)
    recorder = OracleRecording(profile(), fixture())
    writes = []
    with pytest.raises(ValueError, match="PROTOBUF"):
        recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: writes.append((stamp, error)))
    assert writes == [(None, "INVALID_TRUTH_PROTOBUF")]
    assert recorder.finish(False)["execution_outcome"] == "INFRA_ERROR"


def test_storage_and_later_callback_failure_cannot_erase_first_unsafe(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    recorder = OracleRecording(profile(), fixture())
    def writer_failure(stamp, error):
        raise RuntimeError("disk failed")
    with pytest.raises(RuntimeError, match="disk failed"):
        recorder.receive(wire(), 1, 120_000_000, writer_failure)
    assert recorder.first_unsafe is not None
    first = deepcopy(recorder.first_unsafe)
    recorder.fail("later callback failed")
    result = recorder.finish(False)
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert recorder.exit_code(False) == 1
    assert recorder.first_unsafe == first
    assert first["evaluation"]["unsafe"] is True


def test_repeated_truth_stamp_is_not_complete_coverage(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    free = fixture()
    free["oracle"]["fixture"]["surfaces"][0].update(lo=[2]*3, hi=[3]*3)
    recorder = OracleRecording(profile(), free)
    recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: None)
    recorder.receive(wire(), 2, 120_000_000, lambda stamp, error: None)
    assert recorder.finish(True)["coverage_complete"] is False
    assert recorder.exit_code(True) == 2


def test_schema4_alignment_binds_raw_truth_and_endpoint_deltas(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    recorder = OracleRecording(profile(), fixture())
    sample, packet = recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: None)
    q = (np.sqrt(.5), 0, 0, np.sqrt(.5))
    samples = [OdomSample(t, (0, 0, 0), q, 7) for t in (100_000_000, 140_000_000)]
    transform, contract = make_alignment(np.eye(4), sample.source_ns, samples, 50_000_000)
    manifest = {"recording_schema": 4, "world_from_odom": transform.tolist(),
                "alignment": contract, "truth_entities": fixture()["truth_entities"],
                "alignment_source": {"truth_packet": packet}}
    validate_alignment(manifest, 50_000_000)
    assert contract["bracket_endpoint_delta_ns"] == [-20_000_000, 20_000_000]
    assert contract["interpolation_fraction"] == .5
    manifest["alignment_source"]["truth_packet"]["payload_sha256"] = "0"*64
    with pytest.raises(ValueError, match="TRUTH_PACKET"):
        validate_alignment(manifest, 50_000_000)


def test_absent_body_identity_is_rejected_after_raw_write(monkeypatch):
    packet = decoded()
    packet.pose.pop()
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: packet)
    recorder = OracleRecording(profile(), fixture())
    writes = []
    with pytest.raises(ValueError, match="BODY_IDENTITY_CHANGED"):
        recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: writes.append(stamp))
    assert writes == [120_000_000]


def test_first_unsafe_survives_observer_exception(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    def reject(*args, **kwargs):
        raise ValueError("observer failed")
    monkeypatch.setattr("drone_cv.oracle.ObserverState.observe", reject)
    recorder = OracleRecording(profile(), fixture())
    with pytest.raises(ValueError, match="observer failed"):
        recorder.receive(wire(), 1, 120_000_000, lambda stamp, error: None)
    assert recorder.finish(False)["safety_outcome"] == "UNSAFE_OBSERVED"
    assert recorder.exit_code(False) == 1


def test_end_coverage_gap_is_simulation_only_and_never_full_success(monkeypatch):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    free = fixture()
    free["oracle"]["fixture"]["surfaces"][0].update(lo=[2]*3, hi=[3]*3)
    recorder = OracleRecording(profile(), free)
    recorder.receive(wire(), 50_000_000_000, 120_000_000, lambda stamp, error: None)
    recorder.close_coverage(180_000_000)
    assert recorder.exit_code(True) == 2
    assert "TRUTH_FINAL_ENDPOINT_UNCOVERED" in recorder.finish(True)["execution_reasons"]


def install_fake_ros(monkeypatch, output, *, write_failure=False, close_failure=False,
                     start_failure=False, complete=False):
    import sys
    from types import ModuleType
    import record_sensor_bag as module
    callbacks, writes = {}, []
    wall = [1_000_000_000]
    sim = [120_000_000]
    def make_module(name, **values):
        result = ModuleType(name)
        result.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, result)
        return result
    class Qos:
        def __init__(self, depth, reliability, durability):
            self.depth, self.reliability, self.durability = depth, reliability, durability
            self.history = NS(name="KEEP_LAST")
    class Node:
        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=sim[0]))
        def create_subscription(self, kind, topic, callback, qos):
            callbacks[topic] = callback
            return callback
        def destroy_node(self):
            pass
        def get_publishers_info_by_topic(self, topic):
            return []
    spins = [0]
    def spin_once(node, timeout_sec):
        wall[0] += 1_000_000
        spins[0] += 1
        if complete:
            if spins[0] > 1:
                sim[0] += 10_000_000
            for topic, callback in callbacks.items():
                stamp = NS(sec=0, nanosec=sim[0])
                if topic == TRUTH_TOPIC:
                    callback(wire(str(sim[0]).encode()))
                elif topic == "/clock":
                    callback(NS(clock=stamp))
                elif topic == "/tf_static":
                    if spins[0] == 1:
                        callback(NS(transforms=[]))
                elif topic.endswith("vehicle_odometry"):
                    stamps = (100_000_000, 140_000_000) if spins[0] == 1 else (sim[0]+20_000_000,)
                    for source in stamps:
                        callback(NS(timestamp_sample=source//1000, position=[0., 0., 0.],
                                    q=[1., 0., 0., 0.], reset_counter=7, pose_frame=1))
                elif topic.endswith("vehicle_status_v4"):
                    callback(NS(timestamp=sim[0]//1000, arming_state=1, ARMING_STATE_DISARMED=1))
                else:
                    callback(NS(header=NS(stamp=stamp)))
        elif spins[0] == 1:
            callbacks[TRUTH_TOPIC](wire())
        else:
            raise RuntimeError("executor failed after preflight truth")
    make_module("rclpy", init=lambda: None, shutdown=lambda: None,
                create_node=lambda *args, **kwargs: Node(), spin_once=spin_once)
    make_module("rclpy.parameter", Parameter=lambda *args, **kwargs: None)
    make_module("rclpy.qos", QoSProfile=Qos,
                ReliabilityPolicy=NS(BEST_EFFORT=NS(name="BEST_EFFORT"), RELIABLE=NS(name="RELIABLE")),
                DurabilityPolicy=NS(VOLATILE=NS(name="VOLATILE"), TRANSIENT_LOCAL=NS(name="TRANSIENT_LOCAL")))
    make_module("rclpy.serialization", serialize_message=lambda message: b"serialized")
    make_module("rosidl_runtime_py")
    make_module("rosidl_runtime_py.utilities", get_message=lambda kind: object)
    class Writer:
        def __init__(self, root, topics, policy):
            if start_failure:
                raise ValueError("BAG_WRITER_START_FAILED")
        def write(self, topic, payload, event):
            if write_failure:
                raise RuntimeError("storage failed")
            writes.append((topic, payload, event))
            with (output / "receive_progress.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event)+"\n")
        def close(self):
            if close_failure:
                raise RuntimeError("close failed")
            from collections import Counter
            return {"topic_counts": dict(Counter(topic for topic, _, _ in writes))}
    monkeypatch.setattr(module, "BagWriter", Writer)
    monkeypatch.setattr(module, "time", NS(monotonic_ns=lambda: wall[0], monotonic=lambda: wall[0]/1e9))
    def forbidden_snapshot(*args):
        raise AssertionError("schema4 must not acquire subprocess truth")
    monkeypatch.setattr(module, "truth_snapshot", forbidden_snapshot)
    return module, writes


@pytest.mark.parametrize("failure_kind", ["write", "executor", "close"])
def test_actual_callback_preflight_unsafe_survives_recording_failure(monkeypatch, tmp_path, failure_kind):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    root = tmp_path / "recording"
    module, writes = install_fake_ros(monkeypatch, root, write_failure=failure_kind == "write",
                                      close_failure=failure_kind == "close")
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=1., warmup_wall_s=0., max_output_bytes=1_000_000)
    fixture_path, policy_path, profile_path = (tmp_path / name for name in
                                              ("fixture.json", "policy.json", "profile.json"))
    fixture_path.write_text(json.dumps(recording_fixture))
    policy_path.write_text(json.dumps(policy()))
    profile_path.write_text(json.dumps(profile()))
    assert module.record(root, fixture_path, policy_path, profile_path) == 1
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["complete"] is False
    assert manifest["oracle"]["safety_outcome"] == "UNSAFE_OBSERVED"
    assert manifest["oracle"]["execution_outcome"] == "INFRA_ERROR"
    assert manifest["oracle"]["first_unsafe"]["payload_base64"] == base64.b64encode(b"raw").decode()
    assert (root / "oracle_profile.json").read_bytes() == profile_path.read_bytes()
    if failure_kind != "write":
        assert writes[0][0] == TRUTH_TOPIC
        assert writes[0][2]["phase"] == "preflight"


def test_actual_malformed_truth_callback_is_stored_and_marked_incomplete(monkeypatch, tmp_path):
    def reject(message):
        raise ValueError("INVALID_TRUTH_PROTOBUF")
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", reject)
    root = tmp_path / "recording"
    module, writes = install_fake_ros(monkeypatch, root)
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=1., warmup_wall_s=0., max_output_bytes=1_000_000)
    paths = [tmp_path / name for name in ("fixture.json", "policy.json", "profile.json")]
    for path, value in zip(paths, (recording_fixture, policy(), profile())):
        path.write_text(json.dumps(value))
    assert module.record(root, *paths) == 2
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["complete"] is False
    assert manifest["oracle"]["safety_outcome"] == "UNKNOWN"
    assert writes[0][2]["source_ns"] is None
    assert writes[0][2]["validation_error"] == "INVALID_TRUTH_PROTOBUF"
    assert writes[0][1] == b"serialized"


def test_writer_start_failure_cannot_produce_safe_result(monkeypatch, tmp_path):
    root = tmp_path / "recording"
    module, _ = install_fake_ros(monkeypatch, root, start_failure=True)
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=1., warmup_wall_s=0., max_output_bytes=1_000_000)
    paths = [tmp_path / name for name in ("fixture.json", "policy.json", "profile.json")]
    for path, value in zip(paths, (recording_fixture, policy(), profile())):
        path.write_text(json.dumps(value))
    assert module.record(root, *paths) == 2
    result = json.loads((root / "manifest.json").read_text())["oracle"]
    assert result["safety_outcome"] == "UNKNOWN"
    assert result["coverage_complete"] is False


def schema4_trace(monkeypatch):
    import sys
    from types import ModuleType
    from oracle_recording import packet_evidence
    from record_sensor_bag import TOPICS
    serialize = lambda message: repr(message).encode()
    ros = ModuleType("rclpy.serialization")
    ros.__dict__["serialize_message"] = serialize
    monkeypatch.setitem(sys.modules, "rclpy.serialization", ros)
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded(int(bytes(message.data))))
    topics = {**TOPICS, TRUTH_TOPIC: "std_msgs/msg/UInt8MultiArray"}
    messages = {
        "/clock": NS(clock=NS(sec=0, nanosec=120_000_000)),
        "/tf_static": NS(transforms=[]),
        TRUTH_TOPIC: wire(b"120000000"),
    }
    for topic in topics:
        if topic.endswith("vehicle_status_v4"):
            messages[topic] = NS(timestamp=120_000, arming_state=1, ARMING_STATE_DISARMED=1)
        elif topic.endswith("vehicle_odometry"):
            messages[topic] = NS(timestamp_sample=120_000)
        elif topic not in messages:
            messages[topic] = NS(header=NS(stamp=NS(sec=0, nanosec=120_000_000)))
    records, events = [], []
    for wall, (topic, message) in enumerate(messages.items(), 1):
        records.append((topic, message, 120_000_000))
        event = {"topic": topic, "source_ns": source_stamp_ns(topic, message), "topic_sequence": 1,
                 "received_wall_ns": wall, "received_sim_ns": 120_000_000, "seeded": False,
                 "phase": "preflight", "validation_error": None,
                 "serialized_sha256": hashlib.sha256(serialize(message)).hexdigest()}
        if topic == TRUTH_TOPIC:
            event["truth_payload_sha256"] = hashlib.sha256(bytes(message.data)).hexdigest()
        events.append(event)
    truth_event = next(event for event in events if event["topic"] == TRUTH_TOPIC)
    packet = packet_evidence(messages[TRUTH_TOPIC], truth_event["received_wall_ns"], 120_000_000)
    packet.update(source_ns=120_000_000, entities=fixture()["truth_entities"])
    manifest = {"recording_schema": 4, "topics": topics, "topic_counts": dict.fromkeys(topics, 1),
                "complete": True, "truth_entities": fixture()["truth_entities"],
                "record_start_wall_monotonic_ns": 1000,
                "record_end_wall_monotonic_ns": 2000, "record_end_sim_ns": 120_000_000,
                "alignment_source": {"truth_packet": packet},
                "oracle": {"coverage_complete": True, "max_gap_ns": 50_000_000}}
    return records, manifest, policy(), events


def trace_serialized_hashes(records):
    # Test traces use these exact synthetic CDR bytes; production hashes reader payloads.
    return [(topic, stored_ns, hashlib.sha256(repr(message).encode()).hexdigest())
            for topic, message, stored_ns in records]


def test_validator_checks_actual_decoded_truth_stamps_and_hashes(monkeypatch):
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    validate_receive_log(records, manifest, acquisition_policy, events,
                         serialized_hashes=trace_serialized_hashes(records))
    changed = deepcopy(events)
    next(event for event in changed if event["topic"] == TRUTH_TOPIC)["source_ns"] += 1
    with pytest.raises(ValueError, match="TRUTH_STAMP_NOT_ADVANCING|MESSAGE_MISMATCH"):
        validate_receive_log(records, manifest, acquisition_policy, changed,
                             serialized_hashes=trace_serialized_hashes(records))
    changed = deepcopy(events)
    next(event for event in changed if event["topic"] == TRUTH_TOPIC)["serialized_sha256"] = "0"*64
    with pytest.raises(ValueError, match="SERIALIZED_HASH"):
        validate_receive_log(records, manifest, acquisition_policy, changed,
                             serialized_hashes=trace_serialized_hashes(records))


def test_validator_rejects_missing_truth_and_partial_collection(monkeypatch):
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    with pytest.raises(ValueError, match="REQUIRED_STREAM_MISSING"):
        validate_receive_log([row for row in records if row[0] != TRUTH_TOPIC], manifest,
                             acquisition_policy, [row for row in events if row["topic"] != TRUTH_TOPIC])
    manifest["complete"] = False
    with pytest.raises(ValueError, match="COLLECTION_INCOMPLETE"):
        validate_receive_log(records, manifest, acquisition_policy, events,
                             serialized_hashes=trace_serialized_hashes(records))
    manifest["complete"] = True
    manifest["oracle"]["coverage_complete"] = False
    with pytest.raises(ValueError, match="COVERAGE_INCOMPLETE"):
        validate_receive_log(records, manifest, acquisition_policy, events,
                             serialized_hashes=trace_serialized_hashes(records))


def test_validator_rejects_entity_identity_even_when_trace_counts_match(monkeypatch):
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded(duplicate=True))
    with pytest.raises(ValueError, match="IDENTITY_CHANGED"):
        validate_receive_log(records, manifest, acquisition_policy, events,
                             serialized_hashes=trace_serialized_hashes(records))


def test_startup_health_never_uses_wall_age_for_truth_but_clock_still_has_watchdog():
    progress = {TRUTH_TOPIC: {"source_ns": 100_000_000, "wall_ns": 0}}
    require_fresh(progress, policy(), 0, sim_now_ns=0, allow_startup_missing=True)
    progress["/clock"] = {"source_ns": 110_000_000, "wall_ns": 5_000_000_000}
    require_fresh(progress, policy(), 5_000_000_000, sim_now_ns=110_000_000, allow_startup_missing=True)
    with pytest.raises(ValueError, match="STALLED:/clock"):
        require_fresh(progress, policy(), 7_000_000_000, sim_now_ns=110_000_000, allow_startup_missing=True)
    with pytest.raises(ValueError, match="SIM_AGE_EXCEEDED"):
        require_fresh(progress, policy(), 5_000_000_000, sim_now_ns=700_000_000, allow_startup_missing=True)


@pytest.mark.parametrize("unsafe, expected", [(False, 0), (True, 1)])
def test_complete_recording_does_not_mean_oracle_safe(monkeypatch, tmp_path, unsafe, expected):
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded(int(bytes(message.data))))
    root = tmp_path / "recording"
    module, writes = install_fake_ros(monkeypatch, root, complete=True)
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=.003, warmup_wall_s=0., max_output_bytes=1_000_000)
    if not unsafe:
        recording_fixture["oracle"]["fixture"]["surfaces"][0].update(lo=[2]*3, hi=[3]*3)
    paths = [tmp_path / name for name in ("fixture.json", "policy.json", "profile.json")]
    for path, value in zip(paths, (recording_fixture, policy(), profile())):
        path.write_text(json.dumps(value))
    assert module.record(root, *paths) == expected
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["complete"] is True
    assert manifest["oracle"]["coverage_complete"] is True
    assert manifest["oracle"]["observed_unsafe"] is unsafe
    assert manifest["alignment"]["bracket_endpoint_delta_ns"] == [-20_000_000, 20_000_000]
    assert manifest["alignment_source"]["truth_packet"]["payload_base64"] == base64.b64encode(b"120000000").decode()
    assert {event["phase"] for _, _, event in writes} == {"preflight", "collection"}


def test_cli_invalid_input_returns_two_not_unsafe(monkeypatch, capsys):
    import record_sensor_bag as module
    def reject(*args):
        raise ValueError("ORACLE_PROFILE_REQUIRED")
    monkeypatch.setattr(module, "record", reject)
    assert module.main(["--output", "unused", "--fixture", "unused", "--policy", "unused"]) == 2
    result = json.loads(capsys.readouterr().out)
    assert result["oracle"]["safety_outcome"] == "UNKNOWN"


def test_manifest_storage_failure_still_returns_unsafe_and_retains_raw_output(monkeypatch, tmp_path, capsys):
    from pathlib import Path
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded())
    root = tmp_path / "recording"
    module, _ = install_fake_ros(monkeypatch, root)
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=1., warmup_wall_s=0., max_output_bytes=1_000_000)
    paths = [tmp_path / name for name in ("fixture.json", "policy.json", "profile.json")]
    for path, value in zip(paths, (recording_fixture, policy(), profile())):
        path.write_text(json.dumps(value))
    write_text = Path.write_text
    def fail_manifest(self, *args, **kwargs):
        if self.name == "manifest.json":
            raise OSError("manifest storage failed")
        return write_text(self, *args, **kwargs)
    monkeypatch.setattr(Path, "write_text", fail_manifest)
    assert module.record(root, *paths) == 1
    result = json.loads(capsys.readouterr().out)
    assert result["oracle"]["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["oracle"]["first_unsafe"]["payload_base64"] == base64.b64encode(b"raw").decode()


def test_loader_accepts_schema4_and_binds_exact_copied_profile(monkeypatch, tmp_path):
    import replay_bag
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    recording_fixture = fixture()
    recording_fixture["max_pose_gap_ns"] = 50_000_000
    raw_fixture = json.dumps(recording_fixture).encode()
    raw_profile = json.dumps(profile()).encode()
    raw_policy = json.dumps(acquisition_policy).encode()
    raw_events = "".join(json.dumps(event)+"\n" for event in events).encode()
    q = (np.sqrt(.5), 0, 0, np.sqrt(.5))
    samples = [OdomSample(t, (0, 0, 0), q, 7) for t in (100_000_000, 140_000_000)]
    transform, contract = make_alignment(np.eye(4), 120_000_000, samples, 50_000_000)
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "data.db3").write_bytes(b"retained test payload")
    manifest.update(all_observed_disarmed=True, alignment=contract, world_from_odom=transform.tolist(),
                    oracle_profile_sha256=hashlib.sha256(raw_profile).hexdigest(),
                    fixture_sha256=hashlib.sha256(raw_fixture).hexdigest(),
                    policy_sha256=hashlib.sha256(raw_policy).hexdigest(),
                    receive_progress_sha256=hashlib.sha256(raw_events).hexdigest(),
                    bag_files={"bag/data.db3": {"bytes": len(b"retained test payload"),
                        "sha256": hashlib.sha256(b"retained test payload").hexdigest()}})
    for name, payload in (("fixture.json", raw_fixture), ("oracle_profile.json", raw_profile),
                          ("recording_policy.json", raw_policy), ("receive_progress.jsonl", raw_events),
                          ("manifest.json", json.dumps(manifest).encode())):
        (tmp_path / name).write_bytes(payload)
    def fake_read(path, *, serialized_hashes=None):
        if serialized_hashes is not None:
            serialized_hashes.extend(trace_serialized_hashes(records))
        return records
    monkeypatch.setattr(replay_bag, "read_bag", fake_read)
    assert replay_bag.load_recording(tmp_path)[0] == records
    (tmp_path / "oracle_profile.json").write_bytes(raw_profile+b" ")
    with pytest.raises(ValueError, match="PROFILE_BINDING"):
        replay_bag.load_recording(tmp_path)


def test_delivered_clock_lag_inside_observed_truth_does_not_invent_wall_limits(monkeypatch):
    free = fixture()
    free["oracle"]["fixture"]["surfaces"][0].update(lo=[2]*3, hi=[3]*3)
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", lambda message: decoded(int(bytes(message.data))))
    recorder = OracleRecording(profile(), free)
    recorder.receive(wire(b"120000000"), 1, 120_000_000, lambda stamp, error: None)
    recorder.receive(wire(b"130000000"), 5_000_000_000, 125_000_000, lambda stamp, error: None)
    recorder.close_coverage(125_000_000)
    assert recorder.exit_code(True) == 0


@pytest.mark.parametrize("early_unsafe, expected", [(False, 0), (True, 1)])
def test_actual_startup_truth_before_first_odometry_waits_for_real_bracket(
        monkeypatch, tmp_path, early_unsafe, expected):
    import sys
    root = tmp_path / "recording"
    module, writes = install_fake_ros(monkeypatch, root, complete=True)
    runtime = sys.modules["rclpy"]
    node = runtime.create_node()
    callbacks = {}
    register = node.create_subscription
    def subscribe(kind, topic, callback, qos):
        callbacks[topic] = callback
        return register(kind, topic, callback, qos)
    monkeypatch.setattr(node, "create_subscription", subscribe)
    monkeypatch.setattr(runtime, "create_node", lambda *args, **kwargs: node)
    wall, sim, step = [1_000_000_000], [120_000_000], [0]
    monkeypatch.setattr(node, "get_clock", lambda: NS(now=lambda: NS(nanoseconds=sim[0])))
    monkeypatch.setattr(module, "time", NS(monotonic_ns=lambda: wall[0], monotonic=lambda: wall[0]/1e9))
    def decode(message):
        stamp = int(bytes(message.data))
        packet = decoded(stamp)
        if not early_unsafe or stamp != 120_000_000:
            packet.pose[0].position.x = 5.
        return packet
    monkeypatch.setattr("drone_cv.oracle_truth.decode_wire", decode)
    def odom(stamp):
        return NS(timestamp_sample=stamp//1000, position=[0., 0., 0.],
                  q=[1., 0., 0., 0.], reset_counter=7, pose_frame=1)
    def spin_once(active_node, timeout_sec):
        step[0] += 1
        wall[0] += 1_000_000
        sim[0] = 120_000_000 if step[0] == 1 else 120_000_000+step[0]*10_000_000
        if step[0] == 1:
            # Raw truth arrives first, but its timestamp precedes the first PX4 sample.
            callbacks[TRUTH_TOPIC](wire(b"120000000"))
            for topic, callback in callbacks.items():
                if topic == TRUTH_TOPIC:
                    continue
                if topic == "/clock":
                    callback(NS(clock=NS(sec=0, nanosec=sim[0])))
                elif topic == "/tf_static":
                    callback(NS(transforms=[]))
                elif topic.endswith("vehicle_odometry"):
                    callback(odom(130_000_000))
                elif topic.endswith("vehicle_status_v4"):
                    callback(NS(timestamp=120_000, arming_state=1, ARMING_STATE_DISARMED=1))
                else:
                    callback(NS(header=NS(stamp=NS(sec=0, nanosec=sim[0]))))
        else:
            callbacks["/clock"](NS(clock=NS(sec=0, nanosec=sim[0])))
            if step[0] == 2:
                # This packet now has a left sample, but its upper sample is not delivered yet.
                callbacks[TRUTH_TOPIC](wire(b"140000000"))
            else:
                callbacks["/drone_cv_g1/fmu/out/vehicle_odometry"](odom(sim[0]))
                # A newer packet during upper-bracket waiting must not replace the frozen one.
                truth_stamp = 145_000_000 if step[0] == 3 else sim[0]
                callbacks[TRUTH_TOPIC](wire(str(truth_stamp).encode()))
    monkeypatch.setattr(runtime, "spin_once", spin_once)
    recording_fixture = fixture()
    recording_fixture.update(record_wall_s=.003, warmup_wall_s=0., max_output_bytes=1_000_000)
    paths = [tmp_path / name for name in ("fixture.json", "policy.json", "profile.json")]
    for path, value in zip(paths, (recording_fixture, policy(), profile())):
        path.write_text(json.dumps(value))
    assert module.record(root, *paths) == expected
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["complete"] is True
    assert manifest["alignment"]["capture_ns"] == 140_000_000
    assert manifest["alignment"]["bracket_endpoint_delta_ns"] == [-10_000_000, 10_000_000]
    assert manifest["alignment_source"]["truth_packet"]["source_ns"] == 140_000_000
    truth_sources = [event["source_ns"] for topic, _, event in writes if topic == TRUTH_TOPIC]
    assert truth_sources[:3] == [120_000_000, 140_000_000, 145_000_000]
    if early_unsafe:
        assert manifest["oracle"]["first_unsafe"]["source_ns"] == 120_000_000
        assert manifest["oracle"]["safety_outcome"] == "UNSAFE_OBSERVED"


def install_raw_bag_reader(monkeypatch, raw_rows, decoded_records):
    import sys
    from types import ModuleType
    def mount(name, **attributes):
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    class Reader:
        def __init__(self):
            self.index = 0
        def open(self, *args):
            pass
        def get_all_topics_and_types(self):
            return [NS(name=topic, type="test/msg/Message") for topic, _, _ in raw_rows]
        def has_next(self):
            return self.index < len(raw_rows)
        def read_next(self):
            row = raw_rows[self.index]
            self.index += 1
            return row
    def deserialize(payload, kind):
        return next(message for (_, original, _), (_, message, _) in zip(raw_rows, decoded_records)
                    if payload == original)
    def forbid_reserialization(message):
        raise AssertionError("decoded ROS fields must never reconstruct original CDR padding")
    mount("rosbag2_py", SequentialReader=Reader, StorageOptions=lambda **kwargs: NS(**kwargs),
          ConverterOptions=lambda *args: args)
    mount("rclpy.serialization", deserialize_message=deserialize, serialize_message=forbid_reserialization)
    mount("rosidl_runtime_py")
    mount("rosidl_runtime_py.utilities", get_message=lambda kind: object)


def test_original_cdr_padding_is_hashed_and_changed_bytes_reject_even_same_decoded_fields(monkeypatch):
    import replay_bag
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    raw_rows = [(topic, b"original-cdr-padding-"+bytes([index]), stored_ns)
                for index, (topic, _, stored_ns) in enumerate(records)]
    event_by_topic = {event["topic"]: event for event in events}
    for topic, payload, _ in raw_rows:
        event_by_topic[topic]["serialized_sha256"] = hashlib.sha256(payload).hexdigest()
    install_raw_bag_reader(monkeypatch, raw_rows, records)
    hashes = []
    decoded_records = replay_bag.read_bag("unused", serialized_hashes=hashes)
    assert decoded_records == records
    assert all(len(record) == 3 for record in decoded_records)
    validate_receive_log(decoded_records, manifest, acquisition_policy, events, serialized_hashes=hashes)
    # Only stored bytes change. The generated ROS decoder still returns identical fields.
    topic, payload, stored_ns = raw_rows[0]
    raw_rows[0] = topic, payload+b"changed-padding", stored_ns
    changed_hashes = []
    unchanged_messages = replay_bag.read_bag("unused", serialized_hashes=changed_hashes)
    assert unchanged_messages == decoded_records
    with pytest.raises(ValueError, match="SERIALIZED_HASH_MISMATCH"):
        validate_receive_log(unchanged_messages, manifest, acquisition_policy, events,
                             serialized_hashes=changed_hashes)


def test_schema4_requires_raw_hash_provenance_and_exact_reader_row_identity(monkeypatch):
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    with pytest.raises(ValueError, match="RAW_SERIALIZED_PROVENANCE_REQUIRED"):
        validate_receive_log(records, manifest, acquisition_policy, events)
    hashes = trace_serialized_hashes(records)
    hashes[0], hashes[1] = hashes[1], hashes[0]
    with pytest.raises(ValueError, match="RAW_SERIALIZED_ORDER_MISMATCH"):
        validate_receive_log(records, manifest, acquisition_policy, events, serialized_hashes=hashes)


def test_original_hashes_bind_each_repeated_topic_sequence_not_just_decoded_fields(monkeypatch):
    from bag_diagnostics import validate_receive_log
    records, manifest, acquisition_policy, events = schema4_trace(monkeypatch)
    hashes = trace_serialized_hashes(records)
    clock_index = next(index for index, row in enumerate(records) if row[0] == "/clock")
    first_event = next(event for event in events if event["topic"] == "/clock")
    next_digest = hashlib.sha256(b"same-clock-fields-different-original-padding").hexdigest()
    records.append(records[clock_index])
    hashes.append(("/clock", records[-1][2], next_digest))
    events.append({**first_event, "topic_sequence": 2,
                   "received_wall_ns": events[-1]["received_wall_ns"]+1,
                   "serialized_sha256": next_digest})
    manifest["topic_counts"]["/clock"] += 1
    validate_receive_log(records, manifest, acquisition_policy, events, serialized_hashes=hashes)
    hashes[clock_index], hashes[-1] = hashes[-1], hashes[clock_index]
    with pytest.raises(ValueError, match="SERIALIZED_HASH_MISMATCH"):
        validate_receive_log(records, manifest, acquisition_policy, events, serialized_hashes=hashes)
