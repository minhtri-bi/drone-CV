"""Read-only physical-profile builder and static/declared-moving offline oracle.

Profile output is an exclusive JSON file. Replay/evaluate output is an exclusive
folder containing report.json. Nothing here starts a simulator or publishes.
Offline typed observations are analytical inputs, never live acceptance.
"""

import argparse
from collections import Counter, defaultdict, deque
import hashlib
from importlib import import_module
import json
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[1]


def _replay_api():
    # These existing scripts use sibling imports. Do not assume the caller's cwd
    # or add src: drone_cv must come from the explicitly configured PYTHONPATH.
    directory = str(REPO/"spike/freespace")
    if directory not in sys.path:
        sys.path.insert(0, directory)
    return import_module("replay_bag")


def _truth_api():
    return import_module("drone_cv.oracle_truth")


def _json(path):
    return json.loads(path.read_bytes())


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _reason(error):
    return str(error) or type(error).__name__


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f"INVALID_{name}")
    return value


def exit_code(report):
    """An observed unsafe event dominates subsequent infrastructure loss (S03)."""
    if report["observed_unsafe"]:
        return 1
    return 0 if report["execution_outcome"] == "COMPLETED" else 2


def _verify_payloads(recording, manifest):
    for name, key, reason in (
        ("fixture.json", "fixture_sha256", "FIXTURE_HASH_MISMATCH"),
        ("recording_policy.json", "policy_sha256", "RECEIVE_PROVENANCE_HASH_MISMATCH"),
        ("receive_progress.jsonl", "receive_progress_sha256", "RECEIVE_PROVENANCE_HASH_MISMATCH"),
        ("oracle_profile.json", "oracle_profile_sha256", "ORACLE_PROFILE_HASH_MISMATCH"),
    ):
        if _sha(recording/name) != manifest[key]:
            raise ValueError(reason)
    inventory = manifest["bag_files"]
    if not isinstance(inventory, dict) or not inventory:
        raise ValueError("MISSING_BAG_HASH_INVENTORY")
    root = recording.resolve()
    for name, identity in inventory.items():
        relative = Path(name)
        path = recording/relative
        if (relative.is_absolute() or ".." in relative.parts
                or not path.resolve().is_relative_to(root) or path.is_symlink()):
            raise ValueError("UNSAFE_BAG_PATH")
        if _sha(path) != identity["sha256"]:
            raise ValueError("BAG_HASH_MISMATCH")


def _load_replay(recording, manifest):
    """Retain valid payloads after health/collection failure, never bypass hashes.

    The normal loader owns the existing recording/trace validation. Recovery is
    explicitly nonaccepting: it records that loader failure before decoding the
    same hash-bound raw bag with the same generated ROS decoder.
    """
    _verify_payloads(recording, manifest)
    api = _replay_api()
    errors = []
    try:
        records, loaded, acquisition, events = api.load_recording(recording)
        if loaded != manifest:
            raise ValueError("RECORDING_MANIFEST_CHANGED")
        if events is None:
            raise ValueError("MISSING_RECEIVE_TRACE")
        return records, acquisition, events, errors
    except Exception as error:
        errors.append(_reason(error))
    # A decode/storage failure is not made successful by this recovery. The
    # caller retains these errors in the independent execution axis throughout.
    _verify_payloads(recording, manifest)
    serialized_hashes = []
    records = api.read_bag(recording/"bag", serialized_hashes=serialized_hashes)
    acquisition = _json(recording/"fixture.json")
    events = [json.loads(line) for line in (recording/"receive_progress.jsonl").read_bytes().splitlines()]
    try:
        api.validate_receive_log(records, manifest, _json(recording/"recording_policy.json"), events,
                                 serialized_hashes=serialized_hashes)
    except Exception as error:
        errors.append(_reason(error))
    return records, acquisition, events, errors


def _observe(state, oracle, profile, fixture, observation):
    evaluation = oracle.evaluate(profile, observation["world_from_body"], fixture, fixture["phase"])
    return state.observe(evaluation, sim_ns=observation["sim_ns"], epoch=observation["epoch"],
                         wall_ns=observation["wall_ns"], max_gap_ns=fixture["max_truth_gap_ns"])


def _offline(state, oracle, profile, fixture, path):
    samples = _json(path)
    if isinstance(state, import_module("drone_cv.oracle_observation").MovingObserverState):
        if not isinstance(samples, dict) or set(samples) != {"observations", "discovery"}:
            raise ValueError("MOVING_OBSERVATIONS_REQUIRE_OBSERVATIONS_AND_DISCOVERY")
        if not isinstance(samples["observations"], list) or not isinstance(samples["discovery"], list):
            raise ValueError("MOVING_OBSERVATION_LEDGERS_MUST_BE_ARRAYS")
        for sample in samples["observations"]:
            state = state.observe(sample)
        for snapshot in samples["discovery"]:
            state = state.discover(snapshot)
        return state, True, []
    if not isinstance(samples, list):
        raise ValueError("OBSERVATIONS_MUST_BE_ARRAY")
    for sample in samples:
        try:
            state = _observe(state, oracle, profile, fixture, sample)
        except Exception as error:
            state = state.fail(_reason(error))
    return state, True, []


def _packet_sample(packet, entities):
    _replay_api()
    return import_module("oracle_recording").decode_packet(packet, entities)


def _retained_unsafe(state, oracle, profile, fixture, recording, manifest):
    """Re-evaluate independently retained raw evidence, never trust its flags."""
    if state.observed_unsafe:
        return state, []
    try:
        packet = manifest.get("oracle", {}).get("first_unsafe")
        if packet is None:
            return state, []
        if _sha(recording/"fixture.json") != manifest["fixture_sha256"]:
            raise ValueError("FIXTURE_HASH_MISMATCH")
        acquisition = _json(recording/"fixture.json")
        if (fixture.get("truth_entities") != acquisition.get("truth_entities")
                or acquisition["oracle"]["profile_sha256"] != manifest["oracle_profile_sha256"]):
            raise ValueError("RETAINED_TRUTH_BINDING_MISMATCH")
        sample = _packet_sample(packet, fixture["truth_entities"])
        evaluation = oracle.evaluate(profile, sample.world_from_body, fixture, fixture["phase"])
        if not evaluation["unsafe"]:
            return state, []
        state = state.fail("RETAINED_UNSAFE_RAW_EVIDENCE_OUTSIDE_VALIDATED_BAG")
        observation = {"world_from_body": sample.world_from_body, "sim_ns": sample.source_ns,
                       "epoch": 0, "wall_ns": packet["received_wall_ns"]}
        state = state.observe(evaluation, sim_ns=sample.source_ns, epoch=0,
                              wall_ns=packet["received_wall_ns"],
                              max_gap_ns=fixture["max_truth_gap_ns"])
        return state, [{**observation, "model_id": sample.model_id, "body_id": sample.body_id,
                        "source": "manifest.oracle.first_unsafe", "payload_sha256": packet["payload_sha256"]}]
    except Exception as error:
        return state.fail(_reason(error)), []


def _replay(state, oracle, profile, fixture, recording, profile_hash):
    manifest = _json(recording/"manifest.json")
    if type(manifest.get("recording_schema")) is not int or manifest["recording_schema"] != 4:
        raise ValueError("NOT_CONTINUOUS_TRUTH")
    truth = _truth_api()
    if manifest.get("topics", {}).get(truth.TRUTH_TOPIC) != "std_msgs/msg/UInt8MultiArray":
        raise ValueError("NOT_CONTINUOUS_TRUTH")
    if manifest.get("oracle_profile_sha256") != profile_hash:
        raise ValueError("ORACLE_PROFILE_HASH_MISMATCH")
    try:
        records, acquisition, events, errors = _load_replay(recording, manifest)
    except Exception as error:
        state = state.fail(_reason(error))
        state, retained = _retained_unsafe(state, oracle, profile, fixture, recording, manifest)
        return state, False, retained
    for error in errors:
        state = state.fail(error)
    if acquisition["oracle"]["profile_sha256"] != profile_hash:
        return state.fail("ORACLE_PROFILE_HASH_MISMATCH"), False, []
    if fixture.get("truth_entities") != acquisition.get("truth_entities"):
        return state.fail("TRUTH_ENTITY_CONTRACT_MISMATCH"), False, []
    complete = manifest.get("complete") is True
    if not complete:
        state = state.fail("RECORDING_INCOMPLETE")
    if manifest.get("all_observed_disarmed") is not True:
        state = state.fail("RECORDING_NOT_DISARMED")
    # Storage is grouped only for matching each per-topic sequence. Evaluation
    # order comes solely from the original global receive trace, never sim-time
    # sorting (which could conceal resets or turn wall stamps into fabrications).
    by_topic = defaultdict(deque)
    try:
        for topic, message, stored_ns in records:
            by_topic[topic].append((message, stored_ns))
    except Exception as error:
        state = state.fail(_reason(error))
        complete = False
    sequences, matched = Counter(), Counter()
    epoch, previous_sim, previous_clock = 0, None, None
    reset = manifest.get("alignment", {}).get("reset_counter")
    samples = []
    try:
        for event in events:
            try:
                topic = event["topic"]
                sequences[topic] += 1
                if event["topic_sequence"] != sequences[topic]:
                    state = state.fail("RECEIVE_LOG_SEQUENCE_GAP")
                if not by_topic[topic]:
                    raise ValueError("RECEIVE_LOG_BAG_COUNTS_MISMATCH")
                message, stored_ns = by_topic[topic].popleft()
                matched[topic] += 1
                received_sim = _integer(event["received_sim_ns"], "RECEIVE_SIM_STAMP")
                wall_ns = _integer(event["received_wall_ns"], "RECEIVE_WALL_STAMP")
                if stored_ns != received_sim:
                    state = state.fail("RECEIVE_LOG_MESSAGE_MISMATCH")
                if previous_sim is not None and received_sim < previous_sim:
                    epoch += 1
                    state = state.fail("CLOCK_EPOCH_CHANGED")
                previous_sim = received_sim
                if topic == "/clock":
                    clock = int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
                    if previous_clock is not None and clock < previous_clock:
                        epoch += 1
                        state = state.fail("CLOCK_EPOCH_CHANGED")
                    previous_clock = clock
                elif topic.endswith("vehicle_odometry"):
                    counter = int(message.reset_counter)
                    if reset is not None and counter != reset:
                        epoch += 1
                        state = state.fail("ODOMETRY_RESET_CHANGED")
                    reset = counter
                if topic != truth.TRUTH_TOPIC:
                    continue
                decoded = truth.decode_wire(message)
                sample = truth.body_truth(decoded, fixture["truth_entities"])
                if sample.source_ns != event["source_ns"]:
                    state = state.fail("TRUTH_RECEIVE_SOURCE_MISMATCH")
                observation = {"world_from_body": sample.world_from_body,
                               "sim_ns": sample.source_ns, "epoch": epoch, "wall_ns": wall_ns}
                # Even a later trace mismatch must not erase a geometric unsafe
                # observation from an otherwise hash-bound decoded raw packet.
                state = _observe(state, oracle, profile, fixture, observation)
                samples.append({**observation, "topic_sequence": sequences[topic],
                                "model_id": sample.model_id, "body_id": sample.body_id})
            except Exception as error:
                state = state.fail(_reason(error))
    except Exception as error:
        state = state.fail(_reason(error))
        complete = False
    if any(by_topic.values()) or dict(matched) != manifest.get("topic_counts"):
        state = state.fail("RECEIVE_LOG_BAG_COUNTS_MISMATCH")
    if not samples:
        state = state.fail("NO_CONTINUOUS_TRUTH_SAMPLES")
    else:
        try:
            gap = _integer(fixture["max_truth_gap_ns"], "MAX_TRUTH_GAP_NS", 1)
            start = _integer(manifest["record_start_sim_ns"], "RECORD_START_SIM_NS")
            end = _integer(manifest["record_end_sim_ns"], "RECORD_END_SIM_NS")
            if end < start:
                raise ValueError("INVALID_RECORDING_BOUNDS")
            if not any(sample["epoch"] == 0 and abs(sample["sim_ns"]-start) <= gap
                       for sample in samples):
                state = state.fail("TRUTH_START_UNCOVERED")
            if samples[-1]["epoch"] != 0 or abs(end-samples[-1]["sim_ns"]) > gap:
                state = state.fail("TRUTH_END_UNCOVERED")
        except Exception as error:
            state = state.fail(_reason(error))
    state, retained = _retained_unsafe(state, oracle, profile, fixture, recording, manifest)
    samples.extend(retained)
    return state, complete, samples


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    profile = modes.add_parser("profile", help="Build physical profile using installed pinned sdformat14")
    profile.add_argument("--models", type=Path, required=True)
    profile.add_argument("--output", type=Path, required=True, help="Exclusive profile JSON file")
    for name in ("replay", "evaluate"):
        command = modes.add_parser(name)
        command.add_argument("--profile", type=Path, required=True)
        command.add_argument("--fixture", type=Path, required=True, help="Separate frozen oracle fixture JSON")
        command.add_argument("--output", type=Path, required=True, help="Exclusive report directory")
        command.add_argument("--recording" if name == "replay" else "--observations",
                             type=Path, required=True)
    return parser


def main(argv=None):
    args = _parser().parse_args(argv)
    if args.mode == "profile":
        try:
            profile = import_module("drone_cv.oracle_profile").build_profile(args.models)
            payload = json.dumps(profile, indent=2, allow_nan=False)+"\n"
            with args.output.open("x", encoding="utf-8", newline="\n") as stream:
                stream.write(payload)
            print(json.dumps({"output": str(args.output), "sha256": _sha(args.output)}))
            return 0
        except Exception as error:
            print(json.dumps({"execution_outcome": "INFRA_ERROR", "error": _reason(error)}))
            return 2
    try:
        args.output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        print(json.dumps({"execution_outcome": "INFRA_ERROR", "error": _reason(error)}))
        return 2
    failure = "ORACLE_INITIALIZATION_FAILED"
    state, samples, complete, inputs = None, [], False, {}
    moving = False
    fixture = {}
    try:
        oracle = import_module("drone_cv.oracle")
        state = oracle.ObserverState()
        inputs = {"profile_sha256": _sha(args.profile), "fixture_sha256": _sha(args.fixture)}
        profile, fixture = _json(args.profile), _json(args.fixture)
        moving = "moving_contract" in fixture
        if moving:
            state = None
            if args.mode != "evaluate":
                raise ValueError("MOVING_CONTRACT_REQUIRES_INDEPENDENT_OFFLINE_OBSERVATION_LEDGERS")
            state = import_module("drone_cv.oracle_observation").MovingObserverState.create(profile, fixture)
        if args.mode == "evaluate":
            inputs["observations_sha256"] = _sha(args.observations)
            state, complete, samples = _offline(state, oracle, profile, fixture, args.observations)
        else:
            inputs["manifest_sha256"] = _sha(args.recording/"manifest.json")
            state, complete, samples = _replay(state, oracle, profile, fixture, args.recording,
                                               inputs["profile_sha256"])
    except Exception as error:
        if state is not None:
            state = state.fail(_reason(error))
        else:
            failure = _reason(error)
    if state is None:
        result = {"safety_outcome": "UNKNOWN", "execution_outcome": "INFRA_ERROR",
                  "observed_unsafe": False, "reasons": [failure], "execution_reasons": [failure],
                  "ledger": [], "sample_count": 0, "coverage_complete": False,
                  "coverage_intervals": [], "coverage_gaps": [], "minimum_clearance_m": None,
                  "continuous_clearance_certified": False}
    else:
        result = state.finish(complete=complete)
    if moving and state is None:
        # Initialization failure must not silently revert to the static schema.
        result.pop("continuous_clearance_certified")
        result.update(clearance_conditional_on_declared_motion_bounds=False,
                      moving_intervals=[], motion_assumptions=None,
                      **import_module("drone_cv.oracle_observation")._motion_scope(
                          fixture["moving_contract"]))
    result.update(schema=1, mode=args.mode, inputs=inputs, truth_samples=samples,
                  scope=("OFFLINE_DECLARED_MOVING_CONTRACT" if moving else
                         "OFFLINE_TYPED_FIXTURE" if args.mode == "evaluate" else "RECORDED_CONTINUOUS_TRUTH"),
                  live_acceptance_certified=False, flight_admission=False,
                  sampling_scope=("Declared bounded-motion enclosures; independent evaluator phase and "
                                  "cooperative discovery ledgers; offline assumptions only."
                                  if moving
                                  else "Sampled static-fixture endpoints only; no inter-sample swept-volume certificate."))
    try:
        (args.output/"report.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n",
                                                encoding="utf-8")
    except Exception as error:
        print(json.dumps({"safety_outcome": result["safety_outcome"], "execution_outcome": "INFRA_ERROR",
                          "error": _reason(error)}))
        return 1 if result["observed_unsafe"] else 2
    print(json.dumps({key: result[key] for key in ("safety_outcome", "execution_outcome", "sample_count", "scope")}))
    return exit_code(result)


if __name__ == "__main__":
    raise SystemExit(main())
