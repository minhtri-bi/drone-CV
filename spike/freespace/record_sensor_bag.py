"""Bounded unarmed recording inside the pinned ROS container. Never publishes control."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from capture_pose import quaternion_rotation
from recording_integrity import (SCHEMA, ORACLE_SCHEMA, TRUTH_TOPIC, advance_progress,
                                 make_alignment, odom_sample, require_fresh,
                                 source_stamp_ns, validate_policy)
from bag_writer import BagWriter

NS = "/drone_cv_g1"
TOPICS = {
    f"{NS}/camera/depth/image_raw": "sensor_msgs/msg/Image",
    f"{NS}/camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    f"{NS}/fmu/out/vehicle_odometry": "px4_msgs/msg/VehicleOdometry",
    f"{NS}/fmu/out/vehicle_status_v4": "px4_msgs/msg/VehicleStatus",
}


def truth_snapshot(model_name):
    raw = subprocess.check_output(
        ["timeout", "8s", "gz", "topic", "-e", "-t", "/world/default/dynamic_pose/info",
         "-n", "1", "--json-output"], text=True, timeout=10)
    data = json.loads(raw)
    model = next(item for item in data["pose"] if item.get("name") == model_name)
    body = next(item for item in data["pose"] if item.get("name") == "base_link")
    def rigid(item):
        position, orientation = item["position"], item["orientation"]
        transform = np.eye(4)
        transform[:3, :3] = quaternion_rotation(
            [orientation.get(k, 1.0 if k == "w" else 0.0) for k in ("w", "x", "y", "z")])
        transform[:3, 3] = [position.get(k, 0.0) for k in ("x", "y", "z")]
        return transform
    # Gazebo reports the model in world and its base_link in model coordinates.
    return rigid(model) @ rigid(body), data


def record(output: Path, fixture_path: Path, policy_path: Path,
           oracle_profile: Path | None = None) -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message

    fixture_bytes, policy_bytes = fixture_path.read_bytes(), policy_path.read_bytes()
    fixture, policy = json.loads(fixture_bytes), json.loads(policy_bytes)
    validate_policy(policy)
    schema = policy["schema"]
    topics = dict(TOPICS)
    oracle = None
    profile_bytes = None
    if schema == ORACLE_SCHEMA:
        from oracle_recording import OracleRecording, validate_binding
        if oracle_profile is None:
            raise ValueError("ORACLE_PROFILE_REQUIRED")
        profile_bytes = oracle_profile.read_bytes()
        oracle = OracleRecording(validate_binding(profile_bytes, fixture), fixture)
        topics[TRUTH_TOPIC] = "std_msgs/msg/UInt8MultiArray"
    if (schema not in (SCHEMA, ORACLE_SCHEMA) or "writer_queue_items" not in policy
            or "writer_shutdown_wall_s" not in policy
            or set(policy["stream_age_sim_s"]) != set(topics)-{"/tf_static", "/clock"}):
        raise ValueError("POLICY_REQUIRED_STREAMS_MISMATCH")
    output.mkdir(parents=True, exist_ok=False)
    (output / "fixture.json").write_bytes(fixture_bytes)
    (output / "recording_policy.json").write_bytes(policy_bytes)
    if profile_bytes is not None:
        (output / "oracle_profile.json").write_bytes(profile_bytes)
    rclpy.init()
    node = rclpy.create_node("unarmed_sensor_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
    static = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cached, observed_wall, progress, counts = {}, {}, {}, Counter()
    callback_counts = Counter()
    odometry, writer, begin_ns = (), None, None
    failure, alignment_counter = [], None
    last_clock = 0
    # Disk writes run in a bounded worker process, not in the ROS callback thread.
    manifest = {"recording_schema": schema, "scope": "unarmed acquisition only",
                "control_publishers_created": 0, "arming_commands_sent": 0,
                "fixture_sha256": hashlib.sha256(fixture_bytes).hexdigest(),
                "policy_sha256": hashlib.sha256(policy_bytes).hexdigest(),
                "all_observed_disarmed": True, "complete": False, "storage_id": "sqlite3"}
    if oracle is not None and profile_bytes is not None:
        manifest.update(oracle_profile_sha256=hashlib.sha256(profile_bytes).hexdigest(),
                        truth_entities=oracle.entities)

    def write_message(topic, message, wall_ns, seeded=False, *, source_ns=None,
                      stamp_known=False, validation_error=None, received_sim_ns=None):
        sim_ns = node.get_clock().now().nanoseconds if received_sim_ns is None else received_sim_ns
        sequence = counts[topic]+1
        event = {"topic": topic, "source_ns": source_ns if stamp_known else source_stamp_ns(topic, message),
                 "received_wall_ns": wall_ns, "received_sim_ns": sim_ns,
                 "seeded": seeded, "topic_sequence": sequence}
        if oracle is not None:
            event.update(phase="preflight" if begin_ns is None else "collection",
                         validation_error=validation_error)
        payload = serialize_message(message)
        if oracle is not None:
            event["serialized_sha256"] = hashlib.sha256(payload).hexdigest()
            if topic == TRUTH_TOPIC:
                event["truth_payload_sha256"] = hashlib.sha256(bytes(message.data)).hexdigest()
        if writer is None:
            raise ValueError("BAG_WRITER_UNAVAILABLE")
        writer.write(topic, payload, event)
        counts[topic] = sequence

    def receive(topic, message):
        nonlocal last_clock, progress, odometry
        wall_ns = time.monotonic_ns()
        callback_counts[topic] += 1
        try:
            if topic == TRUTH_TOPIC:
                if oracle is None:
                    raise ValueError("UNEXPECTED_TRUTH_STREAM")
                sim_ns = node.get_clock().now().nanoseconds
                sample, _ = oracle.receive(message, wall_ns, sim_ns,
                    lambda stamp, error: write_message(topic, message, wall_ns,
                        source_ns=stamp, stamp_known=True, validation_error=error, received_sim_ns=sim_ns))
                source_ns = sample.source_ns
                if "/clock" in progress and sim_ns > 0:
                    age_ns = sim_ns-source_ns
                    if age_ns < -policy["max_source_clock_lead_ns"]:
                        raise ValueError(f"SOURCE_AHEAD_OF_CLOCK:{topic}")
                    if age_ns > policy["stream_age_sim_s"][topic]*1e9:
                        raise ValueError(f"STREAM_SIM_AGE_EXCEEDED:{topic}")
            else:
                if writer is not None:
                    # Preserve the offending raw observation before semantic rejection.
                    write_message(topic, message, wall_ns)
                source_ns = source_stamp_ns(topic, message)
            if source_ns is not None:
                progress = advance_progress(progress, topic, source_ns, wall_ns)
            if topic.endswith("vehicle_status_v4") and message.arming_state != message.ARMING_STATE_DISARMED:
                manifest["all_observed_disarmed"] = False
                raise ValueError("STATUS_NOT_DISARMED")
            if topic.endswith("vehicle_odometry"):
                sample = odom_sample(message)
                if alignment_counter is not None and sample.reset_counter != alignment_counter:
                    raise ValueError("ALIGNMENT_RESET_CHANGED")
                odometry = ((*odometry[-(policy["alignment_buffer_samples"]-1):], sample)
                            if alignment_counter is None else ())
            if topic == "/clock" and source_ns is not None and source_ns > last_clock:
                last_clock = source_ns
            cached[topic], observed_wall[topic] = message, wall_ns
        except Exception as error:
            failure.append(f"{type(error).__name__}:{error}")
            if oracle is not None:
                oracle.fail(f"RECEIVE:{topic}:{error}")

    def qos_record(qos, discovered=False):
        unknown = discovered and qos.history.name in ("UNKNOWN", "SYSTEM_DEFAULT")
        return {"reliability": qos.reliability.name, "durability": qos.durability.name,
                "history": None if unknown else qos.history.name,
                "depth": None if discovered and (unknown or qos.depth == 0) else qos.depth,
                "source": "DDS discovery; null means unavailable" if discovered else "requested subscriber configuration"}

    try:
        if oracle is not None:
            # Start storage before subscribing: preflight truth is part of the same bag.
            writer = BagWriter(output, topics, policy)
        subscriptions = [node.create_subscription(get_message(kind), topic,
                         lambda message, topic=topic: receive(topic, message),
                         static if topic == "/tf_static" else sensor)
                         for topic, kind in topics.items()]
        ready_deadline = time.monotonic()+policy["startup_wall_s"]
        while len(cached) != len(topics) or last_clock == 0:
            if failure:
                raise ValueError(failure[0])
            if time.monotonic() >= ready_deadline:
                raise ValueError("REQUIRED_STREAMS_MISSING")
            rclpy.spin_once(node, timeout_sec=.05)
            if oracle is not None:
                require_fresh(progress, policy, time.monotonic_ns(),
                              sim_now_ns=node.get_clock().now().nanoseconds, allow_startup_missing=True)
        if failure:
            raise ValueError(failure[0])
        # Freeze a packet from the continuous stream; schema3 retains its historical
        # independent subprocess snapshot acquisition without any schema upgrade.
        if oracle is not None:
            deadline = time.monotonic()+policy["alignment_wait_wall_s"]
            # Startup delivery order is not acquisition order. Wait for an actual
            # stream packet at/after the oldest delivered PX4 sample before freezing;
            # an older cached packet can never obtain a left bracket by waiting.
            while (oracle.latest is None or not odometry
                   or oracle.latest[0].source_ns < odometry[0].timestamp_ns):
                if failure:
                    raise ValueError(failure[0])
                if time.monotonic() >= deadline:
                    raise ValueError("ALIGNMENT_POSE_TIMEOUT")
                rclpy.spin_once(node, timeout_sec=.05)
                require_fresh(progress, policy, time.monotonic_ns(),
                              sim_now_ns=node.get_clock().now().nanoseconds)
            if oracle.latest is None:
                raise ValueError("REQUIRED_TRUTH_STREAM_MISSING")
            truth_sample, truth = oracle.latest
            world_body = np.asarray(truth_sample.world_from_body)
            capture_ns = truth_sample.source_ns
        else:
            world_body, truth = truth_snapshot(fixture["model_name"])
            stamp = truth["header"]["stamp"]
            capture_ns = int(stamp["sec"])*1_000_000_000+int(stamp.get("nsec", 0))
            deadline = time.monotonic()+policy["alignment_wait_wall_s"]
        while not odometry or odometry[-1].timestamp_ns < capture_ns:
            if failure:
                raise ValueError(failure[0])
            if time.monotonic() >= deadline:
                raise ValueError("ALIGNMENT_POSE_TIMEOUT")
            rclpy.spin_once(node, timeout_sec=.05)
            if oracle is not None:
                require_fresh(progress, policy, time.monotonic_ns(),
                              sim_now_ns=node.get_clock().now().nanoseconds)
        alignment, contract = make_alignment(world_body, capture_ns, odometry,
                                             policy["max_pose_gap_ns"])
        alignment_counter = contract["reset_counter"]
        if odometry[-1].reset_counter != alignment_counter:
            raise ValueError("ALIGNMENT_RESET_CHANGED")
        odometry = ()
        manifest.update(world_from_odom=alignment.tolist(), alignment=contract,
                        alignment_source={"truth_packet" if oracle is not None else "gazebo_pose_snapshot": truth,
                                          "note": "Capture-time bracket; frozen before collection; not fitted to depth."})
        # Catch up every required stream after the bounded snapshot call before recording.
        while True:
            if failure:
                raise ValueError(failure[0])
            try:
                require_fresh(progress, policy, time.monotonic_ns(), sim_now_ns=node.get_clock().now().nanoseconds)
                break
            except ValueError:
                if oracle is not None:
                    raise
                if time.monotonic() >= deadline:
                    raise
                rclpy.spin_once(node, timeout_sec=.05)
        manifest["publishers"] = {topic: [{"type": p.topic_type, **qos_record(p.qos_profile, discovered=True)}
            for p in node.get_publishers_info_by_topic(topic)] for topic in topics}
        manifest["subscribers"] = {topic: qos_record(static if topic == "/tf_static" else sensor)
                                   for topic in topics}
        if writer is None:
            writer = BagWriter(output, topics, policy)
        begin_ns = time.monotonic_ns()
        manifest.update(record_start_wall_monotonic_ns=begin_ns, record_start_sim_ns=node.get_clock().now().nanoseconds)
        if oracle is None:
            for topic, message in cached.items():
                write_message(topic, message, observed_wall[topic], seeded=True)
        while (time.monotonic_ns()-begin_ns)/1e9 < fixture["record_wall_s"]:
            rclpy.spin_once(node, timeout_sec=.05)
            wall_ns = time.monotonic_ns()
            if "warmup_end_sim_ns" not in manifest and (wall_ns-begin_ns)/1e9 >= fixture["warmup_wall_s"]:
                manifest["warmup_end_sim_ns"] = node.get_clock().now().nanoseconds
            if failure:
                raise ValueError(failure[0])
            require_fresh(progress, policy, wall_ns, sim_now_ns=node.get_clock().now().nanoseconds)
            if sum(p.stat().st_size for p in output.rglob("*") if p.is_file()) > fixture["max_output_bytes"]:
                raise ValueError("OUTPUT_BUDGET_EXCEEDED")
        manifest["complete"] = True
    except Exception as error:
        manifest["failure"] = f"{type(error).__name__}: {error}"
        if oracle is not None:
            oracle.fail(manifest["failure"])
    finally:
        end_ns = time.monotonic_ns()
        try:
            end_sim_ns = node.get_clock().now().nanoseconds
        except Exception as error:
            end_sim_ns = None
            manifest["complete"] = False
            manifest.setdefault("failure", f"FINAL_CLOCK:{error}")
            if oracle is not None:
                oracle.fail(f"FINAL_CLOCK:{error}")
        if oracle is not None and end_sim_ns is not None:
            oracle.close_coverage(end_sim_ns)
        if writer is not None:
            try:
                writer_result = writer.close()
                manifest["storage_writer"] = writer_result
                if writer_result["topic_counts"] != dict(counts):
                    raise ValueError("BAG_WRITER_COUNTS_MISMATCH")
            except Exception as error:
                manifest["complete"] = False
                manifest["writer_failure"] = str(error)
                manifest.setdefault("failure", str(error))
                if oracle is not None:
                    oracle.fail(f"WRITER_CLOSE:{error}")
        try:
            if not (output / "receive_progress.jsonl").exists():
                (output / "receive_progress.jsonl").write_text("", encoding="utf-8")
        except OSError as error:
            manifest["complete"] = False
            manifest.setdefault("failure", f"RECEIVE_TRACE:{error}")
            if oracle is not None:
                oracle.fail(f"RECEIVE_TRACE:{error}")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as error:
            manifest["complete"] = False
            manifest.setdefault("failure", f"CLEANUP:{error}")
            if oracle is not None:
                oracle.fail(f"CLEANUP:{error}")
        elapsed = (end_ns-begin_ns)/1e9 if begin_ns is not None else 0.
        manifest.update(recorded_wall_s=elapsed, record_end_wall_monotonic_ns=end_ns,
                        record_end_sim_ns=end_sim_ns, topic_counts=dict(counts),
                        topics=topics, warmup_wall_s=fixture["warmup_wall_s"])
        manifest["depth_path_audit"] = {
            "schema": 1, "callback_counts": dict(callback_counts),
            "enqueued_counts": dict(counts),
            "source_and_bridge_authoritative_taps": False,
            "loss_location_certified": False,
            "scope": "callback/enqueue/storage only; warmup seeded events are separate"}
        try:
            manifest["bag_files"] = {(p.relative_to(output).as_posix() if oracle is not None
                                       else str(p.relative_to(output))): {
                "bytes": p.stat().st_size, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                for p in (output / "bag").rglob("*") if p.is_file()}
            manifest["receive_progress_sha256"] = hashlib.sha256((output / "receive_progress.jsonl").read_bytes()).hexdigest()
        except OSError as error:
            manifest["complete"] = False
            manifest.setdefault("failure", f"INVENTORY:{error}")
            if oracle is not None:
                oracle.fail(f"INVENTORY:{error}")
        if oracle is not None:
            manifest["oracle"] = oracle.finish(manifest["complete"])
        manifest_persisted = False
        try:
            (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
            manifest_persisted = True
        except (OSError, ValueError) as error:
            manifest["complete"] = False
            manifest["manifest_failure"] = str(error)
            manifest.setdefault("failure", f"MANIFEST_STORAGE:{error}")
            if oracle is not None:
                oracle.fail(f"MANIFEST_STORAGE:{error}")
                manifest["oracle"] = oracle.finish(False)
    summary = {key: manifest.get(key) for key in
               ("complete", "failure", "recorded_wall_s", "topic_counts", "all_observed_disarmed")}
    outcome = manifest.get("oracle", {})
    if oracle is not None:
        summary["oracle"] = {key: outcome[key] for key in
                            ("safety_outcome", "execution_outcome", "coverage_complete", "sample_count")}
        if not manifest_persisted:
            # If manifest storage itself failed, retain the raw unsafe evidence in the
            # caller's captured JSON output rather than losing the observation to exit 2.
            summary["manifest_failure"] = manifest["manifest_failure"]
            summary["oracle"] = outcome
    print(json.dumps(summary))
    if oracle is not None:
        return (1 if outcome["observed_unsafe"] else
                0 if manifest["complete"] and outcome["coverage_complete"] else 2)
    return 0 if manifest["complete"] else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--policy", type=Path, default=Path(__file__).with_name("recording_policy.json"))
    parser.add_argument("--oracle-profile", type=Path,
                        help="Frozen physical oracle profile; required by explicit schema4 policy")
    options = parser.parse_args(argv)
    try:
        return record(options.output, options.fixture, options.policy, options.oracle_profile)
    except Exception as error:
        # Input/startup errors have no safety observation; never classify them as unsafe.
        print(json.dumps({"complete": False, "failure": f"{type(error).__name__}: {error}",
                          "oracle": {"safety_outcome": "UNKNOWN", "execution_outcome": "INFRA_ERROR",
                                     "coverage_complete": False, "sample_count": 0}}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
