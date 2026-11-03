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
from recording_integrity import (SCHEMA, advance_progress, make_alignment, odom_sample,
                                 require_fresh, source_stamp_ns, validate_policy)

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


def record(output: Path, fixture_path: Path, policy_path: Path) -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    fixture_bytes, policy_bytes = fixture_path.read_bytes(), policy_path.read_bytes()
    fixture, policy = json.loads(fixture_bytes), json.loads(policy_bytes)
    validate_policy(policy)
    if set(policy["stream_progress_wall_s"]) != set(TOPICS)-{"/tf_static"}:
        raise ValueError("POLICY_REQUIRED_STREAMS_MISMATCH")
    output.mkdir(parents=True, exist_ok=False)
    (output / "fixture.json").write_bytes(fixture_bytes)
    (output / "recording_policy.json").write_bytes(policy_bytes)
    rclpy.init()
    node = rclpy.create_node("unarmed_sensor_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
    static = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cached, observed_wall, progress, counts = {}, {}, {}, Counter()
    odometry, writer, begin_ns = [], None, None
    failure, alignment_counter = [], None
    last_clock, clock_progress_ns = 0, time.monotonic_ns()
    events = (output / "receive_progress.jsonl").open("w", encoding="utf-8")
    manifest = {"recording_schema": SCHEMA, "scope": "unarmed acquisition only",
                "control_publishers_created": 0, "arming_commands_sent": 0,
                "fixture_sha256": hashlib.sha256(fixture_bytes).hexdigest(),
                "policy_sha256": hashlib.sha256(policy_bytes).hexdigest(),
                "all_observed_disarmed": True, "complete": False, "storage_id": "sqlite3"}

    def write_message(topic, message, wall_ns, seeded=False):
        sim_ns = node.get_clock().now().nanoseconds
        writer.write(topic, serialize_message(message), sim_ns)
        counts[topic] += 1
        events.write(json.dumps({"topic": topic, "source_ns": source_stamp_ns(topic, message),
                                "received_wall_ns": wall_ns, "received_sim_ns": sim_ns,
                                "seeded": seeded, "topic_sequence": counts[topic]})+"\n")

    def receive(topic, message):
        nonlocal last_clock, clock_progress_ns, progress, odometry
        wall_ns = time.monotonic_ns()
        try:
            if writer is not None:
                # Preserve the offending raw observation before recording its failure reason.
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
                odometry = [*odometry, sample]
            if topic == "/clock" and source_ns > last_clock:
                last_clock, clock_progress_ns = source_ns, wall_ns
            cached[topic], observed_wall[topic] = message, wall_ns
        except ValueError as error:
            failure.append(str(error))

    def qos_record(qos):
        return {"reliability": qos.reliability.name, "durability": qos.durability.name,
                "history": qos.history.name, "depth": qos.depth}

    try:
        subscriptions = [node.create_subscription(get_message(kind), topic,
                         lambda message, topic=topic: receive(topic, message),
                         static if topic == "/tf_static" else sensor)
                         for topic, kind in TOPICS.items()]
        ready_deadline = time.monotonic()+policy["startup_wall_s"]
        while len(cached) != len(TOPICS) or last_clock == 0:
            if failure:
                raise ValueError(failure[0])
            if time.monotonic() >= ready_deadline:
                raise ValueError("REQUIRED_STREAMS_MISSING")
            rclpy.spin_once(node, timeout_sec=.05)
        # Bracket the independent simulator snapshot instead of pairing it with a cached pose.
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
        alignment, contract = make_alignment(world_body, capture_ns, odometry,
                                             policy["max_pose_gap_ns"])
        alignment_counter = contract["reset_counter"]
        if odometry[-1].reset_counter != alignment_counter:
            raise ValueError("ALIGNMENT_RESET_CHANGED")
        manifest.update(world_from_odom=alignment.tolist(), alignment=contract,
                        alignment_source={"gazebo_pose_snapshot": truth,
                                          "note": "Capture-time bracket; frozen before recording; not fitted to depth."})
        # Catch up every required stream after the bounded snapshot call before recording.
        while True:
            if failure:
                raise ValueError(failure[0])
            try:
                require_fresh(progress, policy["stream_progress_wall_s"], time.monotonic_ns())
                break
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                rclpy.spin_once(node, timeout_sec=.05)
        manifest["publishers"] = {topic: [{"type": p.topic_type, **qos_record(p.qos_profile)}
            for p in node.get_publishers_info_by_topic(topic)] for topic in TOPICS}
        manifest["subscribers"] = {topic: qos_record(static if topic == "/tf_static" else sensor)
                                   for topic in TOPICS}
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output / "bag"), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic_id, (topic, kind) in enumerate(TOPICS.items()):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id, name=topic, type=kind, serialization_format="cdr"))
        begin_ns = time.monotonic_ns()
        manifest.update(record_start_wall_monotonic_ns=begin_ns, record_start_sim_ns=last_clock)
        for topic, message in cached.items():
            write_message(topic, message, observed_wall[topic], seeded=True)
        while (time.monotonic_ns()-begin_ns)/1e9 < fixture["record_wall_s"]:
            rclpy.spin_once(node, timeout_sec=.05)
            wall_ns = time.monotonic_ns()
            if "warmup_end_sim_ns" not in manifest and (wall_ns-begin_ns)/1e9 >= fixture["warmup_wall_s"]:
                manifest["warmup_end_sim_ns"] = last_clock
            if failure:
                raise ValueError(failure[0])
            require_fresh(progress, policy["stream_progress_wall_s"], wall_ns)
            if (wall_ns-clock_progress_ns)/1e9 > fixture["max_clock_stall_wall_s"]:
                raise ValueError("CLOCK_STALLED")
            if sum(p.stat().st_size for p in output.rglob("*") if p.is_file()) > fixture["max_output_bytes"]:
                raise ValueError("OUTPUT_BUDGET_EXCEEDED")
        manifest["complete"] = True
    except Exception as error:
        manifest["failure"] = f"{type(error).__name__}: {error}"
    finally:
        end_ns = time.monotonic_ns()
        writer = None
        events.close()
        node.destroy_node()
        rclpy.shutdown()
        elapsed = (end_ns-begin_ns)/1e9 if begin_ns is not None else 0.
        manifest.update(recorded_wall_s=elapsed, record_end_wall_monotonic_ns=end_ns,
                        record_end_sim_ns=last_clock, topic_counts=dict(counts),
                        topics=TOPICS, warmup_wall_s=fixture["warmup_wall_s"])
        manifest["bag_files"] = {str(p.relative_to(output)): {
            "bytes": p.stat().st_size, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in (output / "bag").rglob("*") if p.is_file()}
        manifest["receive_progress_sha256"] = hashlib.sha256((output / "receive_progress.jsonl").read_bytes()).hexdigest()
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: manifest.get(key) for key in
                      ("complete", "failure", "recorded_wall_s", "topic_counts", "all_observed_disarmed")}))
    return 0 if manifest["complete"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--policy", type=Path, default=Path(__file__).with_name("recording_policy.json"))
    options = parser.parse_args()
    raise SystemExit(record(options.output, options.fixture, options.policy))
