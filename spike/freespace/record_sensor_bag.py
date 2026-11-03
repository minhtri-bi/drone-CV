"""Bounded unarmed recording inside the pinned ROS container. Never publishes control."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from capture_pose import OdomSample, interpolate_pose, quaternion_rotation

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


def record(output, fixture_path):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    fixture = json.loads(fixture_path.read_text())
    output.mkdir(parents=True, exist_ok=False)
    fixture_bytes = fixture_path.read_bytes()
    (output / "fixture.json").write_bytes(fixture_bytes)
    rclpy.init()
    node = rclpy.create_node("unarmed_sensor_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
    static = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cached, counts, writer = {}, Counter(), None
    failure = []
    last_clock, progress_wall = 0, time.monotonic()
    begin, elapsed = None, 0.0
    manifest = {"scope": "unarmed recording only", "control_publishers_created": 0,
                "arming_commands_sent": 0, "fixture_sha256": hashlib.sha256(fixture_bytes).hexdigest(),
                "all_observed_disarmed": True, "complete": False, "storage_id": "sqlite3"}

    def receive(topic, message):
        nonlocal last_clock, progress_wall
        cached[topic] = message
        if topic.endswith("vehicle_status_v4") and message.arming_state != message.ARMING_STATE_DISARMED:
            manifest["all_observed_disarmed"] = False
            failure.append("STATUS_NOT_DISARMED")
        if topic == "/clock":
            stamp = message.clock.sec*1_000_000_000+message.clock.nanosec
            if stamp < last_clock:
                failure.append("CLOCK_RESET_DURING_RECORDING")
            if stamp > last_clock:
                last_clock, progress_wall = stamp, time.monotonic()
        if writer is not None:
            writer.write(topic, serialize_message(message), node.get_clock().now().nanoseconds)
            counts[topic] += 1

    try:
        subscriptions = [node.create_subscription(get_message(kind), topic,
                         lambda message, topic=topic: receive(topic, message),
                         static if topic == "/tf_static" else sensor)
                         for topic, kind in TOPICS.items()]
        ready_deadline = time.monotonic()+20
        while (len(cached) != len(TOPICS) or last_clock == 0) and not failure:
            if time.monotonic() >= ready_deadline:
                raise ValueError("REQUIRED_STREAMS_MISSING")
            rclpy.spin_once(node, timeout_sec=.1)
        if failure:
            raise ValueError(failure[0])
        # Alignment uses only simulator localization and PX4 pose, never target depth.
        world_body, truth = truth_snapshot(fixture["model_name"])
        odom = cached[f"{NS}/fmu/out/vehicle_odometry"]
        sample = OdomSample(int(odom.timestamp_sample)*1000, tuple(odom.position),
                            tuple(odom.q), int(odom.reset_counter), int(odom.pose_frame))
        pose = interpolate_pose([sample], sample.timestamp_ns)
        odom_body = np.eye(4)
        odom_body[:3, :3], odom_body[:3, 3] = pose.world_from_body(), pose.position
        alignment = world_body @ np.linalg.inv(odom_body)
        manifest.update(world_from_odom=alignment.tolist(), alignment_source={
            "gazebo_pose_snapshot": truth, "odom_timestamp_sample_ns": sample.timestamp_ns,
            "odom_position_ned": sample.position_ned, "odom_q_wxyz": sample.quaternion_wxyz,
            "note": "Frozen before recording; independent resting-pose alignment, not fitted to depth."})
        manifest["publishers"] = {topic: [{"type": p.topic_type,
            "reliability": p.qos_profile.reliability.name,
            "durability": p.qos_profile.durability.name} for p in node.get_publishers_info_by_topic(topic)]
            for topic in TOPICS}
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output / "bag"), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic_id, (topic, kind) in enumerate(TOPICS.items()):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id, name=topic, type=kind, serialization_format="cdr"))
        # Actual cached messages preserve the latched TF and startup state in the bag.
        from rclpy.serialization import serialize_message
        for topic, message in cached.items():
            writer.write(topic, serialize_message(message), node.get_clock().now().nanoseconds)
            counts[topic] += 1
        begin = time.monotonic()
        progress_wall = begin
        manifest["record_start_sim_ns"] = last_clock
        while time.monotonic()-begin < fixture["record_wall_s"]:
            rclpy.spin_once(node, timeout_sec=.05)
            if ("warmup_end_sim_ns" not in manifest
                    and time.monotonic()-begin >= fixture["warmup_wall_s"]):
                manifest["warmup_end_sim_ns"] = last_clock
            if failure:
                raise ValueError(failure[0])
            if time.monotonic()-progress_wall > fixture["max_clock_stall_wall_s"]:
                raise ValueError("CLOCK_STALLED")
            size = sum(p.stat().st_size for p in output.rglob("*") if p.is_file())
            if size > fixture["max_output_bytes"]:
                raise ValueError("OUTPUT_BUDGET_EXCEEDED")
        manifest["complete"] = True
    except Exception as error:
        manifest["failure"] = f"{type(error).__name__}: {error}"
    finally:
        elapsed = time.monotonic()-begin if begin is not None else 0
        writer = None
        node.destroy_node()
        rclpy.shutdown()
        manifest.update(recorded_wall_s=elapsed, topic_counts=dict(counts),
                        topics=TOPICS, warmup_wall_s=fixture["warmup_wall_s"])
        manifest["bag_files"] = {str(p.relative_to(output)): {
            "bytes": p.stat().st_size, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in (output / "bag").rglob("*") if p.is_file()}
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: manifest.get(key) for key in
                      ("complete", "failure", "recorded_wall_s", "topic_counts", "all_observed_disarmed")}))
    return 0 if manifest["complete"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    options = parser.parse_args()
    raise SystemExit(record(options.output, options.fixture))
