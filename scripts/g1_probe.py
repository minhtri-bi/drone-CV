"""Read-only, bounded G1 probe inside the pinned ROS/PX4/Gazebo container.

This script never publishes or arms. Exit 2 means partial/incomplete environment,
not a successful flight gate. Source the image's ros2-entrypoint.sh first.
"""

import base64
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np


NAMESPACE = "/drone_cv_g1"


def command(args: list[str], timeout_s: float = 15) -> dict:
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout_s)
        return {"exit": result.returncode, "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip()[:2000]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"exit": None, "error": str(error)}


def topic_json(topic: str) -> dict:
    result = command(["timeout", "10s", "gz", "topic", "-e", "-t", topic,
                      "-n", "1", "--json-output"], timeout_s=12)
    if result["exit"] != 0:
        return {"error": result}
    try:
        return json.loads(result["stdout"])
    except ValueError as error:
        return {"error": f"invalid Gazebo JSON: {error}"}


def depth_summary(message: dict) -> dict:
    """Validate payload shape before interpreting Gazebo R_FLOAT32 optical Z."""
    if "error" in message:
        return message
    width, height, step = message.get("width"), message.get("height"), message.get("step")
    if (type(width) is not int or type(height) is not int or width <= 0 or height <= 0
            or message.get("pixelFormatType") != "R_FLOAT32" or step != width*4):
        raise ValueError("unsupported depth metadata/stride")
    payload = base64.b64decode(message["data"], validate=True)
    if len(payload) != height*step:
        raise ValueError("depth payload size does not match image metadata")
    depth = np.frombuffer(payload, dtype="<f4")
    finite = depth[np.isfinite(depth)]
    return {"width": width, "height": height, "step_bytes": step,
            "format": "R_FLOAT32", "header": message.get("header"),
            "finite_pixels": int(finite.size), "nan_pixels": int(np.isnan(depth).sum()),
            "positive_inf_pixels": int(np.isposinf(depth).sum()),
            "negative_inf_pixels": int(np.isneginf(depth).sum()),
            "finite_min_m": float(finite.min()) if finite.size else None,
            "finite_max_m": float(finite.max()) if finite.size else None,
            "payload_sha256": hashlib.sha256(payload).hexdigest(),
            "no_return_semantics_certified": False}


def observe_dds(seconds: float = 6) -> dict:
    import rclpy
    from px4_msgs.msg import VehicleOdometry, VehicleStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("drone_cv_g1_readonly_probe")
    samples = {"odometry": [], "status": []}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def odometry(message):
        samples["odometry"].append({"timestamp_us": message.timestamp,
                                   "sample_us": message.timestamp_sample,
                                   "pose_frame": message.pose_frame,
                                   "position": [float(v) for v in message.position],
                                   "quaternion_wxyz": [float(v) for v in message.q]})

    def status(message):
        samples["status"].append({"timestamp_us": message.timestamp,
                                 "arming_state": message.arming_state,
                                 "armed": message.arming_state == VehicleStatus.ARMING_STATE_ARMED})

    odom_topic = f"{NAMESPACE}/fmu/out/vehicle_odometry"
    status_topic = f"{NAMESPACE}/fmu/out/vehicle_status_v4"
    try:
        subscriptions = [node.create_subscription(VehicleOdometry, odom_topic, odometry, qos),
                         node.create_subscription(VehicleStatus, status_topic, status, qos)]
        start = time.monotonic()
        while time.monotonic()-start < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic()-start
        result = {}
        for label, values in samples.items():
            stamps = [v["timestamp_us"] for v in values]
            result[label] = {"count": len(values), "wall_window_s": elapsed,
                             "wall_rate_hz": len(values)/elapsed,
                             "strictly_increasing_stamps": len(stamps) > 1 and all(
                                 a < b for a, b in zip(stamps, stamps[1:])),
                             "first": values[0] if values else None,
                             "last": values[-1] if values else None}
        result["all_observed_disarmed"] = bool(samples["status"]) and not any(
            value["armed"] for value in samples["status"])
        result["publishers"] = {
            topic: [{"type": p.topic_type,
                     "reliability": p.qos_profile.reliability.name,
                     "durability": p.qos_profile.durability.name}
                    for p in node.get_publishers_info_by_topic(topic)]
            for topic in (odom_topic, status_topic)}
        result["ros_camera_topics"] = [t for t, types in node.get_topic_names_and_types()
                                       if "sensor_msgs/msg/Image" in types
                                       or "sensor_msgs/msg/CameraInfo" in types]
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    report = {"probe": "read-only G1 environment, never flight acceptance",
              "control_publishers_created": 0, "arming_commands_sent": 0,
              "namespace": NAMESPACE, "versions": {}, "assets_sha256": {}}
    report["versions"]["packages"] = command([
        "dpkg-query", "-W", "-f=${Package} ${Version}\n", "px4-gazebo", "ros-jazzy-rclpy",
        "libgz-sim8", "libgz-rendering8"])
    report["versions"]["firmware"] = command(["px4-ver", "all"])
    report["bridge"] = command(["ros2", "pkg", "executables", "ros_gz_bridge"])
    base = Path("/opt/px4-gazebo")
    for relative in ["share/gz/models/x500_depth/model.sdf",
                     "share/gz/models/OakD-Lite/model.sdf", "share/gz/models/x500/model.sdf",
                     "share/gz/models/x500_base/model.sdf", "share/gz/worlds/default.sdf"]:
        path = base/relative
        report["assets_sha256"][relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    report["ros_source_manifest"] = Path("/opt/px4_ros2/ros2.repos").read_text()
    report["depth"] = depth_summary(topic_json("/depth_camera"))
    report["camera_info"] = topic_json("/camera_info")
    report["world_stats"] = topic_json("/stats")
    report["dds"] = observe_dds()
    report["commander_status"] = command(["px4-commander", "status"])
    missing = []
    if report["bridge"]["exit"] != 0:
        missing.append("ros_gz_bridge missing: no ROS image/CameraInfo/clock bridge")
    if not report["dds"]["all_observed_disarmed"]:
        missing.append("no complete disarmed status evidence")
    if not report["dds"]["odometry"]["strictly_increasing_stamps"]:
        missing.append("no advancing DDS odometry")
    if not report["dds"]["ros_camera_topics"]:
        missing.append("no ROS camera topics; camera QoS/clock/frame alignment unverified")
    report["blockers"] = missing
    report["observation_complete"] = not missing
    report["acceptance_evaluated"] = False
    report["scope_note"] = "Gazebo Transport sensor evidence and DDS only; not a ROS camera adapter or aligned-clock proof"
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(2 if missing else 0)


if __name__ == "__main__":
    main()
