"""Bounded evaluator-only native Gazebo-to-ROS Pose_V adapter; no controls or simulator."""

import argparse
from importlib import import_module
from pathlib import Path
from queue import Empty, Full, Queue
import time

from drone_cv.oracle_truth import MAX_PAYLOAD_BYTES, TRUTH_TOPIC, WIRE_LABEL


def run(gazebo_topic: str, seconds: float, queue_items: int, report_path: Path) -> int:
    import json
    import math

    if (not gazebo_topic.startswith("/world/") or not gazebo_topic.endswith("/dynamic_pose/info")
            or not math.isfinite(seconds) or not 1 <= seconds <= 600
            or type(queue_items) is not int or not 1 <= queue_items <= 512):
        raise ValueError("INVALID_TRUTH_ADAPTER_CONFIGURATION")
    if report_path.exists():
        raise ValueError("ADAPTER_REPORT_ALREADY_EXISTS")
    rclpy = import_module("rclpy")
    messages = import_module("std_msgs.msg")
    pose_type = import_module("gz.msgs10.pose_v_pb2").Pose_V
    native = import_module("gz.transport13").Node()
    incoming = Queue(maxsize=queue_items)
    failures = Queue(maxsize=1)

    def receive(message) -> None:
        try:
            payload = message.SerializeToString()
            if not 0 < len(payload) <= MAX_PAYLOAD_BYTES:
                raise ValueError("TRUTH_PAYLOAD_BUDGET_EXCEEDED")
            incoming.put_nowait(payload)
        except (Full, ValueError) as error:
            try:
                failures.put_nowait(f"TRUTH_ADAPTER_FAILURE:{type(error).__name__}:{error}")
            except Full:
                pass  # The first failure remains latched; the adapter will stop.

    rclpy.init()
    node = rclpy.create_node("drone_cv_evaluator_truth_adapter")
    publisher = node.create_publisher(messages.UInt8MultiArray, TRUTH_TOPIC, 10)
    report = {"schema": 1, "scope": "lossless Pose_V content, evaluator-only",
              "gazebo_topic": gazebo_topic, "ros_topic": TRUTH_TOPIC,
              "wire_label": WIRE_LABEL, "published": 0, "complete": False,
              "control_publishers_created": 0, "arming_commands_sent": 0}
    started = time.monotonic()
    try:
        native.subscribe(pose_type, gazebo_topic, receive)
        while time.monotonic()-started < seconds:
            if not failures.empty():
                raise ValueError(failures.get_nowait())
            try:
                payload = incoming.get_nowait()
            except Empty:
                rclpy.spin_once(node, timeout_sec=.01)
                continue
            message = messages.UInt8MultiArray()
            dimension = messages.MultiArrayDimension(label=WIRE_LABEL, size=len(payload), stride=len(payload))
            message.layout.dim = [dimension]
            message.data = list(payload)
            publisher.publish(message)
            report["published"] += 1
        if not failures.empty():
            raise ValueError(failures.get_nowait())
        if not incoming.empty():
            raise ValueError("TRUTH_ADAPTER_UNDRAINED_QUEUE")
        if report["published"] == 0:
            raise ValueError("TRUTH_ADAPTER_NO_SOURCE_MESSAGES")
        report["complete"] = True
    except Exception as error:
        report["failure"] = f"{type(error).__name__}:{error}"
    finally:
        native.unsubscribe(gazebo_topic)
        node.destroy_node()
        rclpy.shutdown()
        report["wall_s"] = time.monotonic()-started
        with report_path.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    return 0 if report["complete"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gazebo-topic", required=True)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument("--queue-items", type=int, default=128)
    parser.add_argument("--report", type=Path, required=True)
    options = parser.parse_args()
    raise SystemExit(run(options.gazebo_topic, options.seconds, options.queue_items, options.report))
