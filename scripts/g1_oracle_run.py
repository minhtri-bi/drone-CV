"""Run one frozen unarmed oracle fixture inside the approved pinned container."""

import argparse
from importlib import import_module
import hashlib
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

from drone_cv.oracle_truth import TRUTH_TOPIC, decode_wire, stamp_ns


ROOT = Path(__file__).resolve().parents[1]
CASES = {
    "support": {"obstacle": None, "fault": None},
    "body_intersection": {"obstacle": ([.19, -.01, .25], [.21, .01, .27]), "fault": None},
    "rotor_intersection": {"obstacle": ([.305, .165, .290], [.315, .175, .296]), "fault": None},
    "truth_loss": {"obstacle": None, "fault": "truth_loss"},
    "unsafe_truth_loss": {"obstacle": ([.19, -.01, .25], [.21, .01, .27]), "fault": "truth_loss"},
}


def save(path: Path, value: dict | list) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+"\n")


def add_box(world, name: str, lower: list, upper: list) -> None:
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(str((a+b)/2) for a, b in zip(lower, upper))+" 0 0 0"
    link = ET.SubElement(model, "link", name="target")
    for tag in ("collision", "visual"):
        geometry = ET.SubElement(ET.SubElement(link, tag, name=tag), "geometry")
        ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(
            str(b-a) for a, b in zip(lower, upper))


def frozen_world(output: Path, case: str) -> list:
    tree = ET.parse(ROOT/"docs/replay/unarmed-r35/default.sdf")
    world = tree.getroot().find("world")
    surfaces = [{"id": "floor", "lo": [-250., -250., -1.], "hi": [250., 250., 0.]},
                {"id": "front_target", "lo": [3.5, -.3, .2], "hi": [3.7, .3, .9]},
                {"id": "offaxis_target", "lo": [4.5, 1., .2], "hi": [4.7, 1.8, .9]}]
    y_target = {"id": "side_target", "lo": [1.8, .9, .2], "hi": [3., 1.1, .9]}
    add_box(world, y_target["id"], y_target["lo"], y_target["hi"])
    surfaces.append(y_target)
    if CASES[case]["obstacle"] is not None:
        lo, hi = CASES[case]["obstacle"]
        add_box(world, "oracle_intersection_target", lo, hi)
        surfaces.append({"id": "oracle_intersection_target", "lo": lo, "hi": hi})
    tree.write(output/"world.sdf", encoding="utf-8", xml_declaration=True)
    return surfaces


def discover_entities(seconds: float, settle_sim_ns: int = 0) -> dict:
    rclpy = import_module("rclpy")
    message_type = import_module("std_msgs.msg").UInt8MultiArray
    messages = []
    rclpy.init()
    node = rclpy.create_node("oracle_entity_discovery")
    subscription = node.create_subscription(message_type, TRUTH_TOPIC, messages.append, 10)
    try:
        deadline = time.monotonic()+seconds
        first_stamp, frozen_identity = None, None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if not messages:
                continue
            decoded = decode_wire(messages.pop())
            messages.clear()
            names = [pose.name for pose in decoded.pose]
            if "x500_depth_0" not in names or "base_link" not in names:
                continue  # Scene publication starts before PX4 has spawned the vehicle.
            def identity(name):
                matches = [pose for pose in decoded.pose if pose.name == name]
                if len(matches) != 1:
                    raise ValueError("AMBIGUOUS_TRUTH_DISCOVERY")
                return int(matches[0].id)
            found = {"model_name": "x500_depth_0", "model_id": identity("x500_depth_0"),
                     "body_name": "base_link", "body_id": identity("base_link"),
                     "link_pose_convention": "model_relative"}
            if settle_sim_ns == 0:
                return found
            now = stamp_ns(decoded)
            if first_stamp is None:
                first_stamp, frozen_identity = now, found
            if found != frozen_identity or now < first_stamp:
                raise ValueError("BOOTSTRAP_IDENTITY_OR_CLOCK_CHANGED")
            if now-first_stamp >= settle_sim_ns:
                return found
        raise ValueError("TRUTH_DISCOVERY_TIMEOUT")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def execute(output: Path, case: str, profile_path: Path, wall_seconds: float) -> int:
    if case not in CASES or not 15 <= wall_seconds <= 120:
        raise ValueError("INVALID_UNARMED_FIXTURE_ARGUMENT")
    output.mkdir(parents=True, exist_ok=False)
    profile_bytes = profile_path.read_bytes()
    profile = json.loads(profile_bytes)
    model_from_body = np.asarray(profile["model_from_body"], dtype=float)
    if not np.allclose(model_from_body[:3, :3], np.eye(3), atol=1e-12, rtol=0):
        raise ValueError("UNSUPPORTED_INITIAL_BODY_ORIENTATION")
    initial_z = -float(profile["z_lo_m"])-float(model_from_body[2, 3])
    initial_pose = f"0,0,{initial_z:.12g},0,0,0"
    (output/"profile.json").write_bytes(profile_bytes)
    surfaces = frozen_world(output, case)
    manifest = {"schema": 1, "case": case, "suite": "oracle_safety_negative" if "intersection" in case or case == "unsafe_truth_loss" else "oracle_static",
                "scope": "unarmed static observer only; conservative geometry, not flight acceptance",
                "control_publishers_created": 0, "arming_commands_sent": 0,
                "world_sha256": hashlib.sha256((output/"world.sdf").read_bytes()).hexdigest(),
                "profile_sha256": hashlib.sha256(profile_bytes).hexdigest(),
                "initial_model_pose": initial_pose,
                "fixture_limits": {"record_wall_s": wall_seconds, "max_truth_gap_ns": 50_000_000,
                                   "fault_after_collection_launch_wall_s": 8.,
                                   "bootstrap_settle_sim_ns": 10_000_000_000,
                                   "bootstrap_deadline_wall_s": 120.,
                                   "bootstrap_clearance_claimed": False},
                "processes": {}, "complete": False}
    save(output/"frozen_run.json", manifest)
    processes, logs = {}, []
    environment = {**os.environ, "GZ_CONFIG_PATH": "/usr/share/gz",
                   "GZ_SIM_RESOURCE_PATH": "/opt/px4-gazebo/share/gz/models:/opt/px4-gazebo/share/gz/worlds",
                   "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/px4-gazebo/lib/gz/plugins",
                   "GZ_SIM_SERVER_CONFIG_PATH": "/opt/px4-gazebo/share/gz/server.config",
                   "PX4_PARAM_UXRCE_DDS_SYNCT": "0", "PX4_GZ_MODEL_POSE": initial_pose,
                   "PX4_GZ_NO_FOLLOW": "1"}

    def start(name, arguments):
        log = (output/f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(arguments, stdin=subprocess.PIPE, stdout=log,
                                   stderr=subprocess.STDOUT, env=environment,
                                   start_new_session=True)
        processes[name] = process
        manifest["processes"][name] = {"pid": process.pid, "argv": arguments}
        return process

    try:
        start("gazebo", ["gz", "sim", "-r", "-s", str(output/"world.sdf")])
        start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888", "-v", "2"])
        start("px4", ["px4-gazebo"])
        # Isolate clock delivery from image serialization/transport backpressure.
        # Preserve the approved one-way mappings and all acquisition health limits.
        mappings = import_module("yaml").safe_load(Path("/opt/drone_cv/g1/bridge.yaml").read_text())
        for name, clock_only in (("camera_bridge", False), ("clock_bridge", True)):
            selected = [entry for entry in mappings if (entry["ros_topic_name"] == "/clock") == clock_only]
            if len(selected) != (1 if clock_only else 2):
                raise ValueError("PINNED_BRIDGE_MAPPING_MISMATCH")
            configuration = output/f"{name}.json"
            save(configuration, selected)
            start(name, ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                "-r", f"__node:=drone_cv_g1_{name}", "-p", f"config_file:={configuration}",
                "-p", "use_sim_time:=true", "-p", "override_timestamps_with_wall_time:=false"])
        start("static_tf", ["ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", "0.13233", "--y", "0", "--z", "0.02078", "--qx", "-0.5", "--qy", "0.5",
            "--qz", "-0.5", "--qw", "0.5", "--frame-id", "drone_cv_g1/base_link",
            "--child-frame-id", "drone_cv_g1/camera_optical"])
        adapter = start("truth_adapter", [sys.executable, str(ROOT/"scripts/g1_truth_adapter.py"),
            "--gazebo-topic", "/world/default/dynamic_pose/info", "--seconds", str(wall_seconds+200),
            "--report", str(output/"truth_adapter.json")])
        # Fixed bootstrap interval, not a search for a passing clearance window.
        # No startup clearance is claimed; oracle preflight begins at acquisition.
        entities = discover_entities(manifest["fixture_limits"]["bootstrap_deadline_wall_s"],
                                     manifest["fixture_limits"]["bootstrap_settle_sim_ns"])
        oracle_fixture = {"truth_entities": entities, "phase": "REST", "surfaces": surfaces,
            "clearance_by_phase": {"REST": 0., "NAVIGATE": .5}, "max_truth_gap_ns": 50_000_000,
            "support_allowlist": [{"part": part["id"], "surface": "floor", "phases": ["REST"],
                "zone_lo": [-1., -1., -.005], "zone_hi": [1., 1., .005]}
                for part in profile["parts"] if part["role"] == "gear"]}
        save(output/"oracle_fixture.json", oracle_fixture)
        fixture = json.loads((ROOT/"spike/freespace/replay_fixture.json").read_bytes())
        fixture = {**fixture, "record_wall_s": wall_seconds, "truth_entities": entities,
                   "world_sha256": manifest["world_sha256"],
                   "oracle": {"profile_sha256": manifest["profile_sha256"], "phase": "REST", "fixture": oracle_fixture},
                   "targets": fixture["targets"]+[{"name": "side", "axis": 1, "coordinate": .9,
                       "lo": [1.8, .9, .2], "hi": [3., .9, .9]}]}
        save(output/"fixture.json", fixture)
        policy = json.loads((ROOT/"spike/freespace/recording_policy.json").read_bytes())
        policy = {**policy, "schema": 4, "stream_age_sim_s": {**policy["stream_age_sim_s"], TRUTH_TOPIC: .2}}
        save(output/"policy.json", policy)
        recorder = start("recorder", [sys.executable, str(ROOT/"spike/freespace/record_sensor_bag.py"),
            "--fixture", str(output/"fixture.json"), "--policy", str(output/"policy.json"),
            "--oracle-profile", str(output/"profile.json"), "--output", str(output/"recording")])
        if CASES[case]["fault"] == "truth_loss":
            try:
                recorder.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                adapter.terminate()
                adapter.wait(timeout=5.)
                manifest["fault_applied"] = {"kind": "truth_loss", "monotonic_ns": time.monotonic_ns()}
        manifest["recorder_exit"] = recorder.wait(timeout=wall_seconds+60)
        result = subprocess.run([sys.executable, str(ROOT/"scripts/g1_oracle.py"), "replay",
            "--recording", str(output/"recording"), "--profile", str(output/"profile.json"),
            "--fixture", str(output/"oracle_fixture.json"), "--output", str(output/"replay")],
            capture_output=True, text=True, env=environment, timeout=120)
        (output/"replay.log").write_text(result.stdout+result.stderr)
        manifest["replay_exit"] = result.returncode
        manifest["complete"] = True
    except Exception as error:
        manifest["failure"] = f"{type(error).__name__}:{error}"
    finally:
        for name, process in reversed(tuple(processes.items())):
            # ros2 launch wrappers may exit before their children. Signal the
            # run-owned process group, never leave an orphan bridge behind.
            try:
                getattr(os, "killpg")(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                getattr(os, "killpg")(process.pid, getattr(signal, "SIGKILL"))
                process.wait(timeout=5.)
            manifest["processes"][name]["exit"] = process.returncode
        for log in logs:
            log.close()
        save(output/"run.json", manifest)
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["complete"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=30.)
    options = parser.parse_args()
    raise SystemExit(execute(options.output, options.case, options.profile, options.seconds))
