"""Bounded ROS camera/clock/TF observations; no control publishers."""

from math import isfinite
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np


NS = "/drone_cv_g1"
OPTICAL = "drone_cv_g1/camera_optical"
BODY = "drone_cv_g1/base_link"


def sensor_translation() -> list[float]:
    """Derive sensor origin relative to base_link, including the merged base pose."""
    base = Path("/opt/px4-gazebo/share/gz/models")
    model = ET.parse(base/"x500_depth/model.sdf")
    include = next(entry for entry in model.findall(".//include")
                   if entry.findtext("uri") == "model://OakD-Lite")
    mount = [float(v) for v in include.findtext("pose").split()]
    camera = ET.parse(base/"OakD-Lite/model.sdf")
    sensor = camera.find(".//sensor[@name='StereoOV7251']")
    offset = [float(v) for v in sensor.findtext("pose").split()]
    body = [float(v) for v in ET.parse(base/"x500_base/model.sdf").findtext("model/pose").split()]
    if any(value[3:] != [0, 0, 0] for value in (mount, offset, body)):
        raise ValueError("model rotation changed; zero-RPY extrinsics no longer apply")
    return [a+b-c for a, b, c in zip(mount[:3], offset[:3], body[:3])]


def image_record(message) -> dict:
    if message.encoding != "32FC1" or message.step != message.width*4:
        raise ValueError("ROS depth is not packed 32FC1")
    if message.width <= 0 or message.height <= 0 or len(message.data) != message.step*message.height:
        raise ValueError("ROS image dimensions/byte count mismatch")
    dtype = ">f4" if message.is_bigendian else "<f4"
    pixels = np.frombuffer(bytes(message.data), dtype=dtype)
    finite = pixels[np.isfinite(pixels)]
    return {"width": message.width, "height": message.height, "encoding": message.encoding,
            "frame_id": message.header.frame_id, "step_bytes": message.step,
            "finite_pixels": int(finite.size), "nan_pixels": int(np.isnan(pixels).sum()),
            "positive_inf_pixels": int(np.isposinf(pixels).sum()),
            "finite_min_m": float(finite.min()) if finite.size else None,
            "finite_max_m": float(finite.max()) if finite.size else None}


def stamp_ns(stamp) -> int:
    return stamp.sec*1_000_000_000 + stamp.nanosec


def observe_ros(seconds: float = 8.0) -> dict:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    from px4_msgs.msg import VehicleOdometry
    if not isfinite(seconds) or not 1 <= seconds <= 60:
        raise ValueError("observation duration must be 1..60 wall seconds")
    rclpy.init()
    node = rclpy.create_node("drone_cv_g1_camera_observer",
                            parameter_overrides=[Parameter("use_sim_time", value=True)])
    sensors = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    static = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    images, infos, clocks, transforms, errors = {}, {}, [], {}, []
    odometry_stamps = []
    image_topic = f"{NS}/camera/depth/image_raw"
    info_topic = f"{NS}/camera/depth/camera_info"

    def image(message):
        try:
            images[stamp_ns(message.header.stamp)] = image_record(message)
        except ValueError as error:
            errors.append(str(error))

    def info(message):
        infos[stamp_ns(message.header.stamp)] = {
            "width": message.width, "height": message.height,
            "frame_id": message.header.frame_id, "K": [float(v) for v in message.k]}

    def tf(message):
        for item in message.transforms:
            translation, rotation = item.transform.translation, item.transform.rotation
            transforms[(item.header.frame_id, item.child_frame_id)] = {
                "translation": [translation.x, translation.y, translation.z],
                "quaternion_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w]}

    try:
        subscriptions = [node.create_subscription(Image, image_topic, image, sensors),
                         node.create_subscription(CameraInfo, info_topic, info, sensors),
                         node.create_subscription(Clock, "/clock", lambda m: clocks.append(stamp_ns(m.clock)), sensors),
                         node.create_subscription(TFMessage, "/tf_static", tf, static)]
        subscriptions.append(node.create_subscription(
            VehicleOdometry, f"{NS}/fmu/out/vehicle_odometry",
            lambda message: odometry_stamps.append(message.timestamp*1000), sensors))
        started = time.monotonic()
        while time.monotonic()-started < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.monotonic()-started
        paired = sorted(images.keys() & infos.keys())
        latest = paired[-1] if paired else None
        image_meta = images.get(latest)
        info_meta = infos.get(latest)
        extrinsics = transforms.get((BODY, OPTICAL))
        checks = {
            "image_samples": len(images) > 1,
            "paired_image_camera_info": len(paired) > 1,
            "clock_progress": len(clocks) > 1 and clocks[-1] > clocks[0],
            "no_invalid_image": not errors,
            "sensor_stamp_in_sim_clock_window": bool(paired and clocks) and (
                clocks[0] <= paired[-1] <= clocks[-1]+4_000_000),
            "image_optical_frame": bool(image_meta) and image_meta["frame_id"] == OPTICAL,
            "camera_info_contract": bool(image_meta and info_meta) and (
                info_meta["frame_id"] == OPTICAL and info_meta["width"] == image_meta["width"]
                and info_meta["height"] == image_meta["height"]
                and all(isfinite(v) for v in info_meta["K"])
                and info_meta["K"][0] > 0 and info_meta["K"][4] > 0),
            "optical_extrinsics_match_model": bool(extrinsics) and bool(
                np.allclose(extrinsics["translation"], sensor_translation(), atol=1e-8, rtol=0)
                and np.allclose(extrinsics["quaternion_xyzw"], [-.5, .5, -.5, .5], atol=1e-8, rtol=0)),
            "node_uses_sim_time": node.get_parameter("use_sim_time").value is True,
            "odometry_in_sim_clock_domain": bool(odometry_stamps and clocks) and (
                clocks[0] <= odometry_stamps[-1] <= clocks[-1]+4_000_000),
        }
        return {"checks": checks, "all_checks_pass": all(checks.values()),
                "wall_window_s": elapsed, "image_count": len(images), "camera_info_count": len(infos),
                "paired_count": len(paired), "image_wall_hz": len(images)/elapsed,
                "image_sim_stamp_first_ns": min(images) if images else None,
                "image_sim_stamp_last_ns": max(images) if images else None,
                "clock_first_ns": clocks[0] if clocks else None,
                "clock_last_ns": clocks[-1] if clocks else None,
                "odometry_stamp_first_ns": odometry_stamps[0] if odometry_stamps else None,
                "odometry_stamp_last_ns": odometry_stamps[-1] if odometry_stamps else None,
                "node_ros_time_ns": node.get_clock().now().nanoseconds,
                "last_paired_image": image_meta, "last_paired_camera_info": info_meta,
                "body_to_optical": extrinsics, "errors": errors,
                "publishers": {topic: [{"type": p.topic_type,
                                         "reliability": p.qos_profile.reliability.name,
                                         "durability": p.qos_profile.durability.name}
                                        for p in node.get_publishers_info_by_topic(topic)]
                               for topic in (image_topic, info_topic, "/clock")},
                "clock_domain_note": "Gazebo /clock is the only clock source; UXRCE_DDS_SYNCT=0 preserves PX4 simulation timestamps. This observes common-domain stamps, not a capture-time pose interpolation implementation."}
    finally:
        node.destroy_node()
        rclpy.shutdown()
