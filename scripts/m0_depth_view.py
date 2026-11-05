"""Capture one ROS depth frame in an existing container and open its PNG on Windows."""

import argparse
import os
from pathlib import Path
import subprocess
import uuid


TOPIC = "/drone_cv_g1/camera/depth/image_raw"
ENTRYPOINT = "/usr/local/bin/ros2-entrypoint.sh"


def capture(output: Path) -> None:
    """Read one 32FC1 optical-Z image; magenta marks invalid pixels in the preview."""
    import numpy as np
    from PIL import Image as Preview
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.task import Future
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node(
        "m0_depth_view", parameter_overrides=[Parameter("use_sim_time", value=True)]
    )
    frame = Future()
    subscription = node.create_subscription(
        Image, TOPIC, lambda message: frame.set_result(message), qos_profile_sensor_data
    )
    try:
        # Acquisition wait only, not a sensor-freshness or flight watchdog.
        rclpy.spin_until_future_complete(node, frame, timeout_sec=30.0)
        if not frame.done():
            raise TimeoutError(f"No depth image received from {TOPIC} in 30 wall seconds")
        message = frame.result()
        if (message.encoding != "32FC1" or message.width <= 0 or message.height <= 0
                or message.step < message.width * 4
                or len(message.data) != message.step * message.height):
            raise ValueError("Expected a complete 32FC1 depth image")
        pixels = np.ndarray(
            (message.height, message.width),
            dtype=">f4" if message.is_bigendian else "<f4",
            buffer=bytes(message.data), strides=(message.step, 4),
        )
        valid = np.isfinite(pixels) & (pixels > 0)
        values = pixels[valid]
        minimum = float(values.min()) if values.size else None
        maximum = float(values.max()) if values.size else None
        print(f"size={message.width}x{message.height} encoding={message.encoding} "
              f"min_m={minimum} max_m={maximum} invalid={int((~valid).sum())} "
              f"total={pixels.size}", flush=True)
        # Scale only for viewing; the printed values above remain metric optical Z.
        scale = (max(maximum - minimum, 1e-6)
                 if minimum is not None and maximum is not None else 1.0)
        gray = np.clip((np.where(valid, pixels, minimum or 0) - (minimum or 0))
                       / scale * 255, 0, 255).astype(np.uint8)
        rgb = np.where(valid[..., None], gray[..., None], [255, 0, 255]).astype(np.uint8)
        with output.open("xb") as stream:
            Preview.fromarray(rgb).save(stream, format="PNG")
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", help="Existing, running pinned simulator container")
    parser.add_argument("--output", type=Path, required=True, help="New PNG path; never overwritten")
    parser.add_argument("--capture", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.capture:
        capture(args.output)
        return
    if not args.container or os.name != "nt":
        parser.error("Host mode requires Windows and --container; ROS capture runs inside it")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    remote = f"/tmp/m0_depth_{uuid.uuid4().hex}"
    subprocess.run(["docker", "cp", str(Path(__file__).resolve()),
                    f"{args.container}:{remote}.py"], check=True, timeout=30)
    subprocess.run(["docker", "exec", args.container, ENTRYPOINT, "python3",
                    f"{remote}.py", "--capture", "--output", f"{remote}.png"],
                   check=True, timeout=60)
    subprocess.run(["docker", "cp", f"{args.container}:{remote}.png", str(output)],
                   check=True, timeout=30)
    os.startfile(str(output))
    print(f"Opened depth preview: {output}")


if __name__ == "__main__":
    main()
