"""Pure ROS-message boundary helpers; no ROS runtime imports or control output."""

from bisect import bisect_left
from dataclasses import dataclass, field
from math import atan2

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from drone_cv.px4_quaternion import normalize_px4_quaternion

OPTICAL_FRAME = "drone_cv_g1/camera_optical"
ENU_FROM_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_FROM_FLU = np.diag([1., -1., -1.])


def unit_quaternion(value):
    return np.asarray(normalize_px4_quaternion(value).wxyz, dtype=np.float64)


def quaternion_rotation(value):
    w, x, y, z = unit_quaternion(value)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass(frozen=True)
class OdomSample:
    timestamp_ns: int
    position_ned: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    reset_counter: int
    pose_frame: int = 1
    quaternion_squared_norm_error: float = field(init=False)

    def __post_init__(self):
        if type(self.timestamp_ns) is not int or self.timestamp_ns < 0:
            raise ValueError("INVALID_SAMPLE_TIME")
        if self.pose_frame != 1:
            raise ValueError("UNSUPPORTED_POSE_FRAME")
        position = np.asarray(self.position_ned, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("INVALID_POSITION")
        if type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255:
            raise ValueError("INVALID_RESET_COUNTER")
        object.__setattr__(self, "position_ned", tuple(float(x) for x in position))
        normalized = normalize_px4_quaternion(self.quaternion_wxyz)
        object.__setattr__(self, "quaternion_wxyz", normalized.wxyz)
        object.__setattr__(self, "quaternion_squared_norm_error", normalized.squared_norm_error)


def interpolate_pose(samples, capture_ns, max_gap_ns=50_000_000):
    if type(capture_ns) is not int or capture_ns < 0 or max_gap_ns <= 0:
        raise ValueError("INVALID_CAPTURE_TIME_OR_GAP")
    stamps = [sample.timestamp_ns for sample in samples]
    if not stamps or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("NONMONOTONIC_ODOMETRY")
    index = bisect_left(stamps, capture_ns)
    if index < len(stamps) and stamps[index] == capture_ns:
        left = right = samples[index]
        fraction = 0.0
    elif index == 0 or index == len(stamps):
        raise ValueError("UNBRACKETED_POSE")
    else:
        left, right = samples[index-1:index+1]
        if right.timestamp_ns-left.timestamp_ns > max_gap_ns:
            raise ValueError("POSE_GAP")
        fraction = (capture_ns-left.timestamp_ns)/(right.timestamp_ns-left.timestamp_ns)
    if left.reset_counter != right.reset_counter:
        raise ValueError("POSE_RESET")
    q0, q1 = np.array(left.quaternion_wxyz), np.array(right.quaternion_wxyz)
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    dot = float(np.clip(dot, -1, 1))
    if dot > .9995:
        q = q0 + fraction*(q1-q0)
        q = q/np.linalg.norm(q)
    else:
        angle = np.arccos(dot)
        q = (np.sin((1-fraction)*angle)*q0 + np.sin(fraction*angle)*q1)/np.sin(angle)
    rotation = ENU_FROM_NED @ quaternion_rotation(q) @ FRD_FROM_FLU
    position = ENU_FROM_NED @ ((1-fraction)*np.array(left.position_ned)
                               + fraction*np.array(right.position_ned))
    return Pose(*position, atan2(rotation[1, 0], rotation[0, 0]), rotation)


def decode_depth(message):
    if message.encoding != "32FC1" or message.header.frame_id != OPTICAL_FRAME:
        raise ValueError("DEPTH_ENCODING_OR_FRAME")
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width*4 or step % 4:
        raise ValueError("DEPTH_DIMENSIONS_OR_STRIDE")
    payload = bytes(message.data)
    if len(payload) != height*step:
        raise ValueError("DEPTH_BYTE_COUNT")
    depth = np.ndarray((height, width), dtype=">f4" if message.is_bigendian else "<f4",
                       buffer=payload, strides=(step, 4)).astype(np.float32, copy=True)
    depth.setflags(write=False)
    return depth


def camera_from_info(message, mount=None):
    k = np.asarray(message.k, dtype=np.float64)
    if (message.header.frame_id != OPTICAL_FRAME or k.shape != (9,)
            or message.distortion_model not in ("", "plumb_bob")
            or not np.isfinite(k).all() or not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1], atol=1e-12)
            or any(not np.isfinite(d) or d != 0 for d in message.d)
            or message.binning_x not in (0, 1) or message.binning_y not in (0, 1)
            or message.roi.x_offset or message.roi.y_offset
            or message.roi.width not in (0, message.width)
            or message.roi.height not in (0, message.height)):
        raise ValueError("UNSUPPORTED_CAMERA_INFO")
    return CameraModel(Intrinsics(message.width, message.height, k[0], k[4], k[2], k[5]),
                       mount or Mount())
