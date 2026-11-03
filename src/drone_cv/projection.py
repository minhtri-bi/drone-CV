"""Rectified pinhole Z-depth projection using a capture-time camera pose."""

from dataclasses import dataclass
from math import isfinite

import numpy as np
from numpy.typing import ArrayLike, NDArray

from drone_cv.depth import DepthFrame, Validity
from drone_cv.frames import vector3
from drone_cv.timing import Clock, FreshnessPolicy, Stamp, require_fresh


def positive_int(value: int, name: str) -> None:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        positive_int(self.width, "width")
        positive_int(self.height, "height")
        if not all(isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("focal lengths must be positive")

    def crop_resize(self, *, left: int, top: int, width: int, height: int,
                    output_width: int, output_height: int) -> "Camera":
        for name, value in (("width", width), ("height", height),
                            ("output_width", output_width), ("output_height", output_height)):
            positive_int(value, name)
        if type(left) is not int or type(top) is not int:
            raise ValueError("crop offsets must be integers")
        if left < 0 or top < 0 or left+width > self.width or top+height > self.height:
            raise ValueError("crop outside image")
        sx, sy = output_width / width, output_height / height
        # Pixel-centre convention for ordinary half-pixel resize.
        return Camera(output_width, output_height, self.fx*sx, self.fy*sy,
                      (self.cx-left+0.5)*sx-0.5, (self.cy-top+0.5)*sy-0.5)


@dataclass(frozen=True)
class Pose:
    """Rotation maps optical vectors to ENU; position is optical origin in ENU."""

    rotation: ArrayLike
    position_enu: tuple[float, float, float]
    stamp: Stamp

    def __post_init__(self) -> None:
        rotation = np.array(self.rotation, dtype=np.float64, copy=True)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-8, rtol=0)):
            raise ValueError("rotation must be a proper orthonormal 3x3 matrix")
        position = tuple(float(v) for v in vector3(self.position_enu, "camera position"))
        rotation.setflags(write=False)
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "position_enu", position)


def validate_view(frame: DepthFrame, camera: Camera, pose: Pose, now: Clock,
                  policy: FreshnessPolicy, max_pose_skew_s: float) -> None:
    if frame.metres.shape != (camera.height, camera.width):
        raise ValueError("depth shape does not match CameraInfo")
    if not isfinite(max_pose_skew_s) or max_pose_skew_s < 0:
        raise ValueError("pose synchronization tolerance must be finite and nonnegative")
    require_fresh(frame.stamp, now, policy)
    require_fresh(pose.stamp, now, policy)
    if abs(frame.stamp.sim_s - pose.stamp.sim_s) > max_pose_skew_s:
        raise ValueError("pose is not synchronized to depth capture")


def project_depth(frame: DepthFrame, camera: Camera, pose: Pose, now: Clock,
                  policy: FreshnessPolicy, *, max_pose_skew_s: float) -> NDArray[np.float64]:
    """Return only physical obstacle returns; NO_RETURN is not a 3D obstacle."""
    validate_view(frame, camera, pose, now, policy, max_pose_skew_s)
    rows, cols = np.nonzero(frame.validity == Validity.VALID)
    z = frame.metres[rows, cols]
    optical = np.column_stack(((cols-camera.cx)*z/camera.fx,
                               (rows-camera.cy)*z/camera.fy, z))
    world = optical @ pose.rotation.T + pose.position_enu
    world.setflags(write=False)
    return world
