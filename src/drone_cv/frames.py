"""Right-handed ROS/PX4 basis changes. Quaternions are explicitly w,x,y,z."""

from math import isfinite, pi

import numpy as np
from numpy.typing import ArrayLike, NDArray

OPTICAL_TO_FLU = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
OPTICAL_TO_FLU.setflags(write=False)


def vector3(value: ArrayLike, name: str) -> NDArray[np.float64]:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite 3-vector")
    return vector


def enu_to_ned(vector: ArrayLike) -> NDArray[np.float64]:
    x, y, z = vector3(vector, "ENU vector")
    return np.array([y, x, -z])


def flu_to_frd(vector: ArrayLike) -> NDArray[np.float64]:
    x, y, z = vector3(vector, "FLU vector")
    return np.array([x, -y, -z])


def rotation_wxyz(quaternion: ArrayLike) -> NDArray[np.float64]:
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must contain finite w,x,y,z")
    squared_norm = float(q @ q)
    if abs(squared_norm - 1.0) > 1e-8:
        raise ValueError("quaternion must be unit length within 1e-8 squared norm")
    # Remove only roundoff accepted by the unit-norm check, not malformed inputs.
    w, x, y, z = q / np.sqrt(squared_norm)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def orientation_enu_flu_to_ned_frd(quaternion_wxyz: ArrayLike) -> NDArray[np.float64]:
    """Convert an FLU-to-ENU orientation into an FRD-to-NED rotation."""
    ned_from_enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    flu_from_frd = np.diag([1, -1, -1])
    return ned_from_enu @ rotation_wxyz(quaternion_wxyz) @ flu_from_frd


def wrap_angle(angle_rad: float) -> float:
    if not isfinite(angle_rad):
        raise ValueError("angle must be finite")
    wrapped = (angle_rad + pi) % (2*pi) - pi
    return pi if wrapped == -pi else wrapped
