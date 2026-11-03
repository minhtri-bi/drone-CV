"""Pinhole camera and full capture pose for the offline spike.

Frames: world ENU, body FLU, optical x-right/y-down/z-forward; depth is optical Z.
The default optical translation is body-relative (0.13233, 0, 0.02078 m),
including the merged x500_base model pose. Historical model-origin fixtures
must pass their old mount explicitly; they are not calibrated body-frame data.
"""

from dataclasses import dataclass
from math import atan, cos, degrees, isfinite, radians, sin, tan

import numpy as np
from numpy.typing import ArrayLike, NDArray

# body FLU -> optical: x_o = -y_b, y_o = -z_b, z_o = x_b
BODY_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
OPTICAL_TO_BODY = BODY_TO_OPTICAL.T


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if type(self.width) is not int or type(self.height) is not int:
            raise ValueError("image size must be integers")
        if self.width < 2 or self.height < 2:
            raise ValueError("image must be at least 2x2 pixels")
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(isfinite(v) for v in values) or self.fx <= 0 or self.fy <= 0:
            raise ValueError("intrinsics must be finite with positive focal lengths")

    @property
    def hfov_deg(self) -> float:
        return degrees(2 * atan((self.width / 2) / self.fx))

    @property
    def vfov_deg(self) -> float:
        return degrees(2 * atan((self.height / 2) / self.fy))

    @staticmethod
    def pinned_x500_depth(width: int = 160, height: int = 120) -> "Intrinsics":
        """Same FOV as the 640x480 Gazebo evidence, at a smaller resolution."""
        scale = width / 640
        if abs(height / 480 - scale) > 1e-12:
            raise ValueError("keep the 4:3 aspect ratio of the evidence camera")
        f = 432.496042035043 * scale
        return Intrinsics(width, height, f, f, width / 2, height / 2)

    @staticmethod
    def from_hfov(hfov_deg: float, width: int = 160, height: int = 120) -> "Intrinsics":
        f = (width / 2) / tan(radians(hfov_deg) / 2)
        return Intrinsics(width, height, f, f, width / 2, height / 2)


@dataclass(frozen=True)
class Mount:
    """Camera optical origin in body FLU and a downward pitch of the camera."""

    translation_flu: tuple[float, float, float] = (0.13233, 0.0, 0.02078)
    pitch_down_deg: float = 0.0

    def __post_init__(self) -> None:
        if len(self.translation_flu) != 3 or not all(isfinite(v) for v in self.translation_flu):
            raise ValueError("mount translation must be a finite 3-vector")
        if not isfinite(self.pitch_down_deg) or not -60 <= self.pitch_down_deg <= 60:
            raise ValueError("mount pitch must be finite and within +-60 degrees")

    def body_from_optical(self) -> NDArray[np.float64]:
        """Rotation mapping optical vectors into body FLU."""
        p = radians(self.pitch_down_deg)
        # Pitch down = rotate about body +y by +p (x forward tilts toward -z).
        pitch = np.array([[cos(p), 0, sin(p)], [0, 1, 0], [-sin(p), 0, cos(p)]])
        return pitch @ OPTICAL_TO_BODY


@dataclass(frozen=True)
class Pose:
    """Base-link pose in ENU; yaw-only construction remains valid for synthetic scenes."""

    x: float
    y: float
    z: float
    yaw_rad: float = 0.0
    rotation_enu_flu: tuple[tuple[float, ...], ...] | None = None

    def __post_init__(self) -> None:
        if not all(isfinite(v) for v in (self.x, self.y, self.z, self.yaw_rad)):
            raise ValueError("pose must be finite")
        if self.rotation_enu_flu is not None:
            rotation = np.asarray(self.rotation_enu_flu, dtype=np.float64)
            if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
                    or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8, rtol=0)):
                raise ValueError("capture rotation must be a proper orthonormal 3x3 matrix")
            object.__setattr__(self, "rotation_enu_flu",
                               tuple(tuple(float(v) for v in row) for row in rotation))

    @property
    def position(self) -> NDArray[np.float64]:
        return np.array([self.x, self.y, self.z])

    def world_from_body(self) -> NDArray[np.float64]:
        if self.rotation_enu_flu is not None:
            return np.array(self.rotation_enu_flu, dtype=np.float64)
        c, s = cos(self.yaw_rad), sin(self.yaw_rad)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@dataclass(frozen=True)
class CameraModel:
    intrinsics: Intrinsics
    mount: Mount = Mount()
    near_m: float = 0.2      # Gazebo clip/near of the pinned OakD-Lite SDF
    far_m: float = 19.1      # Gazebo clip/far of the pinned OakD-Lite SDF

    def __post_init__(self) -> None:
        if not (isfinite(self.near_m) and isfinite(self.far_m) and 0 < self.near_m < self.far_m):
            raise ValueError("clip range must satisfy 0 < near < far")

    def world_from_optical(self, pose: Pose) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return (rotation optical->world, camera origin in world)."""
        r_wb = pose.world_from_body()
        rotation = r_wb @ self.mount.body_from_optical()
        origin = pose.position + r_wb @ np.asarray(self.mount.translation_flu)
        return rotation, origin

    def to_optical(self, pose: Pose, points_world: ArrayLike) -> NDArray[np.float64]:
        rotation, origin = self.world_from_optical(pose)
        points = np.asarray(points_world, dtype=np.float64)
        return (points - origin) @ rotation

    def pixel_rays_world(self, pose: Pose) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Origin and per-pixel world directions scaled so optical z == 1.

        A point origin + t * direction then has optical Z depth exactly t.
        """
        k = self.intrinsics
        v, u = np.mgrid[0:k.height, 0:k.width].astype(np.float64)
        optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones_like(u)), axis=-1)
        rotation, origin = self.world_from_optical(pose)
        return origin, optical @ rotation.T
