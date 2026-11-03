"""72x3 conservative free-path bounds under an explicit pixel-cone model.

Each radial segment encloses the swept circular footprint for the ENTIRE 5-degree
bin in a box. All intersected image pixels must support the box's furthest Z.
This deliberately over-blocks rather than creating gaps by depth reduction.
"""

from dataclasses import dataclass
from itertools import product
from math import ceil, cos, floor, isfinite, pi, sin

import numpy as np
from numpy.typing import NDArray

from drone_cv.depth import DepthFrame, Validity
from drone_cv.frames import vector3
from drone_cv.projection import Camera, Pose, project_depth, validate_view
from drone_cv.timing import Clock, FreshnessPolicy, Stamp, require_fresh

BINS = 72
BANDS = 3


@dataclass(frozen=True)
class HistogramConfig:
    radius_m: float
    half_height_m: float
    horizon_m: float
    step_m: float
    pixel_support_verified: bool = False

    def __post_init__(self) -> None:
        values = (self.radius_m, self.half_height_m, self.horizon_m, self.step_m)
        if not all(isfinite(v) and v > 0 for v in values):
            raise ValueError("footprint, horizon and radial step must be finite and positive")
        if self.step_m > self.horizon_m or ceil(self.horizon_m/self.step_m) > 1000:
            raise ValueError("radial step must give between 1 and 1000 segments")
        if type(self.pixel_support_verified) is not bool:
            raise ValueError("pixel support verification must be explicit boolean")


@dataclass(frozen=True)
class LocalClearance:
    """Independent free sphere, NOT inferred from a forward camera.

    G0 uses analytic synthetic truth. A future runtime must provide justified,
    fresh preflight/local-volume evidence; without it motion starts blocked.
    """

    centre_enu: tuple[float, float, float]
    radius_m: float
    stamp: Stamp

    def __post_init__(self) -> None:
        centre = tuple(float(v) for v in vector3(self.centre_enu, "clearance centre"))
        if not isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError("local clearance radius must be finite and positive")
        object.__setattr__(self, "centre_enu", centre)


@dataclass(frozen=True)
class Histogram:
    free_distance_m: NDArray[np.float64]
    reason: NDArray[np.str_]
    stamp: Stamp
    origin_enu: tuple[float, float, float]
    config: HistogramConfig

    def __post_init__(self) -> None:
        distance = np.array(self.free_distance_m, dtype=np.float64, copy=True)
        reason = np.array(self.reason, dtype="U24", copy=True)
        if distance.shape != (BINS, BANDS) or reason.shape != distance.shape:
            raise ValueError("histogram must have 72 directions and 3 height bands")
        if not np.isfinite(distance).all() or np.any(distance < 0):
            raise ValueError("free distance must be finite and nonnegative")
        if np.any(distance > self.config.horizon_m):
            raise ValueError("free distance exceeds horizon")
        distance.setflags(write=False)
        reason.setflags(write=False)
        object.__setattr__(self, "free_distance_m", distance)
        object.__setattr__(self, "reason", reason)
        object.__setattr__(self, "origin_enu", tuple(vector3(self.origin_enu, "origin")))


def _swept_box(start: float, end: float, angle: float, low_z: float,
               high_z: float, radius: float, origin: NDArray) -> NDArray:
    half_bin = pi / BINS
    along = (start*cos(half_bin)-radius, end+radius)
    side = end*sin(half_bin)+radius
    local = np.array(list(product(along, (-side, side), (low_z, high_z))))
    x, y = local[:, 0], local[:, 1]
    return np.column_stack((x*cos(angle)-y*sin(angle),
                            x*sin(angle)+y*cos(angle), local[:, 2])) + origin


def _box_evidence(corners: NDArray, frame: DepthFrame, camera: Camera,
                  pose: Pose, verified: bool) -> str:
    if not verified:
        return "UNVERIFIED_SUPPORT"
    optical = (corners - pose.position_enu) @ pose.rotation
    if np.any(optical[:, 2] <= 0):
        return "OUT_OF_VIEW"
    u = camera.fx*optical[:, 0]/optical[:, 2] + camera.cx
    v = camera.fy*optical[:, 1]/optical[:, 2] + camera.cy
    if (u.min() < -0.5 or u.max() > camera.width-0.5
            or v.min() < -0.5 or v.max() > camera.height-0.5):
        return "OUT_OF_VIEW"
    # Include pixels on both sides of a numerical pixel-cell boundary.
    x0, x1 = max(0, floor(u.min()+0.5-1e-12)), min(camera.width, ceil(u.max()+0.5+1e-12))
    y0, y1 = max(0, floor(v.min()+0.5-1e-12)), min(camera.height, ceil(v.max()+0.5+1e-12))
    states = frame.validity[y0:y1, x0:x1]
    depths = frame.metres[y0:y1, x0:x1]
    known = (states == Validity.VALID) | (states == Validity.NO_RETURN)
    if np.any(states == Validity.TOO_CLOSE):
        return "TOO_CLOSE"
    if np.any(known & (depths <= optical[:, 2].max())):
        return "OBSTACLE_OR_RANGE"
    if states.size == 0 or np.any(~known):
        return "UNKNOWN"
    return "CLEAR"


def _validate_certificate(clearance: LocalClearance, frame: DepthFrame,
                          camera: Camera, pose: Pose, now: Clock,
                          policy: FreshnessPolicy, skew: float) -> None:
    require_fresh(clearance.stamp, now, policy)
    if clearance.stamp != frame.stamp:
        raise ValueError("local clearance must be certified for this exact frame")
    points = project_depth(frame, camera, pose, now, policy, max_pose_skew_s=skew)
    if np.any(np.linalg.norm(points-clearance.centre_enu, axis=1) <= clearance.radius_m):
        raise ValueError("local clearance conflicts with observed obstacle")


def build_histogram(
    frame: DepthFrame, camera: Camera, pose: Pose,
    origin_enu: tuple[float, float, float], now: Clock, policy: FreshnessPolicy,
    config: HistogramConfig, *, local_clearance: LocalClearance | None = None,
    max_pose_skew_s: float,
) -> Histogram:
    validate_view(frame, camera, pose, now, policy, max_pose_skew_s)
    origin = vector3(origin_enu, "histogram origin")
    if local_clearance is not None:
        _validate_certificate(local_clearance, frame, camera, pose, now,
                              policy, max_pose_skew_s)
    distances = np.zeros((BINS, BANDS))
    reasons = np.full((BINS, BANDS), "TOO_CLOSE", dtype="U24")
    # A below-minimum return cannot be localized reliably. Stop all translation.
    if np.any(frame.validity == Validity.TOO_CLOSE):
        return Histogram(distances, reasons, frame.stamp, tuple(origin), config)
    heights = np.linspace(-config.half_height_m, config.half_height_m, BANDS+1)
    ends = np.minimum(np.arange(1, ceil(config.horizon_m/config.step_m)+1)
                      * config.step_m, config.horizon_m)
    for direction, band in product(range(BINS), range(BANDS)):
        start = 0.0
        for end in ends:
            corners = _swept_box(start, end, direction*2*pi/BINS,
                                 heights[band], heights[band+1], config.radius_m, origin)
            certified = local_clearance is not None and np.all(
                np.linalg.norm(corners-local_clearance.centre_enu, axis=1)
                <= local_clearance.radius_m)
            reason = "CLEAR" if certified else _box_evidence(
                corners, frame, camera, pose, config.pixel_support_verified)
            reasons[direction, band] = reason
            if reason != "CLEAR":
                break
            distances[direction, band] = end
            start = float(end)
    return Histogram(distances, reasons, frame.stamp, tuple(origin), config)
