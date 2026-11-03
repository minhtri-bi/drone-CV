"""Accumulate observed free space from depth frames and query a swept envelope.

Design (offline spike, static world):
- Evidence is a finite set of frames, each with its capture pose, epoch and time.
  No global map: a query point is transformed into every retained frame.
- A point is FREE only if some frame saw past it: the point projects inside the
  image, its depth is inside the trusted range, and ALL four pixel rays around
  its projection returned a known depth beyond the point plus a noise margin.
- Any observed surface point (from any retained frame) inside the envelope
  blocks motion, so a new near obstacle always beats older free evidence.
- Unknown, too-close, out-of-view or occluded points are never free.

Ground truth is not used here.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from math import ceil, cos, isfinite, sin

import numpy as np
from numpy.typing import NDArray

from camera import CameraModel, Pose


class Pixel(IntEnum):
    UNKNOWN = 0
    VALID = 1
    TOO_CLOSE = 2
    NO_RETURN = 3


class Status(IntEnum):
    FREE = 0
    OBSTACLE = 1
    OCCLUDED = 2      # inside some frustum, but never supported as free
    UNSEEN = 3        # never inside any retained frame's trusted frustum


@dataclass(frozen=True)
class SensorProfile:
    """Trusted sensing contract. Values are spike defaults, not calibration."""

    near_trusted_m: float = 0.3
    far_trusted_m: float = 10.0
    no_return_certified: bool = True   # synthetic renderer: +Inf means nothing within far clip
    sigma0: float = 0.004
    sigma2: float = 0.0015
    sigma_k: float = 3.0
    fixed_margin_m: float = 0.03

    def __post_init__(self) -> None:
        values = (self.near_trusted_m, self.far_trusted_m, self.sigma0, self.sigma2,
                  self.sigma_k, self.fixed_margin_m)
        if not all(isfinite(v) and v >= 0 for v in values):
            raise ValueError("sensor profile values must be finite and nonnegative")
        if not 0 < self.near_trusted_m < self.far_trusted_m:
            raise ValueError("trusted range must satisfy 0 < near < far")
        if type(self.no_return_certified) is not bool:
            raise ValueError("no-return certification must be an explicit boolean")

    def margin(self, z: NDArray) -> NDArray:
        return self.fixed_margin_m + self.sigma_k * (self.sigma0 + self.sigma2 * z * z)

    def classify(self, depth: NDArray) -> NDArray[np.uint8]:
        states = np.full(depth.shape, Pixel.UNKNOWN, dtype=np.uint8)
        finite = np.isfinite(depth)
        states[finite & (depth >= self.near_trusted_m)] = Pixel.VALID
        states[finite & (depth > 0) & (depth < self.near_trusted_m)] = Pixel.TOO_CLOSE
        states[np.isneginf(depth)] = Pixel.TOO_CLOSE
        if self.no_return_certified:
            states[np.isposinf(depth)] = Pixel.NO_RETURN
        return states


@dataclass(frozen=True)
class Frame:
    depth: NDArray[np.float32]
    pose: Pose
    epoch: str
    time_s: float
    sequence: int

    def __post_init__(self) -> None:
        depth = np.array(self.depth, dtype=np.float32, copy=True)
        if depth.ndim != 2 or depth.size == 0:
            raise ValueError("depth must be a nonempty 2D array")
        if not isinstance(self.epoch, str) or not self.epoch:
            raise ValueError("epoch must be a nonempty string")
        if not isfinite(self.time_s) or self.time_s < 0:
            raise ValueError("time must be finite and nonnegative")
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        depth.setflags(write=False)
        object.__setattr__(self, "depth", depth)


@dataclass(frozen=True)
class Envelope:
    """Swept vertical capsule: segment in xy, radius, and a z band (ENU)."""

    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    radius_m: float
    z_lo: float
    z_hi: float

    def __post_init__(self) -> None:
        values = (*self.start_xy, *self.end_xy, self.radius_m, self.z_lo, self.z_hi)
        if not all(isfinite(v) for v in values):
            raise ValueError("envelope must be finite")
        if self.radius_m <= 0 or self.z_lo >= self.z_hi:
            raise ValueError("envelope needs radius > 0 and z_lo < z_hi")

    def distance_xy(self, points: NDArray) -> NDArray:
        a, b = np.asarray(self.start_xy), np.asarray(self.end_xy)
        ab = b - a
        denom = float(ab @ ab)
        rel = points[:, :2] - a
        t = np.zeros(len(points)) if denom == 0 else np.clip(rel @ ab / denom, 0, 1)
        return np.linalg.norm(rel - t[:, None] * ab, axis=1)

    def contains(self, points: NDArray, inflate_m: float = 0.0) -> NDArray[np.bool_]:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        z = points[:, 2]
        return ((self.distance_xy(points) <= self.radius_m + inflate_m)
                & (z >= self.z_lo - inflate_m) & (z <= self.z_hi + inflate_m))

    def sample(self, spacing_m: float) -> NDArray[np.float64]:
        """Grid points inside the capsule, not a cover of its continuous boundary."""
        if not isfinite(spacing_m) or spacing_m <= 0:
            raise ValueError("spacing must be finite and positive")
        a, b, r = np.asarray(self.start_xy), np.asarray(self.end_xy), self.radius_m
        lo, hi = np.minimum(a, b) - r, np.maximum(a, b) + r
        axes = [np.linspace(lo[i], hi[i], max(2, ceil((hi[i] - lo[i]) / spacing_m) + 1))
                for i in range(2)]
        zs = np.linspace(self.z_lo, self.z_hi, max(2, ceil((self.z_hi - self.z_lo) / spacing_m) + 1))
        gx, gy, gz = np.meshgrid(axes[0], axes[1], zs, indexing="ij")
        points = np.column_stack((gx.ravel(), gy.ravel(), gz.ravel()))
        # Keep boundary samples despite float roundoff; boundary must be checked too.
        return points[self.contains(points, inflate_m=1e-9)]


def swept_envelope(pose: Pose, distance_m: float, *, body_radius_m: float = 0.35,
                   safety_margin_m: float = 0.10, pose_error_m: float = 0.05,
                   half_height_m: float = 0.20) -> Envelope:
    """Forward straight motion along current yaw, including the current footprint."""
    if not isfinite(distance_m) or distance_m < 0:
        raise ValueError("distance must be finite and nonnegative")
    end = (pose.x + distance_m * cos(pose.yaw_rad), pose.y + distance_m * sin(pose.yaw_rad))
    return Envelope((pose.x, pose.y), end, body_radius_m + safety_margin_m + pose_error_m,
                    pose.z - half_height_m, pose.z + half_height_m)


@dataclass(frozen=True)
class EvidenceStore:
    camera: CameraModel
    profile: SensorProfile
    frames: tuple[Frame, ...] = ()

    def __post_init__(self) -> None:
        if (self.profile.near_trusted_m < self.camera.near_m
                or self.profile.far_trusted_m > self.camera.far_m):
            raise ValueError("trusted range must lie within the camera clip range")

    def add(self, frame: Frame) -> "EvidenceStore":
        if self.frames and frame.epoch == self.frames[-1].epoch and frame.sequence <= self.frames[-1].sequence:
            raise ValueError("frame sequence must increase within an epoch")
        return EvidenceStore(self.camera, self.profile, self.frames + (frame,))

    def retained(self, epoch: str, now_s: float, ttl_s: float) -> tuple[Frame, ...]:
        """Same epoch, not from the future, not older than ttl."""
        if not isfinite(ttl_s) or ttl_s <= 0:
            raise ValueError("ttl must be finite and positive")
        return tuple(f for f in self.frames
                     if f.epoch == epoch and f.time_s <= now_s and now_s - f.time_s <= ttl_s)


def _frame_support(frame: Frame, camera: CameraModel, profile: SensorProfile,
                   points: NDArray) -> tuple[NDArray, NDArray, NDArray]:
    """Per point: (in trusted frustum, certified free, hidden behind a nearer return)."""
    k = camera.intrinsics
    optical = camera.to_optical(frame.pose, points)
    z = optical[:, 2]
    margin = profile.margin(z)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = k.fx * optical[:, 0] / z + k.cx
        v = k.fy * optical[:, 1] / z + k.cy
    in_view = ((z >= profile.near_trusted_m) & (z + margin <= profile.far_trusted_m)
               & (u >= 0) & (u <= k.width - 1) & (v >= 0) & (v <= k.height - 1))
    free = np.zeros(len(points), dtype=bool)
    hidden = np.zeros(len(points), dtype=bool)
    if not in_view.any():
        return in_view, free, hidden
    idx = np.nonzero(in_view)[0]
    u0 = np.clip(np.floor(u[idx]).astype(int), 0, k.width - 2)
    v0 = np.clip(np.floor(v[idx]).astype(int), 0, k.height - 2)
    states = profile.classify(frame.depth)
    depth = frame.depth.astype(np.float64)
    ok = np.ones(len(idx), dtype=bool)
    nearest = np.full(len(idx), np.inf)
    nearest_valid = np.full(len(idx), np.inf)
    for du in (0, 1):
        for dv in (0, 1):
            s = states[v0 + dv, u0 + du]
            d = np.where(s == Pixel.NO_RETURN, profile.far_trusted_m, depth[v0 + dv, u0 + du])
            ok &= (s == Pixel.VALID) | (s == Pixel.NO_RETURN)
            nearest = np.minimum(nearest, np.where(ok, d, -np.inf))
            nearest_valid = np.minimum(nearest_valid, np.where(s == Pixel.VALID, d, np.inf))
    free[idx] = ok & (nearest >= z[idx] + margin[idx])
    hidden[idx] = nearest_valid < z[idx] - margin[idx]
    return in_view, free, hidden


def observed_surface_points(frame: Frame, camera: CameraModel, profile: SensorProfile) -> NDArray:
    """World points of every trusted VALID return in a frame."""
    states = profile.classify(frame.depth)
    origin, dirs = camera.pixel_rays_world(frame.pose)
    mask = (states == Pixel.VALID) & (frame.depth <= profile.far_trusted_m)
    return origin + dirs[mask] * frame.depth[mask].astype(np.float64)[:, None]


@dataclass(frozen=True)
class Decision:
    allowed: bool
    reason: str
    counts: dict = field(default_factory=dict)
    sample_points: int = 0
    frames_used: int = 0
    obstacle_returns: int = 0


def classify_points(points: NDArray, frames: tuple[Frame, ...], camera: CameraModel,
                    profile: SensorProfile) -> NDArray[np.uint8]:
    seen = np.zeros(len(points), dtype=bool)
    free = np.zeros(len(points), dtype=bool)
    for frame in frames:
        in_view, certified, _ = _frame_support(frame, camera, profile, points)
        seen |= in_view
        free |= certified
    status = np.full(len(points), Status.UNSEEN, dtype=np.uint8)
    status[seen] = Status.OCCLUDED
    status[free] = Status.FREE
    return status


def _counts(status: NDArray) -> dict:
    return {s.name: int((status == s).sum()) for s in Status}


def decide(store: EvidenceStore, envelope: Envelope, *, epoch: str, now_s: float,
           ttl_s: float = 10.0, spacing_m: float = 0.05) -> tuple[Decision, NDArray, NDArray]:
    """Allow the swept motion only if every envelope sample is FREE and no observed
    surface lies inside the (margin-inflated) envelope. Returns decision, samples, status."""
    frames = store.retained(epoch, now_s, ttl_s)
    samples = envelope.sample(spacing_m)
    if not frames:
        status = np.full(len(samples), Status.UNSEEN, dtype=np.uint8)
        return Decision(False, "NO_FRESH_FRAMES", _counts(status), len(samples)), samples, status
    if len(samples) == 0:
        status = np.empty(0, dtype=np.uint8)
        return Decision(False, "NO_ENVELOPE_SAMPLES", _counts(status),
                        frames_used=len(frames)), samples, status
    latest = max(frames, key=lambda f: (f.time_s, f.sequence))
    # A dropout cannot clear an unlocalized near hazard. Keep it for the same
    # retained epoch/TTL window as the free evidence; expiry alone is not motion permission.
    if any(np.any(store.profile.classify(frame.depth) == Pixel.TOO_CLOSE) for frame in frames):
        status = np.full(len(samples), Status.OCCLUDED, dtype=np.uint8)
        return Decision(False, "TOO_CLOSE", _counts(status), len(samples), len(frames)), samples, status
    status = classify_points(samples, frames, store.camera, store.profile)
    hits = [s[envelope.contains(s, inflate_m=store.profile.fixed_margin_m)]
            for s in (observed_surface_points(f, store.camera, store.profile) for f in frames)]
    hits = np.concatenate(hits) if hits else np.empty((0, 3))
    obstacle_returns = len(hits)
    if obstacle_returns:
        # A return inside the envelope means the scene there may have changed:
        # older free evidence for points the latest frame sees hidden behind a
        # nearer return no longer counts.
        _, _, hidden = _frame_support(latest, store.camera, store.profile, samples)
        status[(status == Status.FREE) & hidden] = Status.OCCLUDED
        # Audit/plot: samples within one grid step of an observed return are OBSTACLE.
        hits = np.unique(np.round(hits / spacing_m), axis=0) * spacing_m
        for chunk in np.array_split(np.arange(len(samples)), max(1, len(samples) // 2000)):
            d = np.linalg.norm(samples[chunk, None, :] - hits[None, :, :], axis=2).min(axis=1)
            status[chunk[d <= spacing_m]] = Status.OBSTACLE
    counts = _counts(status)
    if obstacle_returns:
        reason = "OBSTACLE_IN_ENVELOPE"
    elif counts["UNSEEN"]:
        reason = "UNSEEN_VOLUME"
    elif counts["OCCLUDED"]:
        reason = "UNSUPPORTED_VOLUME"
    else:
        reason = "ALLOWED"
    decision = Decision(reason == "ALLOWED", reason, counts, len(samples), len(frames), obstacle_returns)
    return decision, samples, status
