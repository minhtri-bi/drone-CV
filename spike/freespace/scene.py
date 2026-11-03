"""Synthetic static world and a ray-cast depth renderer.

Ground truth lives here and is used ONLY to render depth images and to grade
decisions in tests. The free-space evidence code never imports this module.
"""

from dataclasses import dataclass, field
from math import isfinite

import numpy as np
from numpy.typing import NDArray

from camera import CameraModel, Pose


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in ENU."""

    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not all(isfinite(a) and isfinite(b) and a < b for a, b in zip(self.lo, self.hi)):
            raise ValueError("box must have finite lo < hi on every axis")


@dataclass(frozen=True)
class Cylinder:
    """Vertical cylinder (pole) in ENU."""

    cx: float
    cy: float
    radius: float
    z0: float
    z1: float

    def __post_init__(self) -> None:
        if not all(isfinite(v) for v in (self.cx, self.cy, self.radius, self.z0, self.z1)):
            raise ValueError("cylinder must be finite")
        if self.radius <= 0 or self.z0 >= self.z1:
            raise ValueError("cylinder needs radius > 0 and z0 < z1")


@dataclass(frozen=True)
class Scene:
    boxes: tuple[Box, ...] = ()
    cylinders: tuple[Cylinder, ...] = ()
    ground: bool = True
    walls: tuple[Box, ...] = field(default=())   # rendered like boxes; kept separate for plots

    def all_boxes(self) -> tuple[Box, ...]:
        return self.boxes + self.walls


def _ray_box(origin: NDArray, dirs: NDArray, box: Box) -> NDArray:
    """Slab test. dirs: (..., 3). Returns entry t (inf when missed or behind)."""
    lo, hi = np.asarray(box.lo), np.asarray(box.hi)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / dirs
        t1 = (lo - origin) * inv
        t2 = (hi - origin) * inv
    # Axis-parallel rays: inside the slab -> unbounded, outside -> miss.
    parallel = dirs == 0
    inside = (origin >= lo) & (origin <= hi)
    near_axis = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
    far_axis = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
    t_near = np.max(near_axis, axis=-1)
    t_far = np.min(far_axis, axis=-1)
    hit = (t_far >= t_near) & (t_far > 0)
    entry = np.where(t_near > 0, t_near, 0.0)   # origin inside box -> hit at 0
    return np.where(hit, entry, np.inf)


def _ray_cylinder(origin: NDArray, dirs: NDArray, cyl: Cylinder) -> NDArray:
    ox, oy, oz = origin[0] - cyl.cx, origin[1] - cyl.cy, origin[2]
    dx, dy, dz = dirs[..., 0], dirs[..., 1], dirs[..., 2]
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - cyl.radius ** 2
    best = np.full(dx.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        disc = b * b - 4 * a * c
        root = np.sqrt(np.where(disc >= 0, disc, np.nan))
        for t in ((-b - root) / (2 * a), (-b + root) / (2 * a)):
            z = oz + t * dz
            ok = np.isfinite(t) & (t > 0) & (z >= cyl.z0) & (z <= cyl.z1)
            best = np.where(ok & (t < best), t, best)
        for plane in (cyl.z0, cyl.z1):
            t = (plane - oz) / dz
            px, py = ox + t * dx, oy + t * dy
            ok = np.isfinite(t) & (t > 0) & (px * px + py * py <= cyl.radius ** 2)
            best = np.where(ok & (t < best), t, best)
    if c <= 0 and cyl.z0 <= oz <= cyl.z1:      # origin inside the pole
        best = np.zeros_like(best)
    return best


def render_depth(scene: Scene, camera: CameraModel, pose: Pose, *,
                 noise_seed: int | None = None, sigma0: float = 0.004,
                 sigma2: float = 0.0015) -> NDArray[np.float32]:
    """Gazebo-like 32FC1 optical-Z depth.

    +Inf beyond the far clip (no return), -Inf in front of the near clip.
    Optional noise sigma(Z) = sigma0 + sigma2 * Z^2 (stereo-like growth).
    """
    origin, dirs = camera.pixel_rays_world(pose)
    depth = np.full(dirs.shape[:2], np.inf)
    if scene.ground:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -origin[2] / dirs[..., 2]
        depth = np.where(np.isfinite(t) & (t > 0), t, depth)
    for box in scene.all_boxes():
        depth = np.minimum(depth, _ray_box(origin, dirs, box))
    for cyl in scene.cylinders:
        depth = np.minimum(depth, _ray_cylinder(origin, dirs, cyl))
    # Clip to the sensor range BEFORE noise: a grazing ground hit at hundreds of
    # metres must become +Inf, not a huge noise sample that crosses zero.
    depth = np.where(depth > camera.far_m, np.inf, depth)
    if noise_seed is not None:
        rng = np.random.default_rng(noise_seed)
        finite = np.isfinite(depth)
        noise = rng.normal(size=depth.shape) * (sigma0 + sigma2 * np.where(finite, depth, 0) ** 2)
        depth = np.where(finite, depth + noise, depth)
        depth = np.where(depth > camera.far_m, np.inf, depth)
    depth = np.where(depth < camera.near_m, -np.inf, depth)
    return depth.astype(np.float32)


# ---------------------------------------------------------------- grading only

def _segment_point_distance_2d(a: NDArray, b: NDArray, p: NDArray) -> float:
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom == 0 else float(np.clip((p - a) @ ab / denom, 0, 1))
    return float(np.linalg.norm(a + t * ab - p))


def _segments_intersect(p1, p2, q1, q2) -> bool:
    def orient(a, b, c):
        return np.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    o1, o2 = orient(p1, p2, q1), orient(p1, p2, q2)
    o3, o4 = orient(q1, q2, p1), orient(q1, q2, p2)
    return o1 != o2 and o3 != o4


def segment_rect_distance_2d(a: NDArray, b: NDArray, lo: NDArray, hi: NDArray) -> float:
    def inside(p):
        return bool(np.all(p >= lo) and np.all(p <= hi))
    if inside(a) or inside(b):
        return 0.0
    corners = [np.array(c) for c in ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    if any(_segments_intersect(a, b, c, d) for c, d in edges):
        return 0.0
    dists = [_segment_point_distance_2d(a, b, c) for c in corners]
    for p in (a, b):
        q = np.clip(p, lo, hi)
        dists.append(float(np.linalg.norm(p - q)))
    return min(dists)


def capsule_hits_scene(scene: Scene, start_xy, end_xy, radius: float,
                       z_lo: float, z_hi: float) -> bool:
    """Ground-truth: does the swept capsule (x z-band) touch any obstacle?"""
    a, b = np.asarray(start_xy, float), np.asarray(end_xy, float)
    if scene.ground and z_lo <= 0:
        return True
    for box in scene.all_boxes():
        if box.hi[2] < z_lo or box.lo[2] > z_hi:
            continue
        if segment_rect_distance_2d(a, b, np.asarray(box.lo[:2]), np.asarray(box.hi[:2])) <= radius:
            return True
    for cyl in scene.cylinders:
        if cyl.z1 < z_lo or cyl.z0 > z_hi:
            continue
        if _segment_point_distance_2d(a, b, np.array([cyl.cx, cyl.cy])) <= radius + cyl.radius:
            return True
    return False
