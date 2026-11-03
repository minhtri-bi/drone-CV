"""Generate frame sequences along synthetic trajectories (rendering uses ground truth)."""

from math import cos, pi, sin

import numpy as np

from camera import CameraModel, Pose
from evidence import EvidenceStore, Frame, SensorProfile
from scene import Scene, render_depth


def capture(scene: Scene, camera: CameraModel, profile: SensorProfile, poses: list[Pose], *,
            epoch: str = "run", t0: float = 0.0, dt: float = 0.1, noise_seed: int | None = 7,
            store: EvidenceStore | None = None,
            pose_noise: tuple[float, float] | None = None) -> tuple[EvidenceStore, float]:
    """Render one frame per pose; returns the store and the time of the last frame.

    pose_noise=(sigma_xyz_m, sigma_yaw_rad): depth is rendered at the TRUE pose but
    the frame records a perturbed pose, like odometry error.
    """
    store = store or EvidenceStore(camera, profile)
    start = len(store.frames)
    rng = np.random.default_rng(None if noise_seed is None else noise_seed + 10_000)
    t = t0
    for i, pose in enumerate(poses):
        seed = None if noise_seed is None else noise_seed + start + i
        depth = render_depth(scene, camera, pose, noise_seed=seed,
                             sigma0=profile.sigma0, sigma2=profile.sigma2)
        recorded = pose
        if pose_noise is not None:
            dp = rng.normal(scale=pose_noise[0], size=3)
            recorded = Pose(pose.x + dp[0], pose.y + dp[1], pose.z + dp[2],
                            pose.yaw_rad + rng.normal(scale=pose_noise[1]))
        store = store.add(Frame(depth, recorded, epoch, t, start + i))
        t += dt
    return store, t - dt


def straight(a: Pose, b: Pose, step_m: float = 0.1) -> list[Pose]:
    """Constant-yaw straight line from a to b (both included)."""
    d = float(np.linalg.norm(b.position - a.position))
    n = max(1, int(np.ceil(d / step_m)))
    return [Pose(*(a.position + (b.position - a.position) * k / n), a.yaw_rad) for k in range(n + 1)]


def climb(x: float, y: float, z0: float, z1: float, yaw: float = 0.0, step_m: float = 0.1) -> list[Pose]:
    n = max(1, int(np.ceil(abs(z1 - z0) / step_m)))
    return [Pose(x, y, z0 + (z1 - z0) * k / n, yaw) for k in range(n + 1)]


def yaw_scan(x: float, y: float, z: float, yaw0: float = 0.0, step_deg: float = 10.0) -> list[Pose]:
    n = int(round(360 / step_deg))
    return [Pose(x, y, z, yaw0 + 2 * pi * k / n) for k in range(n + 1)]


def ahead(pose: Pose, distance: float) -> Pose:
    return Pose(pose.x + distance * cos(pose.yaw_rad), pose.y + distance * sin(pose.yaw_rad),
                pose.z, pose.yaw_rad)
