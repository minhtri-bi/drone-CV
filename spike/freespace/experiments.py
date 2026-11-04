"""Offline experiments: safety sweep against ground truth, and the takeoff blind zone.

Usage: python experiments.py --profile complete-oracle-profile.json --output out/
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from capture import capture, climb, straight, yaw_scan
from evidence import SensorProfile, decide, swept_envelope
from scene import Box, Cylinder, Scene, capsule_hits_scene

WALLS = (Box((-4, 2.5, 0), (12, 2.7, 3)), Box((-4, -2.7, 0), (12, -2.5, 3)))
B = Pose(2.0, 0.0, 1.2)
QUERY_M = 1.0

CAMERAS = {
    "pinned_73deg": CameraModel(Intrinsics.pinned_x500_depth()),
    "wide_110deg": CameraModel(Intrinsics.from_hfov(110.0)),
    "pinned_pitch20": CameraModel(Intrinsics.pinned_x500_depth(), Mount(pitch_down_deg=20.0)),
}


def hard_scene(rng, env) -> Scene:
    """Obstacles concentrated around the envelope boundary, many of them thin."""
    boxes, poles = [], []
    for _ in range(rng.integers(1, 5)):
        side = rng.choice([-1, 1])
        y = side * (env.radius_m + rng.uniform(-0.3, 0.3))
        x = rng.uniform(env.start_xy[0] - 0.2, env.end_xy[0] + env.radius_m + 0.5)
        r = rng.uniform(0.01, 0.12)
        poles.append(Cylinder(x, y, r, 0.0, rng.uniform(0.9, 3.0)))
    for _ in range(rng.integers(0, 3)):
        x, y = rng.uniform(2.3, 4.0), rng.uniform(-1.2, 1.2)
        w, d = rng.uniform(0.03, 0.5, size=2)
        z0 = rng.choice([0.0, rng.uniform(0.9, 1.5)])
        boxes.append(Box((x, y, z0), (x + w, y + d, z0 + rng.uniform(0.03, 1.5))))
    return Scene(walls=WALLS, boxes=tuple(boxes), cylinders=tuple(poles))


def safety_sweep(n: int, seed: int, pose_noise: tuple[float, float] | None = None,
                 pose_error_m: float = 0.05) -> dict:
    if type(n) is not int or n < 0:
        raise ValueError("INVALID_SWEEP_TRIAL_COUNT")
    rng = np.random.default_rng(seed)
    camera, profile = CAMERAS["pinned_73deg"], SensorProfile()
    env = swept_envelope(B, QUERY_M, pose_error_m=pose_error_m)
    counts = {"trials": 0, "skipped_unsafe_approach": 0, "allowed_true_clear": 0,
              "allowed_true_hit": 0, "blocked_true_hit": 0, "blocked_true_clear": 0}
    reasons: dict[str, int] = {}
    cases = []
    approach = straight(Pose(0.0, 0.0, 1.2), B)
    for trial in range(n):
        scene = hard_scene(rng, env)
        case = {"trial": trial, "scene": asdict(scene)}
        if capsule_hits_scene(scene, (0.0, 0.0), (B.x, B.y), env.radius_m, env.z_lo, env.z_hi):
            counts["skipped_unsafe_approach"] += 1
            cases.append({**case, "classification": "SKIPPED_UNSAFE_APPROACH"})
            continue
        capture_seed = int(rng.integers(1 << 30))
        store, t = capture(scene, camera, profile, approach, noise_seed=capture_seed,
                           pose_noise=pose_noise)
        decision, _, _ = decide(store, env, epoch="run", now_s=t)
        hit = capsule_hits_scene(scene, env.start_xy, env.end_xy, env.radius_m, env.z_lo, env.z_hi)
        key = f"{'allowed' if decision.allowed else 'blocked'}_true_{'hit' if hit else 'clear'}"
        counts[key] += 1
        counts["trials"] += 1
        reasons[decision.reason] = reasons.get(decision.reason, 0) + 1
        cases.append({**case, "capture_noise_seed": capture_seed, "classification": key,
                      "truth_query_intersection": hit, "decision": asdict(decision)})
    return {"counts": counts, "reasons": reasons, "seed": seed, "requested_trials": n,
            "cases": cases, "envelope": asdict(env), "sensor_profile": asdict(profile),
            "all_cases_retained": len(cases) == n,
            "false_admission_count": counts["allowed_true_hit"],
            "conservative_block_count": counts["blocked_true_clear"],
            "pose_noise_sigma": None if pose_noise is None else
            {"xyz_m": pose_noise[0], "yaw_deg": round(float(np.degrees(pose_noise[1])), 3)},
            "envelope_pose_error_m": pose_error_m}


def strategies(*, rest_height_m: float) -> dict[str, list[Pose]]:
    """Synthetic paths starting at an explicitly supplied body-frame rest height."""
    if not np.isfinite(rest_height_m) or not 0 < rest_height_m <= B.z:
        raise ValueError("INVALID_REST_HEIGHT")
    plans = {
        "climb_only": climb(B.x, B.y, rest_height_m, B.z),
        "climb_then_yaw_scan": climb(B.x, B.y, rest_height_m, B.z) + yaw_scan(B.x, B.y, B.z),
    }
    for d in (0.25, 0.5, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 2.0, 2.5, 3.0):
        start = Pose(B.x - d, B.y, B.z)
        # Realistic: take off at A' = B - d, climb, then fly level to B.
        plans[f"approach_{d:.2f}m"] = climb(start.x, start.y, rest_height_m, B.z)[:-1] + straight(start, B)
        # Level-only: frames only from the level segment (no climb history).
        plans[f"level_{d:.2f}m"] = straight(start, B)
    return plans


def blind_zone(*, rest_height_m: float) -> dict:
    clear = Scene(walls=WALLS)
    env = swept_envelope(B, QUERY_M)
    out = {}
    for cam_name, camera in CAMERAS.items():
        out[cam_name] = {}
        for plan_name, poses in strategies(rest_height_m=rest_height_m).items():
            store, t = capture(clear, camera, SensorProfile(), poses)
            decision, samples, status = decide(store, env, epoch="run", now_s=t, ttl_s=60.0)
            behind = samples[:, 0] < B.x
            out[cam_name][plan_name] = {
                "allowed": decision.allowed, "reason": decision.reason,
                "free_fraction": round(decision.counts["FREE"] / decision.sample_points, 4),
                "unseen_fraction": round(decision.counts["UNSEEN"] / decision.sample_points, 4),
                "unseen_behind_body_fraction": round(
                    float(((status == 3) & behind).sum()) / max(1, int(behind.sum())), 4),
                "frames": decision.frames_used,
            }
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=400)
    parser.add_argument("--profile", type=Path, required=True,
                        help="Complete oracle profile used to derive physical support/rest height")
    args = parser.parse_args()
    from model_liveness import physical_fixture
    fixture = physical_fixture(json.loads(args.profile.read_text(encoding="utf-8")))
    args.output.mkdir(parents=True, exist_ok=True)
    results = {"camera_fov_deg": {k: [round(c.intrinsics.hfov_deg, 1), round(c.intrinsics.vfov_deg, 1)]
                                  for k, c in CAMERAS.items()},
               "camera_mount_flu": {k: list(c.mount.translation_flu) for k, c in CAMERAS.items()},
               "envelope_scope": "Historical spike diagnostic; not calibrated navigation clearance",
               "takeoff_fixture": fixture,
               "historical_takeoff_fixture": {"rest_height_m": .15,
                   "status": "Historical reference only; below complete-profile support and not rerun"},
               "motion_authority": False,
               "query": {"start": [B.x, B.y, B.z], "forward_m": QUERY_M,
                         "envelope_radius_m": swept_envelope(B, QUERY_M).radius_m},
               "safety_sweep": safety_sweep(args.trials, seed=11),
               "safety_sweep_pose_noise": [
                   safety_sweep(args.trials, seed=11, pose_noise=(0.02, np.radians(0.5))),
                   safety_sweep(args.trials, seed=11, pose_noise=(0.05, np.radians(1.5))),
                   safety_sweep(args.trials, seed=11, pose_noise=(0.05, np.radians(1.5)), pose_error_m=0.15),
               ],
               "blind_zone": blind_zone(rest_height_m=fixture["rest_height_m"])}
    with (args.output / "results.json").open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(results, indent=2, allow_nan=False)+"\n")
    print(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
