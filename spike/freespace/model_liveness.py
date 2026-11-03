"""Compare sampled coverage using a pinned collision profile; never authorize flight."""

import argparse
import json
from pathlib import Path

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from capture import capture, climb, straight, yaw_scan
from evidence import SensorProfile, decide, swept_envelope
from model_profile import diagnostic_envelope
from scene import Box, Scene


def run(profile: dict, observed: dict) -> dict:
    b = Pose(2., 0., 1.2)
    scene = Scene(walls=(Box((-4, 2.5, 0), (12, 2.7, 3)), Box((-4, -2.7, 0), (12, -2.5, 3))))
    ptp = observed["diagnostics"]["odometry"]["position_ned_peak_to_peak_m"]
    if any(value is None or not np.isfinite(value) or value < 0 for value in ptp):
        raise ValueError("OBSERVED_DRIFT_DIAGNOSTIC_UNAVAILABLE")
    xy, z = float(np.hypot(*ptp[:2])), float(ptp[2])
    profiles = {
        "historical_synthetic": {"mount": [.13233, 0, .26078], "clearance_m": .10,
                                  "pose_xy_m": .05, "pose_z_m": 0., "historical": True},
        "corrected_mount_historical_envelope": {"mount": profile["optical_translation_flu"],
                                                 "clearance_m": .10, "pose_xy_m": .05, "pose_z_m": 0., "historical": True},
        "model_navigation_diagnostic": {"mount": profile["optical_translation_flu"],
                                        "clearance_m": .5, "pose_xy_m": xy, "pose_z_m": z, "historical": False}}
    distances = (1., 1.5, 2., 2.5, 3., 3.5, 4.)
    plans = {"climb_only": climb(b.x, b.y, .15, b.z),
             "climb_then_scan": climb(b.x, b.y, .15, b.z)+yaw_scan(b.x, b.y, b.z)}
    for distance in distances:
        a = Pose(b.x-distance, b.y, b.z)
        plans[f"approach_{distance:.1f}m"] = climb(a.x, a.y, .15, a.z)[:-1]+straight(a, b)
        plans[f"level_{distance:.1f}m"] = straight(a, b)
    rows = []
    for name, config in profiles.items():
        envelope = (swept_envelope(b, 1.) if config["historical"] else diagnostic_envelope(
            profile, b, 1., clearance_m=config["clearance_m"], pose_xy_m=xy, pose_z_m=z))
        for pitch in (0., 20.):
            camera = CameraModel(Intrinsics.pinned_x500_depth(), Mount(tuple(config["mount"]), pitch))
            for strategy, poses in plans.items():
                store, now = capture(scene, camera, SensorProfile(), poses)
                decision, _, _ = decide(store, envelope, epoch="run", now_s=now, ttl_s=60.)
                rows.append({"profile": name, "pitch_down_deg": pitch, "strategy": strategy,
                             "sampled_coverage_allowed": decision.allowed, "reason": decision.reason,
                             "counts": decision.counts, "sample_count": decision.sample_points,
                             "radius_m": envelope.radius_m, "z_lo_m": envelope.z_lo, "z_hi_m": envelope.z_hi})
    summary = []
    for name in profiles:
        for pitch in (0., 20.):
            selected = [row for row in rows if row["profile"] == name and row["pitch_down_deg"] == pitch]
            summary.append({"profile": name, "pitch_down_deg": pitch,
                            **{prefix+"_first_allowed_tested_m": next((distance for distance in distances
                                if any(row["strategy"] == f"{prefix}_{distance:.1f}m" and row["sampled_coverage_allowed"]
                                       for row in selected)), None) for prefix in ("approach", "level")}})
    return {"scope": "Synthetic sampled-coverage diagnostic only", "flight_admission": False,
            "physical_profile": {key: profile[key] for key in ("radius_m", "z_lo_m", "z_hi_m", "source_sha256")},
            "profiles": profiles, "tested_distances_m": distances, "summary": summary, "rows": rows,
            "uncertainty_status": {"diagnostic_xy_span_m": xy, "diagnostic_z_span_m": z,
                                    "source": "r39 stationary trace peak-to-peak displacement, not calibrated error",
                                    "tracking_bound_m": None, "relative_angular_bound_rad": None,
                                    "calibrated_pose_bound_m": None},
            "limitations": ["Observed spans are illustrative input floors, not worst-case localization bounds.",
                            "Navigation clearance is included once; physical geometry itself is not inflated.",
                            "Level/yaw collision proxy excludes visual-mesh completeness and tilted body motion.",
                            "Synthetic no-return is certified by the renderer only; real sensor +Inf remains uncertified.",
                            "An ALLOWED sampled query in this fixture is not runtime motion authority."]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--observed-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(json.loads(args.profile.read_text()), json.loads(args.observed_report.read_text()))
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report["summary"]))


if __name__ == "__main__":
    main()
