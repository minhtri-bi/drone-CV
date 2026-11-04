"""Sampled liveness with complete physical geometry and observed truth error only.

Neither a stationary observed error maximum nor sampled free coverage is a
certified uncertainty bound or motion authority. Old collision-only profiles and
odometry-span reports are deliberately rejected; retained reports stay historic.
"""

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from capture import capture, climb, straight, yaw_scan
from evidence import EvidenceSource, SensorProfile, decide, swept_envelope
from model_profile import diagnostic_envelope
from scene import Box, Scene


def _rigid(value, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-10, rtol=0)):
        raise ValueError(f"INVALID_{name}")
    return matrix


def physical_fixture(profile: dict, *, rest_height_m: float | None = None) -> dict:
    """Derive a level floor-support fixture from every complete oracle part.

    Heights refer to the base-link origin, not the outer model origin. The radial
    proxy encloses every part AABB corner (including visual and full rotor bounds).
    No support allowlist is applied to sensing or navigation clearance.
    """
    if (profile.get("schema") != 1 or profile.get("frame") != "base_link FLU"
            or not profile.get("parts") or not profile.get("source_sha256")
            or "optical_mount_body" not in profile or "model_from_body" not in profile):
        raise ValueError("COMPLETE_ORACLE_PROFILE_REQUIRED")
    lower, upper, identities, roles = [], [], set(), []
    for part in profile["parts"]:
        lo, hi = np.asarray(part["lo"], dtype=float), np.asarray(part["hi"], dtype=float)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all()
                or np.any(lo > hi) or part["id"] in identities
                or part["role"] not in {"body", "rotor", "gear"}):
            raise ValueError("INVALID_COMPLETE_PROFILE_PART")
        identities.add(part["id"])
        lower.append(lo)
        upper.append(hi)
        roles.append(part["role"])
    lower, upper = np.asarray(lower), np.asarray(upper)
    z_lo, z_hi = float(lower[:, 2].min()), float(upper[:, 2].max())
    if (not np.isclose(profile["z_lo_m"], z_lo, atol=1e-12, rtol=0)
            or not np.isclose(profile["z_hi_m"], z_hi, atol=1e-12, rtol=0)):
        raise ValueError("PROFILE_VERTICAL_BOUNDS_MISMATCH")
    support_indices = np.flatnonzero(np.isclose(lower[:, 2], z_lo, atol=1e-12, rtol=0))
    if z_lo >= 0 or any(roles[index] != "gear" for index in support_indices):
        raise ValueError("LOWEST_PROFILE_PART_NOT_GEAR")
    model_from_body = _rigid(profile["model_from_body"], "MODEL_FROM_BODY")
    optical = _rigid(profile["optical_mount_body"], "OPTICAL_MOUNT_BODY")
    if (not np.allclose(model_from_body[:3, :3], np.eye(3), atol=1e-10, rtol=0)
            or not np.allclose(optical[:3, :3], np.eye(3), atol=1e-10, rtol=0)):
        raise ValueError("UNSUPPORTED_NONLEVEL_PROFILE_TRANSFORM")
    rest = -z_lo if rest_height_m is None else float(rest_height_m)
    if not np.isfinite(rest) or rest < -z_lo:
        raise ValueError("REST_HEIGHT_BELOW_SUPPORT")
    radius = float(np.linalg.norm(np.maximum(abs(lower[:, :2]), abs(upper[:, :2])), axis=1).max())
    canonical = json.dumps(profile, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return {"radius_m": radius, "z_lo_m": z_lo, "z_hi_m": z_hi,
            "rest_height_m": rest, "minimum_support_height_m": -z_lo,
            "model_rest_height_m": rest-float(model_from_body[2, 3]),
            "optical_translation_flu": optical[:3, 3].tolist(),
            "support_part_ids": [profile["parts"][index]["id"] for index in support_indices],
            "source_sha256": profile["source_sha256"],
            "profile_canonical_sha256": hashlib.sha256(canonical).hexdigest(),
            "geometry_scope": "Outer radial bound of all complete oracle part AABB corners"}


def _truth_diagnostic(observed: dict) -> dict:
    if (observed.get("scope") != "STATIONARY_TRUTH_DIAGNOSTIC_NOT_CALIBRATION"
            or observed.get("motion_authority") is not False):
        raise ValueError("STATIONARY_TRUTH_DIAGNOSTIC_REQUIRED")
    relative = observed.get("relative_error", {})
    count = relative.get("pair_count")
    fields = ("maximum_translation_m", "maximum_rotation_rad", "maximum_bound_m", "rho_m")
    if (type(count) is not int or count <= 0
            or any(type(relative.get(key)) not in (int, float)
                   or not np.isfinite(relative[key]) or relative[key] < 0 for key in fields)
            or relative["maximum_rotation_rad"] > np.pi or relative["rho_m"] <= 0
            or relative["maximum_bound_m"] + 1e-12 < relative["maximum_translation_m"]
            or relative["maximum_bound_m"] + 1e-12 <
                2*np.sin(relative["maximum_rotation_rad"]/2)*relative["rho_m"]):
        raise ValueError("TRUTH_RELATIVE_ERROR_UNAVAILABLE")
    source = observed.get("source", {})
    digest = source.get("recording_manifest_sha256")
    if (not isinstance(digest, str) or len(digest) != 64
            or any(char not in "0123456789abcdefABCDEF" for char in digest)
            or not isinstance(observed.get("coverage"), dict)):
        raise ValueError("TRUTH_DIAGNOSTIC_SOURCE_REQUIRED")
    return relative


def run(profile: dict, observed: dict) -> dict:
    """Consume a full oracle profile and all-pairs stationary truth diagnostic.

    maximum_bound_m is the observed maximum of the per-pair translation plus
    angular lever-arm error. It inflates XY and Z equally for this diagnostic;
    it is never inferred from odometry spans or presented as calibration.
    """
    fixture = physical_fixture(profile)
    relative = _truth_diagnostic(observed)
    error = float(relative["maximum_bound_m"])
    b = Pose(2., 0., 1.2)
    rest = fixture["rest_height_m"]
    if rest > b.z:
        raise ValueError("INVALID_REST_HEIGHT")
    sensor = SensorProfile(0., 0., EvidenceSource.HISTORICAL_ANALYSIS,
                           "Historical sensing-cap comparison; no calibrated uncertainty", False)
    caps = {"near_trusted_m": sensor.near_trusted_m, "far_trusted_m": sensor.far_trusted_m,
            "rho_m": relative["rho_m"]}
    if "sensing_caps" in observed and any(observed["sensing_caps"].get(key) != caps[key]
                                         for key in ("near_trusted_m", "far_trusted_m")):
        raise ValueError("TRUTH_DIAGNOSTIC_SENSOR_CAP_MISMATCH")
    scene = Scene(walls=(Box((-4, 2.5, 0), (12, 2.7, 3)), Box((-4, -2.7, 0), (12, -2.5, 3))))
    profiles = {
        "historical_synthetic": {"mount": [.13233, 0, .26078], "clearance_m": .10,
                                  "pose_xy_m": .05, "pose_z_m": 0., "historical": True},
        "corrected_mount_historical_envelope": {"mount": fixture["optical_translation_flu"],
                                                 "clearance_m": .10, "pose_xy_m": .05,
                                                 "pose_z_m": 0., "historical": True},
        "model_navigation_diagnostic": {"mount": fixture["optical_translation_flu"],
                                        "clearance_m": .5, "pose_xy_m": error,
                                        "pose_z_m": error, "historical": False}}
    distances = (1., 1.5, 2., 2.5, 3., 3.5, 4.)
    plans = {"climb_only": climb(b.x, b.y, rest, b.z),
             "climb_then_scan": climb(b.x, b.y, rest, b.z)+yaw_scan(b.x, b.y, b.z)}
    for distance in distances:
        a = Pose(b.x-distance, b.y, b.z)
        plans[f"approach_{distance:.1f}m"] = climb(a.x, a.y, rest, a.z)[:-1]+straight(a, b)
        plans[f"level_{distance:.1f}m"] = straight(a, b)
    rows = []
    for name, config in profiles.items():
        sensor = SensorProfile(
            near_translation_uncertainty_m=relative["maximum_translation_m"] if not config["historical"] else 0.,
            near_rotation_uncertainty_rad=relative["maximum_rotation_rad"] if not config["historical"] else 0.,
            source=(EvidenceSource.ANALYTIC_SYNTHETIC if config["historical"]
                    else EvidenceSource.STATIONARY_DIAGNOSTIC),
            provenance=("Historical analytic comparison with explicit zero uncertainty" if config["historical"]
                        else "Same-bag all-pairs stationary truth-relative observed maxima; not calibration"),
            no_return_certified=config["historical"])
        config["sensor_profile"] = asdict(sensor)
        envelope = (swept_envelope(b, 1.) if config["historical"] else diagnostic_envelope(
            fixture, b, 1., clearance_m=config["clearance_m"], pose_xy_m=error, pose_z_m=error))
        for pitch in (0., 20.):
            camera = CameraModel(Intrinsics.pinned_x500_depth(), Mount(tuple(config["mount"]), pitch))
            for strategy, poses in plans.items():
                store, now = capture(scene, camera, sensor, poses)
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
            "motion_authority": False, "physical_profile": fixture,
            "profiles": profiles, "tested_distances_m": distances, "summary": summary, "rows": rows,
            "sensing_caps": caps,
            "uncertainty_status": {"observed_relative_error": relative,
                                    "diagnostic_inflation_all_axes_m": error,
                                    "source": observed["source"], "coverage": observed["coverage"],
                                    "calibrated_pose_bound_m": None, "tracking_bound_m": None},
            "limitations": ["The all-pairs stationary observed error maximum is not a certified localization bound.",
                            "Navigation clearance is included once; physical geometry itself is not inflated.",
                            "All takeoff paths use complete-profile support height, including historical-envelope comparisons.",
                            "Level/yaw proxy encloses full oracle AABBs but excludes tilted-body motion.",
                            "Finite near cones and capsule squares use conservative outer boxes and may overblock.",
                            "Near hazard translation/rotation inflation uses declared observed stationary maxima only in the model diagnostic.",
                            "Near floor observations remain hazards; no support exemption clears sensing evidence.",
                            "Synthetic no-return is certified by the renderer only; real sensor +Inf remains uncertified.",
                            "An ALLOWED sampled query in this fixture is not runtime motion authority."]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True, help="Complete oracle profile JSON")
    parser.add_argument("--observed-report", type=Path, required=True,
                        help="All-pairs stationary truth-relative diagnostic JSON, not odometry spans")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = run(json.loads(args.profile.read_text(encoding="utf-8")),
                 json.loads(args.observed_report.read_text(encoding="utf-8")))
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report["summary"]))


if __name__ == "__main__":
    main()
