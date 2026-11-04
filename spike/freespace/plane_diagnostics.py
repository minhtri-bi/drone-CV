"""Evaluator-only plane residual crosscheck; never alter primary grading or fit poses."""

from collections import Counter

import numpy as np

from camera import Pose
from drone_cv.oracle_truth import TRUTH_TOPIC, body_truth, decode_wire
from dy_crosscheck import independent_dy
from replay_fixture import aligned_camera, grade_frame
from truth_error import interpolate_truth, rigid


def _statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0, "mean_signed_m": None, "median_signed_m": None,
                "minimum_signed_m": None, "maximum_signed_m": None,
                "p95_absolute_m": None, "maximum_absolute_m": None}
    return {"count": int(values.size), "mean_signed_m": float(values.mean()),
            "median_signed_m": float(np.median(values)), "minimum_signed_m": float(values.min()),
            "maximum_signed_m": float(values.max()),
            "p95_absolute_m": float(np.quantile(abs(values), .95)),
            "maximum_absolute_m": float(abs(values).max())}


def _projection(camera, pose, alignment, target, edge):
    """Mirror grade_frame ray/erosion selection; caller asserts its pixel counts."""
    rotation, origin = aligned_camera(camera, pose, alignment)
    k = camera.intrinsics
    v, u = np.mgrid[:k.height, :k.width]
    rays = np.stack(((u-k.cx)/k.fx, (v-k.cy)/k.fy, np.ones_like(u)), axis=-1) @ rotation.T
    axis = target["axis"]
    with np.errstate(divide="ignore", invalid="ignore"):
        expected = (target["coordinate"]-origin[axis])/rays[..., axis]
        intersections = origin+rays*expected[..., None]
    roi = np.isfinite(expected) & (expected > camera.near_m) & (expected < camera.far_m)
    for index in range(3):
        if index != axis:
            roi &= ((intersections[..., index] >= target["lo"][index])
                    & (intersections[..., index] <= target["hi"][index]))
    padded = np.pad(roi, edge, constant_values=False)
    eroded = np.ones_like(roi)
    for dy in range(2*edge+1):
        for dx in range(2*edge+1):
            eroded &= padded[dy:dy+k.height, dx:dx+k.width]
    return origin, rays, expected, eroded


def _targets(fixture):
    targets = [(target, "auxiliary_side" if target["name"] == "side" else "original")
               for target in fixture["targets"]]
    if not any(target["name"] == "side" for target, _ in targets):
        surfaces = fixture.get("oracle", {}).get("fixture", {}).get("surfaces", [])
        sides = [surface for surface in surfaces if surface["id"] == "side_target"]
        if len(sides) > 1:
            raise ValueError("AMBIGUOUS_SIDE_TARGET")
        if sides:
            # Only the frozen schema-four side surface is eligible. Never invent
            # an absent target, widen any original ROI or borrow a different wall.
            side = sides[0]
            if side["lo"] != [1.8, .9, .2] or side["hi"] != [3., 1.1, .9]:
                raise ValueError("UNEXPECTED_FROZEN_SIDE_TARGET")
            targets.append(({"name": "side", "axis": 1, "coordinate": .9,
                             "lo": [1.8, .9, .2], "hi": [3., .9, .9]}, "auxiliary_side"))
    return targets


def _target_row(frame, camera, alignment, actual_pose, fixture, target, scope, grade):
    depth = frame.depth
    axis = target["axis"]
    estimate_origin, estimate_rays, estimate_z, roi = _projection(
        camera, frame.pose, alignment, target, fixture["edge_pixels"])
    true_origin, true_rays, true_z, true_roi = _projection(
        camera, actual_pose, np.eye(4), target, fixture["edge_pixels"])
    finite_depth = np.isfinite(depth) & (depth >= camera.near_m) & (depth <= camera.far_m)
    valid = roi & finite_depth
    total, valid_count = int(roi.sum()), int(valid.sum())
    if total != grade["roi_pixels"] or valid_count != grade["valid_pixels"]:
        raise ValueError("PRIMARY_GRADER_ROI_DIVERGENCE")
    estimated_points = estimate_origin+estimate_rays[valid]*depth[valid, None]
    true_points = true_origin+true_rays[valid]*depth[valid, None]
    paired = valid & np.isfinite(true_z) & (true_z > camera.near_m) & (true_z < camera.far_m)
    true_total = int(true_roi.sum())
    true_valid = int((true_roi & finite_depth).sum())
    support = valid_count/total if total else 0.
    return {"name": target["name"], "scope": scope, "axis": axis,
            "normal_world": np.eye(3)[axis].tolist(), "coordinate_m": target["coordinate"],
            "roi": {"lo": list(target["lo"]), "hi": list(target["hi"]),
                    "edge_pixels": fixture["edge_pixels"], "pixels": total,
                    "valid_pixels": valid_count, "support": support,
                    "minimum_target_support_reference": fixture["minimum_target_support"],
                    "support_below_reference": support < fixture["minimum_target_support"],
                    "source": "Unchanged estimator-projected grader ROI"},
            "original_grader": grade,
            "same_estimator_roi": {
                "estimated_plane_normal_residual_m": _statistics(estimated_points[:, axis]-target["coordinate"]),
                "truth_plane_normal_residual_m": _statistics(true_points[:, axis]-target["coordinate"]),
                "estimate_minus_truth_point_normal_m": _statistics(estimated_points[:, axis]-true_points[:, axis]),
                "estimated_optical_z_residual_m": _statistics(depth[valid]-estimate_z[valid]),
                "truth_optical_z_residual_m": _statistics(depth[paired]-true_z[paired]),
                "truth_minus_estimated_expected_optical_z_m": _statistics(true_z[paired]-estimate_z[paired]),
                "truth_projectable_valid_pixels": int(paired.sum()),
                "truth_unprojectable_valid_pixels": valid_count-int(paired.sum()),
                "valid_pixels_inside_truth_roi": int((valid & true_roi).sum()),
                "valid_pixels_outside_truth_roi": valid_count-int((valid & true_roi).sum())},
            "independent_truth_projected_roi": {"pixels": true_total, "valid_pixels": true_valid,
                "support": true_valid/true_total if true_total else 0.,
                "pixels_excluded_from_original_roi": int((roi & ~true_roi).sum()),
                "pixels_added_beyond_original_roi": int((true_roi & ~roi).sum()),
                "note": "Diagnostic only; does not replace or change primary grading ROI"}}


def plane_crosscheck(records, manifest, fixture, frames, camera):
    """Crosscheck original per-frame ROIs against same-bag bracketed body truth.

    Inputs are decoded strict load_recording records and accepted Frame tuples
    from replay_semantics.load_frames. frame.pose is odometry-relative; the frozen
    manifest world_from_odom is applied exactly once. No alignment is fitted.
    Incomplete truth returns diagnostic_complete=False with explicit rejection
    reasons. Low depth support is reported, never converted into diagnostic success
    or motion admission. A completed diagnostic says only that truth was covered.
    """
    if manifest.get("recording_schema") != 4:
        raise ValueError("CONTINUOUS_TRUTH_SCHEMA_REQUIRED")
    if not frames or len({frame.epoch for frame in frames}) != 1:
        raise ValueError("PLANE_DIAGNOSTIC_REQUIRES_ONE_NONEMPTY_EPOCH")
    alignment = np.asarray(manifest["world_from_odom"], dtype=float)
    # Existing validated camera helper owns the rigid-alignment contract.
    aligned_camera(camera, frames[0].pose, alignment)
    maximum_gap = fixture["oracle"]["fixture"]["max_truth_gap_ns"]
    truth, rejected, decode_errors = [], Counter(), []
    for topic, message, _ in records:
        if topic != TRUTH_TOPIC:
            continue
        try:
            sample = body_truth(decode_wire(message), manifest["truth_entities"])
            truth.append((sample.source_ns, np.asarray(sample.world_from_body)))
        except (ValueError, TypeError, KeyError) as error:
            rejected["decode:"+str(error)] += 1
            decode_errors.append(str(error))
    targets = _targets(fixture)
    rows, missing = [], []
    for frame in frames:
        stamp = round(frame.time_s*1e9)
        try:
            actual, bracket = interpolate_truth(truth, stamp, maximum_gap)
        except ValueError as error:
            rejected["capture:"+str(error)] += 1
            missing.append({"stamp_ns": stamp, "sequence": frame.sequence, "reason": str(error)})
            continue
        estimate = alignment @ rigid(frame.pose)
        actual_pose = Pose(*actual[:3, 3], rotation_enu_flu=tuple(tuple(float(v) for v in row)
                                                               for row in actual[:3, :3]))
        # Call the real API on the untouched fixture, not a loosened equivalent.
        original_grade = grade_frame(frame.depth, camera, frame.pose, alignment, fixture)
        target_rows = []
        for target, scope in targets:
            if target["name"] in original_grade["targets"]:
                grade = original_grade["targets"][target["name"]]
            else:
                auxiliary_fixture = {**fixture, "targets": [target]}
                grade = grade_frame(frame.depth, camera, frame.pose, alignment, auxiliary_fixture)["targets"][target["name"]]
            target_rows.append(_target_row(frame, camera, alignment, actual_pose, fixture, target, scope, grade))
        residuals = {row["name"]: row["same_estimator_roi"]["estimated_plane_normal_residual_m"]["median_signed_m"]
                     for row in target_rows}
        required = ("side", "front", "horizontal")
        if all(residuals.get(name) is not None for name in required):
            dy = independent_dy(
                (0., 1., 0.), residuals["side"], residuals["front"], residuals["horizontal"],
                nuisance_corrections_m=(0., 0., 0.), budget=None,
                truth_minus_estimate_y_m=float(actual[1, 3]-estimate[1, 3]))
            dy["geometry_eligible"] = all(row["original_grader"]["pass"] for row in target_rows)
            dy["nuisance_status"] = "Explicitly uncorrected; no justified rotation/depth/extrinsic budget"
            if not dy["geometry_eligible"]:
                dy["geometry_failure"] = "ORIGINAL_OR_AUXILIARY_TARGET_GRADING_FAILED"
        else:
            dy = {"motion_authority": False, "comparison_pass": False, "g2_acceptance": False,
                  "failure": "INDEPENDENT_PLANE_RESIDUAL_UNAVAILABLE"}
        rows.append({"stamp_ns": stamp, "sequence": frame.sequence, "truth_bracket_delta_ns": bracket,
                     "truth_minus_estimate_translation_world_m": (actual[:3, 3]-estimate[:3, 3]).tolist(),
                     "estimated_world_from_body": estimate.tolist(), "truth_world_from_body": actual.tolist(),
                     "original_grader_all_pass": original_grade["all_pass"], "targets": target_rows,
                     "independent_dy": dy})
    complete = not rejected and len(rows) == len(frames)
    return {"scope": "DIRECT_PLANE_TRUTH_CROSSCHECK_NOT_CALIBRATION", "motion_authority": False,
            "diagnostic_complete": complete, "rows": rows,
            "coverage": {"available_frames": len(frames), "covered_frames": len(rows),
                         "truth_samples": len(truth), "maximum_truth_gap_ns": maximum_gap,
                         "complete_observed_frames": complete, "missing_frames": missing,
                         "rejected": dict(rejected), "decode_errors": decode_errors,
                         "continuous_time_certified": False},
            "primary_grader_unchanged": True,
            "threshold_references": {key: fixture[key] for key in
                                     ("minimum_target_support", "p95_error_m", "maximum_error_m", "edge_pixels")},
            "residual_sign": "Projected point coordinate minus plane coordinate, along positive world axis",
            "limitations": ["Infinite-plane normal residuals are not finite-surface distance or admission tests.",
                            "Truth and estimate residuals use the same original estimator-selected ROI pixels.",
                            "Optical-Z comparisons require a forward in-clip truth plane intersection; counts are explicit.",
                            "Independent truth ROI support is diagnostic only, never a substituted primary grader ROI.",
                            "No fitted alignment, correlation threshold, navigation admission or calibrated bound is produced.",
                            "Truth interpolation brackets observed source stamps; it does not certify continuous truth between them."]}
