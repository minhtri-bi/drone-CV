"""Offline nonoccluding fixture preflight; no simulator, recording or motion authority."""

import argparse
from dataclasses import asdict, dataclass
import hashlib
from itertools import product
import json
from pathlib import Path

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from model_liveness import physical_fixture
from replay_fixture import grade_frame
from scene import Box, Scene, render_depth


@dataclass(frozen=True)
class AuxiliaryPlane:
    """Finite, two-sided vertical rectangle, with a separate interior metric ROI."""

    center: tuple[float, float, float]
    normal: tuple[float, float, float]
    half_width_m: float
    half_height_m: float
    roi_half_width_m: float
    roi_half_height_m: float

    def __post_init__(self):
        center, normal = np.asarray(self.center, dtype=float), np.asarray(self.normal, dtype=float)
        sizes = (self.half_width_m, self.half_height_m, self.roi_half_width_m, self.roi_half_height_m)
        if (center.shape != (3,) or normal.shape != (3,) or not np.isfinite([center, normal]).all()
                or not np.isclose(np.linalg.norm(normal), 1., atol=1e-12, rtol=0)
                or normal[2] != 0. or not np.isfinite(sizes).all() or min(sizes) <= 0
                or self.roi_half_width_m >= self.half_width_m
                or self.roi_half_height_m >= self.half_height_m):
            raise ValueError("INVALID_AUXILIARY_PLANE")
        object.__setattr__(self, "center", tuple(float(v) for v in center))
        object.__setattr__(self, "normal", tuple(float(v) for v in normal))

    @property
    def tangent(self):
        return np.array([-self.normal[1], self.normal[0], 0.])

    def corners(self, *, roi=False):
        width = self.roi_half_width_m if roi else self.half_width_m
        height = self.roi_half_height_m if roi else self.half_height_m
        return np.array([np.asarray(self.center)+s*width*self.tangent+[0., 0., t*height]
                         for s, t in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


# Candidate only: IDs are derived/frozen by run() only after every included pose passes.
CANDIDATE = AuxiliaryPlane((2.5, -1.2, .55), (2**-.5, 2**-.5, 0.), .5, .45, .25, .25)
OLD_SIDE = {"name": "side_target", "lo": [1.8, .9, .2], "hi": [3., 1.1, .9]}
OLD_SIDE_ROI = {"name": "side", "axis": 1, "coordinate": .9,
                "lo": [1.8, .9, .2], "hi": [3., .9, .9]}
TRANSLATIONS_M = (-.07, 0., .07)
ROTATION_CORNERS_DEG = tuple(product((-.1, .1), repeat=3)) + ((0., 0., 0.),)


def _erode(mask, edge):
    padded = np.pad(mask, edge, constant_values=False)
    result = np.ones_like(mask)
    height, width = mask.shape
    for dy in range(2*edge+1):
        for dx in range(2*edge+1):
            result &= padded[dy:dy+height, dx:dx+width]
    return result


def _plane_depth(origin, rays, plane, *, roi=False):
    """Renderer adapter: normal equation plus local rectangle bounds."""
    normal, center = np.asarray(plane.normal), np.asarray(plane.center)
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = ((center-origin) @ normal)/(rays @ normal)
    valid_distance = np.isfinite(distance) & (distance > 0)
    points = origin+np.where(valid_distance, distance, 0.)[..., None]*rays
    width = plane.roi_half_width_m if roi else plane.half_width_m
    height = plane.roi_half_height_m if roi else plane.half_height_m
    local = points-center
    valid = (valid_distance
             & (abs(local @ plane.tangent) <= width) & (abs(local[..., 2]) <= height))
    return np.where(valid, distance, np.inf)


def _polygon_hit(origin, rays, corners):
    """Independent convex-polygon ray intersection using oriented edge half spaces."""
    a, b, c = corners[:3]
    normal = np.cross(b-a, c-a)
    denominator = rays @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = ((a-origin) @ normal)/denominator
    inside = np.isfinite(distance) & (distance > 0)
    points = origin+rays*np.where(inside, distance, 0.)[..., None]
    for first, second in zip(corners, np.roll(corners, -1, axis=0)):
        inside &= np.cross(second-first, points-first) @ normal >= -1e-10
    return np.where(inside, distance, np.inf)


def _box_faces(box):
    lo, hi = np.asarray(box["lo"], dtype=float), np.asarray(box["hi"], dtype=float)
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        for side in (lo[axis], hi[axis]):
            corners = []
            for s, t in ((0, 0), (1, 0), (1, 1), (0, 1)):
                point = lo.copy()
                point[axis] = side
                point[others[0]] = (lo if s == 0 else hi)[others[0]]
                point[others[1]] = (lo if t == 0 else hi)[others[1]]
                corners.append(point)
            yield np.asarray(corners)


def _identity_hits(origin, rays, boxes, auxiliary):
    """Nearest physical identity, independent of Scene slab tests and ROI grading."""
    floor = np.array([[-250., -250., 0.], [250., -250., 0.],
                      [250., 250., 0.], [-250., 250., 0.]])
    distances = [_polygon_hit(origin, rays, floor)]
    identities = ["ground"]
    for box in boxes:
        distances.append(np.minimum.reduce([_polygon_hit(origin, rays, face) for face in _box_faces(box)]))
        identities.append(box["name"])
    if auxiliary is not None:
        distances.append(_polygon_hit(origin, rays, auxiliary.corners()))
        identities.append("auxiliary")
    distances = np.stack(distances)
    indices = np.argmin(distances, axis=0)
    nearest = np.min(distances, axis=0)
    return nearest, np.asarray(identities)[indices]


def _primary_roi(origin, rays, target, camera, edge):
    axis = target["axis"]
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (target["coordinate"]-origin[axis])/rays[..., axis]
        points = origin+rays*distance[..., None]
    roi = np.isfinite(distance) & (distance > camera.near_m) & (distance < camera.far_m)
    for index in range(3):
        if index != axis:
            roi &= (points[..., index] >= target["lo"][index]) & (points[..., index] <= target["hi"][index])
    return _erode(roi, edge)


def check_pose(fixture, camera, pose, *, auxiliary=None, old_side=False):
    """Use the untouched primary API, then add independent correct-target support."""
    boxes = list(fixture["boxes"]) + ([OLD_SIDE] if old_side else [])
    targets = list(fixture["targets"]) + ([OLD_SIDE_ROI] if old_side else [])
    grading_fixture = {**fixture, "targets": targets}
    scene = Scene(boxes=tuple(Box(tuple(box["lo"]), tuple(box["hi"])) for box in boxes))
    depth = render_depth(scene, camera, pose, noise_seed=None)
    origin, rays = camera.pixel_rays_world(pose)
    if auxiliary is not None:
        extra = _plane_depth(origin, rays, auxiliary)
        extra = np.where(extra > camera.far_m, np.inf, extra)
        extra = np.where(extra < camera.near_m, -np.inf, extra)
        depth = np.minimum(depth, extra).astype(np.float32)
    original = grade_frame(depth, camera, pose, np.eye(4), grading_fixture)
    primary_rois = {target["name"]: _primary_roi(origin, rays, target, camera, fixture["edge_pixels"])
                    for target in targets}
    selected = np.logical_or.reduce(list(primary_rois.values()))
    auxiliary_roi = None
    if auxiliary is not None:
        expected = _plane_depth(origin, rays, auxiliary, roi=True)
        auxiliary_roi = _erode(np.isfinite(expected) & (expected > camera.near_m)
                                & (expected < camera.far_m), fixture["edge_pixels"])
        selected |= auxiliary_roi
    nearest, ray_identities = _identity_hits(origin, rays[selected], boxes, auxiliary)
    identities = np.full(depth.shape, "", dtype="<U32")
    identities[selected] = ray_identities
    visible = np.isfinite(depth) & (depth >= camera.near_m) & (depth <= camera.far_m)
    checked = visible[selected]
    renderer_consistent = bool(np.allclose(depth[selected][checked], nearest[checked], atol=2e-6, rtol=1e-6))
    rows = {}
    for target in targets:
        roi = primary_rois[target["name"]]
        grade = original["targets"][target["name"]]
        if int(roi.sum()) != grade["roi_pixels"]:
            raise ValueError("PRIMARY_ROI_DIVERGENCE")
        identity = {"front": "front_target", "offaxis": "offaxis_target",
                    "horizontal": "ground", "side": "side_target"}[target["name"]]
        total = int(roi.sum())
        correct = int((roi & visible & (identities == identity)).sum())
        support = correct/total if total else 0.
        rows[target["name"]] = {**grade, "correct_target_pixels": correct,
                                "correct_target_support": support,
                                "pass": grade["pass"] and support >= fixture["minimum_target_support"]}
    if auxiliary is not None:
        assert auxiliary_roi is not None
        roi = auxiliary_roi
        total = int(roi.sum())
        valid = roi & visible
        points = origin+rays[valid]*depth[valid, None]
        local = points-np.asarray(auxiliary.center)
        normal = local @ np.asarray(auxiliary.normal)
        tangent_excess = np.maximum(abs(local @ auxiliary.tangent)-auxiliary.half_width_m, 0.)
        vertical_excess = np.maximum(abs(local[:, 2])-auxiliary.half_height_m, 0.)
        errors = np.sqrt(normal**2+tangent_excess**2+vertical_excess**2)
        support = float((roi & visible & (identities == "auxiliary")).sum()/total) if total else 0.
        p95 = float(np.quantile(errors, .95)) if errors.size else None
        maximum = float(errors.max()) if errors.size else None
        passed = (total > 0 and support >= fixture["minimum_target_support"] and p95 is not None
                  and p95 <= fixture["p95_error_m"] and maximum <= fixture["maximum_error_m"])
        rows["auxiliary"] = {"roi_pixels": total, "valid_pixels": int(valid.sum()),
                              "correct_target_support": support, "p95_m": p95,
                              "maximum_m": maximum, "pass": bool(passed)}
    return {"targets": rows, "renderer_identity_depth_agreement": renderer_consistent,
            "all_pass": renderer_consistent and all(row["pass"] for row in rows.values())}


def _rotation(rpy_deg):
    roll, pitch, yaw = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def support_minimum_z(profile, pose):
    """Transform every outer profile corner; no below-ground acquisition is admitted."""
    rotation = pose.world_from_body()
    return min(float((pose.position+rotation @ np.array(corner))[2])
               for part in profile["parts"]
               for corner in product(*zip(part["lo"], part["hi"])))


def perturbations(profile):
    physical = physical_fixture(profile)
    for translation in product(TRANSLATIONS_M, repeat=3):
        for angles in ROTATION_CORNERS_DEG:
            position = np.array([0., 0., physical["rest_height_m"]])+translation
            rotation = tuple(tuple(float(v) for v in row) for row in _rotation(angles))
            pose = Pose(*position, rotation_enu_flu=rotation)
            yield translation, angles, pose, support_minimum_z(profile, pose)


def run(fixture, profile):
    physical = physical_fixture(profile)
    # Thresholds and source ROIs are immutable, not caller-adjustable success knobs.
    references = {"edge_pixels": 3, "minimum_target_support": .95,
                  "p95_error_m": .10, "maximum_error_m": .20}
    if any(fixture[key] != value for key, value in references.items()):
        raise ValueError("ORIGINAL_PREFLIGHT_THRESHOLDS_REQUIRED")
    original = json.loads(Path(__file__).with_name("replay_fixture.json").read_text(encoding="utf-8"))
    if any(fixture[key] != original[key] for key in ("targets", "boxes", "camera_translation_flu")):
        raise ValueError("ORIGINAL_PRIMARY_GEOMETRY_REQUIRED")
    camera = CameraModel(Intrinsics.pinned_x500_depth(640, 480),
                         Mount(tuple(physical["optical_translation_flu"])))
    nominal = Pose(0., 0., physical["rest_height_m"])
    known_visible = check_pose(fixture, camera, nominal)
    old = check_pose(fixture, camera, nominal, old_side=True)
    rows, excluded = [], []
    for translation, angles, pose, minimum_z in perturbations(profile):
        metadata = {"translation_m": translation, "rpy_deg": angles, "minimum_physical_z_m": minimum_z}
        if minimum_z < -1e-12:
            excluded.append({**metadata, "reason": "PHYSICAL_PROFILE_BELOW_SUPPORT"})
            continue
        rows.append({**metadata, **check_pose(fixture, camera, pose, auxiliary=CANDIDATE)})
    passed = (known_visible["all_pass"] and not old["all_pass"] and bool(rows)
              and all(row["all_pass"] for row in rows))
    contract = {"primary_fixture": fixture, "auxiliary": asdict(CANDIDATE),
                "profile_sha256": physical["profile_canonical_sha256"],
                "translation_grid_m": TRANSLATIONS_M, "rotation_samples_deg": ROTATION_CORNERS_DEG,
                "thresholds": references}
    source_paths = (Path(__file__), Path(__file__).with_name("scene.py"),
                    Path(__file__).with_name("replay_fixture.py"), Path(__file__).with_name("camera.py"),
                    Path(__file__).with_name("model_liveness.py"))
    contract["implementation_sha256"] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in source_paths}
    digest = hashlib.sha256(json.dumps(contract, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode()).hexdigest()
    return {"scope": "OFFLINE_FINITE_FIXTURE_PREFLIGHT_NOT_ACQUISITION", "motion_authority": False,
            "all_checks_pass": passed, "primary_grader_unchanged": True,
            "known_visible_primary_only": known_visible, "old_occluding_fixture_negative": old,
            "candidate_contract": contract, "frozen_fixture_id": "nonoccluding-"+digest if passed else None,
            "frozen_world_id": "nonoccluding-world-"+digest if passed else None,
            "frozen_source_id": "fixture-source-"+digest if passed else None,
            "grid": {"sampled_poses": len(rows)+len(excluded), "included_poses": len(rows),
                     "excluded_poses": excluded, "rows": rows, "continuous_coverage_proven": False,
                     "description": "27 Cartesian translation samples times 8 simultaneous RPY corners plus zero rotation"},
            "acquisition_authorized": False, "fresh_primary_baseline_pass": False,
            "dy_budget_status": "MISSING_JUSTIFIED_NUMERICAL_BUDGET",
            "remaining": ["Separate approval and fresh unarmed pinned acquisition after successful preflight",
                          "Freeze justified auxiliary residual/rotation/depth/extrinsic and identifiability budgets before collection",
                          "Fresh primary grading and per-frame same-bag truth comparison; no fitted alignment"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=Path(__file__).with_name("replay_fixture.json"))
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fixture, profile = (json.loads(path.read_text(encoding="utf-8")) for path in (args.fixture, args.profile))
    report = run(fixture, profile)
    report["source_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in (args.fixture, args.profile, Path(__file__),
                                            Path(__file__).with_name("scene.py"),
                                            Path(__file__).with_name("replay_fixture.py"))}
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in ("all_checks_pass", "frozen_fixture_id", "acquisition_authorized")}))
    raise SystemExit(0 if report["all_checks_pass"] else 2)


if __name__ == "__main__":
    main()
