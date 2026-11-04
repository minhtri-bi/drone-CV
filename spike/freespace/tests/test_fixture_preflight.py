"""Independent identity negatives and unchanged primary fixture boundaries."""

from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Mount, Pose
from fixture_preflight import (AuxiliaryPlane, CANDIDATE, _identity_hits, _plane_depth,
                               check_pose, perturbations, run, support_minimum_z)
from model_liveness import physical_fixture


ROOT = Path(__file__).parents[3]
FIXTURE = json.loads((ROOT / "spike/freespace/replay_fixture.json").read_text())
PROFILE = json.loads((ROOT / "docs/replay/evidence-semantics-01/profile.json").read_text())
PHYSICAL = physical_fixture(PROFILE)
CAMERA = CameraModel(Intrinsics.pinned_x500_depth(640, 480), Mount(tuple(PHYSICAL["optical_translation_flu"])))
REST = Pose(0., 0., PHYSICAL["rest_height_m"])


def test_independently_visible_original_fixture_and_old_side_occlusion_negative():
    visible = check_pose(FIXTURE, CAMERA, REST)
    assert visible["all_pass"]
    assert all(row["correct_target_support"] >= .95 for row in visible["targets"].values())
    old = check_pose(FIXTURE, CAMERA, REST, old_side=True)
    assert not old["all_pass"]
    assert old["targets"]["offaxis"]["correct_target_support"] < .95
    assert visible["targets"]["offaxis"]["roi_pixels"] == old["targets"]["offaxis"]["roi_pixels"]


def test_oblique_candidate_preserves_primary_rois_and_has_visible_y_sensitive_interior():
    original = check_pose(FIXTURE, CAMERA, REST)
    candidate = check_pose(FIXTURE, CAMERA, REST, auxiliary=CANDIDATE)
    assert candidate["all_pass"]
    assert abs(CANDIDATE.normal[1]) > .7
    assert candidate["targets"]["auxiliary"]["correct_target_support"] >= .95
    for name in ("front", "offaxis", "horizontal"):
        assert candidate["targets"][name]["roi_pixels"] == original["targets"][name]["roi_pixels"]
        assert candidate["targets"][name]["pass"]
    assert FIXTURE["edge_pixels"] == 3
    assert (FIXTURE["p95_error_m"], FIXTURE["maximum_error_m"]) == (.10, .20)


def test_independent_ray_faces_find_the_known_offaxis_occluder():
    origin = np.array([.13233, 0., .24778])
    point = np.array([4.5, 1.4, .55])
    direction = (point-origin)[None, :]
    side = {"name": "side_target", "lo": [1.8, .9, .2], "hi": [3., 1.1, .9]}
    distance, identity = _identity_hits(origin, direction, FIXTURE["boxes"]+[side], None)
    assert identity[0] == "side_target"
    assert distance[0] == pytest.approx(.9/1.4)
    crossing = origin+distance[0]*direction[0]
    assert 1.8 < crossing[0] < 3. and .2 < crossing[2] < .9


def test_finite_plane_extent_and_roi_are_distinct_and_not_an_infinite_wall():
    origin = np.zeros(3)
    center = np.asarray(CANDIDATE.center)
    tangent = CANDIDATE.tangent
    rays = np.array([center, center+.4*tangent, center+.6*tangent])
    physical = _plane_depth(origin, rays, CANDIDATE)
    interior = _plane_depth(origin, rays, CANDIDATE, roi=True)
    assert np.isfinite(physical[:2]).all() and np.isinf(physical[2])
    assert np.isfinite(interior[0]) and np.isinf(interior[1:]).all()
    _, identities = _identity_hits(origin, rays, [], CANDIDATE)
    assert identities[0] == identities[1] == "auxiliary"


def test_declared_grid_covers_all_translation_corners_and_combined_rotation_extremes():
    samples = list(perturbations(PROFILE))
    assert len(samples) == 27*9
    assert {sample[0][2] for sample in samples} == {-.07, 0., .07}
    assert ((.07, -.07, .07), (.1, -.1, .1)) in {(sample[0], sample[1]) for sample in samples}
    assert any(sample[3] < 0 for sample in samples)
    assert any(sample[3] >= 0 for sample in samples)
    for translation, angles, pose, minimum in samples:
        if translation[2] == -.07:
            assert minimum < 0
        if angles == (0., 0., 0.) and translation[2] == 0.:
            assert minimum == pytest.approx(0., abs=1e-12)
        assert minimum == support_minimum_z(PROFILE, pose)


@pytest.mark.parametrize("changes", [{"normal": (1., 1., 0.)}, {"roi_half_width_m": .5},
                                      {"center": (np.nan, 0., 0.)}, {"half_height_m": -.1}])
def test_invalid_or_noninterior_auxiliary_geometry_fails_closed(changes):
    with pytest.raises(ValueError, match="INVALID_AUXILIARY"):
        replace(CANDIDATE, **changes)


def test_preflight_cannot_loosen_primary_roi_or_threshold():
    with pytest.raises(ValueError, match="THRESHOLDS"):
        run({**FIXTURE, "edge_pixels": 2}, PROFILE)
    changed = {**FIXTURE, "targets": [{**target, "lo": list(target["lo"])} for target in FIXTURE["targets"]]}
    changed["targets"][0]["lo"][1] = -.1
    with pytest.raises(ValueError, match="PRIMARY_GEOMETRY"):
        run(changed, PROFILE)
