import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Mount, Pose
from evidence import Frame, swept_envelope
from evidence_variants import early_obstacle_variant, gap_variant, scheduled_identities
from truth_error import (all_pairs, interpolate_truth, inverse, quaternion, ray_radius,
                         relative_error, transform_error)
from capture_pose import quaternion_rotation


def transform(x=0., angle=0., axis=0):
    q = np.zeros(4)
    q[0], q[axis+1] = np.cos(angle/2), np.sin(angle/2)
    value = np.eye(4)
    value[:3, :3] = quaternion_rotation(q)
    value[0, 3] = x
    return value


@pytest.mark.parametrize("angle", [0., .001, np.pi/2, np.pi, -np.pi+.00001])
@pytest.mark.parametrize("axis", [0, 1, 2])
def test_full_quaternion_conversion_includes_roll_pitch_and_pi(angle, axis):
    rotation = transform(angle=angle, axis=axis)[:3, :3]
    assert np.allclose(quaternion_rotation(quaternion(rotation)), rotation, atol=1e-12)
    measured = transform_error(transform(angle=angle, axis=axis), 3.)
    assert measured["rotation_rad"] == pytest.approx(abs(angle))
    assert measured["bound_m"] == pytest.approx(6*np.sin(abs(angle)/2))
    assert measured["bound_m"] <= measured["linear_bound_m"]+1e-12


def test_all_pairs_finds_nonadjacent_worst_and_inclusive_ttl():
    rows = [(0, transform(0), transform()), (5, transform(1), transform()),
            (10, transform(2), transform())]
    full = all_pairs(rows, rows, 10, 2.)
    assert full["pair_count"] == 6
    assert full["maximum_translation_m"] == 2.
    assert full["worst_pairs"]["maximum_translation_m"]["capture_ns"] == 0
    assert full["worst_pairs"]["maximum_translation_m"]["query_ns"] == 10
    limited = all_pairs(rows, rows, 9, 2.)
    assert limited["pair_count"] == 5
    assert limited["maximum_translation_m"] == 1.


def test_constant_world_alignment_is_not_relative_drift():
    a, capture, query = transform(4., .3, 1), transform(2., .7, 0), transform(-1., -.6, 2)
    error = relative_error(a @ capture, a @ query, capture, query)
    assert np.allclose(error, np.eye(4), atol=1e-12)


def test_exact_rigid_displacement_never_exceeds_translation_rotation_bound():
    error = transform(.25, .9, 1)
    points = np.array([[2., 0., 0.], [0., 2., 0.], [0., 0., -2.]])
    displacement = points @ error[:3, :3].T+error[:3, 3]-points
    assert np.linalg.norm(displacement, axis=1).max() <= transform_error(error, 2.)["bound_m"]
    assert np.allclose(inverse(error) @ error, np.eye(4), atol=1e-12)


def test_corner_range_and_body_origin_radius_are_not_optical_z():
    camera = CameraModel(Intrinsics.pinned_x500_depth(640, 480), Mount())
    result = ray_radius(camera, 19.1)
    assert 25.9 < result["maximum_ray_range_m"] < 26.1
    assert result["rho_m"] > 19.1
    assert result["rho_m"] <= result["triangle_rho_bound_m"]
    assert ray_radius(camera, 10.)["rho_m"] < result["rho_m"]


def test_truth_interpolation_requires_brackets_and_bounded_gap():
    samples = [(0, transform(0., 0., 0)), (10, transform(2., np.pi/2, 0))]
    actual, deltas = interpolate_truth(samples, 5, 10)
    assert actual[0, 3] == 1.
    assert transform_error(actual, 1.)["rotation_rad"] == pytest.approx(np.pi/4)
    assert deltas == [-5, 5]
    with pytest.raises(ValueError, match="TRUTH_GAP"):
        interpolate_truth(samples, 5, 9)
    with pytest.raises(ValueError, match="UNBRACKETED_TRUTH"):
        interpolate_truth(samples, 11, 10)


def test_event_oracle_activates_and_expires_without_using_store_retained():
    result = scheduled_identities([(10, "run", 7), (14, "run", 9)],
                                  [9, 10, 14, 15, 16, 20], 5, epoch="run")
    assert result == {9: [], 10: [7], 14: [7, 9], 15: [7, 9], 16: [9], 20: []}


def test_gap_and_early_obstacle_have_observable_evidence_transitions():
    camera = CameraModel(Intrinsics.from_hfov(73, 16, 12), Mount((0., 0., 0.)))
    pose = Pose(0., 0., 1.)
    frames = tuple(Frame(np.full((12, 16), 5., dtype=np.float32), pose, "epoch", float(t), t)
                   for t in range(21))
    gap = gap_variant(frames, camera, 10_000_000_000, swept_envelope(pose, 1.))
    assert gap["all_checks_pass"]
    assert gap["queries"][1]["decision"]["frames_used"] == 0
    assert gap["queries"][2]["retained_sequences"] == [12]
    early = early_obstacle_variant(frames, camera, 10_000_000_000)
    assert early["all_checks_pass"]
    assert early["queries"][0]["probe_surface_contributors"] == [0]
    assert early["queries"][1]["probe_surface_contributors"] == []
    assert early["queries"][1]["decision"]["allowed"]
    assert np.all(frames[0].depth == 5.)


def test_independent_identity_oracle_separates_reused_sequences_across_epochs():
    rows = [(10, "old", 7), (12, "new", 7), (16, "old", 9)]
    queries = [9, 10, 12, 15, 16, 17, 18]
    assert scheduled_identities(rows, queries, 5, epoch="new") == {
        9: [], 10: [], 12: [7], 15: [7], 16: [7], 17: [7], 18: []}
    assert scheduled_identities(rows, [15, 16, 21, 22], 5, epoch="old") == {
        15: [7], 16: [9], 21: [9], 22: []}


def test_early_obstacle_variant_cannot_claim_early_only_when_later_surface_remains():
    camera = CameraModel(Intrinsics(5, 5, 2., 2., 2., 2.), Mount((0., 0., 0.)))
    frames = tuple(Frame(np.ones((5, 5), dtype=np.float32), Pose(0., 0., 1.),
                         "epoch", float(t), t) for t in range(21))
    report = early_obstacle_variant(frames, camera, 10_000_000_000)
    assert not report["all_checks_pass"]
    assert report["queries"][-1]["probe_surface_contributors"]


def test_gap_variant_rejects_cross_epoch_inputs_instead_of_mixing_identities():
    camera = CameraModel(Intrinsics(5, 5, 2., 2., 2., 2.))
    frames = (Frame(np.ones((5, 5), dtype=np.float32), Pose(0., 0., 1.), "old", 0., 0),
              Frame(np.ones((5, 5), dtype=np.float32), Pose(0., 0., 1.), "new", 20., 0))
    with pytest.raises(ValueError, match="VARIANT_REQUIRES_ONE_ORDERED_EPOCH"):
        gap_variant(frames, camera, 10_000_000_000, swept_envelope(frames[0].pose, 1.))


def test_truth_diagnostic_rejects_invalid_bracket_policy_even_on_exact_sample():
    with pytest.raises(ValueError, match="INVALID_TRUTH_GAP"):
        interpolate_truth([(0, transform())], 0, 0)


def test_plane_crosscheck_preserves_baseline_erosion_and_reports_auxiliary_exclusions(monkeypatch):
    from types import SimpleNamespace
    import plane_diagnostics

    camera = CameraModel(Intrinsics(5, 5, 4., 4., 2., 2.), Mount((0., 0., 0.)))
    frame = Frame(np.full((5, 5), 3., dtype=np.float32), Pose(0., 0., 1.), "run", 1., 0)
    actual = np.eye(4)
    actual[:3, 3] = [0., .8, 1.]
    sample = SimpleNamespace(source_ns=1_000_000_000, world_from_body=actual)
    monkeypatch.setattr(plane_diagnostics, "decode_wire", lambda message: sample)
    monkeypatch.setattr(plane_diagnostics, "body_truth", lambda message, entities: sample)
    fixture = {"edge_pixels": 1, "minimum_target_support": .95,
               "p95_error_m": .02, "maximum_error_m": .1,
               "targets": [{"name": "wall", "axis": 0, "coordinate": 3.,
                            "lo": [3., -1.6, -.6], "hi": [3., 1.6, 2.6]}],
               "oracle": {"fixture": {"max_truth_gap_ns": 50_000_000}}}
    manifest = {"recording_schema": 4, "world_from_odom": np.eye(4).tolist(), "truth_entities": {}}
    report = plane_diagnostics.plane_crosscheck(
        [(plane_diagnostics.TRUTH_TOPIC, sample, 1_000_000_000)], manifest, fixture, (frame,), camera)
    row = report["rows"][0]["targets"][0]
    assert report["primary_grader_unchanged"] and report["diagnostic_complete"]
    assert row["roi"]["pixels"] == 9
    assert row["roi"]["valid_pixels"] == 9
    assert row["original_grader"]["pass"]
    assert row["independent_truth_projected_roi"]["pixels_excluded_from_original_roi"] > 0
    assert report["motion_authority"] is False
