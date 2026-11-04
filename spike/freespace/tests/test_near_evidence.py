"""Consumer-visible near hazards are capture-pose volumes, never center rays."""

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Mount, Pose
from evidence import Envelope, EvidenceStore, Frame, SensorProfile, decide


CAMERA = CameraModel(Intrinsics(5, 5, 2., 2., 2., 2.), Mount((0., 0., 0.)))
PROFILE = SensorProfile(fixed_margin_m=0., sigma0=0., sigma2=0.)
CAPTURE = Pose(0., 0., 1.)


def hazard_store(value=.1, *, pose=CAPTURE, supporting_pose: Pose | None = Pose(-1., 0., 1.)):
    store = EvidenceStore(CAMERA, PROFILE)
    if supporting_pose is not None:
        store = store.add(Frame(np.full((5, 5), np.inf), supporting_pose, "run", 0., 0))
    depth = np.full((5, 5), np.nan)
    depth[2, 2] = value
    return store.add(Frame(depth, pose, "run", 1., len(store.frames)))


def query(store, envelope, *, now=1., ttl=10., spacing=.002):
    return decide(store, envelope, epoch="run", now_s=now, ttl_s=ttl, spacing_m=spacing)[0]


def point_envelope(x, y, z, radius=.002):
    return Envelope((x, y), (x, y), radius, z-.002, z+.002)


@pytest.mark.parametrize("y,z", [(-.07, .93), (-.07, 1.07), (.07, .93), (.07, 1.07)])
def test_full_pixel_corner_blocks_where_center_ray_misses(y, z):
    # At optical Z=.29 every corner reaches |y|=|z-1|=.0725.
    envelope = point_envelope(.29, y, z)
    assert query(hazard_store(), envelope).reason == "TOO_CLOSE"
    assert query(hazard_store(np.nan), envelope).allowed


@pytest.mark.parametrize("optical_z", [.001, .15, .299])
def test_finite_return_blocks_entire_zero_to_trusted_near_range(optical_z):
    # The measured .01 m cannot narrow the unresolved range to that surface.
    assert query(hazard_store(.01), point_envelope(optical_z, 0., 1.)).reason == "TOO_CLOSE"


def test_swept_segment_crossing_blocks_without_any_near_sample():
    envelope = Envelope((.29, -1.), (.29, 1.), .002, .928, .932)
    samples = envelope.sample(1.)
    assert len(samples) and np.all(np.abs(samples[:, 1]) > .3)
    decision = query(hazard_store(), envelope, spacing=1.)
    assert not decision.allowed and decision.reason == "TOO_CLOSE"


@pytest.mark.parametrize("rotation", [
    [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
    [[1., 0., 0.], [0., 2**-.5, -2**-.5], [0., 2**-.5, 2**-.5]],
    [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]],
    [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]],
])
def test_near_cone_uses_full_capture_rotation(rotation):
    pose = Pose(2., -3., 4., rotation_enu_flu=rotation)
    r = pose.world_from_body()
    corner = pose.position + r @ np.array([.29, .07, -.07])
    supporting = pose.position - r @ np.array([1., 0., 0.])
    support_pose = Pose(*supporting, rotation_enu_flu=rotation)
    envelope = point_envelope(*corner)
    assert query(hazard_store(pose=pose, supporting_pose=support_pose), envelope).reason == "TOO_CLOSE"
    assert query(hazard_store(np.nan, pose=pose, supporting_pose=support_pose), envelope).allowed


def test_motion_away_from_finite_near_requires_independent_support():
    envelope = Envelope((1., 0.), (2., 0.), .02, .98, 1.02)
    assert query(hazard_store(), envelope, spacing=.02).allowed
    unsupported = query(hazard_store(supporting_pose=None), envelope, spacing=.02)
    assert not unsupported.allowed and unsupported.reason != "TOO_CLOSE"
    unresolved = query(hazard_store(-np.inf), envelope, spacing=.02)
    assert not unresolved.allowed and unresolved.reason == "TOO_CLOSE"


def test_near_corner_survives_new_clear_frame_and_capture_pose_motion():
    store = hazard_store()
    store = store.add(Frame(np.full((5, 5), np.inf), Pose(10., 0., 1.), "run", 2., 2))
    assert query(store, point_envelope(.29, .07, .93), now=2.).reason == "TOO_CLOSE"


@pytest.mark.parametrize("value", [.1, -np.inf])
def test_true_ttl_expiry_removes_hazard_but_does_not_supply_coverage(value):
    envelope = point_envelope(.29, .07, .93)
    store = hazard_store(value)
    # Boundary remains retained, not silently dropped by a clear/dropout frame.
    store = store.add(Frame(np.full((5, 5), np.nan), CAPTURE, "run", 10.5, 2))
    assert query(store, envelope, now=11.).reason == "TOO_CLOSE"
    assert not query(store, envelope, now=11.01).allowed
    store = store.add(Frame(np.full((5, 5), np.inf), Pose(-1., 0., 1.), "run", 11.01, 3))
    assert query(store, envelope, now=11.01).allowed


@pytest.mark.parametrize("u,v", [(0, 0), (0, 4), (4, 0), (4, 4), (2, 2)])
def test_one_pixel_hazard_is_not_lost_among_many_pixels(u, v):
    depth = np.full((5, 5), np.inf)
    depth[v, u] = .1
    store = EvidenceStore(CAMERA, PROFILE).add(Frame(depth, CAPTURE, "run", 1., 0))
    optical_corner = np.array([((u+.49)-2)/2, ((v+.49)-2)/2, 1.])*.29
    rotation, origin = CAMERA.world_from_optical(CAPTURE)
    corner = origin + rotation @ optical_corner
    assert query(store, point_envelope(*corner)).reason == "TOO_CLOSE"


def test_conservative_world_box_can_block_outside_exact_rotated_cone():
    # Explicit false-positive contract: AABBs and square XY radius inflation
    # enclose the full cone/capsule, rather than certifying exact intersection.
    rotation = ((1., 0., 0.), (0., 0., -1.), (0., 1., 0.))
    pose = Pose(0., 0., 1., rotation_enu_flu=rotation)
    # At x=.01 the true cone half-width is .0025, not .075.
    assert query(hazard_store(pose=pose), point_envelope(.01, .07, 1.)).reason == "TOO_CLOSE"


@pytest.mark.parametrize("translation,rotation", [(.02, 0.), (0., .1)])
def test_declared_pose_uncertainty_inflates_near_hazard(translation, rotation):
    from dataclasses import replace

    store = hazard_store()
    envelope = point_envelope(.31, 0., 1.)
    assert query(store, envelope).reason != "TOO_CLOSE"
    uncertain = replace(store, profile=replace(
        PROFILE, near_translation_uncertainty_m=translation,
        near_rotation_uncertainty_rad=rotation))
    assert query(uncertain, envelope).reason == "TOO_CLOSE"


def test_rotation_uncertainty_uses_body_lever_arm_not_camera_only():
    camera = CameraModel(CAMERA.intrinsics, Mount((1., 0., 0.)))
    depth = np.full((5, 5), np.nan)
    depth[2, 2] = .1
    profile = SensorProfile(fixed_margin_m=0., near_rotation_uncertainty_rad=.1)
    store = EvidenceStore(camera, profile, (Frame(depth, CAPTURE, "run", 1., 0),))
    # The 1 m mount needs >.1 m rotational inflation; a camera-origin-only
    # .3 m ray radius would miss this point.
    assert query(store, point_envelope(1.4, 0., 1.)).reason == "TOO_CLOSE"


@pytest.mark.parametrize("field,value", [
    ("near_translation_uncertainty_m", -.01),
    ("near_translation_uncertainty_m", np.nan),
    ("near_rotation_uncertainty_rad", -.01),
    ("near_rotation_uncertainty_rad", np.pi+.01),
])
def test_invalid_near_uncertainty_is_not_silently_used(field, value):
    with pytest.raises(ValueError):
        SensorProfile(**{field: value})


def test_retained_near_hazard_from_other_epoch_cannot_block_current_epoch():
    store = hazard_store().add(Frame(np.full((5, 5), np.inf), Pose(-1., 0., 1.), "new", 1., 0))
    decision = decide(store, point_envelope(.29, .07, .93), epoch="new", now_s=1., spacing_m=.002)[0]
    assert decision.allowed and decision.frames_used == 1


def test_inclusive_ttl_uses_source_nanoseconds_not_float_subtraction_roundoff():
    frame = Frame(np.full((5, 5), np.inf), CAPTURE, "run", .1, 0)
    store = EvidenceStore(CAMERA, PROFILE, (frame,))
    assert store.retained("run", .4, .3) == (frame,)
    assert store.retained("run", .400000001, .3) == ()
