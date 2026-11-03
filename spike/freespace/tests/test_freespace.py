from math import pi

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Pose
from capture import ahead, capture, climb, straight
from evidence import Envelope, EvidenceStore, Frame, SensorProfile, Status, decide, swept_envelope
from scene import Box, Cylinder, Scene, capsule_hits_scene, render_depth

CAM = CameraModel(Intrinsics.pinned_x500_depth())
PROFILE = SensorProfile()
WALLS = (Box((-2, 2.5, 0), (12, 2.7, 3)), Box((-2, -2.7, 0), (12, -2.5, 3)))
CLEAR = Scene(walls=WALLS)
A, B = Pose(0, 0, 1.2), Pose(2, 0, 1.2)


def run(scene, poses, envelope, *, store=None, t0=0.0, profile=PROFILE, epoch="run", now=None):
    store, t = capture(scene, CAM, profile, poses, store=store, t0=t0, epoch=epoch)
    decision, samples, status = decide(store, envelope, epoch=epoch, now_s=t if now is None else now)
    return decision, store, t


def truth(scene, env):
    return capsule_hits_scene(scene, env.start_xy, env.end_xy, env.radius_m, env.z_lo, env.z_hi)


# ------------------------------------------------------------- geometry

def test_rendered_depth_is_optical_z_and_matches_analytic_values():
    depth = render_depth(Scene(walls=(Box((3, -5, 0), (3.2, 5, 4)),)), CAM, A)
    assert depth[60, 80] == pytest.approx(3 - 0.13233, abs=1e-5)
    assert depth[5, 5] == pytest.approx(3 - 0.13233, abs=1e-5)     # planar wall: same Z off-axis
    pole = render_depth(Scene(cylinders=(Cylinder(2, 0, 0.1, 0, 3),), ground=False), CAM, A)
    assert pole[60, 80] == pytest.approx(2 - 0.1 - 0.13233, abs=1e-5)
    north = render_depth(Scene(walls=(Box((-5, 2, 0), (5, 2.2, 4)),), ground=False),
                         CAM, Pose(0, 0, 1.2, pi / 2))
    assert north[60, 80] == pytest.approx(2 - 0.13233, abs=1e-5)


def test_pixel_ray_projects_back_to_its_pixel():
    origin, dirs = CAM.pixel_rays_world(Pose(1, -2, 1.5, 0.7))
    k = CAM.intrinsics
    for v, u in ((0, 0), (60, 80), (119, 3), (17, 150)):
        point = origin + dirs[v, u] * 2.5
        optical = CAM.to_optical(Pose(1, -2, 1.5, 0.7), point[None])[0]
        assert optical[2] == pytest.approx(2.5)
        assert k.fx * optical[0] / optical[2] + k.cx == pytest.approx(u)
        assert k.fy * optical[1] / optical[2] + k.cy == pytest.approx(v)


def test_clip_semantics_follow_pinned_sdf():
    assert CAM.near_m == 0.2 and CAM.far_m == 19.1
    sky = render_depth(Scene(ground=False), CAM, A)
    assert np.isposinf(sky).all()
    wall = render_depth(Scene(walls=(Box((0.25, -5, 0), (0.4, 5, 4)),), ground=False), CAM, A)
    assert np.isneginf(wall).all()      # 0.25 - 0.132 < near clip


def test_envelope_samples_lie_inside_and_cover_the_footprint():
    env = swept_envelope(B, 1.0)
    samples = env.sample(0.05)
    assert env.contains(samples, inflate_m=1e-9).all()
    assert samples[:, 0].min() == pytest.approx(B.x - env.radius_m, abs=0.05)   # behind body
    assert samples[:, 0].max() == pytest.approx(B.x + 1.0 + env.radius_m, abs=0.05)


# ------------------------------------------------------------- required outcomes

def test_seen_free_corridor_allows_motion():
    env = swept_envelope(B, 1.0)
    decision, _, _ = run(CLEAR, straight(A, B), env)
    assert decision.allowed and decision.reason == "ALLOWED"
    assert decision.counts["FREE"] == decision.sample_points
    assert not truth(CLEAR, env)


def test_same_place_without_history_is_blocked_by_blind_zone():
    env = swept_envelope(B, 1.0)
    decision, _, _ = run(CLEAR, [B], env)
    assert not decision.allowed and decision.reason == "UNSEEN_VOLUME"
    assert decision.counts["UNSEEN"] > 0


@pytest.mark.parametrize("obstacle", [
    Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),                # box across the path
    Cylinder(3.0, 0.0, 0.025, 0.0, 3.0),                 # 5 cm pole
    Cylinder(2.6, 0.42, 0.03, 0.0, 3.0),                 # pole grazing the envelope side
    Box((2.5, -1.0, 1.32), (3.5, 1.0, 1.36)),            # thin horizontal beam in the top band
])
def test_obstacle_inside_envelope_blocks(obstacle):
    scene = Scene(walls=WALLS, boxes=(obstacle,) if isinstance(obstacle, Box) else (),
                  cylinders=(obstacle,) if isinstance(obstacle, Cylinder) else ())
    env = swept_envelope(B, 1.0)
    assert truth(scene, env)
    decision, _, _ = run(scene, straight(A, B), env)
    assert not decision.allowed
    assert decision.reason in {"OBSTACLE_IN_ENVELOPE", "UNSUPPORTED_VOLUME"}


def test_obstacle_just_outside_envelope_does_not_block():
    env = swept_envelope(B, 1.0)
    pole = Cylinder(2.8, 0.0 + env.radius_m + 0.25, 0.03, 0.0, 3.0)
    scene = Scene(walls=WALLS, cylinders=(pole,))
    assert not truth(scene, env)
    decision, _, _ = run(scene, straight(A, B), env)
    assert decision.allowed


def test_new_obstacle_beats_older_free_evidence():
    env = swept_envelope(B, 1.0)
    _, store, t = run(CLEAR, straight(A, B), env)
    changed = Scene(walls=WALLS, boxes=(Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),))
    decision, _, _ = run(changed, [B], env, store=store, t0=t + 0.1)
    assert not decision.allowed and decision.reason == "OBSTACLE_IN_ENVELOPE"


def test_expired_or_reset_history_is_not_evidence():
    env = swept_envelope(B, 1.0)
    _, store, t = run(CLEAR, straight(A, B), env)
    expired, _, _ = decide(store, env, epoch="run", now_s=t + 30, ttl_s=10)
    assert expired.reason == "NO_FRESH_FRAMES"
    # Approach frames expire, a fresh frame at B alone is not enough.
    store, t2 = capture(CLEAR, CAM, PROFILE, [B], store=store, t0=t + 20)
    partial, _, _ = decide(store, env, epoch="run", now_s=t2, ttl_s=10)
    assert partial.reason == "UNSEEN_VOLUME"
    reset, _, _ = decide(store, env, epoch="after-reset", now_s=t2)
    assert reset.reason == "NO_FRESH_FRAMES"
    future, _, _ = decide(store, env, epoch="run", now_s=0.05)
    assert future.frames_used == 1           # frames stamped after "now" are ignored


def test_motion_into_never_viewed_side_is_blocked():
    sideways = Envelope((B.x, B.y), (B.x, B.y + 1.5), 0.5, B.z - 0.2, B.z + 0.2)
    decision, _, _ = run(CLEAR, straight(A, B), sideways)
    assert not decision.allowed and decision.reason == "UNSEEN_VOLUME"


def test_too_close_return_in_latest_frame_stops_everything():
    near_wall = Scene(walls=WALLS, boxes=(Box((2.4, -1, 0), (2.6, 1, 3)),))
    env = swept_envelope(B, 0.1)
    decision, _, _ = run(near_wall, straight(A, B), env)
    assert not decision.allowed and decision.reason == "TOO_CLOSE"


def test_unknown_pixels_never_become_free():
    env = swept_envelope(B, 1.0)
    store, t = capture(CLEAR, CAM, PROFILE, straight(A, B))
    holes = EvidenceStore(CAM, PROFILE)
    for f in store.frames:
        depth = np.array(f.depth)
        depth[45:75, 60:100] = np.nan                     # dropout over the corridor
        holes = holes.add(Frame(depth, f.pose, f.epoch, f.time_s, f.sequence))
    decision, _, _ = decide(holes, env, epoch="run", now_s=t)
    assert not decision.allowed and decision.counts["OCCLUDED"] > 0


def test_no_return_clears_space_only_when_certified():
    open_field = Scene()                                  # ground only; sky is +Inf
    env = swept_envelope(B, 1.0)
    certified, _, _ = run(open_field, straight(A, B), env)
    strict = SensorProfile(no_return_certified=False)
    uncertified, _, _ = run(open_field, straight(A, B), env, profile=strict)
    assert certified.allowed
    assert not uncertified.allowed and uncertified.reason == "UNSUPPORTED_VOLUME"


# ------------------------------------------------------------- safety vs ground truth

def random_scene(rng):
    boxes, cylinders = [], []
    for _ in range(rng.integers(0, 4)):
        x, y = rng.uniform(2.0, 4.5), rng.uniform(-1.5, 1.5)
        w, d, h = rng.uniform(0.05, 0.6, size=3)
        z0 = rng.choice([0.0, rng.uniform(0.8, 1.5)])
        boxes.append(Box((x, y, z0), (x + w, y + d, z0 + max(h, 0.04))))
    for _ in range(rng.integers(0, 4)):
        cylinders.append(Cylinder(rng.uniform(2.0, 4.5), rng.uniform(-1.5, 1.5),
                                  rng.uniform(0.02, 0.15), 0.0, rng.uniform(0.5, 3.0)))
    return Scene(walls=WALLS, boxes=tuple(boxes), cylinders=tuple(cylinders))


def test_allowed_motion_never_touches_ground_truth_obstacles():
    rng = np.random.default_rng(2026)
    env = swept_envelope(B, 1.0)
    allowed = blocked_clear = 0
    for _ in range(80):
        scene = random_scene(rng)
        if capsule_hits_scene(scene, (0, 0), (2, 0), 0.5, 1.0, 1.4):
            continue                                       # approach path itself must be clear
        decision, _, _ = run(scene, straight(A, B), env)
        hit = truth(scene, env)
        if decision.allowed:
            allowed += 1
            assert not hit, f"unsafe allow in {scene}"
        elif not hit:
            blocked_clear += 1
    assert allowed >= 10            # liveness: the check is not "always stop"
    print(f"allowed={allowed}, conservative blocks of clear envelopes={blocked_clear}")


def test_grazing_far_hits_stay_no_return_under_noise():
    # Regression: noise used to be applied before the far clip, so a ground hit at
    # ~475 m got hundreds of metres of noise and turned into a false TOO_CLOSE.
    from camera import Mount
    pitched = CameraModel(Intrinsics.pinned_x500_depth(), Mount(pitch_down_deg=20.0))
    for seed in range(5):
        depth = render_depth(CLEAR, pitched, B, noise_seed=seed)
        assert not np.isneginf(depth).any()
        assert (np.isfinite(depth) & (depth < 0.3)).sum() == 0


def test_points_hidden_behind_a_new_return_lose_old_free_status():
    env = swept_envelope(B, 1.0)
    _, store, t = run(CLEAR, straight(A, B), env)
    changed = Scene(walls=WALLS, boxes=(Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),))
    store, t2 = capture(changed, CAM, PROFILE, [B], store=store, t0=t + 0.1)
    decision, samples, status = decide(store, env, epoch="run", now_s=t2)
    inside_box = ((samples[:, 0] > 2.95) & (samples[:, 0] < 3.15)
                  & (np.abs(samples[:, 1]) < 0.25) & (samples[:, 2] < 2.0))
    assert inside_box.any()
    assert not np.any(status[inside_box] == Status.FREE)


def test_odometry_pose_noise_costs_liveness_not_safety():
    """Depth rendered at the true pose, frames record a noisy pose (2 cm, 0.5 deg)."""
    rng = np.random.default_rng(99)
    env = swept_envelope(B, 1.0)
    allowed = 0
    for i in range(25):
        scene = random_scene(rng)
        if capsule_hits_scene(scene, (0, 0), (2, 0), 0.5, 1.0, 1.4):
            continue
        store, t = capture(scene, CAM, PROFILE, straight(A, B), noise_seed=500 + i,
                           pose_noise=(0.02, np.radians(0.5)))
        decision, _, _ = decide(store, env, epoch="run", now_s=t)
        if decision.allowed:
            allowed += 1
            assert not truth(scene, env)
    assert allowed >= 3


def test_too_close_history_survives_dropout_until_it_expires():
    env = swept_envelope(B, 1.0)
    clear, store, t = run(CLEAR, straight(A, B), env)
    assert clear.allowed
    depth = np.full((120, 160), np.nan, dtype=np.float32)
    depth[60, 80] = -np.inf
    store = store.add(Frame(depth, B, "run", t + 0.1, len(store.frames)))
    near, _, _ = decide(store, env, epoch="run", now_s=t + 0.1)
    assert not near.allowed and near.reason == "TOO_CLOSE"
    dropout = np.full((120, 160), np.nan, dtype=np.float32)
    store = store.add(Frame(dropout, B, "run", t + 0.2, len(store.frames)))
    after_dropout, _, _ = decide(store, env, epoch="run", now_s=t + 0.2)
    assert not after_dropout.allowed and after_dropout.reason == "TOO_CLOSE"
    # Expiring the hazard alone is not permission to move: fresh coverage is needed.
    expired, _, _ = decide(store, env, epoch="run", now_s=t + 20.0)
    assert not expired.allowed and expired.reason == "NO_FRESH_FRAMES"
    recovered_store, recovered_t = capture(
        CLEAR, CAM, PROFILE, straight(A, B), store=store, t0=t + 20.0)
    recovered, _, _ = decide(recovered_store, env, epoch="run", now_s=recovered_t)
    assert recovered.allowed


@pytest.mark.parametrize("spacing_m", [2.0, 100.0])
def test_empty_sampling_grid_never_grants_motion(spacing_m):
    depth = np.full((120, 160), np.nan, dtype=np.float32)
    store = EvidenceStore(CAM, PROFILE).add(Frame(depth, B, "run", 0.0, 0))
    decision, samples, _ = decide(
        store, swept_envelope(B, 1.0), epoch="run", now_s=0.0,
        spacing_m=spacing_m)
    assert samples.shape == (0, 3)
    assert not decision.allowed and decision.reason == "NO_ENVELOPE_SAMPLES"


@pytest.mark.parametrize("profile", [
    SensorProfile(near_trusted_m=0.1),
    SensorProfile(far_trusted_m=20.0),
])
def test_profile_cannot_claim_support_outside_camera_clip(profile):
    with pytest.raises(ValueError, match="trusted range"):
        EvidenceStore(CAM, profile)


def test_short_camera_range_cannot_clear_an_obstacle_beyond_its_clip():
    short_camera = CameraModel(Intrinsics.pinned_x500_depth(), far_m=1.0)
    scene = Scene(
        boxes=(Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),), ground=False)
    env = swept_envelope(B, 1.0)
    assert truth(scene, env)
    # This mismatch previously converted a clipped wall to trusted free space.
    with pytest.raises(ValueError, match="trusted range"):
        capture(scene, short_camera, PROFILE, [A], noise_seed=None)
    matching = SensorProfile(near_trusted_m=0.2, far_trusted_m=1.0)
    store, t = capture(scene, short_camera, matching, [A], noise_seed=None)
    assert np.isposinf(store.frames[0].depth).all()
    decision, _, _ = decide(store, env, epoch="run", now_s=t)
    assert not decision.allowed and decision.reason == "UNSEEN_VOLUME"
