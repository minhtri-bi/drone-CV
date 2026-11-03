from math import pi, sqrt

import numpy as np
import pytest

from drone_cv.depth import DepthLimits, normalize_depth
from drone_cv.frames import OPTICAL_TO_FLU, enu_to_ned, flu_to_frd, rotation_wxyz, wrap_angle
from drone_cv.projection import Camera, Pose, project_depth
from drone_cv.histogram import LocalClearance, HistogramConfig, build_histogram
from drone_cv.timing import Clock, FreshnessPolicy, Stamp

STAMP = Stamp("geometry", 1, 1.0, 10.0)
NOW = Clock("geometry", 1.05, 10.05)
FRESH = FreshnessPolicy(0.3, 0.3)
CAMERA = Camera(121, 81, 60.0, 60.0, 60.0, 40.0)
POSE = Pose(OPTICAL_TO_FLU, (0.0, 0.0, 0.0), STAMP)


def frame(image):
    return normalize_depth(np.asarray(image, np.float32), "32FC1",
                           DepthLimits(0.3, 10.0), STAMP, verified_rep117=True)


def histogram(image, *, radius=0.2, support=True, pose=POSE, certificate=True):
    clear = LocalClearance((0, 0, 0), 1.0, STAMP) if certificate else None
    return build_histogram(frame(image), CAMERA, pose, (0, 0, 0), NOW, FRESH,
                           HistogramConfig(radius, 0.15, 4.0, 0.25, support),
                           local_clearance=clear, max_pose_skew_s=0.02)


def test_c08_signed_axis_and_independent_quaternion_answer():
    np.testing.assert_array_equal(enu_to_ned([1, -2, 3]), [-2, 1, -3])
    np.testing.assert_array_equal(flu_to_frd([-1, 2, -3]), [-1, -2, 3])
    np.testing.assert_allclose(rotation_wxyz((sqrt(0.5), 0, 0, sqrt(0.5))) @ [1, 0, 0],
                               [0, 1, 0], atol=1e-14)
    np.testing.assert_array_equal(OPTICAL_TO_FLU @ [2, 3, 4], [4, -2, -3])
    # Roundtrip supplements, not replaces, analytic expected axes.
    np.testing.assert_array_equal(enu_to_ned(enu_to_ned([-2, 5, -7])), [-2, 5, -7])


@pytest.mark.parametrize("quaternion", [(0, 0, 0, 0), (2, 0, 0, 0),
                                        (float("nan"), 0, 0, 1)])
def test_e20_bad_quaternion_rejected(quaternion):
    with pytest.raises(ValueError):
        rotation_wxyz(quaternion)


def test_projection_is_optical_z_not_ray_range():
    camera = Camera(3, 1, 2, 2, 1, 0)
    points = project_depth(frame([[4, 4, np.nan]]), camera, POSE, NOW, FRESH,
                           max_pose_skew_s=0.02)
    np.testing.assert_allclose(points, [[4, 2, 0], [4, 0, 0]])
    assert not points.flags.writeable


def test_e05_resize_crop_and_e06_invalid_rotation():
    cropped = CAMERA.crop_resize(left=10, top=4, width=100, height=60,
                                 output_width=50, output_height=30)
    assert (cropped.fx, cropped.fy, cropped.cx, cropped.cy) == (30, 30, 24.75, 17.75)
    with pytest.raises(ValueError, match="shape"):
        project_depth(frame([[4]]), CAMERA, POSE, NOW, FRESH, max_pose_skew_s=0.02)
    with pytest.raises(ValueError, match="rotation"):
        Pose(np.zeros((3, 3)), (0, 0, 0), STAMP)


def test_e07_stale_pose_and_epoch_never_use_latest_pose():
    with pytest.raises(ValueError, match="synchron"):
        project_depth(frame(np.full((81, 121), 4)), CAMERA,
                      Pose(OPTICAL_TO_FLU, (0, 0, 0), Stamp("geometry", 2, 1.04, 10)),
                      NOW, FRESH, max_pose_skew_s=0.02)
    with pytest.raises(ValueError, match="epoch"):
        project_depth(frame(np.full((81, 121), 4)), CAMERA, POSE,
                      Clock("reset", 0, 10.05), FRESH, max_pose_skew_s=0.02)


def test_b07_wrap_boundary():
    assert wrap_angle(-pi) == pi
    assert wrap_angle(3*pi) == pi
    assert wrap_angle(pi + 0.01) == pytest.approx(-pi + 0.01)


def test_free_wall_corridor_and_blind_rear_limit():
    result = histogram(np.full((81, 121), 6.0))
    assert result.free_distance_m.shape == (72, 3)
    np.testing.assert_allclose(result.free_distance_m[0], [4, 4, 4])
    assert np.max(result.free_distance_m[36]) < 1.0
    assert not result.free_distance_m.flags.writeable


def test_e01_unknown_and_unverified_pixel_support_never_clear_corridor():
    assert np.all(histogram(np.full((81, 121), np.nan), certificate=False).free_distance_m == 0)
    assert np.all(histogram(np.full((81, 121), 6), support=False,
                            certificate=False).free_distance_m == 0)


def test_e02_e03_thin_new_wire_is_not_reduced_away():
    clean = np.full((81, 121), 6.0, np.float32)
    wire = clean.copy()
    wire[40, :] = 2.0  # One row is less than 5% of every column.
    first = histogram(clean)
    second = histogram(wire)
    assert first.free_distance_m[0, 1] == 4
    assert 0 < second.free_distance_m[0, 1] < 2
    assert np.min(second.free_distance_m[0]) < 2
    np.testing.assert_array_equal(clean, np.full((81, 121), 6.0))


def test_a04_small_pole_and_a05_footprint_inflation():
    pole = np.full((81, 121), 6.0)
    pole[:, 60] = 3.0  # At 3 m, one 60px-focal pixel is about 5 cm.
    assert histogram(pole).free_distance_m[0, 1] < 3
    assert histogram(pole, radius=0.4).free_distance_m[0, 1] <= histogram(pole).free_distance_m[0, 1]
    gap = np.full((81, 121), 2.0)
    gap[:, 51:70] = 6.0  # ~0.6m gap on the Z=2m wall plane.
    assert histogram(gap, radius=0.4).free_distance_m[0, 1] < 2
    assert histogram(gap, radius=0.1).free_distance_m[0, 1] > 2


def test_e13_boundary_obstacle_blocks_both_sides_and_unknown_hole():
    scene = np.full((81, 121), 6.0)
    scene[40, 60] = 2.0
    result = histogram(scene)
    assert result.free_distance_m[1, 1] < 2
    assert result.free_distance_m[71, 1] < 2
    scene[40, 60] = np.nan
    assert histogram(scene).free_distance_m[0, 1] < 2


def test_local_clearance_is_explicit_fresh_and_cannot_hide_obstacle():
    scene = frame(np.full((81, 121), 0.5))
    with pytest.raises(ValueError, match="conflict"):
        build_histogram(scene, CAMERA, POSE, (0, 0, 0), NOW, FRESH,
                        HistogramConfig(0.2, 0.15, 4, 0.25, True),
                        local_clearance=LocalClearance((0, 0, 0), 1, STAMP),
                        max_pose_skew_s=0.02)


def test_quaternion_basis_conversion_not_component_permutation():
    from drone_cv.frames import orientation_enu_flu_to_ned_frd

    # ROS yaw=0 faces East; PX4 FRD forward is East, right is South, down is down.
    np.testing.assert_allclose(orientation_enu_flu_to_ned_frd((1, 0, 0, 0)),
                               [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-14)
    north = (sqrt(0.5), 0, 0, sqrt(0.5))
    np.testing.assert_allclose(orientation_enu_flu_to_ned_frd(north), np.eye(3), atol=1e-14)


def test_new_no_return_hole_and_certificate_epoch():
    far = np.full((81, 121), np.inf)
    assert histogram(far).free_distance_m[0, 1] == 4
    far[40, 60] = np.nan
    assert histogram(far).free_distance_m[0, 1] < 1
    with pytest.raises(ValueError, match="epoch"):
        build_histogram(frame(np.full((81, 121), 6)), CAMERA, POSE, (0, 0, 0),
                        NOW, FRESH, HistogramConfig(0.2, 0.15, 4, 0.25, True),
                        local_clearance=LocalClearance((0, 0, 0), 1,
                                                      Stamp("old", 1, 1, 10)),
                        max_pose_skew_s=0.02)


def test_pitch_and_translated_pose_match_analytic_projection():
    # Optical forward becomes world up after a 90-degree body pitch.
    pitch_up = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    pose = Pose(pitch_up @ OPTICAL_TO_FLU, (1, -2, 3), STAMP)
    point = project_depth(frame([[2]]), Camera(1, 1, 10, 10, 0, 0),
                          pose, NOW, FRESH, max_pose_skew_s=0.02)
    np.testing.assert_allclose(point, [[1, -2, 5]])
