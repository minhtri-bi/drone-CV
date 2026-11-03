from types import SimpleNamespace as NS

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Pose
from capture_pose import (OdomSample, camera_from_info, decode_depth, interpolate_pose,
                          quaternion_rotation)


def sample(t, q=(1, 0, 0, 0), p=(0, 0, 0), reset=0):
    return OdomSample(t, p, q, reset)


def test_ned_axes_and_full_tilt_have_independent_enu_answers():
    q = (np.sqrt(.5), np.sqrt(.5), 0, 0)
    pose = interpolate_pose([sample(10, q, (1, 2, 3))], 10)
    np.testing.assert_allclose(pose.position, [2, 1, -3])
    expected = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
    np.testing.assert_allclose(pose.world_from_body(), expected, atol=1e-12)
    camera = CameraModel(Intrinsics.pinned_x500_depth())
    rotation, origin = camera.world_from_optical(pose)
    np.testing.assert_allclose(origin, [2.26078, 1.13233, -3], atol=1e-12)
    np.testing.assert_allclose(rotation[:, 2], [0, 1, 0], atol=1e-12)


def test_capture_interpolation_uses_sample_time_and_shortest_arc():
    a = sample(0)
    b = sample(40_000_000, (0, 0, 0, 1), (2, 4, -6))
    pose = interpolate_pose([a, b], 20_000_000)
    np.testing.assert_allclose(pose.position, [2, 1, 3])
    np.testing.assert_allclose(pose.world_from_body(), np.eye(3), atol=1e-12)
    same = interpolate_pose([a, sample(40_000_000, (-1, 0, 0, 0))], 10_000_000)
    np.testing.assert_allclose(same.world_from_body(), [[0, -1, 0], [1, 0, 0], [0, 0, 1]])


@pytest.mark.parametrize("samples,capture,reason", [
    ([], 1, "NONMONOTONIC"),
    ([sample(10), sample(10)], 10, "NONMONOTONIC"),
    ([sample(20), sample(10)], 15, "NONMONOTONIC"),
    ([sample(10), sample(20)], 5, "UNBRACKETED"),
    ([sample(10), sample(20)], 25, "UNBRACKETED"),
    ([sample(0), sample(100_000_000)], 20_000_000, "POSE_GAP"),
    ([sample(0), sample(40_000_000, reset=1)], 20_000_000, "POSE_RESET"),
])
def test_invalid_pose_windows_do_not_extrapolate(samples, capture, reason):
    with pytest.raises(ValueError, match=reason):
        interpolate_pose(samples, capture)


def test_float32_roundoff_is_normalized_but_malformed_norm_is_rejected():
    q = np.array([.5, .5, .5, .5], dtype=np.float32)
    q[0] = np.nextafter(q[0], np.float32(1))
    rotation = quaternion_rotation(q)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(rotation, quaternion_rotation(-q))
    for bad in ([0, 0, 0, 0], [1.01, 0, 0, 0], [np.nan, 0, 0, 0]):
        with pytest.raises(ValueError):
            sample(0, bad)
    with pytest.raises(ValueError, match="UNSUPPORTED_POSE_FRAME"):
        OdomSample(0, (0, 0, 0), (1, 0, 0, 0), 0, 2)


def image_message(endian=False):
    data = np.array([[1, 2, 99], [3, np.inf, 99]], dtype=">f4" if endian else "<f4")
    return NS(encoding="32FC1", header=NS(frame_id="drone_cv_g1/camera_optical"),
              width=2, height=2, step=12, is_bigendian=endian, data=data.tobytes())


@pytest.mark.parametrize("endian", [False, True])
def test_depth_padding_and_endianness_preserve_optical_z(endian):
    decoded = decode_depth(image_message(endian))
    np.testing.assert_equal(decoded, [[1, 2], [3, np.inf]])
    assert not decoded.flags.writeable


def test_depth_truncation_and_wrong_frame_are_rejected():
    image = image_message()
    with pytest.raises(ValueError, match="BYTE_COUNT"):
        decode_depth(NS(**{**vars(image), "data": image.data[:-1]}))
    with pytest.raises(ValueError, match="FRAME"):
        decode_depth(NS(**{**vars(image), "header": NS(frame_id="body")}))


def info_message():
    return NS(header=NS(frame_id="drone_cv_g1/camera_optical"), width=640, height=480,
              k=[432., 0, 320., 0, 432., 240., 0, 0, 1], d=[], distortion_model="plumb_bob",
              binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0))


def test_recorded_camera_info_is_used_and_incompatible_geometry_rejected():
    info = info_message()
    camera = camera_from_info(info)
    assert camera.intrinsics.fx == 432.
    with pytest.raises(ValueError, match="CAMERA_INFO"):
        camera_from_info(NS(**{**vars(info), "d": [.1, 0, 0, 0, 0]}))
    with pytest.raises(ValueError, match="CAMERA_INFO"):
        camera_from_info(NS(**{**vars(info), "distortion_model": "equidistant"}))
    with pytest.raises(ValueError, match="rotation"):
        Pose(0, 0, 0, rotation_enu_flu=np.diag([1, 1, -1]))
