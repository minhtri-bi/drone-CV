"""Shared real-wire ingress policy; no ROS runtime required."""
from math import pi, sin, cos, sqrt
from types import SimpleNamespace

import numpy as np
import pytest

from drone_cv.px4_bridge import decode_odometry
from drone_cv.px4_quaternion import MAX_SQUARED_NORM_ERROR, normalize_px4_quaternion


@pytest.mark.parametrize("angle", [0., .3, pi-1e-8, -pi+1e-8])
def test_float32_sign_and_bounded_scale_do_not_bias_yaw(angle):
    raw = np.array([cos(angle/2), 0., 0., sin(angle/2)], dtype=np.float32)
    message = SimpleNamespace(pose_frame=1, velocity_frame=1, q=raw,
        timestamp_sample=1000000, position=(0., 0., -1.), velocity=(0., 0., 0.),
        reset_counter=0, position_variance=(0., 0., 0.), velocity_variance=(0., 0., 0.))
    first = decode_odometry(message, "epoch", 1.)
    message.q = -raw.astype(float)*sqrt(1 + MAX_SQUARED_NORM_ERROR*.5)
    second = decode_odometry(message, "epoch", 1.)
    assert second.yaw_ned == pytest.approx(first.yaw_ned, abs=1e-12)
    assert abs(first.yaw_ned-angle) < 1e-7
    assert second.quaternion_squared_norm_error != first.quaternion_squared_norm_error


@pytest.mark.parametrize("squared", [1-1.01e-6, 1+1.01e-6, 0.])
def test_malformed_norm_cannot_be_normalized_into_acceptance(squared):
    with pytest.raises(ValueError, match="QUATERNION_NORM"):
        normalize_px4_quaternion((sqrt(squared), 0., 0., 0.))


@pytest.mark.parametrize("squared", [1-.99e-6, 1+.99e-6])
def test_both_sides_of_ingress_boundary_are_normalized(squared):
    result = normalize_px4_quaternion((sqrt(squared), 0., 0., 0.))
    assert result.wxyz == (1., 0., 0., 0.)
    assert result.squared_norm_error == pytest.approx(squared-1.)


@pytest.mark.parametrize("raw", [(float('nan'), 0., 0., 0.), (1., 0., 0.), (1e308, 0., 0., 0.)])
def test_nonfinite_shape_and_overflow_fail_closed(raw):
    with pytest.raises(ValueError):
        normalize_px4_quaternion(raw)
