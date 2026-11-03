import numpy as np
import pytest

from drone_cv.depth import DepthLimits, Validity, normalize_depth
from drone_cv.timing import Clock, FreshnessPolicy, Stamp, freshness_error


def stamp(**kwargs):
    return Stamp(**{"epoch": "run-a", "sequence": 4, "sim_s": 2.0,
                    "received_wall_s": 10.0, **kwargs})


def test_e01_invalid_boot_is_not_free():
    image = np.array([[np.nan, 0.0, -1.0, np.inf, -np.inf]], np.float32)
    frame = normalize_depth(image, "32FC1", DepthLimits(0.3, 10), stamp())
    assert np.all(frame.validity == Validity.UNKNOWN)
    assert np.isnan(frame.metres).all()
    assert np.isnan(image[0, 0]) and image[0, 1] == 0


def test_verified_rep117_keeps_near_far_distinct():
    image = np.array([[-np.inf, np.inf, 0.2, 4.0, 11.0]], np.float32)
    frame = normalize_depth(image, "32FC1", DepthLimits(0.3, 10), stamp(),
                            verified_rep117=True)
    assert frame.validity.tolist() == [[Validity.TOO_CLOSE, Validity.NO_RETURN,
                                       Validity.TOO_CLOSE, Validity.VALID,
                                       Validity.UNKNOWN]]
    assert frame.metres[0, 0] == 0
    assert frame.metres[0, 1] == 10
    assert frame.metres[0, 3] == 4
    assert not frame.metres.flags.writeable
    assert not frame.validity.flags.writeable


def test_e04_millimetres_and_unknown_encoding():
    raw = np.array([[0, 200, 4000, 10000, 12000]], np.uint16)
    before = raw.copy()
    frame = normalize_depth(raw, "16UC1", DepthLimits(0.3, 10), stamp())
    np.testing.assert_array_equal(raw, before)
    assert frame.validity.tolist() == [[Validity.UNKNOWN, Validity.TOO_CLOSE,
                                       Validity.VALID, Validity.VALID,
                                       Validity.UNKNOWN]]
    assert frame.metres[0, 2] == 4
    with pytest.raises(ValueError, match="encoding"):
        normalize_depth(raw, "rgb8", DepthLimits(0.3, 10), stamp())
    with pytest.raises(ValueError, match="dtype"):
        normalize_depth(raw.astype(np.float32), "16UC1", DepthLimits(0.3, 10), stamp())


@pytest.mark.parametrize("near,far", [(0, 10), (-1, 10), (3, 2),
                                      (np.nan, 10), (0.3, np.inf)])
def test_e20_invalid_depth_configuration(near, far):
    with pytest.raises(ValueError):
        DepthLimits(near, far)


def test_e08_epoch_future_and_two_clock_deadlines():
    policy = FreshnessPolicy(sim_ttl_s=0.3, wall_ttl_s=0.2)
    now = Clock("run-a", sim_s=2.1, wall_s=10.1)
    assert freshness_error(stamp(), now, policy) is None
    assert freshness_error(stamp(epoch="old"), now, policy) == "epoch"
    assert freshness_error(stamp(sim_s=2.2), now, policy) == "future_sim"
    assert freshness_error(stamp(received_wall_s=11), now, policy) == "future_wall"
    assert freshness_error(stamp(sim_s=1.0), now, policy) == "stale_sim"
    # Sim paused: wall progress still expires independently.
    assert freshness_error(stamp(), Clock("run-a", 2.0, 10.5), policy) == "stale_wall"


@pytest.mark.parametrize("kwargs", [{"sequence": -1}, {"sequence": 1.5},
                                   {"epoch": ""}, {"sim_s": float("nan")},
                                   {"received_wall_s": -1}])
def test_stamp_rejects_invalid_identity_and_time(kwargs):
    with pytest.raises(ValueError):
        stamp(**kwargs)


def test_deadlines_must_be_finite_and_positive():
    with pytest.raises(ValueError):
        FreshnessPolicy(0.3, float("inf"))
    with pytest.raises(ValueError):
        FreshnessPolicy(0, 0.3)
