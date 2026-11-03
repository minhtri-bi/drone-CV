from math import sqrt

import numpy as np
import pytest

from drone_cv.braking import BrakingLimits, MotionState, Proposal, decide_velocity, speed_limit
from drone_cv.histogram import Histogram, HistogramConfig
from drone_cv.timing import Clock, FreshnessPolicy, Stamp

STAMP = Stamp("safety", 3, 1.0, 10.0)
NOW = Clock("safety", 1.1, 10.1)
POLICY = FreshnessPolicy(0.3, 0.3)
LIMITS = BrakingLimits(2.0, 0.3, 0.5, 3.0)


def scene(forward=4.0, rear=0.0):
    distance = np.zeros((72, 3))
    distance[0] = forward
    distance[36] = rear
    return Histogram(distance, np.full((72, 3), "UNKNOWN"), STAMP, (0, 0, 0),
                     HistogramConfig(0.2, 0.15, 4, 0.25, True))


def proposal(velocity=(2, 0, 0), stamp=STAMP):
    return Proposal(velocity, 0.0, stamp)


def motion(velocity=(0, 0, 0), position=(0, 0, 0)):
    return MotionState(position, velocity, STAMP)


def test_braking_example_and_distance_boundary():
    example = BrakingLimits(2.0, 0.3, 1.0, 10.0)
    assert speed_limit(4, example) == pytest.approx(2.915679166)
    assert speed_limit(0.5, LIMITS) == 0
    assert speed_limit(0, LIMITS) == 0
    # Invert the limit against independently integrated constant-deceleration motion.
    for distance in (0.6, 1.0, 2.0, 4.0):
        v = speed_limit(distance, LIMITS)
        assert v*0.3 + v*v/4 <= distance-0.5+1e-12
    assert speed_limit(2, LIMITS) < speed_limit(4, LIMITS)
    assert speed_limit(4, BrakingLimits(2, 0.6, 0.5, 3)) < speed_limit(4, LIMITS)


@pytest.mark.parametrize("distance", [-1, float("nan"), float("inf")])
def test_invalid_distance_never_becomes_speed(distance):
    with pytest.raises(ValueError):
        speed_limit(distance, LIMITS)


@pytest.mark.parametrize("args", [(0, .3, .5, 3), (-1, .3, .5, 3),
                                  (2, -.1, .5, 3), (2, .3, -1, 3),
                                  (2, .3, .5, float("inf"))])
def test_invalid_braking_config(args):
    with pytest.raises(ValueError):
        BrakingLimits(*args)


def test_speed_clamped_and_signed_reverse_is_valid_but_requires_coverage():
    safe = decide_velocity(proposal((8, 0, 0)), motion(), scene(), NOW, POLICY, LIMITS)
    assert safe.velocity_enu == pytest.approx((speed_limit(4, LIMITS), 0, 0))
    assert safe.reason == "CLAMPED"
    rear = decide_velocity(proposal((-1, 0, 0)), motion(), scene(), NOW, POLICY, LIMITS)
    assert rear.velocity_enu == (0, 0, 0)
    assert rear.reason == "NO_CORRIDOR"
    visible_rear = decide_velocity(proposal((-1, 0, 0)), motion(), scene(rear=4), NOW, POLICY, LIMITS)
    assert visible_rear.velocity_enu == (-1, 0, 0)


def test_e12_alive_caller_cannot_extend_proposal_expiry():
    old = Stamp("safety", 1, 0.5, 9.5)
    result = decide_velocity(proposal(stamp=old), motion(), scene(), NOW, POLICY, LIMITS)
    assert result.velocity_enu == (0, 0, 0)
    assert result.reason == "PROPOSAL_stale_sim"
    reset = decide_velocity(proposal(stamp=Stamp("old", 9, 1, 10)), motion(),
                            scene(), NOW, POLICY, LIMITS)
    assert reset.reason == "PROPOSAL_epoch"


def test_actual_side_slip_and_excess_speed_force_braking():
    sideways = decide_velocity(proposal(), motion((0, 2, 0)), scene(), NOW, POLICY, LIMITS)
    assert sideways.brake_required and sideways.velocity_enu == (0, 0, 0)
    assert sideways.reason == "STOPPING_ROOM"
    overspeed = decide_velocity(proposal((0.1, 0, 0)), motion((4, 0, 0)), scene(),
                                NOW, POLICY, LIMITS)
    assert overspeed.brake_required and overspeed.reason == "STOPPING_ROOM"


def test_turning_while_moving_does_not_cut_through_unknown_space():
    full = Histogram(np.full((72, 3), 4), np.full((72, 3), "CLEAR"), STAMP,
                     (0, 0, 0), HistogramConfig(.2, .15, 4, .25, True))
    result = decide_velocity(proposal((0, 1, 0)), motion((1, 0, 0)), full, NOW, POLICY, LIMITS)
    assert result.brake_required and result.reason == "STOP_BEFORE_TURN"


def test_no_vertical_motion_and_no_reuse_at_different_origin():
    assert decide_velocity(proposal((0, 0, 1)), motion(), scene(), NOW, POLICY,
                           LIMITS).reason == "VERTICAL_MOTION"
    assert decide_velocity(proposal(), motion(position=(0, .1, 0)), scene(), NOW, POLICY,
                           LIMITS).reason == "ORIGIN_CHANGED"


def test_unknown_one_height_band_blocks_whole_vehicle():
    distances = np.full((72, 3), 4.0)
    distances[0, 2] = 0
    partial = Histogram(distances, np.full((72, 3), "UNKNOWN"), STAMP, (0, 0, 0),
                        HistogramConfig(.2, .15, 4, .25, True))
    assert decide_velocity(proposal(), motion(), partial, NOW, POLICY,
                           LIMITS).reason == "NO_CORRIDOR"


def test_nan_actual_motion_rejected_not_hovered():
    with pytest.raises(ValueError):
        motion((float("nan"), 0, 0))
