"""Synthetic conditional policy checks; no live PX4 or calibration claim."""
from dataclasses import replace
from math import hypot
import struct
from typing import Any

import pytest

from drone_cv.px4_policy import (
    HomeSnapshot, LandingCorridor, LocalReference, PolicyGeometry,
    conservative_upper_float_parameter_bound, float32_comparison_bounds,
    validate_geometry_policy, validate_motion_policy, validate_parameter_policy,
)


def policy():
    return {
        "COM_OF_LOSS_T": .33333, "COM_OBL_RC_ACT": 5, "COM_FAIL_ACT_T": 0.,
        "COM_RC_LOSS_T": .5, "COM_RCL_EXCEPT": 0, "NAV_RCL_ACT": 1,
        "COM_RC_IN_MODE": 4,
        "NAV_DLL_ACT": 1, "COM_DL_LOSS_T": 10, "COM_DLL_EXCEPT": 0,
        "COM_LOW_BAT_ACT": 2, "BAT_LOW_THR": .15, "BAT_CRIT_THR": .07,
        "BAT_EMERGEN_THR": .05, "GF_ACTION": 2, "GF_SOURCE": 0,
        "GF_MAX_HOR_DIST": 20., "GF_MAX_VER_DIST": 10.,
        "MPC_XY_VEL_MAX": 1., "MPC_Z_VEL_MAX_UP": .5, "MPC_Z_VEL_MAX_DN": .5,
        "MPC_POS_MODE": 4, "MPC_ACC_HOR": 3., "MPC_ACC_HOR_MAX": 5.,
        "MPC_ACC_UP_MAX": 4., "MPC_ACC_DOWN_MAX": 3.,
        "MPC_JERK_MAX": 8., "MPC_JERK_AUTO": 4., "UXRCE_DDS_SYNCT": 0,
    }


def geometry():
    reference = LocalReference(47., 8., 500., 123)
    home = HomeSnapshot((0., 0., 0.), 47., 8., 500., 1, reference, "epoch", (0, 0, 0, 0))
    landing = LandingCorridor((-5., -5., -6.), (5., 5., 0.), 0., .25,
                              "synthetic-body", "synthetic-support-survey",
                              "synthetic-eroded-all-phases")
    return PolicyGeometry(home, 20., 10., landing, .5, .1,
                          "Synthetic local/global frame bound; not measured")


def validate_geometry(values=None, context=None, **kwargs):
    context = geometry() if context is None else context
    defaults: dict[str, Any] = dict(
        current_home=context.home, minimum_ned=(-2., -2., -4.),
        maximum_ned=(2., 2., -1.), required_motion_reserve_m=.5,
        body_provenance="synthetic-body")
    return validate_geometry_policy(policy() if values is None else values,
                                    context, **(defaults | kwargs))


@pytest.mark.parametrize("name,value", [
    ("COM_OBL_RC_ACT", 4), ("COM_OBL_RC_ACT", 8), ("NAV_RCL_ACT", 7),
    ("NAV_DLL_ACT", 4), ("GF_ACTION", 0), ("COM_LOW_BAT_ACT", 0),
    ("COM_LOW_BAT_ACT", 1), ("COM_FAIL_ACT_T", .02), ("COM_RCL_EXCEPT", 4),
    ("COM_DLL_EXCEPT", 4), ("GF_SOURCE", 1), ("GF_MAX_HOR_DIST", 0.),
    ("GF_MAX_VER_DIST", 10001.), ("MPC_Z_VEL_MAX_UP", .49),
    ("MPC_Z_VEL_MAX_DN", 4.01), ("MPC_JERK_AUTO", 0.),
    ("COM_DL_LOSS_T", 4), ("COM_DL_LOSS_T", 10.),
    ("COM_OBL_RC_ACT", 5.), ("BAT_CRIT_THR", .2),
    ("COM_RC_IN_MODE", 3), ("MPC_XY_VEL_MAX", 1), ("BAT_LOW_THR", .51),
    ("UXRCE_DDS_SYNCT", False),
])
def test_policy_rejects_bad_domains_types_and_unsupported_actions(name, value):
    with pytest.raises(ValueError):
        validate_parameter_policy(policy() | {name: value}, "synthetic")


def test_policy_rejects_incomplete_and_extended_inventory():
    values = policy()
    del values["MPC_Z_VEL_MAX_DN"]
    with pytest.raises(ValueError, match="complete"):
        validate_parameter_policy(values, "synthetic")
    with pytest.raises(ValueError, match="complete"):
        validate_parameter_policy(policy() | {"UNKNOWN": 1}, "synthetic")


def test_land_policy_requires_independently_bound_complete_corridor():
    validate_geometry()
    with pytest.raises(ValueError, match="landing"):
        replace(geometry(), landing=None)
    with pytest.raises(ValueError, match="provenance"):
        replace(geometry().landing, physical_provenance="")
    with pytest.raises(ValueError, match="body"):
        validate_geometry(body_provenance="different-body")
    with pytest.raises(ValueError, match="landing"):
        validate_geometry(context=replace(geometry(), landing=replace(
            geometry().landing, maximum_ned=(1., 5., 0.))))
    with pytest.raises(ValueError, match="support"):
        replace(geometry().landing, support_plane_down_m=-.1)


@pytest.mark.parametrize("change", [
    {"position_ned": (1., 0., 0.)}, {"latitude_deg": 47.001},
    {"altitude_m": 501.}, {"update_count": 2}, {"epoch": "other"},
    {"reset_counters": (1, 0, 0, 0)},
    {"local_reference": LocalReference(47., 8., 500., 124)},
    {"local_reference": LocalReference(47.001, 8., 500., 123)},
])
def test_geometry_binds_actual_home_origin_epoch_and_resets(change):
    with pytest.raises(ValueError, match="Home"):
        validate_geometry(current_home=replace(geometry().home, **change))


def test_outer_cylinder_checks_whole_box_and_room_not_single_extents():
    # Each individual horizontal extent is smaller than 20, but their resultant is not.
    with pytest.raises(ValueError, match="cylinder"):
        validate_geometry(minimum_ned=(14., 14., -4.), maximum_ned=(15., 15., -1.))
    with pytest.raises(ValueError, match="reserve"):
        validate_geometry(required_motion_reserve_m=.6)
    with pytest.raises(ValueError, match="cylinder"):
        validate_geometry(minimum_ned=(-2., -2., -9.5))
    with pytest.raises(ValueError, match="cylinder"):
        validate_geometry(context=replace(geometry(), horizontal_radius_m=19.))
    with pytest.raises(ValueError):
        validate_geometry(values=policy() | {"GF_MAX_VER_DIST": 0.})
    with pytest.raises(ValueError, match="Home"):
        validate_geometry(current_home=None)
    with pytest.raises(ValueError, match="frame"):
        replace(geometry().landing, frame="body_frd")


def test_three_dimensional_resultant_includes_comparison_reserve():
    with pytest.raises(ValueError, match="resultant"):
        validate_motion_policy(policy(), maximum_speed_mps=1.01,
                               minimum_deceleration_mps2=1.)
    with pytest.raises(ValueError, match="resultant"):
        validate_motion_policy(policy(), maximum_speed_mps=hypot(1., .5),
                               minimum_deceleration_mps2=1.)
    validate_motion_policy(policy(), maximum_speed_mps=1.2, minimum_deceleration_mps2=1.)


def test_acceleration_limits_only_reject_impossible_minimum_not_certify_response():
    assert validate_motion_policy(policy(), maximum_speed_mps=1.2,
                                  minimum_deceleration_mps2=1.) is None
    with pytest.raises(ValueError, match="commanded"):
        validate_motion_policy(policy(), maximum_speed_mps=1.2,
                               minimum_deceleration_mps2=3.1)


def test_unused_manual_acceleration_limit_does_not_become_a_braking_certificate():
    values = policy() | {"MPC_ACC_HOR_MAX": 2.}
    validate_motion_policy(values, maximum_speed_mps=1.2, minimum_deceleration_mps2=2.5)
    with pytest.raises(ValueError, match="commanded"):
        validate_motion_policy(values | {"MPC_POS_MODE": 0},
                               maximum_speed_mps=1.2, minimum_deceleration_mps2=2.5)


def f32(value):
    return struct.unpack("!f", struct.pack("!f", value))[0]


@pytest.mark.parametrize("value", [.333, .33333, 0., 1., 20.])
def test_comparison_bounds_cover_exact_float32_acceptance_on_both_sides(value):
    lower, upper = float32_comparison_bounds(value)
    target = f32(value)
    limit = f32(1e-7)
    assert lower <= target <= upper
    assert f32(abs(f32(lower-target))) < limit
    assert f32(abs(f32(upper-target))) < limit
    # The next representable values outside either endpoint must fail the pinned compare.
    def adjacent(number, direction):
        bits = struct.unpack("!I", struct.pack("!f", number))[0]
        bits += direction if number >= 0 else -direction
        return struct.unpack("!f", struct.pack("!I", bits))[0]
    assert f32(abs(f32(adjacent(lower, -1)-target))) >= limit
    assert f32(abs(f32(adjacent(upper, 1)-target))) >= limit
    assert conservative_upper_float_parameter_bound("COM_OF_LOSS_T", value) == upper


def test_upper_bound_never_uses_four_decimal_text_as_authoritative():
    declared = .33333
    upper = conservative_upper_float_parameter_bound("COM_OF_LOSS_T", declared)
    assert upper >= f32(declared) > .3333
    with pytest.raises(ValueError, match="float"):
        conservative_upper_float_parameter_bound("COM_OBL_RC_ACT", 5)
