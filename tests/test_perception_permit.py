"""Relative versus absolute uncertainty in consumer-visible stopping decisions."""
from dataclasses import replace

import pytest

from drone_cv.relative_profile import CaptureQueryBound
from drone_cv.supervisor import Stage, transition
from drone_cv.timing import Stamp
from test_supervisor import active, config, inputs


def perception_case(translation=.04, rotation=0., max_age=.5):
    bound = CaptureQueryBound(max_age, translation, rotation, 3., "a"*64, "b"*64,
        "c"*64, "d"*64, "base_link_origin", "gazebo_sim", "epoch", 0,
        conditional_model_scope_sha256="e"*64)
    cfg = replace(config(), envelope=replace(config().envelope, position_error_m=.01,
                                             capture_query_bound=bound))
    data = inputs(1.3, stage=14, armed=True)
    assert data.permit is not None and data.lease is not None
    permit = replace(data.permit, minimum_ned=(-.3, -.5, -1.5), maximum_ned=(2., .5, -.5),
        evidence_kind="perception_supported", capture_position_ned=(0., 0., -1.), evidence_rho_m=3.,
        relative_bound_sha256=bound.sha256, capture_source_sha256=bound.capture_source_sha256,
        query_source_sha256=bound.query_source_sha256)
    return cfg, replace(data, permit=permit, lease=replace(data.lease, kind="mission"))


def test_relative_error_not_absolute_radius_controls_perception_room():
    cfg, data = perception_case(.02)
    assert transition(active(), data, cfg)[0].stage == Stage.ACTIVE
    # Opposite +/-0.04 m absolute errors can each fit a 0.05 m radius yet
    # disagree by 0.08 m. The relative reserve independently denies this box.
    cfg, data = perception_case(.08)
    cfg = replace(cfg, envelope=replace(cfg.envelope, position_error_m=.05))
    state, output = transition(active(), data, cfg)
    assert state.reason == "STOPPING_ROOM" and not output.heartbeat


def test_rotation_uses_full_supported_point_radius_not_only_body_origin():
    cfg, data = perception_case(.0, .1)
    state, output = transition(active(), data, cfg)
    assert state.reason == "STOPPING_ROOM" and not output.heartbeat


def test_relative_reserve_does_not_get_added_to_absolute_geofence_twice():
    cfg, data = perception_case(.02)
    # Independent absolute position uncertainty can reject a geofence while
    # the same relative evidence still fits its complete supported box.
    cfg = replace(cfg, envelope=replace(cfg.envelope, minimum_ned=(-.29, -5., -5.),
                                        maximum_ned=(5., 5., 0.), position_error_m=.01))
    assert transition(active(), data, cfg)[0].stage == Stage.ACTIVE
    larger = replace(cfg, envelope=replace(cfg.envelope, position_error_m=.08))
    state, output = transition(active(), data, larger)
    assert state.reason == "PREDICTED_ENVELOPE" and not output.heartbeat


@pytest.mark.parametrize("mutation,reason", [
    ({"relative_bound_sha256": "f"*64}, "RELATIVE_UNCERTAINTY_BINDING"),
    ({"capture_source_sha256": "f"*64}, "RELATIVE_UNCERTAINTY_BINDING"),
    ({"query_source_sha256": "f"*64}, "RELATIVE_UNCERTAINTY_BINDING"),
    ({"evidence_rho_m": 4.}, "RELATIVE_UNCERTAINTY_RADIUS"),
    ({"capture_stamp": Stamp("epoch", 1_260_000, 1.26, 1.3)}, "RELATIVE_UNCERTAINTY_AGE"),
])
def test_bound_source_and_support_scope_are_not_labels(mutation, reason):
    cfg, data = perception_case(.02, max_age=.02)
    assert data.permit is not None
    state, output = transition(active(), replace(data, permit=replace(data.permit, **mutation)), cfg)
    assert state.reason == reason and not output.heartbeat


def test_missing_relative_claim_never_falls_back_to_absolute_profile():
    cfg, data = perception_case(.02)
    assert data.permit is not None
    state, output = transition(active(), data, replace(cfg, envelope=replace(cfg.envelope,
                                                                           capture_query_bound=None)))
    assert state.reason == "RELATIVE_UNCERTAINTY_MISSING" and not output.heartbeat
    with pytest.raises(ValueError, match="whole certified box"):
        replace(data.permit, evidence_rho_m=.1)
    with pytest.raises(ValueError, match="original capture"):
        replace(data.permit, capture_position_ned=None)


def test_bootstrap_keeps_independent_absolute_uncertainty():
    cfg = replace(config(), envelope=replace(config().envelope, position_error_m=.08))
    state, output = transition(active(), inputs(1.3, stage=14, armed=True), cfg)
    assert state.reason == "STOPPING_ROOM" and not output.heartbeat
