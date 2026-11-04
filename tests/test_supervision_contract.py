"""Conditional offline bounds; these synthetic rows are not runtime acceptance."""
from dataclasses import replace
from math import hypot, nan
from types import SimpleNamespace

import pytest

from drone_cv.supervision_contract import LossResponse, RateWindow, SourceBudget
from drone_cv.supervisor import Stage, State, transition
from drone_cv.timing import Clock, Stamp
from test_supervisor import active, bare_adapter, config, inputs
from test_px4_policy import geometry, policy as declared_policy


def conditional_runtime_inputs(cfg):
    """Generous synthetic room for pure prerequisites, never a runtime profile."""
    from drone_cv.px4_bridge import FIRMWARE_PIN
    policy = {**declared_policy(), "COM_OF_LOSS_T": .05,
              "GF_MAX_HOR_DIST": 50., "GF_MAX_VER_DIST": 50.}
    cfg = replace(cfg, envelope=replace(cfg.envelope, target="exploratory_simulation",
                  minimum_ned=(-5., -5., -15.), maximum_ned=(5., 5., -6.), maximum_speed_mps=1.2),
                  loss_response=replace(cfg.loss_response, firmware=FIRMWARE_PIN,
                                        failsafe_delay_s=0., scope="exploratory_conditional"))
    context = geometry()
    context = replace(context, horizontal_radius_m=50., height_above_home_m=50., motion_reserve_m=5.,
        landing=replace(context.landing, minimum_ned=(-15., -15., -25.), maximum_ned=(15., 15., 0.),
                        body_provenance=cfg.envelope.bounds_provenance))
    return cfg, policy, context


def test_loss_timeout_transition_and_upper_rtf_each_consume_stopping_room():
    cfg = config()
    assert cfg.response_delay_s == pytest.approx(.3*1. + .05 + .02 + .01 + .02)
    data = inputs(1.3, stage=14, armed=True)
    assert transition(active(), data, cfg)[0].stage == Stage.ACTIVE
    for component in ("offboard_loss_s", "failsafe_delay_s", "mode_transition_s", "transport_clock_s"):
        response = replace(cfg.loss_response, **{component: getattr(cfg.loss_response, component)+1.})
        state, output = transition(active(), data, replace(cfg, loss_response=response))
        assert state.reason == "STOPPING_ROOM" and not output.heartbeat
    faster = replace(cfg.clock_contract, maximum_rtf=10.)
    assert transition(active(), data, replace(cfg, clock_contract=faster))[0].reason == "STOPPING_ROOM"


def test_crash_immediately_after_admitted_tick_needs_no_future_supervisor_tick():
    cfg, data = config(), inputs(1.3, stage=14, armed=True)
    state, output = transition(active(), data, cfg)
    assert state.stage == Stage.ACTIVE and output.velocity_ned == (.1, 0., 0.)
    # Independent worst-case path: keep the entire speed bound until every
    # response leg elapses, then reduce speed at the declared minimum deceleration.
    speed = max(cfg.envelope.maximum_speed_mps, state.issued_speed_bound_mps,
                hypot(*(abs(v)+e for v, e in zip(data.odometry.velocity_ned,
                                                cfg.envelope.velocity_error_mps))))
    response = (.3*1. + .05 + .02 + .01 + .02)
    residual = .1+.01+.01
    for axis in range(3):
        for direction in (-1, 1):
            travelled = 0.
            total = response+speed/.25
            step = total/1000
            for index in range(1000):
                t = index*step
                travelled += max(0., speed-max(0., t-response)*.25)*step
                p = data.odometry.position_ned[axis]+direction*(travelled+residual)
                assert data.permit.minimum_ned[axis] <= p <= data.permit.maximum_ned[axis]
                assert cfg.envelope.minimum_ned[axis] <= p <= cfg.envelope.maximum_ned[axis]


def test_zero_new_request_cannot_erase_possibly_effective_previous_output():
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, command=replace(data.command, velocity_ned=(0.,)*3),
                   permit=replace(data.permit, velocity_ned=(0.,)*3))
    previous = replace(active(), issued_speed_bound_mps=.5)
    state, output = transition(previous, data, config())
    assert state.reason == "STOPPING_ROOM" and not output.heartbeat
    assert state.issued_speed_bound_mps == .5


def test_parameter_timeout_mismatch_is_refused_not_hidden_by_free_text():
    cfg = config()
    values = {"COM_OF_LOSS_T": .05, "COM_FAIL_ACT_T": .02, "COM_OBL_RC_ACT": 5}
    for field in values:
        with pytest.raises(ValueError, match="RESPONSE_POLICY_MISMATCH"):
            cfg.loss_response.bind_policy({**values, field: values[field]+1}, "a"*40)
    with pytest.raises(ValueError, match="firmware"):
        cfg.loss_response.bind_policy(values, "b"*40)
    # A smaller positive frozen timeout tightens the horizon, never enlarges it.
    tightened = replace(cfg.loss_response, offboard_loss_s=.025,
                        provenance="Synthetic tighter positive timeout; heartbeat margin separately required")
    assert tightened.required_delay(.3, cfg.clock_contract) < cfg.response_delay_s


def test_missing_response_leg_or_scalar_health_shortcut_fails_closed():
    with pytest.raises(TypeError):
        LossResponse(epoch="epoch", firmware="a"*40, offboard_loss_s=.05)
    for changes in ({"offboard_loss_s": nan}, {"mode_transition_s": -1.},
                    {"scope": "calibrated"}, {"offboard_loss_s": 0.}):
        with pytest.raises(ValueError):
            replace(config().loss_response, **changes)
    with pytest.raises((TypeError, ValueError)):
        replace(config().envelope, position_variance_max=.01)


def test_health_ceilings_apply_by_ned_axis_not_vector_maximum():
    data = inputs(1.3, stage=14, armed=True)
    cfg = replace(config(), envelope=replace(config().envelope,
                  position_variance_max=(.001, .01, .02), velocity_variance_max=(.002, .003, .004)))
    data = replace(data, odometry=replace(data.odometry, position_variance=(.0011, .001, .001)))
    state, output = transition(active(), data, cfg)
    assert state.reason == "ODOMETRY_UNCERTAINTY" and not output.heartbeat


def test_sim_cadence_deadline_uses_rtf_floor_and_wall_cadence_does_not():
    clock = config().clock_contract
    source = SourceBudget("STATUS", 3., .504, .1, "simulation", "Synthetic declared cadence")
    assert source.wall_deadline(clock) == pytest.approx(.504/.196+.1)
    assert replace(source, cadence="wall").wall_deadline(clock) == pytest.approx(.604)
    slower = replace(clock, minimum_rtf=.15)
    assert source.wall_deadline(slower) == pytest.approx(3.46)
    assert source.freshness(slower).sim_ttl_s == 3.
    with pytest.raises(ValueError, match="clock wall watchdog"):
        replace(config(), clock_stall_wall_s=.001)
    with pytest.raises(ValueError, match="one explicit budget"):
        replace(config(), source_budgets=config().source_budgets[:-1])


def test_source_wall_age_remains_finite_under_slow_simulation():
    cfg = config()
    state = replace(active(), previous=None, started=Clock("epoch", 9., 9.), streams=())
    data = inputs(10., stage=14, armed=True)
    status = replace(data.status, stamp=Stamp("epoch", 9_496_000, 9.496, 7.5),
                     nav_state_since_s=9., armed_since_s=9.)
    accepted, _ = transition(state, replace(data, status=status), cfg)
    assert accepted.stage == Stage.ACTIVE
    stale = replace(status, stamp=replace(status.stamp, received_wall_s=7.))
    rejected, output = transition(state, replace(data, status=stale), cfg)
    assert rejected.reason == "STATUS_stale_wall" and not output.heartbeat


@pytest.mark.parametrize("rtf", [0., .195, 1.001])
def test_observed_rate_outside_supported_window_revokes_with_specific_reason(rtf):
    data = inputs(10., stage=14, armed=True)
    rate = RateWindow(Clock("epoch", 10.-rtf, 9.), Clock("epoch", 10., 10.))
    state = replace(active(), previous=None, started=Clock("epoch", 9., 9.))
    result, output = transition(state, replace(data, rate=rate), config())
    assert result.reason == "RTF_OUTSIDE_SUPPORTED_RANGE" and not output.heartbeat


def test_missing_short_future_stale_and_wrong_epoch_rates_never_imply_support():
    data, cfg = inputs(10., stage=14, armed=True), config()
    windows = [None,
        RateWindow(Clock("epoch", 9.999, 9.999), Clock("epoch", 10., 10.)),
        RateWindow(Clock("epoch", 10., 10.), Clock("epoch", 10.1, 10.1)),
        RateWindow(Clock("epoch", 9., 9.), Clock("epoch", 9.1, 9.1)),
        RateWindow(Clock("other", 9., 9.), Clock("other", 10., 10.))]
    for window in windows:
        result, output = transition(replace(active(), previous=None), replace(data, rate=window), cfg)
        assert result.reason.startswith("RTF_") and not output.heartbeat


def test_rtf_never_extends_human_authorization_or_process_session():
    data = inputs(10., stage=14, armed=True)
    rate = RateWindow(Clock("epoch", 9.8, 9.), Clock("epoch", 10., 10.))
    expired = replace(data.lease, stamp=Stamp("epoch", 1, 9.9, 8.), wall_ttl_s=1.)
    state = replace(active(), previous=None, started=Clock("epoch", 9.9, 9.9))
    result, output = transition(state, replace(data, rate=rate, lease=expired), config())
    assert result.reason.startswith("AUTHORIZATION_") and not output.heartbeat


def test_clock_callback_retains_actual_rate_and_sticky_reset(monkeypatch):
    from drone_cv import px4_bridge
    adapter = bare_adapter(State())
    adapter.clock_value, adapter.clock_progress, adapter.clock_samples = None, 0., ()
    node_times = iter((1_000_000_000, 1_020_000_000))
    adapter.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=next(node_times))))
    times = iter((1., 1., 1.1, 1.1))
    monkeypatch.setattr(px4_bridge.time, "monotonic", lambda: next(times))
    adapter._clock(SimpleNamespace(clock=SimpleNamespace(sec=1, nanosec=0)))
    adapter._clock(SimpleNamespace(clock=SimpleNamespace(sec=1, nanosec=20_000_000)))
    window = px4_bridge.rate_window(adapter.clock_samples, adapter.cfg.clock_contract)
    assert window.rtf == pytest.approx(.2)
    adapter._clock(SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0)))
    assert adapter.state.reason == "CLOCK_RESET"


def test_empirical_missing_evidence_stops_runtime_before_ros_import(tmp_path, monkeypatch):
    from drone_cv import px4_bridge
    cfg, policy, context = conditional_runtime_inputs(config())
    authorization = px4_bridge.RuntimeAuthorization(
        owner=cfg.owner, namespace="/vehicle", epoch="epoch",
        firmware=px4_bridge.FIRMWARE_PIN, messages=px4_bridge.MESSAGES_PIN,
        image=px4_bridge.IMAGE_PIN,
        confirmation="ALLOW_CONTROL_PUBLISHERS:/vehicle:test_owner",
        lock_directory=str(tmp_path), discovery_wall_s=2., discovery_stable_s=.2,
        tick_wall_s=.05, target="exploratory_simulation", parameter_policy=policy,
        policy_rationale="Synthetic negative policy; never applied", readback_wall_s=1.,
        terminal_observation_wall_s=1., instance=0, empirical_profile={}, empirical_evaluation={},
        workload="Synthetic boundary fixture, not a supported workload", policy_geometry=context)
    def forbidden(*args, **kwargs):
        raise AssertionError("No ROS import or resource construction before complete admission")
    monkeypatch.setattr(px4_bridge, "import_module", forbidden)
    with pytest.raises(ValueError, match="EMPIRICAL_PROFILE_INELIGIBLE"):
        px4_bridge.RosBridge(cfg, authorization)
    with pytest.raises(ValueError, match="RESPONSE_POLICY_MISMATCH"):
        replace(authorization, parameter_policy={**policy, "COM_OF_LOSS_T": .5}).validate(cfg)


def test_conditional_profile_prerequisite_binds_workload_without_granting_authority(tmp_path):
    from drone_cv import px4_bridge
    from drone_cv.empirical_profile import Applicability, derive_profile, digest, evaluate_profile
    from test_empirical_profile import protocol, sample
    cfg, policy, context = conditional_runtime_inputs(config())
    app = Applicability(px4_bridge.IMAGE_PIN, px4_bridge.FIRMWARE_PIN, px4_bridge.MESSAGES_PIN,
        digest(policy), "gazebo_sim", cfg.clock_contract.minimum_rtf,
        cfg.clock_contract.maximum_rtf, cfg.clock_contract.window_wall_s,
        "frozen workload identity", "base_link_origin", "local_ned")
    frozen = protocol(holdout_kind="untouched_blinded", blinding_evidence_sha256="a"*64,
                      tracking_error_m=.01, tracking_evidence_sha256="b"*64, applicability=app)
    profile = derive_profile(frozen, [sample(1)])
    evaluation = evaluate_profile(profile, [sample(2)])
    candidate = evaluation["candidate"]
    cfg = replace(cfg, envelope=replace(cfg.envelope, target="exploratory_simulation",
                  position_error_m=candidate["position_box_enclosing_radius_m"],
                  velocity_error_mps=tuple(candidate["velocity_axis_mps"])),
                  loss_response=replace(cfg.loss_response, firmware=px4_bridge.FIRMWARE_PIN,
                                        scope="exploratory_conditional"))
    authorization = px4_bridge.RuntimeAuthorization(
        cfg.owner, "/vehicle", "epoch", px4_bridge.FIRMWARE_PIN, px4_bridge.MESSAGES_PIN,
        px4_bridge.IMAGE_PIN, "ALLOW_CONTROL_PUBLISHERS:/vehicle:test_owner", str(tmp_path),
        2., .2, .05, "exploratory_simulation", policy, "Synthetic no-operation boundary",
        1., 1., 0, profile.to_dict(), evaluation, app.workload, context)
    authorization.validate(cfg)  # Pure prerequisite check; no ROS construction.
    assert evaluation["runtime_admission"] is False
    assert evaluation["deterministic_bound_certified"] is False
    with pytest.raises(ValueError, match="EMPIRICAL_APPLICABILITY_MISMATCH"):
        replace(authorization, workload="different workload").validate(cfg)
    with pytest.raises(ValueError, match="EMPIRICAL_APPLICABILITY_MISMATCH"):
        authorization.validate(replace(cfg, clock_contract=replace(cfg.clock_contract, minimum_rtf=.15)))
    with pytest.raises(ValueError, match="EMPIRICAL_PROFILE_INELIGIBLE"):
        authorization.validate(replace(cfg, envelope=replace(cfg.envelope, velocity_error_mps=(.01,)*3)))
    from drone_cv.relative_profile import CaptureQueryBound, evaluate_relative_profile, relative_profile_payload
    from test_relative_profile import J, MODEL, setup_profile
    relative_protocol, rows = setup_profile()
    relative_protocol = replace(relative_protocol, holdout_kind="untouched_source_disjoint",
                                blinding_evidence_sha256="a"*64, conditional_model_scope_sha256=J)
    relative_payload = relative_profile_payload(relative_protocol, rows)
    relative_evaluation = evaluate_relative_profile(relative_payload)
    bound = CaptureQueryBound(.5, .05, .05, 2., relative_protocol.source_sha256,
        relative_evaluation["evaluation_sha256"], relative_protocol.provenance_sha256, MODEL,
        "base_link_origin", "gazebo_sim", "epoch", 0, conditional_model_scope_sha256=J)
    relative_cfg = replace(cfg, envelope=replace(cfg.envelope, bounds_provenance=MODEL,
                                                capture_query_bound=bound))
    relative_authorization = replace(authorization, relative_profile=relative_payload,
        relative_evaluation=relative_evaluation, policy_geometry=replace(context,
            landing=replace(context.landing, body_provenance=MODEL)))
    relative_authorization.validate(relative_cfg)
    assert not relative_evaluation["runtime_admission"]
    with pytest.raises(ValueError, match="MISSING_RELATIVE_EVIDENCE"):
        replace(relative_authorization, relative_profile=None).validate(relative_cfg)
    with pytest.raises(ValueError, match="RELATIVE_EVALUATION_CONTENT_OR_HASH_MISMATCH"):
        replace(relative_authorization, relative_evaluation={
            **relative_evaluation, "runtime_admission": True}).validate(relative_cfg)
