"""Offline state-machine regressions; none is flight or fault-injection evidence."""
from dataclasses import replace
from math import nan

import pytest

from drone_cv.supervisor import (
    Ack, Command, Config, Envelope, Inputs, Lease, Odometry, OperatorRequest,
    Permit, Stage, State, Status, transition,
)
from drone_cv.timing import Clock, FreshnessPolicy, Stamp


def config():
    return Config(
        owner="test_owner", planner="planner", safety="independent_safety",
        source_system=42, source_component=191, target_system=1, target_component=1,
        freshness=FreshnessPolicy(.5, .5), status_freshness=FreshnessPolicy(3., 3.),
        progress_wall_s=3., clock_stall_wall_s=.5, max_clock_step_s=.5,
        max_tick_gap_wall_s=.3, prime_sim_s=.2, prime_wall_s=.2,
        request_timeout_s=1., session_timeout_s=20., max_command_ttl_s=.5,
        envelope=Envelope((-5., -5., -5.), (5., 5., 0.), 1., .1, .2, .1,
                          .2, .01, .01, "offline synthetic bounds, not calibrated"),
    )


def sample(t, seq=None):
    return Stamp("epoch", int(t * 1000000) if seq is None else seq, t, t)


def inputs(t=1., *, stage=2, armed=False, action=None, command=True):
    stamp = sample(t)
    cmd = Command("planner", stamp, .4, (.1, 0., 0.), 0.) if command else None
    permit = Permit("independent_safety", stamp, .4, "planner", stamp.sequence,
                    (.1, 0., 0.), 2., stamp) if command else None
    return Inputs(
        Clock("epoch", t, t),
        Odometry(stamp, (0., 0., -1.), (0., 0., 0.), 0., 0,
                 (.001,) * 3, (.001,) * 3),
        Status(stamp, stage, 2 if armed else 1, False, 0, True, 1, 1,
               t, t if armed else 0.),
        cmd, permit, Lease("test_owner", stamp, 10., 10., "bootstrap"),
        True, OperatorRequest("test_owner", stamp, action) if action else None,
    )


def advance(state, t, **kwargs):
    return transition(state, inputs(t, **kwargs), config())


def active():
    state, output = advance(State(), 1., action="prime")
    assert state.stage == Stage.PRIMING and output.heartbeat
    assert output.position_ned is None
    state, output = advance(state, 1.21, action="enter")
    assert state.stage == Stage.MODE_WAIT and output.vehicle_command == "offboard"
    ack = Ack(sample(1.22), 176, 0, 42, 191, False)
    data = replace(inputs(1.23, stage=14), ack=ack)
    state, output = transition(state, data, config())
    assert state.stage == Stage.OFFBOARD_DISARMED
    state, output = advance(state, 1.24, stage=14, action="arm")
    assert output.vehicle_command == "arm" and state.stage == Stage.ARM_WAIT
    data = replace(inputs(1.25, stage=14, armed=True),
                   ack=Ack(sample(1.25), 400, 0, 42, 191, False))
    state, output = transition(state, data, config())
    assert state.stage == Stage.ACTIVE and output.velocity_ned == (.1, 0., 0.)
    return state


def test_explicit_prime_mode_ack_status_arm_ack_status_path():
    active()


def test_import_and_idle_have_no_control_effect():
    state, output = transition(State(), inputs(), config())
    assert state.stage == Stage.IDLE and not output.heartbeat
    assert output.vehicle_command is None


def test_prime_without_command_never_requests_mode_or_arm():
    state, output = advance(State(), 1., action="prime", command=False)
    state, output = advance(state, 1.21, command=False)
    assert output.heartbeat and output.velocity_ned is None
    assert output.vehicle_command is None and state.stage == Stage.PRIMING
    state, output = advance(state, 1.22, action="enter", command=False)
    assert state.stage == Stage.REVOKED and not output.heartbeat


def test_no_automatic_arming_after_mode_confirmation():
    state, _ = advance(State(), 1., action="prime")
    state, _ = advance(state, 1.21, action="enter")
    state, output = transition(state, replace(inputs(1.22, stage=14),
        ack=Ack(sample(1.22), 176, 0, 42, 191, False)), config())
    state, output = advance(state, 1.23, stage=14)
    assert state.stage == Stage.OFFBOARD_DISARMED and output.vehicle_command is None
    assert output.velocity_ned is None


@pytest.mark.parametrize("ack", [None, Ack(sample(1.22), 176, 0, 7, 191, False),
                                  Ack(sample(1.1), 176, 0, 42, 191, False),
                                  Ack(sample(1.22), 400, 0, 42, 191, False),
                                  Ack(sample(1.22), 176, 0, 42, 191, True)])
def test_mode_status_without_matching_px4_ack_cannot_activate(ack):
    state, _ = advance(State(), 1., action="prime")
    state, _ = advance(state, 1.21, action="enter")
    state, output = transition(state, replace(inputs(1.23, stage=14), ack=ack), config())
    assert state.stage == Stage.MODE_WAIT and output.velocity_ned is None


def test_ack_without_status_does_not_confirm_mode():
    state, _ = advance(State(), 1., action="prime")
    state, _ = advance(state, 1.21, action="enter")
    state, output = transition(state, replace(inputs(1.22),
        ack=Ack(sample(1.22), 176, 0, 42, 191, False)), config())
    assert state.stage == Stage.MODE_WAIT


@pytest.mark.parametrize("fault,reason", [
    ({"graph_exclusive": False}, "OWNER_CONFLICT"),
    ({"odometry": None}, "MISSING_ODOMETRY"),
    ({"status": None}, "MISSING_STATUS"),
    ({"command": None}, "MISSING_COMMAND"),
    ({"permit": None}, "MISSING_PERMIT"),
    ({"lease": None}, "MISSING_AUTHORIZATION"),
])
def test_faults_revoke_and_never_publish_a_fake_hold(fault, reason):
    state, output = transition(active(), replace(inputs(1.3, stage=14, armed=True),
                                               **fault), config())
    assert state.stage == Stage.REVOKED and reason in state.reason
    assert not output.heartbeat and output.position_ned is None
    assert output.velocity_ned is None and output.vehicle_command is None
    state, output = advance(state, 1.4, stage=14, armed=True, action="prime")
    assert state.stage == Stage.REVOKED and not output.heartbeat


@pytest.mark.parametrize("change,reason", [
    ({"reset_counter": 1}, "ODOMETRY_RESET"),
    ({"velocity_ned": (1.1, 0., 0.)}, "MEASURED_SPEED"),
    ({"position_ned": (6., 0., -1.)}, "MEASURED_ENVELOPE"),
    ({"velocity_ned": (-.2, 0., 0.)}, "DIRECTION_UNSUPPORTED"),
    ({"position_variance": (.1,) * 3}, "ODOMETRY_UNCERTAINTY"),
])
def test_measured_telemetry_not_request_or_truth_controls_authority(change, reason):
    data = inputs(1.3, stage=14, armed=True)
    state, output = transition(active(), replace(data, odometry=replace(data.odometry,
                                                                      **change)), config())
    assert state.stage == Stage.REVOKED and reason in state.reason
    assert not output.heartbeat


@pytest.mark.parametrize("change,reason", [
    ({"failsafe": True}, "PX4_FAILSAFE"),
    ({"executor_in_charge": 1}, "PX4_OWNER_CONFLICT"),
    ({"nav_state": 4}, "OFFBOARD_LOST"),
    ({"arming_state": 1}, "ARMING_LOST"),
])
def test_px4_override_latches_and_does_not_reclaim(change, reason):
    data = inputs(1.3, stage=14, armed=True)
    state, output = transition(active(), replace(data, status=replace(data.status,
                                                                    **change)), config())
    assert reason in state.reason and state.stage == Stage.REVOKED
    assert output.vehicle_command is None


def test_replayed_command_sequence_cannot_be_retimestamped_by_bridge():
    state = active()
    data = inputs(1.3, stage=14, armed=True)
    command = replace(data.command, stamp=sample(1.3, 1250000))
    state, output = transition(state, replace(data, command=command), config())
    assert state.stage == Stage.REVOKED and "COMMAND_SEQUENCE" in state.reason
    assert not output.heartbeat


def test_expired_original_command_is_not_refreshed_by_heartbeat():
    state = active()
    data = inputs(1.3, stage=14, armed=True)
    command = replace(data.command, stamp=sample(.8, 1300000))
    state, output = transition(state, replace(data, command=command), config())
    assert state.stage == Stage.REVOKED and "COMMAND" in state.reason
    assert not output.heartbeat


def test_independent_permit_must_bind_command_identity_and_sequence():
    data = inputs(1.3, stage=14, armed=True)
    state, output = transition(active(), replace(data, permit=replace(data.permit,
        command_sequence=42)), config())
    assert state.reason == "PERMIT_BINDING" and not output.heartbeat


def test_actual_speed_is_used_for_stopping_room():
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, odometry=replace(data.odometry, velocity_ned=(.9, 0., 0.)),
                   permit=replace(data.permit, free_distance_m=.1))
    state, output = transition(active(), data, config())
    assert state.reason == "STOPPING_ROOM" and not output.heartbeat


def test_clock_reset_and_discontinuity_revoke():
    for now in (Clock("new", 1.3, 1.3), Clock("epoch", .5, 1.3),
                Clock("epoch", 2., 1.3), Clock("epoch", 1.3, 1.1)):
        state, output = transition(active(), replace(inputs(1.3, stage=14, armed=True),
                                                     now=now), config())
        assert state.stage == Stage.REVOKED and "CLOCK" in state.reason
        assert not output.heartbeat


def test_stalled_clock_and_tick_watchdog_are_wall_based():
    state = active()
    data = inputs(1.3, stage=14, armed=True)
    state, _ = transition(state, data, config())
    for wall in (1.5, 1.7, 1.9):
        state, output = transition(state, replace(data, now=Clock("epoch", 1.3, wall)), config())
    assert state.stage == Stage.REVOKED and "CLOCK_STALLED" in state.reason
    assert not output.heartbeat
    state, output = transition(active(), inputs(1.6, stage=14, armed=True), config())
    assert state.stage == Stage.REVOKED and "TICK_DEADLINE" in state.reason


def test_repeated_status_receive_cannot_fake_source_progress():
    state = active()
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, status=replace(data.status, stamp=sample(1.3, 1250000)))
    state, _ = transition(state, data, config())
    assert state.stage == Stage.REVOKED and "STATUS_SEQUENCE" in state.reason


def test_mode_request_deadline_does_not_retry_or_keep_heartbeat():
    cfg = replace(config(), request_timeout_s=.15)
    state, _ = transition(State(), inputs(1., action="prime"), cfg)
    state, _ = transition(state, inputs(1.21, action="enter"), cfg)
    state, output = transition(state, inputs(1.4), cfg)
    assert state.reason == "REQUEST_TIMEOUT" and state.stage == Stage.REVOKED
    assert output.vehicle_command is None and not output.heartbeat


def test_command_rejection_is_terminal():
    state, _ = advance(State(), 1., action="prime")
    state, _ = advance(state, 1.21, action="enter")
    state, output = transition(state, replace(inputs(1.22),
        ack=Ack(sample(1.22), 176, 1, 42, 191, False)), config())
    assert state.reason == "COMMAND_REJECTED_1" and not output.heartbeat


def test_handoff_requires_ack_and_status_then_stops_forever():
    state, output = advance(active(), 1.3, stage=14, armed=True, action="handoff")
    assert output.vehicle_command == "loiter" and state.stage == Stage.HANDOFF_WAIT
    assert not output.heartbeat and output.velocity_ned is None
    state, output = transition(state, replace(inputs(1.32, stage=4, armed=True),
        ack=Ack(sample(1.31), 176, 0, 42, 191, False)), config())
    assert state.stage == Stage.RELEASED and not output.heartbeat
    state, output = advance(state, 1.4, action="prime")
    assert state.stage == Stage.RELEASED and output.vehicle_command is None


@pytest.mark.parametrize("factory", [
    lambda: replace(config(), max_command_ttl_s=nan),
    lambda: replace(config(), planner="independent_safety"),
    lambda: replace(config(), source_component=1000),
    lambda: replace(inputs().command, velocity_ned=(nan, 0., 0.)),
    lambda: replace(inputs().permit, free_distance_m=nan),
    lambda: replace(inputs().lease, kind="navigation"),
])
def test_invalid_configuration_and_physical_values_are_rejected(factory):
    with pytest.raises(ValueError):
        factory()


@pytest.mark.parametrize("capture,reason", [
    (Stamp("previous_epoch", 1, 1.3, 1.3), "PERCEPTION_epoch"),
    (Stamp("epoch", 1, .7, 1.3), "PERCEPTION_stale_sim"),
    (Stamp("epoch", 1, 1.4, 1.3), "PERCEPTION_future_sim"),
])
def test_new_permit_does_not_refresh_original_capture(capture, reason):
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, permit=replace(data.permit, capture_stamp=capture))
    state, output = transition(active(), data, config())
    assert state.stage == Stage.REVOKED and state.reason == reason
    assert not output.heartbeat


def test_repeated_capture_with_new_decisions_has_its_own_wall_progress_deadline():
    cfg = replace(config(), progress_wall_s=.04)
    state = active()
    capture = sample(1.25)
    for t in (1.27, 1.3):
        data = inputs(t, stage=14, armed=True)
        data = replace(data, permit=replace(data.permit, capture_stamp=capture))
        state, output = transition(state, data, cfg)
    assert state.reason == "PERCEPTION_PROGRESS_STALLED" and not output.heartbeat


def test_capture_resequence_cannot_relabel_the_same_acquisition_time():
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, permit=replace(data.permit, capture_stamp=sample(1.25, 1300000)))
    state, output = transition(active(), data, config())
    assert state.reason == "PERCEPTION_SEQUENCE" and not output.heartbeat


def test_cached_one_shot_operator_request_is_not_renewable_authority():
    state = active()
    cached = OperatorRequest("test_owner", sample(1.24), "arm")
    for t in (1.4, 1.6, 1.8):
        data = replace(inputs(t, stage=14, armed=True), request=cached)
        state, output = transition(state, data, config())
    assert state.stage == Stage.ACTIVE and output.heartbeat
    stale_new = OperatorRequest("test_owner", sample(1.25), "handoff")
    state, output = transition(state, replace(inputs(1.9, stage=14, armed=True),
                                             request=stale_new), config())
    assert state.reason == "REQUEST_stale_sim" and not output.heartbeat


def test_zero_request_with_measured_drift_cannot_skip_swept_direction():
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, command=replace(data.command, velocity_ned=(0., 0., 0.)),
                   permit=replace(data.permit, velocity_ned=(0., 0., 0.)),
                   odometry=replace(data.odometry, position_ned=(4.99, 0., -1.),
                                    velocity_ned=(.9, 0., 0.)))
    state, output = transition(active(), data, config())
    assert state.reason == "DIRECTION_UNSUPPORTED" and not output.heartbeat


def test_priming_and_mode_wait_do_not_inject_non_offboard_setpoints():
    state, output = advance(State(), 1., action="prime")
    assert output.heartbeat and output.position_ned is None and output.velocity_ned is None
    state, output = advance(state, 1.21, action="enter")
    assert output.heartbeat and output.vehicle_command == "offboard"
    assert output.position_ned is None and output.velocity_ned is None


def test_ack_cached_at_request_time_cannot_confirm_a_command_not_yet_sent():
    state, _ = advance(State(), 1., action="prime")
    ack = Ack(sample(1.21), 176, 0, 42, 191, False)
    state, output = transition(state, replace(inputs(1.21, action="enter"), ack=ack), config())
    assert state.stage == Stage.MODE_WAIT and output.vehicle_command == "offboard"
    state, output = transition(state, replace(inputs(1.22, stage=14), ack=ack), config())
    assert state.stage == Stage.MODE_WAIT and output.velocity_ned is None


def test_status_transition_timestamp_cannot_be_in_the_future():
    with pytest.raises(ValueError, match="transition timestamp"):
        replace(inputs().status, nav_state_since_s=1.1)
    with pytest.raises(ValueError, match="transition timestamp"):
        replace(inputs().status, armed_since_s=1.1)


def test_session_deadline_cannot_be_extended_by_new_operator_leases():
    cfg = replace(config(), session_timeout_s=.2)
    state, output = transition(active(), inputs(1.3, stage=14, armed=True), cfg)
    assert state.reason == "SESSION_TIMEOUT" and not output.heartbeat


@pytest.mark.parametrize("field", ["command", "permit"])
def test_command_and_permit_expire_at_their_exact_original_ttl(field):
    data = inputs(1.5, stage=14, armed=True)
    value = replace(getattr(data, field), stamp=sample(1.25), ttl_s=.25)
    state, output = transition(active(), replace(data, **{field: value}), config())
    assert state.reason == field.upper() + "_EXPIRED" and not output.heartbeat


def test_lease_expires_at_its_exact_wall_deadline():
    lease = Lease("test_owner", sample(1.26), 10., .05, "bootstrap")
    deadline = lease.stamp.received_wall_s + lease.wall_ttl_s
    for wall_s, expected in ((deadline-1e-6, Stage.ACTIVE), (deadline, Stage.REVOKED)):
        data = replace(inputs(wall_s, stage=14, armed=True), lease=lease)
        state, output = transition(active(), data, config())
        assert state.stage == expected
        assert output.heartbeat == (expected == Stage.ACTIVE)
        if expected == Stage.REVOKED:
            assert output.velocity_ned is None and output.vehicle_command is None


def test_offboard_loss_before_mode_ack_is_not_reclaimed():
    state, _ = advance(State(), 1., action="prime")
    state, _ = advance(state, 1.21, action="enter")
    state, _ = advance(state, 1.22, stage=14)
    state, output = advance(state, 1.23, stage=2)
    assert state.reason == "OFFBOARD_LOST" and not output.heartbeat


def test_disarming_before_arm_ack_is_not_reclaimed():
    state = replace(active(), stage=Stage.ARM_WAIT, ack_accepted=False)
    state, output = advance(state, 1.3, stage=14)
    assert state.reason == "ARMING_LOST" and not output.heartbeat


def test_revoke_keeps_first_reason_and_never_changes_released_state():
    from drone_cv.supervisor import revoke
    revoked, _ = revoke(State(), "OWNER_CONFLICT")
    again, output = revoke(revoked, "LATER_CLOCK_FAILURE")
    assert again == revoked and not output.heartbeat
    released = State(stage=Stage.RELEASED, reason="released")
    assert revoke(released, "LATER_FAILURE")[0] == released


def test_wire_permit_preserves_capture_identity_and_rejects_wall_stamp_spoofing():
    import json
    from drone_cv.px4_bridge import decode_json
    wire = {
        "stamp": {"epoch": "epoch", "sequence": 10, "sim_s": 1.3},
        "capture_stamp": {"epoch": "epoch", "sequence": 7, "sim_s": 1.1},
        "source": "independent_safety", "ttl_s": .4, "command_source": "planner",
        "command_sequence": 9, "velocity_ned": [.1, 0., 0.], "free_distance_m": 2.,
    }
    permit = decode_json(json.dumps(wire), "permit", "epoch", 12.)
    assert permit.capture_stamp == Stamp("epoch", 7, 1.1, 12.)
    wire["capture_stamp"]["received_wall_s"] = 99.
    with pytest.raises(ValueError, match="wire stamp"):
        decode_json(json.dumps(wire), "permit", "epoch", 12.)
    del wire["capture_stamp"]
    with pytest.raises(KeyError):
        decode_json(json.dumps(wire), "permit", "epoch", 12.)


def test_heartbeat_only_encoding_does_not_construct_a_trajectory():
    from types import SimpleNamespace
    from drone_cv.px4_bridge import encode
    from drone_cv.supervisor import Output
    types = SimpleNamespace(OffboardControlMode=SimpleNamespace)
    messages = encode(Output(heartbeat=True), 1000000, config(), types)
    assert set(messages) == {"offboard_control_mode"}
    assert messages["offboard_control_mode"].position
    assert not messages["offboard_control_mode"].velocity


def test_velocity_encoding_keeps_all_unused_fields_nan():
    from math import isnan
    from types import SimpleNamespace
    from drone_cv.px4_bridge import encode
    from drone_cv.supervisor import Output
    types = SimpleNamespace(OffboardControlMode=SimpleNamespace, TrajectorySetpoint=SimpleNamespace)
    messages = encode(Output(True, velocity_ned=(.1, 0., 0.), yaw_ned=0.),
                      1000000, config(), types)
    point, mode = messages["trajectory_setpoint"], messages["offboard_control_mode"]
    assert mode.velocity and not mode.position
    assert point.velocity == [.1, 0., 0.]
    assert all(isnan(v) for v in (*point.position, *point.acceleration, *point.jerk, point.yawspeed))


def bare_adapter(state):
    """Callback-only instance: no constructor, node, publishers, init or lock."""
    from types import SimpleNamespace
    from drone_cv.px4_bridge import RosBridge
    adapter = object.__new__(RosBridge)
    adapter.state, adapter.cfg = state, config()
    adapter.authorization = SimpleNamespace(epoch="epoch")
    adapter.latest = {}
    return adapter


def status_message(t, **changes):
    from types import SimpleNamespace
    values = dict(timestamp=round(t*1e6), nav_state=14, arming_state=2, failsafe=False,
                  failsafe_and_user_took_over=False, failsafe_defer_state=0,
                  executor_in_charge=0, pre_flight_checks_pass=True, system_id=1,
                  component_id=1, nav_state_timestamp=1210000, armed_time=1250000)
    return SimpleNamespace(**(values | changes))


@pytest.mark.parametrize("changes", [
    {"failsafe": True}, {"executor_in_charge": 1},
    {"nav_state": 4}, {"arming_state": 1},
])
def test_transient_px4_authority_loss_latches_between_supervisor_ticks(changes):
    adapter = bare_adapter(active())
    adapter._receive_px4("status", status_message(1.3, **changes))
    assert adapter.state.stage == Stage.REVOKED
    first_reason = adapter.state.reason
    adapter._receive_px4("status", status_message(1.31))
    assert adapter.state.stage == Stage.REVOKED and adapter.state.reason == first_reason


def test_rejected_ack_cannot_be_overwritten_by_later_acceptance_before_tick():
    from types import SimpleNamespace
    state = replace(active(), stage=Stage.ARM_WAIT, pending_since=Clock("epoch", 1.24, 0.))
    adapter = bare_adapter(state)
    values = dict(timestamp=1300000, command=400, result=2,
                  target_system=42, target_component=191, from_external=False)
    adapter._receive_px4("ack", SimpleNamespace(**values))
    assert adapter.state.stage == Stage.REVOKED
    adapter._receive_px4("ack", SimpleNamespace(**(values | {"timestamp": 1310000, "result": 0})))
    assert adapter.state.stage == Stage.REVOKED


def test_runtime_idle_has_a_finite_wall_session_without_any_operator_request(monkeypatch):
    from drone_cv import px4_bridge
    adapter = bare_adapter(State())
    adapter.cfg = replace(config(), session_timeout_s=.2)
    adapter.started_wall, adapter.node, adapter.timer = 1., object(), None
    adapter.publishers = {}
    monkeypatch.setattr(px4_bridge.time, "monotonic", lambda: 1.3)
    adapter._tick()
    assert adapter.state.reason == "RUNTIME_SESSION_TIMEOUT"


def test_external_command_identity_cannot_impersonate_px4_target():
    with pytest.raises(ValueError, match="identity"):
        replace(config(), source_system=1, source_component=1)


def test_capture_cannot_follow_the_verdict_that_authorizes_it():
    data = inputs(1.3, stage=14, armed=True)
    data = replace(data, permit=replace(data.permit, stamp=sample(1.28),
                                       capture_stamp=sample(1.29)))
    state, output = transition(active(), data, config())
    assert state.reason == "PERCEPTION_FUTURE_DECISION" and not output.heartbeat


def test_refreshed_permit_receive_time_does_not_extend_capture_wall_age():
    state = active()
    cfg = replace(config(), freshness=FreshnessPolicy(2., .04))
    for t in (1.27, 1.3):
        data = inputs(t, stage=14, armed=True)
        repeated = Stamp("epoch", 1250000, 1.25, t)
        data = replace(data, permit=replace(data.permit, capture_stamp=repeated))
        state, output = transition(state, data, cfg)
    assert state.reason == "PERCEPTION_stale_wall" and not output.heartbeat


def test_unrequested_offboard_entry_is_latched_before_tick():
    state, _ = advance(State(), 1., action="prime")
    adapter = bare_adapter(state)
    adapter._receive_px4("status", status_message(1.3, arming_state=1, armed_time=0))
    assert adapter.state.stage == Stage.REVOKED


@pytest.mark.parametrize("fault", [
    None, "duplicate_owner", "foreign_control", "version_alias", "missing_source",
    "lost_own_endpoint", "changed_gid",
])
def test_dds_exclusivity_checks_endpoint_identity_and_all_control_aliases(fault):
    from types import SimpleNamespace
    from drone_cv.px4_bridge import INPUT_TYPES, OUTPUT_TYPES, graph_exclusive
    namespace, owner = "/vehicle", "test_owner"
    nodes = [(name, namespace) for name in (owner, "planner", "independent_safety", owner + "_operator")]
    if fault == "duplicate_owner":
        nodes.append((owner, namespace))
    publishers, readers = {}, {}
    gids = {key: bytes([i+1]) for i, key in enumerate(OUTPUT_TYPES)}
    def endpoint(name, kind, gid=b"x"):
        return SimpleNamespace(node_name=name, node_namespace=namespace,
                               topic_type=kind, endpoint_gid=gid)
    for key, name in OUTPUT_TYPES.items():
        topic = f"{namespace}/fmu/in/{key}"
        publishers[topic] = [endpoint(owner, f"px4_msgs/msg/{name}", gids[key])]
        readers[topic] = [endpoint("agent", f"px4_msgs/msg/{name}")]
    for key, name in INPUT_TYPES.items():
        publishers[f"{namespace}/fmu/out/{key}"] = [endpoint("agent", f"px4_msgs/msg/{name}")]
    for leaf, name in (("command", "planner"), ("permit", "independent_safety"),
                       ("lease", owner + "_operator"), ("request", owner + "_operator")):
        publishers[f"{namespace}/supervisor/{leaf}"] = [endpoint(name, "std_msgs/msg/String")]
    publishers["/clock"] = [endpoint("simulator", "rosgraph_msgs/msg/Clock")]
    if fault in ("foreign_control", "version_alias"):
        leaf = "actuator_motors" if fault == "foreign_control" else "trajectory_setpoint_v0"
        publishers[f"{namespace}/fmu/in/{leaf}"] = [endpoint("foreign", "px4_msgs/msg/TrajectorySetpoint")]
    elif fault == "missing_source":
        publishers[f"{namespace}/supervisor/permit"] = []
    elif fault == "lost_own_endpoint":
        publishers[f"{namespace}/fmu/in/trajectory_setpoint"] = []
    elif fault == "changed_gid":
        publishers[f"{namespace}/fmu/in/trajectory_setpoint"][0].endpoint_gid = b"replaced"
    node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: nodes,
        get_topic_names_and_types=lambda: [(name, []) for name in publishers],
        get_publishers_info_by_topic=lambda topic: publishers.get(topic, []),
        get_subscriptions_info_by_topic=lambda topic: readers.get(topic, []))
    assert graph_exclusive(node, namespace, owner, "planner", "independent_safety", gids) == (fault is None)
