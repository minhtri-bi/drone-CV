"""Read-only context and sticky faults; synthetic values are not flight evidence."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from drone_cv.px4_observation import (
    HARD_FAILURES, HOLD_REQUIREMENTS, BatteryObservation, FallbackObservation,
    HomeObservation, LocalObservation, decode_battery, decode_fallback_flags, decode_home,
    decode_local_position,
)
from drone_cv.px4_runtime_guard import runtime_context_error
from drone_cv.supervisor import Stage, State
from drone_cv.timing import Clock, Stamp
from test_px4_policy import geometry, policy
from test_supervisor import bare_adapter, config, inputs


def context_inputs():
    cfg, context = config(), geometry()
    context = replace(context, landing=replace(context.landing,
        minimum_ned=(-7., -7., -7.), maximum_ned=(7., 7., 1.),
        support_down_m=1., support_plane_down_m=1.25,
        body_provenance=cfg.envelope.bounds_provenance))
    data = inputs(1.)
    stamp = data.now
    acquired = Stamp(stamp.epoch, 1_000_000, stamp.sim_s, stamp.wall_s)
    latest = {
        "home": HomeObservation(acquired, context.home.position_ned, 47., 8., 500., 1),
        "local": LocalObservation(acquired, context.home.local_reference, (0, 0, 0, 0), 0, ()),
        "flags": FallbackObservation(acquired, (), (), True, False, 0),
        "battery": BatteryObservation(acquired, 1, .8, True, 0, 0, False),
        "odometry": data.odometry, "status": data.status,
    }
    return cfg, context, latest, Clock("epoch", 1., 1.)


def flags_message(**updates):
    fields = {"timestamp": 1_000_000, "mode_req_other": 0, "mode_req_manual_control": 0,
              "offboard_control_signal_lost": True, "gcs_connection_lost": False,
              "battery_warning": 0}
    fields = {**fields, **{name: 0 for name, _ in HOLD_REQUIREMENTS},
              **{name: False for _, name in HOLD_REQUIREMENTS},
              **{name: False for name in HARD_FAILURES}, **updates}
    return SimpleNamespace(**fields)


def test_complete_healthy_context_not_rc_link_is_conditional_prerequisite():
    cfg, context, latest, now = context_inputs()
    assert runtime_context_error(latest, now, cfg, context, policy(), State()) is None
    # Headless policy ignores missing manual control, not a lost GCS or Hold capability.
    flags = decode_fallback_flags(flags_message(manual_control_signal_lost=True), "epoch", 1.)
    assert runtime_context_error({**latest, "flags": flags}, now, cfg, context, policy(), State()) is None
    failed = decode_fallback_flags(flags_message(mode_req_local_position=16,
                                                local_position_invalid=True), "epoch", 1.)
    reason = runtime_context_error({**latest, "flags": failed}, now, cfg, context, policy(), State())
    assert reason is not None and reason.startswith("HOLD_UNAVAILABLE:")


@pytest.mark.parametrize("source,change,reason", [
    ("flags", {"gcs_lost": True}, "GCS_LINK_UNHEALTHY"),
    ("flags", {"battery_warning": 1}, "BATTERY_WARNING"),
    ("battery", {"remaining": .1}, "BATTERY_RESERVE_UNHEALTHY"),
    ("battery", {"faults": 1}, "BATTERY_UNHEALTHY"),
    ("battery", {"connected": False}, "BATTERY_UNHEALTHY"),
    ("local", {"invalid": ("dead_reckoning",)}, "LOCAL_REFERENCE_INVALID:"),
    ("home", {"update_count": 2}, "PX4_GEOMETRY:"),
    ("local", {"reset_counters": (0, 1, 0, 0)}, "PX4_GEOMETRY:"),
])
def test_unhealthy_or_changed_context_has_explicit_reason(source, change, reason):
    cfg, context, latest, now = context_inputs()
    changed = {**latest, source: replace(latest[source], **change)}
    actual = runtime_context_error(changed, now, cfg, context, policy(), State())
    assert actual is not None and actual.startswith(reason)


def test_each_dynamic_context_retains_independent_finite_wall_freshness():
    cfg, context, latest, now = context_inputs()
    for source, budget in (("local", "LOCAL_POSITION"), ("flags", "FALLBACK_FLAGS"), ("battery", "BATTERY")):
        changed = {**latest, source: replace(latest[source],
            stamp=Stamp("epoch", 1_000_000, 1., 1.))}
        overdue = replace(now, wall_s=1.+cfg.source(budget).wall_deadline(cfg.clock_contract)+.001)
        # Other channels arrive now; no unrelated stale observation masks this one.
        changed = {name: replace(value, stamp=replace(value.stamp, received_wall_s=overdue.wall_s))
                   if name not in (source, "home") else value for name, value in changed.items()}
        assert runtime_context_error(changed, overdue, cfg, context, policy(), State()) == budget+"_stale_wall"
    ancient_home = replace(latest["home"], stamp=Stamp("epoch", 1, .001, .001))
    assert runtime_context_error({**latest, "home": ancient_home}, now, cfg, context, policy(), State()) is None


def test_missing_context_cannot_be_substituted_by_graph_or_clock_health():
    cfg, context, latest, now = context_inputs()
    assert runtime_context_error({name: value for name, value in latest.items() if name != "battery"},
                                 now, cfg, context, policy(), State()) == "MISSING_BATTERY_CONTEXT"
    offboard = replace(State(), stage=Stage.OFFBOARD_DISARMED)
    assert runtime_context_error(latest, now, cfg, context, policy(), offboard) == "OFFBOARD_SIGNAL_LOST"


def test_hold_loss_is_latched_even_if_next_flags_are_healthy():
    adapter = bare_adapter(State())
    adapter._receive_px4("flags", flags_message(mode_req_local_alt=16, local_altitude_invalid=True))
    first = adapter.state.reason
    assert first == "INVALID_FLAGS:HOLD_UNAVAILABLE:local_altitude_invalid"
    adapter._receive_px4("flags", flags_message(timestamp=1_100_000, offboard_control_signal_lost=False))
    assert adapter.state.stage == Stage.REVOKED and adapter.state.reason == first


def test_home_and_reference_changes_latch_without_auto_rebinding():
    home = SimpleNamespace(timestamp=1_000_000, x=0., y=0., z=0., lat=47., lon=8., alt=500.,
                           update_count=1, valid_lpos=True, valid_hpos=True, valid_alt=True)
    adapter = bare_adapter(State())
    adapter._receive_px4("home", home)
    adapter._receive_px4("home", home)  # An identical static snapshot is not fake source progress.
    assert adapter.state.stage != Stage.REVOKED
    adapter._receive_px4("home", SimpleNamespace(**{**vars(home), "timestamp": 1_100_000, "update_count": 2}))
    assert adapter.state.reason == "INVALID_HOME:frozen Home changed"
    adapter._receive_px4("home", SimpleNamespace(**{**vars(home), "timestamp": 1_200_000}))
    assert adapter.state.reason == "INVALID_HOME:frozen Home changed"
    local = SimpleNamespace(timestamp_sample=1_000_000, ref_lat=47., ref_lon=8., ref_alt=500.,
        ref_timestamp=123, xy_reset_counter=0, z_reset_counter=0, vxy_reset_counter=0,
        vz_reset_counter=0, heading_reset_counter=0, xy_valid=True, z_valid=True, v_xy_valid=True,
        v_z_valid=True, xy_global=True, z_global=True, heading_good_for_control=True, dead_reckoning=False)
    adapter = bare_adapter(State())
    adapter._receive_px4("local", local)
    adapter._receive_px4("local", SimpleNamespace(**{**vars(local), "timestamp_sample": 1_100_000,
                                                    "ref_timestamp": 124}))
    assert adapter.state.reason == "INVALID_LOCAL:frozen local reference or reset changed"


def test_decoders_reject_invalid_boolean_geometry_and_battery_domain():
    with pytest.raises(ValueError):
        decode_home(SimpleNamespace(valid_lpos=1, valid_hpos=True, valid_alt=True), "epoch", 1.)
    with pytest.raises(ValueError):
        decode_battery(SimpleNamespace(timestamp=1, id=1, remaining=-1., connected=True,
                                       warning=0, faults=0, is_powering_off=False), "epoch", 1.)
    with pytest.raises(ValueError):
        decode_local_position(SimpleNamespace(timestamp_sample=1, ref_lat=91., ref_lon=8.,
                                              ref_alt=500., ref_timestamp=1), "epoch", 1.)
