"""Offline moving-contract behavior; these fixtures are not flight evidence."""
from dataclasses import FrozenInstanceError
import importlib.util
import json
from math import pi
from pathlib import Path

import numpy as np
import pytest

from drone_cv.oracle import evaluate
from drone_cv.oracle_observation import MovingObserverState


def profile(lo=(-.1, -.1, -.1), hi=(.1, .1, .1), role="body"):
    return {"schema": 1, "frame": "base_link FLU", "source_sha256": {"model.sdf": "a"*64},
            "parts": [{"id": role, "role": role, "lo": list(lo), "hi": list(hi),
                       "source": "model.sdf"}]}


def fixture(surface=None):
    return {"phase": "rest", "surfaces": [surface or
            {"id": "wall", "lo": [10, -10, -10], "hi": [11, 10, 10]}],
            "support_allowlist": [], "clearance_by_phase":
            {"rest": 0., "takeoff": .5, "navigation": .5, "landing": 0.},
            "max_truth_gap_ns": 1_000_000_000, "moving_contract": {
                "schema": 1, "truth_source": "truth/1", "telemetry_source": "px4/1",
                "support_source": "contact/1", "max_source_age_ns": 0,
                "max_wall_gap_ns": 1_000_000_000, "max_observations": 100,
                "motion": {"translation_speed_m_s": 4., "angular_speed_rad_s": pi,
                           "provenance": "analytic test constraints, not calibrated"},
                "schedule": [
                    {"phase": "rest", "start_sim_ns": 0, "end_sim_ns": 2_000_000_000},
                    {"phase": "takeoff", "start_sim_ns": 2_000_000_000, "end_sim_ns": 3_000_000_000},
                    {"phase": "navigation", "start_sim_ns": 3_000_000_000, "end_sim_ns": 4_000_000_000},
                    {"phase": "landing", "start_sim_ns": 4_000_000_000, "end_sim_ns": 5_000_000_000},
                    {"phase": "rest", "start_sim_ns": 5_000_000_000, "end_sim_ns": 6_000_000_000}],
                "discovery": {"source": "graph/1", "max_gap_ns": 1_000_000_000,
                              "max_snapshots": 100, "max_endpoints": 10,
                              "allowed_readers": [{"gid": "01", "node": "oracle",
                                  "namespace": "/eval", "topic": "/truth"}]}}}


def observation(stamp=0, x=0., angle=0., phase="rest", epoch=0):
    c, s = np.cos(angle), np.sin(angle)
    transform = [[c, -s, 0, x], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    return {"sim_ns": stamp, "wall_ns": stamp, "epoch": epoch, "truth_source": "truth/1",
            "source_ns": stamp, "world_from_body": transform,
            "telemetry": {"source": "px4/1", "source_ns": stamp, "epoch": epoch,
                          "armed": phase != "rest", "landed": phase == "rest",
                          "nav_state": {"rest": "DISARMED", "takeoff": "TAKEOFF",
                                        "navigation": "OFFBOARD", "landing": "LAND"}[phase]},
            "support": {"source": "contact/1", "source_ns": stamp, "epoch": epoch,
                        "contacts": []}}


def graph(stamp, readers=None, error=None):
    return {"wall_ns": stamp, "source": "graph/1", "readers": [] if readers is None else readers,
            "error": error}


def finish(state, end=1_000_000_000):
    return state.discover(graph(0)).discover(graph(end)).finish(complete=True)


def test_thin_crossing_is_possible_not_demonstrated_sampled_intersection():
    world = fixture({"id": "wire", "lo": [-.01, -10, -10], "hi": [.01, 10, 10]})
    state = MovingObserverState.create(profile(), world)
    state = state.observe(observation(x=-1)).observe(observation(1_000_000_000, x=1))
    result = finish(state)
    assert not result["observed_unsafe"]
    assert result["moving_intervals"][0]["outcome"] == "POTENTIAL_VIOLATION"
    assert result["safety_outcome"] == "UNKNOWN"
    assert not result["clearance_conditional_on_declared_motion_bounds"]
    # Analytic straight-line midpoint is a witness, not an observed ledger sample.
    assert evaluate(profile(), observation(x=0)["world_from_body"], world, "rest")["unsafe"]


def test_rotation_only_uses_full_offset_corner_radius():
    world = fixture({"id": "corner", "lo": [1.3, 1.3, -.1], "hi": [1.4, 1.4, .1]})
    world["moving_contract"]["motion"]["translation_speed_m_s"] = 0.
    state = MovingObserverState.create(profile((1.8, -.05, -.05), (2., .05, .05)), world)
    state = state.observe(observation()).observe(observation(1_000_000_000, angle=pi/2))
    result = finish(state)
    interval = result["moving_intervals"][0]
    assert not result["observed_unsafe"]
    assert interval["pairs"][0]["part_radius_m"] > 2.
    assert interval["outcome"] == "POTENTIAL_VIOLATION"
    midpoint = observation(angle=pi/4)["world_from_body"]
    assert evaluate(profile((1.8, -.05, -.05), (2., .05, .05)), midpoint, world, "rest")["unsafe"]


def test_identical_endpoints_do_not_exclude_curved_excursion():
    world = fixture({"id": "obstacle", "lo": [1., -.1, -.1], "hi": [1.1, .1, .1]})
    state = MovingObserverState.create(profile(), world)
    state = state.observe(observation()).observe(observation(1_000_000_000))
    assert finish(state)["moving_intervals"][0]["outcome"] == "POTENTIAL_VIOLATION"
    # A return excursion of length 2 m satisfies the declared 4 m/s path bound.
    assert evaluate(profile(), observation(x=1)["world_from_body"], world, "rest")["unsafe"]


@pytest.mark.parametrize("change,reason", [
    ("motion", "MOTION_BOUNDS_MISSING"), ("source", "TRUTH_SOURCE_INVALID"),
    ("epoch", "EPOCH_CHANGED"), ("gap", "SIM_SAMPLE_GAP"),
    ("rate", "TRANSLATION_BOUND_EXCEEDED"), ("angular", "ANGULAR_BOUND_EXCEEDED")])
def test_unsupported_intervals_are_unknown(change, reason):
    world = fixture()
    if change == "motion":
        del world["moving_contract"]["motion"]
    if change == "rate":
        world["moving_contract"]["motion"]["translation_speed_m_s"] = 0.
    if change == "angular":
        world["moving_contract"]["motion"]["angular_speed_rad_s"] = 0.
    state = MovingObserverState.create(profile(), world).observe(observation())
    item = observation(1_000_000_001 if change == "gap" else 1_000_000_000,
                       x=1 if change == "rate" else 0,
                       angle=.1 if change == "angular" else 0, epoch=1 if change == "epoch" else 0)
    if change == "source":
        item["truth_source"] = "other"
    result = finish(state.observe(item), item["wall_ns"])
    assert reason in result["moving_intervals"][0]["reasons"]
    assert result["moving_intervals"][0]["outcome"] == "UNKNOWN"
    assert result["safety_outcome"] == "UNKNOWN"


def test_exact_gap_and_translation_rate_boundary_is_conditional_only():
    world = fixture()
    state = MovingObserverState.create(profile(), world)
    state = state.observe(observation()).observe(observation(1_000_000_000, x=4))
    result = finish(state)
    assert result["moving_intervals"][0]["outcome"] == "CLEAR"
    assert result["clearance_conditional_on_declared_motion_bounds"]
    assert result["moving_intervals"][0]["clearance_conditional_on_declared_motion_bounds"]
    assert result["motion_bounds_provenance"] == world["moving_contract"]["motion"]["provenance"]
    assert not result["motion_bounds_calibrated"]
    assert not result["independent_motion_bound_evidence_present"]
    assert result["motion_bounds_scope"] == "supplied_offline_intervals_only_no_independent_bound_evidence"
    assert "continuous_clearance_certified" not in result
    assert all("continuous_clearance_certified" not in item
               for item in result["coverage_intervals"] + result["moving_intervals"])
    assert not result["flight_admission"]
    assert not result["live_acceptance_certified"]


def test_sampled_violation_survives_source_loss_and_discovery_error():
    state = MovingObserverState.create(profile(), fixture()).observe(observation(x=10))
    item = observation(1_000_000_000)
    del item["telemetry"]
    result = state.observe(item).discover(graph(0, error="graph unavailable")).finish(complete=False)
    assert result["observed_unsafe"]
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["moving_intervals"][0]["outcome"] == "SAMPLED_VIOLATION"
    assert not result["coverage_complete"]
    assert not result["clearance_conditional_on_declared_motion_bounds"]
    assert not result["moving_intervals"][0]["clearance_conditional_on_declared_motion_bounds"]


def test_phase_schedule_rejects_skips_backwards_and_planner_stage():
    state = MovingObserverState.create(profile(), fixture()).observe(observation())
    skip = observation(3_000_000_000, phase="navigation")
    skip["planner_stage"] = "landing"
    state = state.observe(skip).observe(observation(2_000_000_000, phase="takeoff"))
    result = state.finish(complete=True)
    assert "PHASE_TRANSITION_SKIPPED" in result["reasons"]
    assert "PHASE_TRANSITION_BACKWARD" in result["reasons"]
    assert not result["phase_ledger"][1]["valid"]


def test_frozen_inputs_and_states_do_not_alias_callers():
    world = fixture()
    state = MovingObserverState.create(profile(), world)
    world["moving_contract"]["motion"]["translation_speed_m_s"] = 0.
    sample = observation()
    next_state = state.observe(sample)
    sample["world_from_body"][0][3] = 100
    assert not next_state.finish(complete=False)["observed_unsafe"]
    assert state.finish(complete=False)["sample_count"] == 0
    with pytest.raises(FrozenInstanceError):
        setattr(next_state, "records", ())


def support_world():
    world = fixture({"id": "floor", "lo": [-10, -10, -1], "hi": [10, 10, 0]})
    world["support_allowlist"] = [{"part": "gear", "surface": "floor",
        "phases": ["rest", "landing"], "zone_lo": [-1, -1, -.2], "zone_hi": [1, 1, 0]}]
    return world


def test_early_landing_and_uncorroborated_support_never_grant_exemption():
    world = support_world()
    state = MovingObserverState.create(profile(role="gear"), world)
    item = observation()
    item["planner_stage"] = "landing"
    item["telemetry"].update(armed=True, landed=True, nav_state="LAND")
    item["support"]["contacts"] = [["gear", "floor"]]
    result = state.observe(item).finish(complete=True)
    assert result["safety_outcome"] == "UNKNOWN"
    assert not result["ledger"][0]["evaluation"]["pairs"][0]["allowed_support"]


def test_valid_support_does_not_exempt_body_or_rotor():
    vehicle = profile(role="gear")
    for role in ("body", "rotor"):
        vehicle["parts"].append({"id": role, "role": role, "lo": [-.1]*3, "hi": [.1]*3,
                                  "source": "model.sdf"})
    state = MovingObserverState.create(vehicle, support_world())
    for stamp, phase in [(0, "rest"), (2_000_000_000, "takeoff"),
                         (3_000_000_000, "navigation"), (4_000_000_000, "landing")]:
        item = observation(stamp, phase=phase)
        if phase in {"rest", "landing"}:
            item["telemetry"]["landed"] = True
            item["support"]["contacts"] = [["gear", "floor"]]
        state = state.observe(item)
    pairs = state.finish(complete=True)["ledger"][-1]["evaluation"]["pairs"]
    assert pairs[0]["allowed_support"]
    assert all(pair["unsafe"] for pair in pairs[1:])


def test_unexpected_reader_remains_detected_after_disappearing():
    state = MovingObserverState.create(profile(), fixture())
    state = state.observe(observation()).observe(observation(1_000_000_000))
    bad = {"gid": "99", "node": "planner", "namespace": "/", "topic": "/truth"}
    state = state.discover(graph(0)).discover(graph(500_000_000, [bad])).discover(graph(1_000_000_000))
    result = state.finish(complete=True)
    assert result["discovery"]["unexpected_readers"] == [bad]
    assert not result["discovery"]["cooperative_coverage_complete"]
    assert not result["discovery"]["dds_security_certified"]
    assert result["safety_outcome"] == "UNKNOWN"


@pytest.mark.parametrize("kind", ["missing", "gap", "error", "overflow"])
def test_discovery_coverage_fails_closed(kind):
    world = fixture()
    if kind == "overflow":
        world["moving_contract"]["discovery"]["max_snapshots"] = 1
    state = MovingObserverState.create(profile(), world)
    state = state.observe(observation()).observe(observation(1_000_000_000))
    if kind != "missing":
        state = state.discover(graph(0, error="unavailable" if kind == "error" else None))
        state = state.discover(graph(1_000_000_001 if kind == "gap" else 1_000_000_000))
    assert not state.finish(complete=True)["discovery"]["cooperative_coverage_complete"]


def test_cli_moving_contract_is_attached_to_report_and_exit_code(tmp_path):
    spec = importlib.util.spec_from_file_location("g1_oracle_cli", Path(__file__).parents[1]/"scripts/g1_oracle.py")
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    world = fixture({"id": "wire", "lo": [-.01, -10, -10], "hi": [.01, 10, 10]})
    values = {"profile": profile(), "fixture": world, "observations": {
        "observations": [observation(x=-1), observation(1_000_000_000, x=1)],
        "discovery": [graph(0), graph(1_000_000_000)]}}
    for name, value in values.items():
        (tmp_path/f"{name}.json").write_text(json.dumps(value), encoding="utf-8")
    code = cli.main(["evaluate", "--profile", str(tmp_path/"profile.json"),
                     "--fixture", str(tmp_path/"fixture.json"), "--observations",
                     str(tmp_path/"observations.json"), "--output", str(tmp_path/"report")])
    report = json.loads((tmp_path/"report/report.json").read_text(encoding="utf-8"))
    assert code == 2
    assert report["moving_intervals"][0]["outcome"] == "POTENTIAL_VIOLATION"
    assert report["safety_outcome"] == "UNKNOWN"
    assert report["sampling_scope"].startswith("Declared bounded-motion")


@pytest.mark.parametrize("field,value", [
    ("translation_speed_m_s", -1.), ("translation_speed_m_s", float("nan")),
    ("angular_speed_rad_s", float("inf")), ("provenance", "")])
def test_invalid_motion_declarations_reject_before_observation(field, value):
    world = fixture()
    world["moving_contract"]["motion"][field] = value
    with pytest.raises(ValueError):
        MovingObserverState.create(profile(), world)


def test_exact_angular_boundary_and_large_rotation_saturation():
    world = fixture()
    world["moving_contract"]["motion"]["translation_speed_m_s"] = 0.
    state = MovingObserverState.create(profile(), world)
    result = finish(state.observe(observation()).observe(observation(1_000_000_000, angle=pi)))
    assert result["moving_intervals"][0]["outcome"] == "CLEAR"
    radius = result["moving_intervals"][0]["pairs"][0]["part_radius_m"]
    assert result["moving_intervals"][0]["pairs"][0]["displacement_bound_m"] == pytest.approx(2*radius)


def test_discovery_matches_full_endpoint_identity_not_just_node_name():
    state = MovingObserverState.create(profile(), fixture())
    wrong_gid = {"gid": "02", "node": "oracle", "namespace": "/eval", "topic": "/truth"}
    result = state.discover(graph(0, [wrong_gid])).finish(complete=False)
    assert result["discovery"]["unexpected_readers"] == [wrong_gid]


def test_missing_epoch_and_recovery_remain_unknown():
    state = MovingObserverState.create(profile(), fixture()).observe(observation())
    item = observation(500_000_000)
    del item["epoch"]
    state = state.observe(item).observe(observation(1_000_000_000))
    result = finish(state)
    assert result["moving_intervals"][0]["outcome"] == "UNKNOWN"
    assert not result["coverage_complete"]


def test_capacity_overflow_retains_new_unsafe_latch():
    world = fixture()
    world["moving_contract"]["max_observations"] = 1
    state = MovingObserverState.create(profile(), world).observe(observation())
    result = state.observe(observation(1_000_000_000, x=10)).finish(complete=True)
    assert result["sample_count"] == 1
    assert result["observed_unsafe"]
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert "OBSERVATION_CAPACITY_EXCEEDED" in result["reasons"]


def test_valid_rest_support_requires_independent_report_and_geometry():
    state = MovingObserverState.create(profile(role="gear"), support_world())
    item = observation()
    item["support"]["contacts"] = [["gear", "floor"]]
    result = state.observe(item).finish(complete=True)
    assert result["phase_ledger"][0]["valid"]
    assert result["ledger"][0]["evaluation"]["pairs"][0]["allowed_support"]
    missing = observation()
    result = state.observe(missing).finish(complete=True)
    assert "REST_SUPPORT_UNOBSERVED" in result["reasons"]
    assert not result["phase_ledger"][0]["valid"]


def test_planner_landing_label_cannot_exempt_navigation_contact():
    state = MovingObserverState.create(profile(role="gear"), support_world())
    initial = observation()
    initial["support"]["contacts"] = [["gear", "floor"]]
    state = state.observe(initial)
    airborne = observation(2_000_000_000, phase="takeoff")
    airborne["world_from_body"][2][3] = 2
    state = state.observe(airborne)
    early = observation(3_000_000_000, phase="navigation")
    early["planner_stage"] = "landing"
    result = state.observe(early).finish(complete=True)
    assert result["phase_ledger"][-1]["valid"]
    assert result["phase_ledger"][-1]["phase"] == "navigation"
    assert result["observed_unsafe"]
    assert not result["ledger"][-1]["evaluation"]["pairs"][0]["allowed_support"]


@pytest.mark.parametrize("case", ["thin", "rotation", "curved", "gap", "missing_bounds",
                                  "source_loss", "latch", "phase", "reader", "clear"])
def test_cli_offline_contract_smoke_cases(tmp_path, case):
    spec = importlib.util.spec_from_file_location("g1_oracle_smoke", Path(__file__).parents[1]/"scripts/g1_oracle.py")
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    vehicle, world = profile(), fixture()
    first, last = observation(), observation(1_000_000_000)
    snapshots = [graph(0), graph(1_000_000_000)]
    if case == "thin":
        world["surfaces"] = [{"id": "wire", "lo": [-.01, -10, -10], "hi": [.01, 10, 10]}]
        first, last = observation(x=-1), observation(1_000_000_000, x=1)
    elif case == "rotation":
        vehicle = profile((1.8, -.05, -.05), (2., .05, .05))
        world["surfaces"] = [{"id": "corner", "lo": [1.3, 1.3, -.1], "hi": [1.4, 1.4, .1]}]
        world["moving_contract"]["motion"]["translation_speed_m_s"] = 0.
        last = observation(1_000_000_000, angle=pi/2)
    elif case == "curved":
        world["surfaces"] = [{"id": "excursion", "lo": [1., -.1, -.1], "hi": [1.1, .1, .1]}]
    elif case == "gap":
        last = observation(1_000_000_001)
    elif case == "missing_bounds":
        del world["moving_contract"]["motion"]
    elif case == "source_loss":
        del last["telemetry"]
    elif case == "latch":
        first = observation(x=10)
        del last["telemetry"]
    elif case == "phase":
        last = observation(3_000_000_000, phase="navigation")
    elif case == "reader":
        snapshots.insert(1, graph(500_000_000, [
            {"gid": "bad", "node": "planner", "namespace": "/", "topic": "/truth"}]))
    values = {"profile": vehicle, "fixture": world,
              "observations": {"observations": [first, last], "discovery": snapshots}}
    for name, value in values.items():
        (tmp_path/f"{name}.json").write_text(json.dumps(value), encoding="utf-8")
    code = cli.main(["evaluate", "--profile", str(tmp_path/"profile.json"),
                     "--fixture", str(tmp_path/"fixture.json"), "--observations",
                     str(tmp_path/"observations.json"), "--output", str(tmp_path/"result")])
    report = json.loads((tmp_path/"result/report.json").read_text(encoding="utf-8"))
    assert code == (1 if case == "latch" else 0 if case == "clear" else 2)
    assert report["safety_outcome"] == ("UNSAFE_OBSERVED" if case == "latch" else
                                      "NO_VIOLATION_OBSERVED" if case == "clear" else "UNKNOWN")
    assert report["clearance_conditional_on_declared_motion_bounds"] is (case == "clear")
    assert not report["motion_bounds_calibrated"]
    assert not report["independent_motion_bound_evidence_present"]
    assert report["motion_bounds_provenance"] == (
        None if case == "missing_bounds" else world["moving_contract"]["motion"]["provenance"])
    assert not report["flight_admission"]
    assert not report["live_acceptance_certified"]
    assert "continuous_clearance_certified" not in report
    assert all("continuous_clearance_certified" not in item
               for item in report["moving_intervals"] + report["coverage_intervals"])
    assert report["moving_intervals"][0]["clearance_conditional_on_declared_motion_bounds"] is (
        case in {"clear", "reader"})
    if case in {"thin", "rotation", "curved"}:
        assert report["moving_intervals"][0]["outcome"] == "POTENTIAL_VIOLATION"
    if case == "reader":
        assert report["discovery"]["unexpected_readers"][0]["gid"] == "bad"


@pytest.mark.parametrize("moving,expected", [(False, "CLEAR"), (True, "POTENTIAL_VIOLATION")])
def test_interval_support_requires_whole_enclosed_contact_zone(moving, expected):
    world = support_world()
    world["moving_contract"]["motion"].update(translation_speed_m_s=1. if moving else 0.,
                                               angular_speed_rad_s=0.)
    state = MovingObserverState.create(profile(role="gear"), world)
    for stamp in (0, 1_000_000_000):
        item = observation(stamp)
        item["support"]["contacts"] = [["gear", "floor"]]
        state = state.observe(item)
    interval = finish(state)["moving_intervals"][0]
    assert interval["outcome"] == expected
    assert interval["pairs"][0]["allowed_support_enclosure"] is (not moving)


def test_invalid_truth_source_cannot_create_a_new_demonstrated_violation():
    state = MovingObserverState.create(profile(), fixture())
    item = observation(x=10)
    item["truth_source"] = "untrusted"
    result = state.observe(item).finish(complete=True)
    assert result["safety_outcome"] == "UNKNOWN"
    assert not result["observed_unsafe"]


@pytest.mark.parametrize("change", ["missing", "stale", "future", "epoch", "identity"])
def test_independent_phase_source_failures_are_unknown_not_exemptions(change):
    state = MovingObserverState.create(profile(role="gear"), support_world())
    item = observation()
    item["support"]["contacts"] = [["gear", "floor"]]
    if change == "missing":
        del item["telemetry"]
    elif change == "stale":
        item["sim_ns"] = item["source_ns"] = item["wall_ns"] = 1
        item["support"]["source_ns"] = 1
    elif change == "future":
        item["telemetry"]["source_ns"] = 1
    elif change == "epoch":
        item["telemetry"]["epoch"] = 1
    else:
        item["telemetry"]["source"] = "planner"
    result = state.observe(item).finish(complete=True)
    assert result["safety_outcome"] == "UNKNOWN"
    assert not result["phase_ledger"][0]["valid"]
    assert not result["ledger"][0]["evaluation"]["pairs"][0]["allowed_support"]


def test_lost_phase_authority_never_reenables_later_support():
    state = MovingObserverState.create(profile(role="gear"), support_world())
    initial = observation()
    initial["support"]["contacts"] = [["gear", "floor"]]
    state = state.observe(initial)
    skipped = observation(3_000_000_000, phase="navigation")
    skipped["world_from_body"][2][3] = 2
    state = state.observe(skipped)
    landing = observation(4_000_000_000, phase="landing")
    landing["telemetry"]["landed"] = True
    landing["support"]["contacts"] = [["gear", "floor"]]
    result = state.observe(landing).finish(complete=True)
    assert "PHASE_AUTHORITY_LOST" in result["phase_ledger"][-1]["reasons"]
    assert not result["phase_ledger"][-1]["valid"]
    assert not result["ledger"][-1]["evaluation"]["pairs"][0]["allowed_support"]


@pytest.mark.parametrize("calibrated", [False, True])
def test_declaration_labels_cannot_supply_calibration_evidence(calibrated):
    world = fixture()
    world["moving_contract"]["motion"].update(
        calibrated=calibrated, provenance="calibrated: caller assertion without bound evidence")
    state = MovingObserverState.create(profile(), world)
    report = finish(state.observe(observation()).observe(observation(1_000_000_000)))
    assert report["clearance_conditional_on_declared_motion_bounds"]
    assert not report["motion_bounds_calibrated"]
    assert not report["independent_motion_bound_evidence_present"]
    assert not report["flight_admission"]
    assert not report["live_acceptance_certified"]


def test_prior_failure_and_incomplete_collection_never_aggregate_conditional_success():
    state = MovingObserverState.create(profile(), fixture()).observe(observation())
    failed = state.fail("SOURCE_LOST").observe(observation(1_000_000_000))
    report = finish(failed)
    assert report["moving_intervals"][0]["outcome"] == "UNKNOWN"
    assert not report["moving_intervals"][0]["clearance_conditional_on_declared_motion_bounds"]
    assert not report["clearance_conditional_on_declared_motion_bounds"]
    clear = state.observe(observation(1_000_000_000)).discover(graph(0)).discover(graph(1_000_000_000))
    incomplete = clear.finish(complete=False)
    assert incomplete["moving_intervals"][0]["clearance_conditional_on_declared_motion_bounds"]
    assert not incomplete["clearance_conditional_on_declared_motion_bounds"]


def test_cli_malformed_moving_contract_cannot_fall_back_to_static_claims(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "g1_oracle_invalid", Path(__file__).parents[1]/"scripts/g1_oracle.py")
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    world = fixture()
    world["moving_contract"]["motion"]["translation_speed_m_s"] = -1.
    for name, value in {"profile": profile(), "fixture": world, "observations": {
            "observations": [observation(), observation(1_000_000_000)],
            "discovery": [graph(0), graph(1_000_000_000)]}}.items():
        (tmp_path/f"{name}.json").write_text(json.dumps(value), encoding="utf-8")
    assert cli.main(["evaluate", "--profile", str(tmp_path/"profile.json"),
                     "--fixture", str(tmp_path/"fixture.json"), "--observations",
                     str(tmp_path/"observations.json"), "--output", str(tmp_path/"report")]) == 2
    report = json.loads((tmp_path/"report/report.json").read_text(encoding="utf-8"))
    assert not report["clearance_conditional_on_declared_motion_bounds"]
    assert not report["motion_bounds_calibrated"]
    assert report["motion_bounds_provenance"] == world["moving_contract"]["motion"]["provenance"]
    assert "continuous_clearance_certified" not in report
    assert not report["flight_admission"]


def test_cli_static_contract_retains_sampled_only_schema(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "g1_oracle_static", Path(__file__).parents[1]/"scripts/g1_oracle.py")
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    world = fixture()
    del world["moving_contract"]
    for name, value in {"profile": profile(), "fixture": world, "observations": [
            observation(), observation(1_000_000_000)]}.items():
        (tmp_path/f"{name}.json").write_text(json.dumps(value), encoding="utf-8")
    assert cli.main(["evaluate", "--profile", str(tmp_path/"profile.json"),
                     "--fixture", str(tmp_path/"fixture.json"), "--observations",
                     str(tmp_path/"observations.json"), "--output", str(tmp_path/"report")]) == 0
    report = json.loads((tmp_path/"report/report.json").read_text(encoding="utf-8"))
    assert not report["continuous_clearance_certified"]
    assert "clearance_conditional_on_declared_motion_bounds" not in report
    assert all(not item["continuous_clearance_certified"] for item in report["coverage_intervals"])
    assert not report["flight_admission"]
    assert not report["live_acceptance_certified"]
