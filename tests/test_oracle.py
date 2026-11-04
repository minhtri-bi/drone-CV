from copy import deepcopy
from dataclasses import FrozenInstanceError
from math import sqrt

import numpy as np
import pytest

from drone_cv.oracle import ObserverState, evaluate


def profile(parts=None):
    return {
        "schema": 1,
        "frame": "base_link FLU",
        "parts": parts if parts is not None else [part("body", "body", [-1]*3, [1]*3)],
        "source_sha256": {"model.sdf": "a"*64},
    }


def part(identity, role, lo, hi):
    return {"id": identity, "role": role, "lo": lo, "hi": hi, "source": "model.sdf"}


def fixture(surfaces=None, support=None):
    return {
        "surfaces": surfaces if surfaces is not None else [
            {"id": "wall", "lo": [3, -5, -5], "hi": [4, 5, 5]},
        ],
        "support_allowlist": support if support is not None else [],
        "clearance_by_phase": {"rest": 0.0, "navigation": 0.5, "landing": 0.0},
        "max_truth_gap_ns": 100,
    }


def support_scene():
    vehicle = profile([
        part("gear", "gear", [-.2, -.2, -.1], [.2, .2, .1]),
        part("body", "body", [-.5, -.5, .2], [.5, .5, .8]),
        part("propeller", "rotor", [-1, -1, .9], [1, 1, 1]),
    ])
    world = fixture([
        {"id": "floor", "lo": [-5, -5, -1], "hi": [5, 5, 0]},
    ], [{
        "part": "gear", "surface": "floor", "phases": ["rest", "landing"],
        "zone_lo": [-.3, -.3, -.2], "zone_hi": [.3, .3, 0],
    }])
    return vehicle, world


def pair(result, component, surface):
    return next(item for item in result["pairs"]
                if item["part"] == component and item["surface"] == surface)


def sample(unsafe=False):
    world = fixture()
    if unsafe:
        world["surfaces"][0]["lo"][0] = .5
    return evaluate(profile(), np.eye(4), world, "rest")


def observe(state, evaluation=None, sim_ns=0, epoch=0, wall_ns=0, max_gap_ns=100):
    return state.observe(sample() if evaluation is None else evaluation,
                         sim_ns=sim_ns, epoch=epoch, wall_ns=wall_ns,
                         max_gap_ns=max_gap_ns)


@pytest.mark.parametrize("lo,hi,expected", [
    ([4, 5, -1], [5, 6, 1], 5.0),
    ([1, -1, -1], [2, 1, 1], 0.0),
    ([.5, -.5, -.5], [2, .5, .5], -.5),
])
def test_analytical_signed_box_separation(lo, hi, expected):
    result = evaluate(profile(), np.eye(4), fixture([
        {"id": "box", "lo": lo, "hi": hi},
    ]), "rest")
    assert result["minimum_clearance_m"] == pytest.approx(expected)
    assert result["pairs"][0]["clearance_m"] == pytest.approx(expected)
    assert result["unsafe"] is (expected <= 0)
    assert result["geometry_scope"] == "conservative_world_aabb_not_physics_contact"


def test_rotated_tilted_geometry_encloses_all_eight_corners():
    c = 1/sqrt(2)
    transform = np.array([[c, -c, 0, 3], [0, 0, -1, 4],
                          [c, c, 0, 5], [0, 0, 0, 1]])
    vehicle = profile([part("body", "body", [-2, -1, -.5], [2, 1, .5])])
    world = fixture([{"id": "wall", "lo": [8, 3, 1], "hi": [9, 5, 9]}])
    # Independently: x/z half-extents are (2+1)/sqrt(2), y half-extent is .5.
    result = evaluate(vehicle, transform, world, "rest")
    assert result["minimum_clearance_m"] == pytest.approx(5-3/sqrt(2))
    assert result["unsafe"] is False


def test_exact_gear_support_is_allowed_but_raw_clearance_is_retained():
    vehicle, world = support_scene()
    result = evaluate(vehicle, np.eye(4), world, "rest")
    gear = pair(result, "gear", "floor")
    assert gear["clearance_m"] == pytest.approx(-.1)
    assert gear["allowed_support"] is True
    assert gear["unsafe"] is False
    assert gear["reason"] == "ALLOWED_SUPPORT"
    assert result["minimum_clearance_m"] == pytest.approx(-.1)
    assert result["unsafe"] is False


def test_allowed_support_does_not_exempt_simultaneous_body_wall_intersection():
    vehicle, world = support_scene()
    world["surfaces"].append({"id": "wall", "lo": [.4, -1, .2], "hi": [.6, 1, .8]})
    result = evaluate(vehicle, np.eye(4), world, "rest")
    assert len(result["pairs"]) == 6
    assert pair(result, "gear", "floor")["allowed_support"] is True
    assert pair(result, "body", "wall")["unsafe"] is True
    assert pair(result, "body", "wall")["reason"] == "GEOMETRIC_INTERSECTION"
    assert result["unsafe"] is True


def test_rotor_intersection_is_unsafe_even_with_explicit_allowlist():
    vehicle, world = support_scene()
    world["surfaces"].append({"id": "rotor_box", "lo": [.9, -.1, .9],
                               "hi": [1.1, .1, 1.1]})
    world["support_allowlist"].append({
        "part": "propeller", "surface": "rotor_box", "phases": ["rest"],
        "zone_lo": [-5]*3, "zone_hi": [5]*3,
    })
    result = evaluate(vehicle, np.eye(4), world, "rest")
    assert pair(result, "propeller", "rotor_box")["unsafe"] is True
    assert pair(result, "propeller", "rotor_box")["allowed_support"] is False


@pytest.mark.parametrize("change", ["zone", "phase", "pair", "role", "missing"])
def test_support_requires_exact_pair_phase_role_and_full_intersection_zone(change):
    vehicle, world = support_scene()
    phase = "rest"
    if change == "zone":
        world["support_allowlist"][0]["zone_hi"][0] = .1
    elif change == "phase":
        phase = "navigation"
    elif change == "pair":
        world["support_allowlist"][0]["part"] = "body"
    elif change == "role":
        vehicle["parts"][0]["role"] = "body"
    else:
        world["support_allowlist"] = []
    result = evaluate(vehicle, np.eye(4), world, phase)
    assert pair(result, "gear", "floor")["allowed_support"] is False
    assert pair(result, "gear", "floor")["unsafe"] is True


def test_touching_support_footprint_must_fit_zone_not_only_contact_center():
    vehicle, world = support_scene()
    vehicle["parts"][0]["lo"][2] = 0
    world["support_allowlist"][0]["zone_hi"][0] = .1
    result = evaluate(vehicle, np.eye(4), world, "rest")
    assert pair(result, "gear", "floor")["clearance_m"] == 0
    assert pair(result, "gear", "floor")["unsafe"] is True


def test_separate_partial_zones_do_not_exempt_the_whole_intersection():
    vehicle, world = support_scene()
    second = deepcopy(world["support_allowlist"][0])
    world["support_allowlist"][0]["zone_hi"][0] = 0
    second["zone_lo"][0] = 0
    world["support_allowlist"].append(second)
    assert evaluate(vehicle, np.eye(4), world, "rest")["unsafe"] is True


def test_touching_gear_support_allows_the_full_zero_thickness_footprint():
    vehicle, world = support_scene()
    vehicle["parts"][0]["lo"][2] = 0
    result = evaluate(vehicle, np.eye(4), world, "rest")
    assert pair(result, "gear", "floor")["clearance_m"] == 0
    assert pair(result, "gear", "floor")["allowed_support"] is True
    assert result["unsafe"] is False


def test_support_zone_must_enclose_intersection_depth_as_well_as_footprint():
    vehicle, world = support_scene()
    world["support_allowlist"][0]["zone_lo"][2] = -.05
    assert evaluate(vehicle, np.eye(4), world, "rest")["unsafe"] is True


def test_zero_thickness_decorative_geometry_is_transformed_without_inflation():
    vehicle = profile([part("plane", "body", [-1, -2, 0], [1, 2, 0])])
    transform = np.array([[0, 0, 1, 2], [0, 1, 0, 0],
                          [-1, 0, 0, 0], [0, 0, 0, 1]])
    world = fixture([{"id": "wall", "lo": [3, -5, -5], "hi": [3, 5, 5]}])
    result = evaluate(vehicle, transform, world, "rest")
    assert result["minimum_clearance_m"] == 1
    assert result["unsafe"] is False


@pytest.mark.parametrize("distance,unsafe", [(.49, True), (.5, False), (.51, False)])
def test_navigation_margin_applies_once_to_physical_bounds(distance, unsafe):
    world = fixture([{"id": "wall", "lo": [1+distance, -5, -5], "hi": [3, 5, 5]}])
    result = evaluate(profile(), np.eye(4), world, "navigation")
    assert result["minimum_clearance_m"] == pytest.approx(distance)
    assert result["required_clearance_m"] == .5
    assert result["unsafe"] is unsafe


@pytest.mark.parametrize("kind", ["shape", "nan", "scale", "reflection", "bottom", "translation"])
def test_rejects_nonrigid_or_nonfinite_transforms(kind):
    transform = np.eye(4)
    if kind == "shape":
        transform = np.eye(3)
    elif kind == "nan":
        transform[0, 0] = np.nan
    elif kind == "scale":
        transform[0, 0] = 1.01
    elif kind == "reflection":
        transform[0, 0] = -1
    elif kind == "bottom":
        transform[3, 0] = .1
    else:
        transform[0, 3] = np.inf
    with pytest.raises(ValueError):
        evaluate(profile(), transform, fixture(), "rest")


@pytest.mark.parametrize("kind", [
    "schema", "frame", "empty_parts", "duplicate_part", "role", "source", "hash",
    "hash_path", "hashes_missing", "inverted_part", "nonfinite_part", "empty_surfaces",
    "duplicate_surface", "inverted_surface", "phase", "negative_margin", "nan_margin",
    "zero_gap", "bool_gap", "unknown_part", "unknown_surface", "unknown_phase",
    "empty_support_phases", "inverted_zone", "nonfinite_zone",
])
def test_rejects_invalid_profile_or_fixture_boundaries(kind):
    vehicle, world = support_scene()
    phase = "rest"
    if kind == "schema":
        vehicle["schema"] = True
    elif kind == "frame":
        vehicle["frame"] = "world"
    elif kind == "empty_parts":
        vehicle["parts"] = []
    elif kind == "duplicate_part":
        vehicle["parts"].append(deepcopy(vehicle["parts"][0]))
    elif kind == "role":
        vehicle["parts"][0]["role"] = "support"
    elif kind == "source":
        vehicle["parts"][0]["source"] = ""
    elif kind == "hash":
        vehicle["source_sha256"]["model.sdf"] = "not-a-sha256"
    elif kind == "hash_path":
        vehicle["source_sha256"] = {"../model.sdf": "a"*64}
    elif kind == "hashes_missing":
        vehicle["source_sha256"] = {}
    elif kind == "inverted_part":
        vehicle["parts"][0]["lo"][0] = 100
    elif kind == "nonfinite_part":
        vehicle["parts"][0]["hi"][0] = np.inf
    elif kind == "empty_surfaces":
        world["surfaces"] = []
    elif kind == "duplicate_surface":
        world["surfaces"].append(deepcopy(world["surfaces"][0]))
    elif kind == "inverted_surface":
        world["surfaces"][0]["lo"][0] = 100
    elif kind == "phase":
        phase = "unknown"
    elif kind == "negative_margin":
        world["clearance_by_phase"]["rest"] = -1
    elif kind == "nan_margin":
        world["clearance_by_phase"]["rest"] = np.nan
    elif kind == "zero_gap":
        world["max_truth_gap_ns"] = 0
    elif kind == "bool_gap":
        world["max_truth_gap_ns"] = True
    elif kind == "unknown_part":
        world["support_allowlist"][0]["part"] = "missing"
    elif kind == "unknown_surface":
        world["support_allowlist"][0]["surface"] = "missing"
    elif kind == "unknown_phase":
        world["support_allowlist"][0]["phases"] = ["missing"]
    elif kind == "empty_support_phases":
        world["support_allowlist"][0]["phases"] = []
    elif kind == "inverted_zone":
        world["support_allowlist"][0]["zone_hi"][0] = -100
    else:
        world["support_allowlist"][0]["zone_hi"][0] = np.nan
    with pytest.raises(ValueError):
        evaluate(vehicle, np.eye(4), world, phase)


def test_complete_healthy_samples_only_claim_sampled_fixture_observation():
    initial = ObserverState()
    first = observe(initial)
    state = observe(first, sim_ns=100, wall_ns=100)
    result = state.finish(complete=True)
    assert initial.ledger == ()
    assert len(first.ledger) == 1
    assert result["safety_outcome"] == "NO_VIOLATION_OBSERVED"
    assert result["execution_outcome"] == "COMPLETED"
    assert result["sample_count"] == 2
    assert result["coverage_complete"] is True
    assert result["continuous_clearance_certified"] is False
    assert result["scope"] == "sampled_static_fixture_only_no_inter_sample_swept_volume_certificate"
    assert len(result["coverage_intervals"]) == 1
    assert result["coverage_intervals"][0]["continuous_clearance_certified"] is False
    with pytest.raises(FrozenInstanceError):
        setattr(state, "observed_unsafe", True)


@pytest.mark.parametrize("unsafe", [False, True])
def test_large_wall_interval_at_low_rtf_does_not_invent_simulation_coverage_gap(unsafe):
    state = observe(ObserverState(), sample(unsafe))
    state = observe(state, sim_ns=10, wall_ns=1_000_000)
    result = state.finish(complete=True)
    assert result["safety_outcome"] == ("UNSAFE_OBSERVED" if unsafe
                                         else "NO_VIOLATION_OBSERVED")
    assert result["execution_outcome"] == "COMPLETED"
    assert result["coverage_complete"] is True
    assert result["coverage_gaps"] == []
    assert len(result["coverage_intervals"]) == 1
    assert result["coverage_intervals"][0]["continuous_clearance_certified"] is False


@pytest.mark.parametrize("complete", [True, False])
def test_no_samples_never_claims_safety(complete):
    result = ObserverState().finish(complete=complete)
    assert result["safety_outcome"] == "UNKNOWN"
    assert result["execution_outcome"] == "INCOMPLETE"
    assert result["coverage_complete"] is False
    assert "NO_OBSERVATIONS" in result["reasons"]


def test_incomplete_collection_never_claims_safety():
    result = observe(ObserverState()).finish(complete=False)
    assert result["safety_outcome"] == "UNKNOWN"
    assert result["execution_outcome"] == "INCOMPLETE"
    assert result["coverage_gaps"][-1]["reason"] == "COLLECTION_INCOMPLETE"


@pytest.mark.parametrize("unsafe", [False, True])
def test_observer_loss_preserves_safety_and_execution_independently(unsafe):
    state = observe(ObserverState(), sample(unsafe)).fail("LOGGER_LOST")
    result = state.finish(complete=False)
    assert result["safety_outcome"] == ("UNSAFE_OBSERVED" if unsafe else "UNKNOWN")
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert "LOGGER_LOST" in result["execution_reasons"]
    assert "LOGGER_LOST" in result["reasons"]
    assert result["coverage_complete"] is False


@pytest.mark.parametrize("sim_ns,epoch,wall_ns,reason", [
    (101, 0, 1, "SIM_SAMPLE_GAP"),
    (0, 0, 1, "SIM_STAMP_NOT_ADVANCING"),
    (1, 0, 0, "WALL_STAMP_NOT_ADVANCING"),
    (1, 1, 1, "EPOCH_CHANGED"),
])
def test_gap_nonprogress_and_epoch_reset_remain_unknown_after_recovery(
        sim_ns, epoch, wall_ns, reason):
    state = observe(ObserverState())
    state = observe(state, sim_ns=sim_ns, epoch=epoch, wall_ns=wall_ns)
    state = observe(state, sim_ns=sim_ns+1, epoch=epoch, wall_ns=wall_ns+1)
    result = state.finish(complete=True)
    assert result["safety_outcome"] == "UNKNOWN"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert reason in result["reasons"]
    assert any(gap["reason"] == reason for gap in result["coverage_gaps"])
    assert result["coverage_complete"] is False
    assert len(result["ledger"]) == 3


def test_stamp_rewind_and_epoch_return_cannot_restore_coverage():
    state = observe(ObserverState(), sim_ns=10, wall_ns=10)
    state = observe(state, sim_ns=5, epoch=1, wall_ns=20)
    state = observe(state, sim_ns=30, epoch=0, wall_ns=30)
    assert state.finish(complete=True)["safety_outcome"] == "UNKNOWN"
    rewind = observe(observe(ObserverState(), sim_ns=10, wall_ns=10),
                     sim_ns=5, wall_ns=20)
    assert "SIM_STAMP_NOT_ADVANCING" in rewind.finish(complete=True)["reasons"]


def test_unsafe_latch_survives_gap_and_subsequent_healthy_observation():
    state = observe(ObserverState(), sample(True))
    state = observe(state, sim_ns=101, wall_ns=101)
    state = observe(state, sim_ns=102, wall_ns=102)
    result = state.finish(complete=True)
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert result["minimum_clearance_m"] == pytest.approx(-.5)


def test_failure_before_first_sample_stays_unknown_after_recovery():
    state = observe(ObserverState().fail("TRUTH_MISSING"), sim_ns=10, wall_ns=20)
    result = state.finish(complete=True)
    assert result["safety_outcome"] == "UNKNOWN"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert result["coverage_gaps"][0]["start_sim_ns"] is None
    assert result["coverage_gaps"][0]["end_sim_ns"] == 10


def test_evaluation_and_finish_output_mutation_cannot_rewrite_frozen_ledger():
    evaluation = sample()
    state = observe(ObserverState(), evaluation)
    evaluation["pairs"][0]["clearance_m"] = -100
    evaluation["unsafe"] = True
    first = state.finish(complete=True)
    first["ledger"][0]["evaluation"]["pairs"][0]["unsafe"] = True
    assert state.finish(complete=True)["ledger"][0]["evaluation"]["unsafe"] is False
    assert state.finish(complete=True)["minimum_clearance_m"] == 2


@pytest.mark.parametrize("name,value", [
    ("sim_ns", -1), ("sim_ns", True), ("wall_ns", -1),
    ("epoch", -1), ("epoch", "reset"), ("max_gap_ns", 0),
])
def test_observe_rejects_invalid_metadata_without_swallowing_errors(name, value):
    kwargs = {"sim_ns": 0, "epoch": 0, "wall_ns": 0, "max_gap_ns": 100}
    kwargs[name] = value
    with pytest.raises(ValueError):
        ObserverState().observe(sample(), **kwargs)


def test_gap_policy_cannot_change_or_disagree_with_frozen_fixture():
    state = observe(ObserverState())
    with pytest.raises(ValueError):
        observe(state, sim_ns=1, wall_ns=1, max_gap_ns=200)
    evaluation = sample()
    evaluation["max_truth_gap_ns"] = 200
    with pytest.raises(ValueError):
        observe(state, evaluation, sim_ns=1, wall_ns=1, max_gap_ns=200)


def test_validation_error_is_explicitly_routed_through_fail_by_caller():
    state = observe(ObserverState())
    with pytest.raises(ValueError):
        evaluate(profile(), np.zeros((4, 4)), fixture(), "rest")
    state = state.fail("INVALID_TRUTH_POSE")
    assert state.finish(complete=True)["execution_outcome"] == "INFRA_ERROR"
    assert state.finish(complete=True)["safety_outcome"] == "UNKNOWN"


@pytest.mark.parametrize("evaluation", [{}, {"unsafe": False}, None])
def test_observer_rejects_incomplete_evaluation(evaluation):
    with pytest.raises(ValueError):
        ObserverState().observe(evaluation, sim_ns=0, epoch=0, wall_ns=0, max_gap_ns=100)


@pytest.mark.parametrize("invalid_complete", [1, None, "yes"])
def test_fail_reason_and_completion_flag_are_validated(invalid_complete):
    with pytest.raises(ValueError):
        ObserverState().fail("")
    with pytest.raises(ValueError):
        ObserverState().finish(complete=invalid_complete)
