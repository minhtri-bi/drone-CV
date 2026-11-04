from dataclasses import replace

import pytest

from drone_cv.empirical_profile import (
    Binding, Identity, ProfileProtocol, Sample, derive_profile, evaluate_profile,
)


HASH = "a" * 64
BINDING = Binding(HASH, 0, 12, "b" * 64, "base_link_origin", "local_ned",
                  "gazebo_sim", ("PX4_timestamp_sample", "Gazebo_Pose_V"))


def identity(sequence):
    return Identity(HASH, sequence, sequence * 1_000_000, f"{sequence:064x}")


def protocol(**changes):
    result = ProfileProtocol("frozen-before-outcomes", (BINDING,), (identity(1),),
        (identity(2),), "retrospective_derivation_disjoint", "",
        (.01, .01, .01), (.01, .01, .01), (.01, .01, .01), (.01, .01, .01),
        (0., 0., 0.), (0., 0., 0.), "Zero added margin; descriptive sample maximum only",
        "Unchanged synthetic health reference; no tuning", "frozen workload identity",
        "instantaneous", (50., 95., 99., 100.))
    return replace(result, **changes)


def sample(sequence, **changes):
    result = Sample(identity(sequence), BINDING, (-.002, .003, -.004),
                    (-.001, .002, -.003), (.005, .006, .007), (.005, .006, .007),
                    "instantaneous", (), (sequence * 1_000_000, sequence * 1_000_000))
    return replace(result, **changes)


def test_signed_errors_and_exact_boundaries_are_descriptive_not_authority():
    profile = derive_profile(protocol(), [sample(1)])
    result = evaluate_profile(profile, [sample(2)])
    assert result["heldout_trial_criteria_met"]
    assert not result["runtime_use_eligible"]
    assert not result["runtime_admission"]
    assert result["holdout_kind"] == "retrospective_derivation_disjoint"
    assert result["candidate"]["position_axis_m"] == [.002, .003, .004]
    assert result["heldout"]["position_error"][0]["signed_min"] == -.002


def test_reports_every_violation_and_missing_identity_without_fitting_holdout():
    profile = derive_profile(protocol(heldout=(identity(2), identity(3))), [sample(1)])
    result = evaluate_profile(profile, [sample(2, position_error_ned=(.1, -.2, .3),
                                                  position_variance=(.02, .03, .04))])
    assert not result["heldout_trial_criteria_met"]
    assert len(result["heldout"]["violations"]) == 6
    assert result["heldout"]["uncovered"][0]["identity"]["sequence"] == 3
    assert result["candidate"]["position_axis_m"] == [.002, .003, .004]


def test_finite_difference_never_covers_instantaneous_velocity():
    rows = [sample(i, velocity_scope="interval_average") for i in (1, 2)]
    profile = derive_profile(protocol(), rows[:1])
    result = evaluate_profile(profile, rows[1:])
    assert "INSTANTANEOUS_VELOCITY_UNCOVERED" in result["refusal_reasons"]
    assert not result["heldout_trial_criteria_met"]
    assert result["heldout"]["velocity_scopes"] == ["interval_average"]


@pytest.mark.parametrize("field,value", [
    ("clock_epoch", 1), ("reset_counter", 13), ("alignment_sha256", "c" * 64),
    ("body_point", "camera_optical_origin"), ("frame", "world_enu"),
    ("clock_source", "wall"),
])
def test_binding_mismatch_stays_uncovered(field, value):
    profile = derive_profile(protocol(), [sample(1)])
    bad = sample(2, binding=replace(BINDING, **{field: value}))
    result = evaluate_profile(profile, [bad])
    assert result["heldout"]["uncovered"][0]["reasons"] == ["SOURCE_BINDING_MISMATCH"]
    assert not result["runtime_use_eligible"]


def test_rejects_overlapping_membership_and_nonfinite_contract():
    with pytest.raises(ValueError, match="DISJOINT"):
        protocol(heldout=(identity(1),))
    with pytest.raises(ValueError, match="finite"):
        protocol(position_margin_m=(float("nan"), 0., 0.))
    with pytest.raises(ValueError, match="finite"):
        sample(1, position_error_ned=(float("inf"), 0., 0.))


def test_unexpected_and_duplicate_samples_cannot_silently_disappear():
    profile = derive_profile(protocol(), [sample(1)])
    result = evaluate_profile(profile, [sample(2), sample(2), sample(3)])
    assert not result["heldout_trial_criteria_met"]
    assert len(result["heldout"]["unexpected"]) == 1
    assert result["heldout"]["uncovered"][0]["reasons"] == ["DUPLICATE_SAMPLE"]


def test_three_axis_box_is_not_mislabeled_as_component_radius():
    profile = derive_profile(protocol(), [sample(1, position_error_ned=(.01, .01, .01))])
    result = evaluate_profile(profile, [sample(2)])
    assert result["candidate"]["position_box_enclosing_radius_m"] == pytest.approx(3 ** .5 * .01)
    assert not result["deterministic_bound_certified"]


def test_health_limits_never_derived_from_observed_maximum():
    profile = derive_profile(protocol(), [sample(1, position_variance=(.03, .03, .03))])
    result = evaluate_profile(profile, [sample(2)])
    assert not result["heldout_trial_criteria_met"]
    assert result["candidate"]["position_variance_ceiling"] == [.01, .01, .01]
    assert len(result["derivation"]["violations"]) == 3


def test_protocol_roundtrip_hash_is_stable_and_rejects_empty_rationale():
    original = protocol()
    assert ProfileProtocol.from_dict(original.to_dict()).sha256 == original.sha256
    with pytest.raises(ValueError, match="rationale"):
        protocol(health_rationale="")


def test_missing_truth_preserves_observed_health_and_scalar_norm_exceedance():
    p = protocol(heldout=(identity(2), identity(3)))
    profile = derive_profile(p, [sample(1)])
    result = evaluate_profile(profile, [
        sample(2, position_error_ned=None, velocity_error_ned=None,
               position_variance=(.03, .04, .05), velocity_scope="missing",
               rejection_reasons=("CONTINUOUS_TRUTH_MISSING",), truth_bracket_ns=None),
        sample(3, position_error_ned=(.009, .009, .009))])
    assert result["heldout"]["position_health"][0]["absolute_max"] == .03
    assert len(result["heldout"]["violations"]) >= 3
    assert result["unchanged_reference"]["heldout"]["position_radius_violations"][0]["identity"]["sequence"] == 3
    assert not result["heldout_trial_criteria_met"]


def test_conditional_helper_recomputes_embedded_evaluation_and_exact_envelope():
    from drone_cv.empirical_profile import Applicability, empirical_trial_refusals
    app = Applicability("image", "firmware", "messages", HASH, "gazebo_sim",
                        .2, 1., 1., "frozen workload identity", "base_link_origin", "local_ned")
    p = protocol(holdout_kind="untouched_blinded", blinding_evidence_sha256=HASH,
                 tracking_error_m=.01, tracking_evidence_sha256=HASH, applicability=app)
    profile = derive_profile(p, [sample(1)])
    evaluation = evaluate_profile(profile, [sample(2)])
    candidate = evaluation["candidate"]
    envelope = dict(position_variance_max=(.01, .01, .01), velocity_variance_max=(.01, .01, .01),
                    velocity_error_mps=tuple(candidate["velocity_axis_mps"]),
                    position_error_m=candidate["position_box_enclosing_radius_m"], tracking_error_m=.01)
    assert empirical_trial_refusals(profile.to_dict(), evaluation, **envelope) == ()
    assert not evaluation["runtime_admission"] and not evaluation["runtime_use_eligible"]
    assert empirical_trial_refusals(profile.to_dict(), evaluation, **{**envelope, "position_error_m": .01}) == ("EMPIRICAL_ENVELOPE_MISMATCH",)
    tampered = {**evaluation, "trial_profile_eligible": False}
    assert empirical_trial_refusals(profile.to_dict(), tampered, **envelope) == ("EVALUATION_CONTENT_OR_HASH_MISMATCH",)


def test_different_sequence_or_hash_cannot_alias_same_source_stamp_in_holdout():
    alias = replace(identity(1), sequence=2, serialized_sha256="f" * 64)
    with pytest.raises(ValueError, match="DISJOINT"):
        protocol(heldout=(alias,))
