from dataclasses import asdict, replace
from math import pi

import numpy as np
import pytest

from drone_cv.empirical_profile import Binding, Identity, digest
from drone_cv.relative_profile import (
    CaptureQueryBound, PoseObservation, RelativeProtocol, evaluate_relative_profile,
    relative_profile_payload, verify_capture_query_bound,
)


H = "a" * 64
J = "b" * 64
MODEL = "c" * 64
BINDINGS = tuple(Binding(h, 0, 12, "d" * 64, "base_link_origin", "local_ned",
                         "gazebo_sim", ("PX4_timestamp_sample_NED", "Gazebo_Pose_V_world_ENU"))
                 for h in (H, J))


def identity(source, sequence, stamp):
    return Identity(source, sequence, stamp, digest([source, sequence, stamp]))


def matrix(x=0., angle=0.):
    c, s = np.cos(angle), np.sin(angle)
    return ((float(c), float(-s), 0., x), (float(s), float(c), 0., 0.),
            (0., 0., 1., 0.), (0., 0., 0., 1.))


def setup_profile(*, errors=(.01, .01), ttl=.5, rotation=0.):
    captures = tuple(identity(h, 1, 0) for h in (H, J))
    queries = tuple(identity(h, 2, 500_000_000) for h in (H, J))
    protocol = RelativeProtocol("test-frozen", BINDINGS, captures, queries, (H,), (J,),
                                ttl, 2., .05, .05, "retrospective_source_disjoint",
                                "", MODEL, "Synthetic body-point ball, not calibrated", "e" * 64)
    rows = []
    for binding, capture, query in zip(BINDINGS, captures, queries):
        rows.extend([
            PoseObservation("capture", capture, binding, matrix(errors[0]), matrix(), ()),
            PoseObservation("query", query, binding, matrix(errors[1], rotation), matrix(), ()),
        ])
    return protocol, rows


def payload_and_bound():
    protocol, rows = setup_profile()
    payload = relative_profile_payload(protocol, rows)
    evaluation = evaluate_relative_profile(payload)
    bound = CaptureQueryBound(.5, .05, .05, 2., protocol.source_sha256,
        evaluation["evaluation_sha256"], protocol.provenance_sha256, MODEL,
        "base_link_origin", "gazebo_sim", "trial-epoch-0", 12)
    return protocol, payload, evaluation, bound


def verify(bound, payload, evaluation, **changes):
    kwargs = dict(permit_age_s=.5, max_rho_m=2., capture_clock_epoch="trial-epoch-0",
                  query_clock_epoch="trial-epoch-0", capture_reset_counter=12, query_reset_counter=12,
                  model_sha256=MODEL, body_point="base_link_origin", clock_source="gazebo_sim",
                  provenance_sha256=bound.provenance_sha256,
                  capture_source_sha256=bound.capture_source_sha256,
                  query_source_sha256=bound.query_source_sha256)
    kwargs.update(changes)
    return verify_capture_query_bound(bound, payload, evaluation, **kwargs)


def test_constant_common_absolute_bias_cancels_without_hiding_absolute_error():
    protocol, rows = setup_profile(errors=(.04, .04))
    result = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    report = result["derivation"]
    assert report["pair_count"] == report["covered_pair_count"] == 1
    assert report["metrics"]["translation_m"]["maximum"] == pytest.approx(0.)
    assert report["absolute"]["capture"]["axis_ned"][1]["absolute_max"] == .04
    assert report["absolute"]["query"]["radius_m"]["maximum"] == .04
    assert not result["runtime_admission"]


def test_two_small_absolute_errors_can_violate_relative_claim():
    protocol, rows = setup_profile(errors=(.04, -.04))
    result = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    report = result["heldout"]
    assert report["absolute"]["capture"]["radius_m"]["maximum"] < .05
    assert report["absolute"]["query"]["radius_m"]["maximum"] < .05
    assert report["metrics"]["translation_m"]["maximum"] == pytest.approx(.08)
    assert report["violations"][0]["capture"]["source_ns"] == 0
    assert "translation_m" in report["violations"][0]["fields"]


@pytest.mark.parametrize("angle", [.001, pi / 2, pi - 1e-9, pi])
def test_rotation_uses_whole_point_lever_arm_including_near_pi(angle):
    protocol, rows = setup_profile(errors=(0., 0.), rotation=angle)
    result = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    metrics = result["heldout"]["metrics"]
    assert metrics["rotation_rad"]["maximum"] == pytest.approx(angle)
    assert metrics["displacement_m"]["maximum"] == pytest.approx(4 * np.sin(angle / 2))
    _, _, _, bound = payload_and_bound()
    changed = replace(bound, rotation_rad=angle, translation_m=.03)
    assert changed.displacement_m == pytest.approx(.03 + 4 * np.sin(angle / 2))


def test_ttl_boundaries_and_exhaustive_nonadjacent_counts():
    protocol, rows = setup_profile(errors=(0., 0.))
    captures = tuple(identity(h, i + 1, stamp) for h in (H, J)
                     for i, stamp in enumerate((0, 250_000_000, 500_000_000)))
    queries = tuple(identity(h, i + 10, stamp) for h in (H, J)
                    for i, stamp in enumerate((0, 250_000_000, 500_000_000, 500_000_001)))
    protocol = replace(protocol, captures=captures, queries=queries)
    rows = [PoseObservation(role, item, BINDINGS[0 if item.manifest_sha256 == H else 1],
                            matrix(), matrix(), ())
            for role, identities in (("capture", captures), ("query", queries)) for item in identities]
    result = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    for split in ("derivation", "heldout"):
        assert result[split]["pair_count"] == 8
        assert result[split]["maximum_age_s"] == .5
        assert result[split]["covered_pair_count"] == 8
        assert not result[split]["missing_pairs"]


@pytest.mark.parametrize("field,value", [("clock_epoch", 1), ("reset_counter", 13),
    ("body_point", "model_origin"), ("alignment_sha256", "f" * 64),
    ("clock_source", "wall"), ("frame", "world_enu")])
def test_binding_mismatch_preserves_missing_pair_population(field, value):
    protocol, rows = setup_profile()
    rows[-1] = replace(rows[-1], binding=replace(BINDINGS[1], **{field: value}))
    report = evaluate_relative_profile(relative_profile_payload(protocol, rows))["heldout"]
    assert report["pair_count"] == 1
    assert report["covered_pair_count"] == 0
    assert report["missing_pairs"][0]["query"]["manifest_sha256"] == J
    assert "SOURCE_BINDING_MISMATCH" in report["missing_pairs"][0]["reasons"]


def test_source_disjoint_splits_are_mandatory_and_aliases_cannot_cross():
    protocol, _ = setup_profile()
    with pytest.raises(ValueError, match="SOURCE_SPLIT"):
        replace(protocol, heldout_sources=(H,))
    with pytest.raises(ValueError, match="MEMBERSHIP"):
        replace(protocol, captures=(*protocol.captures, protocol.captures[0]))


def test_schema_two_missing_truth_is_uncovered_never_upgraded():
    protocol, rows = setup_profile()
    rows[-1] = replace(rows[-1], truth_world_from_body=None,
                       rejection_reasons=("CONTINUOUS_TRUTH_MISSING_SCHEMA_2",))
    report = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    assert not report["sampled_claim_met"]
    assert report["heldout"]["missing_pairs"][0]["reasons"] == ["CONTINUOUS_TRUTH_MISSING_SCHEMA_2"]
    assert report["heldout"]["uncovered_observations"]
    assert not report["runtime_admission"]


def test_recomputation_not_claimed_labels_controls_relative_binding():
    _, payload, evaluation, bound = payload_and_bound()
    reasons = verify(bound, payload, evaluation)
    assert "RELATIVE_NO_UNTOUCHED_SOURCE_HOLDOUT" in reasons
    assert "RELATIVE_CONDITIONAL_MODEL_SCOPE_MISSING" in reasons
    tampered = {**evaluation, "runtime_admission": True}
    assert verify(bound, payload, tampered) == ("RELATIVE_EVALUATION_CONTENT_OR_HASH_MISMATCH",)
    empty = {**payload, "observations": []}
    empty["profile_sha256"] = digest({k: v for k, v in empty.items() if k != "profile_sha256"})
    report = evaluate_relative_profile(empty)
    assert not report["sampled_claim_met"]
    assert report["heldout"]["covered_pair_count"] == 0


@pytest.mark.parametrize("changes,reason", [
    ({"permit_age_s": .500000001}, "RELATIVE_PERMIT_AGE_OUT_OF_SCOPE"),
    ({"permit_age_s": -1.}, "RELATIVE_PERMIT_AGE_OUT_OF_SCOPE"),
    ({"max_rho_m": 2.00001}, "RELATIVE_RHO_OUT_OF_SCOPE"),
    ({"capture_clock_epoch": 1}, "RELATIVE_EPOCH_RESET_SCOPE_MISMATCH"),
    ({"query_reset_counter": 13}, "RELATIVE_EPOCH_RESET_SCOPE_MISMATCH"),
    ({"model_sha256": "f" * 64}, "RELATIVE_MODEL_SCOPE_MISMATCH"),
    ({"capture_source_sha256": H}, "RELATIVE_STREAM_SOURCE_SCOPE_MISMATCH"),
    ({"query_source_sha256": "f" * 64}, "RELATIVE_STREAM_SOURCE_SCOPE_MISMATCH"),
    ({"body_point": "model_origin"}, "RELATIVE_BODY_CLOCK_OR_BASIS_MISMATCH"),
])
def test_permit_binding_checks_exact_domain(changes, reason):
    _, payload, evaluation, bound = payload_and_bound()
    assert reason in verify(bound, payload, evaluation, **changes)


def test_missing_duplicate_and_unexpected_rows_are_not_hidden():
    protocol, rows = setup_profile()
    extra = replace(rows[0], identity=identity(H, 50, 20))
    result = evaluate_relative_profile(relative_profile_payload(protocol, [rows[0], *rows, extra]))
    assert result["derivation"]["unexpected_observations"]
    assert "DUPLICATE_OBSERVATION" in result["derivation"]["missing_pairs"][0]["reasons"]
    assert not result["sampled_claim_met"]


def test_finite_bounds_and_payload_roundtrip():
    _, payload, evaluation, bound = payload_and_bound()
    assert CaptureQueryBound.from_dict(asdict(bound)) == bound
    assert evaluate_relative_profile(payload) == evaluation
    for changes in ({"max_age_s": float("nan")}, {"rotation_rad": pi + .01},
                    {"rho_m": 0.}, {"translation_m": -1.}):
        with pytest.raises(ValueError):
            replace(bound, **changes)


def test_future_trial_epoch_is_not_historical_recording_epoch():
    _, payload, evaluation, bound = payload_and_bound()
    future = replace(bound, clock_epoch="future-epoch-7", reset_counter=200)
    reasons = verify(future, payload, evaluation, capture_clock_epoch="future-epoch-7",
                     query_clock_epoch="future-epoch-7", capture_reset_counter=200, query_reset_counter=200)
    assert "RELATIVE_EPOCH_RESET_SCOPE_MISMATCH" not in reasons
    assert "RELATIVE_NO_UNTOUCHED_SOURCE_HOLDOUT" in reasons


def test_untouched_label_and_hash_never_manufacture_deterministic_bound():
    protocol, rows = setup_profile()
    protocol = replace(protocol, holdout_kind="untouched_source_disjoint",
                       blinding_evidence_sha256=H)
    result = evaluate_relative_profile(relative_profile_payload(protocol, rows))
    assert result["sampled_claim_met"]
    assert not result["runtime_admission"]
    assert not result["calibration_established"]
    assert "RELATIVE_CONDITIONAL_MODEL_SCOPE_MISSING" in result["refusal_reasons"]


def test_complete_independent_data_can_satisfy_only_conditional_prerequisite():
    protocol, rows = setup_profile()
    protocol = replace(protocol, holdout_kind="untouched_source_disjoint",
                       blinding_evidence_sha256=H, conditional_model_scope_sha256=J)
    payload = relative_profile_payload(protocol, rows)
    result = evaluate_relative_profile(payload)
    bound = CaptureQueryBound(.5, .05, .05, 2., protocol.source_sha256,
        result["evaluation_sha256"], protocol.provenance_sha256, MODEL,
        "base_link_origin", "gazebo_sim", "new-trial-epoch", 200,
        conditional_model_scope_sha256=J)
    assert verify(bound, payload, result, capture_clock_epoch="new-trial-epoch",
                  query_clock_epoch="new-trial-epoch", capture_reset_counter=200,
                  query_reset_counter=200) == ()
    assert not result["runtime_admission"]
    assert not result["deterministic_bound_certified"]
    assert not result["calibration_established"]
    assert not result["instantaneous_twist_covered"]


def test_unknown_source_population_cannot_disappear_from_coverage():
    protocol, rows = setup_profile()
    unknown = replace(rows[0], identity=identity("f" * 64, 1, 0))
    report = evaluate_relative_profile(relative_profile_payload(protocol, [*rows, unknown]))
    assert report["unbound_observations"][0]["reasons"] == ["SOURCE_BINDING_MISSING"]
    assert not report["sampled_claim_met"]


def test_conditional_model_scope_is_part_of_exact_bound_binding():
    _, payload, evaluation, bound = payload_and_bound()
    changed = replace(bound, conditional_model_scope_sha256=J)
    assert "RELATIVE_CONDITIONAL_MODEL_SCOPE_MISMATCH" in verify(changed, payload, evaluation)


def test_metadata_freeze_is_runnable_without_decoders_or_outcomes(tmp_path, monkeypatch):
    import hashlib
    import json
    from pathlib import Path
    import runpy

    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / "scripts"))
    cli = runpy.run_path(str(root / "scripts/sim_relative_consistency.py"))
    inputs = []
    for number in (1, 2):
        directory = tmp_path / str(number)
        directory.mkdir()
        manifest = dict(recording_schema=2, alignment=dict(clock_epoch=0, reset_counter=12),
                        world_from_odom=matrix(), warmup_end_sim_ns=0, record_end_sim_ns=500_000_000)
        (directory / "manifest.json").write_text(json.dumps({**manifest, "source_number": number}))
        (directory / "fixture.json").write_text(json.dumps(dict(camera_translation_flu=[.13233, 0., .02078])))
        events = [
            dict(topic="/camera/image_raw", source_ns=0, topic_sequence=1,
                 serialized_sha256=digest([number, "camera"])),
            dict(topic="/fmu/out/vehicle_odometry", source_ns=250_000_000, topic_sequence=1,
                 serialized_sha256=digest([number, "odometry"])),
        ]
        (directory / "receive_progress.jsonl").write_text("\n".join(json.dumps(e) for e in events))
        files = {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                 for name in ("manifest.json", "fixture.json", "receive_progress.jsonl")}
        inputs.append(dict(path=str(directory), manifest_sha256=files["manifest.json"], files=files))
    aliases = dict(schema=1, inputs=inputs)
    alias_path = tmp_path / "aliases.json"
    alias_path.write_text(json.dumps({**aliases, "frozen_sha256": digest(aliases)}))
    output = tmp_path / "relative-protocol.json"
    cli["freeze"](alias_path, output)
    frozen = json.loads(output.read_text())
    assert not frozen["evaluation_performed"]
    assert frozen["protocol"]["max_age_s"] == .5
    assert len(frozen["protocol"]["captures"]) == len(frozen["protocol"]["queries"]) == 2
    assert not set(frozen["protocol"]["derivation_sources"]) & set(frozen["protocol"]["heldout_sources"])
    assert "observations" not in frozen
    with pytest.raises(FileExistsError):
        cli["freeze"](alias_path, output)
    with pytest.raises(ValueError, match="TTL_EXCEEDS"):
        cli["freeze"](alias_path, tmp_path / "ten-seconds.json", max_age_s=10.)
