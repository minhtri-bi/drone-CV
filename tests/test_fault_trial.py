from dataclasses import replace

import pytest

from drone_cv.empirical_profile import Applicability, derive_profile, digest, evaluate_profile
from drone_cv.fault_trial import (
    FaultProtocol, ObservedRecord, Trace, classify_trial, summarize_trials,
)
from test_empirical_profile import protocol, sample


HASH = "a" * 64
APP = Applicability("image-frozen", "firmware-frozen", "messages-frozen", HASH,
                    "gazebo_sim", .2, 1., 1., "frozen workload identity",
                    "base_link_origin", "local_ned")
BINDINGS = tuple((key, value) for key, value in {
    "image": APP.image, "firmware": APP.firmware, "messages": APP.messages,
    "parameters": APP.parameter_policy_sha256, "sources": "sources-frozen",
    "workload_rtf": digest({k: getattr(APP, k) for k in
                           ("clock_source", "rtf_min", "rtf_max", "rtf_window_wall_s", "workload")}),
    "fixture": "fixture-frozen", "seed": "seed-frozen", "authorization": "authorization-frozen",
    "scenario": "scenario-frozen"}.items())


def empirical():
    p = protocol(holdout_kind="untouched_blinded", blinding_evidence_sha256=HASH,
                 tracking_error_m=.01, tracking_evidence_sha256=HASH, applicability=APP)
    profile = derive_profile(p, [sample(1)])
    evaluation = evaluate_profile(profile, [sample(2)])
    return profile.to_dict(), evaluation


def records(fault=False, reason="COMMAND_EXPIRED", injection=True, mask=False):
    rows = []
    for t in range(7):
        for kind in ("stage", "px4", "ack", "output", "safety", "envelope"):
            revoked = fault and t >= 4
            rows.append(ObservedRecord(t * 100, kind, "supervisor" if kind == "stage" else "independent",
                HASH if kind == "stage" else "b" * 64, t + 1,
                "revoked" if revoked else "active", reason if revoked and kind == "stage" else "",
                not revoked, True, 14, 2, True, ""))
    if fault and injection:
        rows.append(ObservedRecord(300, "injection", "independent", "b" * 64, 99,
                                   "", "", False, True, 14, 2, True, "stale_command"))
    if mask:
        rows.append(ObservedRecord(350, "masking", "independent", "b" * 64, 100,
                                   "", "ODOMETRY_UNCERTAINTY", False, False, 14, 2, True, ""))
    return tuple(sorted(rows, key=lambda r: r.wall_ns))


def trial(**changes):
    profile, evaluation = empirical()
    baseline = Trace("baseline", BINDINGS, records(), "COMPLETED")
    fault = Trace("fault", BINDINGS, records(True), "COMPLETED")
    frozen = FaultProtocol("stale-command", profile["profile_sha256"], evaluation["evaluation_sha256"],
        baseline.trial_id, fault.trial_id, BINDINGS, "active", 14, 2,
        (0, 300), (300, 600), (0, 300), (300, 600), 300, "stale_command",
        ("COMMAND_EXPIRED",), 100, 100, "frozen exact stale-command response and independent observation cadence")
    return replace(frozen, **changes), baseline, fault, profile, evaluation


def classify(parts):
    return classify_trial(*parts)


def test_valid_baseline_and_causal_fault_pass_without_runtime_claim():
    result = classify(trial())
    assert result["test_validity"] == "VALID_TEST"
    assert result["fault_verdict"] == "PASS"
    assert not result["runtime_acceptance_certified"]


def test_variance_revoke_before_target_stage_invalidates_baseline():
    p, baseline, fault, profile, evaluation = trial()
    rows = tuple(replace(r, stage="revoked", reason="ODOMETRY_UNCERTAINTY") if r.kind == "stage" else r for r in baseline.records)
    baseline = replace(baseline, records=rows)
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["test_validity"] == "INVALID_TEST"
    assert result["fault_verdict"] == "NOT_EVALUATED"
    assert result["baseline_first_reason"] == "ODOMETRY_UNCERTAINTY"


@pytest.mark.parametrize("reason", ["OWNER_CONFLICT", ""])
def test_wrong_or_absent_expected_response_is_fail_in_valid_trial(reason):
    p, baseline, fault, profile, evaluation = trial()
    fault = replace(fault, records=records(True, reason=reason) if reason else records(False))
    if not reason:
        fault = replace(fault, records=tuple(sorted((*fault.records,
            ObservedRecord(300, "injection", "independent", "b" * 64, 99, "", "", False,
                           True, 14, 2, True, "stale_command")), key=lambda r: r.wall_ns)))
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["test_validity"] == "VALID_TEST"
    assert result["fault_verdict"] == "FAIL"


@pytest.mark.parametrize("change", ["no_injection", "masked", "coverage", "binding", "late_suppression"])
def test_activation_masking_coverage_bindings_and_deadline(change):
    p, baseline, fault, profile, evaluation = trial()
    if change == "no_injection":
        fault = replace(fault, records=records(True, injection=False))
    elif change == "masked":
        fault = replace(fault, records=records(True, mask=True))
    elif change == "coverage":
        fault = replace(fault, records=tuple(r for r in fault.records if r.kind != "ack"))
    elif change == "binding":
        fault = replace(fault, bindings=tuple((k, "other" if k == "seed" else v) for k, v in BINDINGS))
    else:
        fault = replace(fault, records=tuple(replace(r, output_eligible=True) if r.kind == "output" else r for r in fault.records))
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["fault_verdict"] == ("FAIL" if change == "late_suppression" else "NOT_EVALUATED")
    assert result["test_validity"] == ("VALID_TEST" if change == "late_suppression" else "INVALID_TEST")


def test_unsafe_latch_survives_invalid_baseline_and_infrastructure_error():
    p, baseline, fault, profile, evaluation = trial()
    unsafe = ObservedRecord(600, "unsafe", "independent", HASH, 999, "", "CONTACT", False,
                            False, 14, 2, True, "")
    fault = replace(fault, records=(*fault.records, unsafe), execution_outcome="INFRA_ERROR")
    p = replace(p, baseline_trial_id="different-baseline")
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["execution_outcome"] == "INFRA_ERROR"
    assert result["exit_code"] == 1
    assert summarize_trials([result])["counts"] == {"planned": 1, "valid": 0, "passed": 0, "failed": 0, "invalid": 1}


def test_retrospective_profile_cannot_enable_fault_eligibility():
    p, baseline, fault, _, _ = trial()
    profile = derive_profile(protocol(), [sample(1)])
    evaluation = evaluate_profile(profile, [sample(2)])
    p = replace(p, profile_sha256=profile.to_dict()["profile_sha256"], evaluation_sha256=evaluation["evaluation_sha256"])
    result = classify_trial(p, baseline, fault, profile.to_dict(), evaluation)
    assert result["test_validity"] == "INVALID_TEST"
    assert any("NO_UNTOUCHED" in r for r in result["invalid_reasons"])


def test_healthy_diagnostic_reason_is_not_itself_a_revocation():
    p, baseline, fault, profile, evaluation = trial()
    baseline = replace(baseline, records=tuple(
        replace(r, reason="active") if r.kind == "stage" else r for r in baseline.records))
    fault = replace(fault, records=tuple(
        replace(r, reason="active") if r.kind == "stage" and r.stage == "active" else r for r in fault.records))
    assert classify_trial(p, baseline, fault, profile, evaluation)["fault_verdict"] == "PASS"


def test_baseline_revocation_before_frozen_pre_window_stays_invalid():
    p, baseline, fault, profile, evaluation = trial()
    baseline = replace(baseline, records=tuple(
        replace(r, stage="revoked", reason="ODOMETRY_UNCERTAINTY") if r.kind == "stage" and r.wall_ns == 0
        else r for r in baseline.records))
    p = replace(p, baseline_pre=(100, 300), fault_pre=(100, 300))
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["test_validity"] == "INVALID_TEST"
    assert "BASELINE_PREVIOUS_OR_CURRENT_REVOCATION_LATCHED" in result["invalid_reasons"]


def test_deadline_does_not_allow_output_after_observed_revoke():
    p, baseline, fault, profile, evaluation = trial(response_deadline_ns=200)
    fault = replace(fault, records=tuple(
        replace(r, output_eligible=True) if r.kind == "output" and r.wall_ns == 400 else r for r in fault.records))
    result = classify_trial(p, baseline, fault, profile, evaluation)
    assert result["test_validity"] == "VALID_TEST"
    assert result["fault_verdict"] == "FAIL"
    assert "OUTPUT_SUPPRESSION_DEADLINE_OR_LATCH_FAILED" in result["failure_reasons"]


@pytest.mark.parametrize("bad_contents", [None, "{not-json"])
def test_cli_counts_missing_or_malformed_side_and_latches_decoded_unsafe(tmp_path, bad_contents):
    from dataclasses import asdict
    import importlib.util
    import json
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("classify_fault_trial", Path(__file__).parents[1] / "scripts/classify_fault_trial.py")
    assert spec is not None and spec.loader is not None
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    p, baseline, fault, profile, evaluation = trial()
    unsafe = ObservedRecord(600, "unsafe", "independent", HASH, 999, "", "CONTACT",
                            False, False, 14, 2, True, "")
    fault = replace(fault, records=(*fault.records, unsafe))
    for name, value in (("profile.json", profile), ("evaluation.json", evaluation), ("fault.json", asdict(fault))):
        (tmp_path / name).write_text(json.dumps(value))
    if bad_contents is not None:
        (tmp_path / "baseline.json").write_text(bad_contents)
    manifest = {"schema": 1, "profile": "profile.json", "evaluation": "evaluation.json",
        "protocols": [asdict(p)], "trials": [{"protocol_sha256": p.sha256,
        "baseline": "baseline.json", "fault": "fault.json",
        "baseline_sha256": baseline.sha256, "fault_sha256": fault.sha256}], "source_artifacts": {}}
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    result = cli.classify_manifest(tmp_path / "manifest.json")
    assert result["counts"] == {"planned": 1, "valid": 0, "passed": 0, "failed": 0, "invalid": 1}
    assert result["trials"][0]["safety_outcome"] == "UNSAFE_OBSERVED"
    assert result["trials"][0]["execution_outcome"] == "INFRA_ERROR"
    assert result["exit_code"] == 1
