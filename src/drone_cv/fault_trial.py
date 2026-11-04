"""Offline classification of frozen, independently observed runtime fault trials.

This consumes observations; it neither creates runtime states nor injects faults.
Content hashes cannot authenticate an observer. CLI verifies referenced artifact
bytes; approval and truthful independent acquisition remain external prerequisites.
"""
from dataclasses import asdict, dataclass

from drone_cv.empirical_profile import _hash, digest, empirical_trial_refusals


BINDING_KEYS = frozenset({"image", "firmware", "messages", "parameters", "sources", "workload_rtf",
                          "fixture", "seed", "authorization", "scenario"})
KINDS = frozenset({"stage", "px4", "ack", "output", "safety", "envelope", "injection", "masking", "unsafe"})
EXECUTION = frozenset({"COMPLETED", "SAFE_ABORT", "TASK_FAIL", "INFRA_ERROR", "INVALID_WORLD"})


def _bindings(values):
    if (not isinstance(values, tuple) or len(values) != len(BINDING_KEYS)
            or {k for k, _ in values} != BINDING_KEYS or any(not isinstance(v, str) or not v for _, v in values)):
        raise ValueError("complete frozen image/policy/source/workload/fixture/seed/auth bindings required")


@dataclass(frozen=True)
class ObservedRecord:
    wall_ns: int
    kind: str
    observer: str
    artifact_sha256: str
    sequence: int
    stage: str
    reason: str
    output_eligible: bool
    healthy: bool
    nav_state: int
    arming_state: int
    ack_accepted: bool
    injection: str

    def __post_init__(self):
        _hash(self.artifact_sha256)
        if (type(self.wall_ns) is not int or self.wall_ns < 0 or self.kind not in KINDS
                or not self.observer or type(self.sequence) is not int or self.sequence < 1
                or any(not isinstance(v, str) for v in (self.observer, self.stage, self.reason, self.injection))
                or type(self.nav_state) is not int or type(self.arming_state) is not int
                or any(type(v) is not bool for v in (self.output_eligible, self.healthy, self.ack_accepted))):
            raise ValueError("invalid typed observed source record")


@dataclass(frozen=True)
class Trace:
    trial_id: str
    bindings: tuple[tuple[str, str], ...]
    records: tuple[ObservedRecord, ...]
    execution_outcome: str

    def __post_init__(self):
        _bindings(self.bindings)
        if (not self.trial_id or not isinstance(self.records, tuple)
                or self.execution_outcome not in EXECUTION
                or any(b.wall_ns < a.wall_ns for a, b in zip(self.records, self.records[1:]))):
            raise ValueError("invalid observed trace ordering/outcome")
        identities = [(r.observer, r.kind, r.sequence) for r in self.records]
        if len(set(identities)) != len(identities):
            raise ValueError("duplicate observed identity")

    @property
    def sha256(self):
        return digest(asdict(self))

    @classmethod
    def from_dict(cls, value):
        return cls(value["trial_id"], tuple(tuple(v) for v in value["bindings"]),
                   tuple(ObservedRecord(**r) for r in value["records"]), value["execution_outcome"])


@dataclass(frozen=True)
class FaultProtocol:
    scenario: str
    profile_sha256: str
    evaluation_sha256: str
    baseline_trial_id: str
    fault_trial_id: str
    bindings: tuple[tuple[str, str], ...]
    target_stage: str
    target_nav_state: int
    target_arming_state: int
    baseline_pre: tuple[int, int]
    baseline_matched: tuple[int, int]
    fault_pre: tuple[int, int]
    fault_post: tuple[int, int]
    injection_wall_ns: int
    injection_kind: str
    expected_first_reasons: tuple[str, ...]
    response_deadline_ns: int
    observation_max_gap_ns: int
    rationale: str

    def __post_init__(self):
        for value in (self.profile_sha256, self.evaluation_sha256):
            _hash(value)
        if (not self.baseline_trial_id or not self.fault_trial_id
                or self.baseline_trial_id == self.fault_trial_id):
            raise ValueError("distinct frozen baseline/fault trial IDs required")
        _bindings(self.bindings)
        for window in (self.baseline_pre, self.baseline_matched, self.fault_pre, self.fault_post):
            if (not isinstance(window, tuple) or len(window) != 2
                    or any(type(v) is not int or v < 0 for v in window) or window[0] >= window[1]):
                raise ValueError("finite nonempty frozen windows required")
        if (not self.scenario or not self.target_stage or self.target_stage == "revoked"
                or not self.injection_kind or not self.rationale or not self.expected_first_reasons
                or any(not r for r in self.expected_first_reasons)
                or type(self.response_deadline_ns) is not int or self.response_deadline_ns <= 0
                or type(self.observation_max_gap_ns) is not int or self.observation_max_gap_ns <= 0
                or self.baseline_pre[1] != self.baseline_matched[0]
                or self.fault_pre[1] != self.injection_wall_ns or self.fault_post[0] != self.injection_wall_ns
                or self.fault_post[1] < self.injection_wall_ns + self.response_deadline_ns
                or self.baseline_pre[1] - self.baseline_pre[0] != self.fault_pre[1] - self.fault_pre[0]
                or self.baseline_matched[1] - self.baseline_matched[0] != self.fault_post[1] - self.fault_post[0]):
            raise ValueError("invalid matched windows/deadline/expected-reason protocol")

    @property
    def sha256(self):
        return digest(asdict(self))

    @classmethod
    def from_dict(cls, value):
        value = dict(value)
        value["bindings"] = tuple(tuple(v) for v in value["bindings"])
        for key in ("baseline_pre", "baseline_matched", "fault_pre", "fault_post", "expected_first_reasons"):
            value[key] = tuple(value[key])
        return cls(**value)


def _coverage(trace, window, max_gap):
    reasons = []
    channels = {}
    for kind in ("stage", "px4", "ack", "output", "safety", "envelope"):
        rows = [r for r in trace.records if r.kind == kind and window[0] <= r.wall_ns <= window[1]]
        channels[kind] = rows
        if (not rows or rows[0].wall_ns != window[0] or rows[-1].wall_ns != window[1]
                or any(b.wall_ns - a.wall_ns > max_gap for a, b in zip(rows, rows[1:]))):
            reasons.append("COVERAGE_" + kind.upper())
    stage_observers = {r.observer for r in channels["stage"]}
    other_observers = {r.observer for kind in ("px4", "ack", "output", "safety", "envelope") for r in channels[kind]}
    if not stage_observers or not other_observers or stage_observers & other_observers:
        reasons.append("INDEPENDENT_OBSERVER_MISSING")
    return reasons


def _healthy(trace, window, protocol, *, exclusive_end=False):
    rows = [r for r in trace.records if window[0] <= r.wall_ns
            and (r.wall_ns < window[1] if exclusive_end else r.wall_ns <= window[1])]
    failures = []
    for row in rows:
        if row.kind == "stage" and (row.stage != protocol.target_stage or not row.healthy
                                    or row.reason.lower() not in ("", row.stage.lower())):
            failures.append("TARGET_STAGE_NOT_HEALTHY")
        elif row.kind == "px4" and (row.nav_state != protocol.target_nav_state
                or row.arming_state != protocol.target_arming_state or not row.healthy):
            failures.append("ACTUAL_PX4_STAGE_MISMATCH")
        elif row.kind == "ack" and not row.ack_accepted:
            failures.append("ADDRESSED_ACK_NOT_ACCEPTED")
        elif row.kind == "output" and not row.output_eligible:
            failures.append("ACTUAL_OUTPUT_INELIGIBLE")
        elif row.kind == "envelope" and not row.healthy:
            failures.append("OUTSIDE_FROZEN_SOURCE_WORKLOAD_RTF_ENVELOPE")
        elif row.kind == "safety" and not row.healthy:
            failures.append("SAFETY_OBSERVER_COVERAGE_INVALID")
    return list(dict.fromkeys(failures))


def classify_trial(protocol: FaultProtocol, baseline: Trace, fault: Trace,
                   profile_payload: dict, evaluation_payload: dict) -> dict:
    """INVALID evidence never passes. Valid wrong/absent response fails. Unsafe latches."""
    invalid, failures = [], []
    unsafe = [asdict(r) for trace in (baseline, fault) for r in trace.records if r.kind == "unsafe"]
    if baseline.trial_id != protocol.baseline_trial_id or fault.trial_id != protocol.fault_trial_id:
        invalid.append("FROZEN_TRACE_ID_MISMATCH")
    if baseline.bindings != protocol.bindings or fault.bindings != protocol.bindings:
        invalid.append("MATCHED_TRIAL_BINDING_MISMATCH")
    if baseline.trial_id == fault.trial_id:
        invalid.append("BASELINE_AND_FAULT_NOT_DISTINCT")
    if (profile_payload.get("profile_sha256") != protocol.profile_sha256
            or evaluation_payload.get("evaluation_sha256") != protocol.evaluation_sha256):
        invalid.append("FROZEN_PROFILE_EVALUATION_MISMATCH")
    try:
        candidate = evaluation_payload["candidate"]
        invalid.extend(empirical_trial_refusals(profile_payload, evaluation_payload,
            position_variance_max=tuple(candidate["position_variance_ceiling"]),
            velocity_variance_max=tuple(candidate["velocity_variance_ceiling"]),
            velocity_error_mps=tuple(candidate["velocity_axis_mps"] or ()),
            position_error_m=candidate["position_box_enclosing_radius_m"],
            tracking_error_m=candidate["tracking_error_m"]))
        applicability = profile_payload["protocol"]["applicability"]
        bindings = dict(protocol.bindings)
        if applicability is not None:
            workload_rtf = {k: applicability[k] for k in
                            ("clock_source", "rtf_min", "rtf_max", "rtf_window_wall_s", "workload")}
            if (bindings["image"] != applicability["image"]
                    or bindings["firmware"] != applicability["firmware"]
                    or bindings["messages"] != applicability["messages"]
                    or bindings["parameters"] != applicability["parameter_policy_sha256"]
                    or bindings["workload_rtf"] != digest(workload_rtf)):
                invalid.append("PROFILE_APPLICABILITY_TRIAL_MISMATCH")
    except (KeyError, TypeError, ValueError):
        invalid.append("PROFILE_ELIGIBILITY_UNRESOLVED")
    baseline_window = (protocol.baseline_pre[0], protocol.baseline_matched[1])
    fault_window = (protocol.fault_pre[0], protocol.fault_post[1])
    invalid.extend("BASELINE_" + r for r in _coverage(baseline, baseline_window, protocol.observation_max_gap_ns))
    invalid.extend("BASELINE_" + r for r in _healthy(baseline, baseline_window, protocol))
    invalid.extend("FAULT_" + r for r in _coverage(fault, fault_window, protocol.observation_max_gap_ns))
    invalid.extend("FAULT_PRE_" + r for r in _healthy(fault, protocol.fault_pre, protocol, exclusive_end=True))
    if any(r.kind in ("safety", "envelope") and not r.healthy
           for trace in (baseline, fault) for r in trace.records):
        invalid.append("INDEPENDENT_SAFETY_OR_ENVELOPE_COVERAGE_INVALID")
    if any(r.kind == "injection" for r in baseline.records):
        invalid.append("BASELINE_NOT_NO_FAULT")
    injections = [r for r in fault.records if r.kind == "injection"]
    if any(r.kind == "stage" and r.stage == "revoked" and r.wall_ns <= baseline_window[1]
           for r in baseline.records):
        invalid.append("BASELINE_PREVIOUS_OR_CURRENT_REVOCATION_LATCHED")
    if (len(injections) != 1 or injections[0].wall_ns != protocol.injection_wall_ns
            or injections[0].injection != protocol.injection_kind
            or injections[0].observer in {r.observer for r in fault.records if r.kind == "stage"}):
        invalid.append("INTENDED_INJECTION_NOT_INDEPENDENTLY_PROVEN")
    if any(r.kind == "masking" for trace in (baseline, fault) for r in trace.records):
        invalid.append("INDEPENDENT_UNRELATED_MASKING_FAULT")
    if any(r.kind == "stage" and r.stage == "revoked" and r.wall_ns < protocol.injection_wall_ns for r in fault.records):
        invalid.append("FAULT_ALREADY_REVOKED_BEFORE_INJECTION")
    if baseline.execution_outcome != "COMPLETED" or fault.execution_outcome in ("INFRA_ERROR", "INVALID_WORLD"):
        invalid.append("EXECUTION_OR_OBSERVER_INCOMPLETE")
    revokes = [r for r in fault.records if r.kind == "stage" and r.stage == "revoked"
               and protocol.injection_wall_ns <= r.wall_ns <= protocol.fault_post[1]]
    first = revokes[0] if revokes else None
    baseline_first = next((r for r in baseline.records if r.kind == "stage"
        and (r.stage == "revoked" or not r.healthy or r.reason.lower() not in ("", r.stage.lower()))), None)
    deadline = protocol.injection_wall_ns + protocol.response_deadline_ns
    if first is None:
        failures.append("EXPECTED_REVOCATION_ABSENT")
    elif first.reason not in protocol.expected_first_reasons:
        failures.append("WRONG_FIRST_REVOCATION_REASON")
    elif first.wall_ns > deadline:
        failures.append("REVOCATION_DEADLINE_EXCEEDED")
    output = [r for r in fault.records if r.kind == "output"
              and protocol.injection_wall_ns <= r.wall_ns <= protocol.fault_post[1]]
    suppressed = next((r for r in output if not r.output_eligible), None)
    if (suppressed is None or suppressed.wall_ns > deadline
            or any(r.output_eligible for r in output if r.wall_ns >= suppressed.wall_ns)
            or first is not None and any(r.kind == "output" and r.output_eligible
                                        and r.wall_ns >= first.wall_ns for r in fault.records)):
        failures.append("OUTPUT_SUPPRESSION_DEADLINE_OR_LATCH_FAILED")
    if first is not None and any(r.kind == "stage" and r.stage != "revoked" and r.wall_ns > first.wall_ns for r in fault.records):
        failures.append("AUTHORITY_REENTERED_AFTER_REVOKE")
    verdict = "NOT_EVALUATED" if invalid else "FAIL" if failures else "PASS"
    safety = "UNSAFE_OBSERVED" if unsafe else "UNKNOWN" if invalid else "NO_VIOLATION_OBSERVED"
    # Preserve S03 execution axis; classifier never manufactures SAFE_ABORT.
    execution = fault.execution_outcome
    payload = {"schema": 1, "scenario": protocol.scenario, "protocol_sha256": protocol.sha256,
        "baseline_sha256": baseline.sha256, "fault_sha256": fault.sha256,
        "profile_sha256": protocol.profile_sha256, "evaluation_sha256": protocol.evaluation_sha256,
        "test_validity": "INVALID_TEST" if invalid else "VALID_TEST", "fault_verdict": verdict,
        "invalid_reasons": list(dict.fromkeys(invalid)), "failure_reasons": failures if not invalid else [],
        "baseline_first_reason": None if baseline_first is None else baseline_first.reason,
        "baseline_first_observation": None if baseline_first is None else asdict(baseline_first),
        "first_revocation": None if first is None else asdict(first),
        "first_output_suppressed": None if suppressed is None else asdict(suppressed),
        "safety_outcome": safety, "execution_outcome": execution, "unsafe_observations": unsafe,
        "baseline_execution_outcome": baseline.execution_outcome,
        "runtime_acceptance_certified": False,
        "exit_code": 1 if unsafe or verdict == "FAIL" or "TASK_FAIL" in (execution, baseline.execution_outcome) else 2 if invalid else 0}
    return {**payload, "classification_sha256": digest(payload)}


def summarize_trials(results) -> dict:
    results = tuple(results)
    return {"counts": {"planned": len(results),
                       "valid": sum(r["test_validity"] == "VALID_TEST" for r in results),
                       "passed": sum(r["fault_verdict"] == "PASS" for r in results),
                       "failed": sum(r["fault_verdict"] == "FAIL" for r in results),
                       "invalid": sum(r["test_validity"] == "INVALID_TEST" for r in results)},
            "unsafe_attempts": sum(r["safety_outcome"] == "UNSAFE_OBSERVED" for r in results),
            "trials": list(results)}
