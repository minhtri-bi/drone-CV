"""Finite evaluator-only NED consistency; empirical agreement is not authority.

No variance-to-error conversion, fitting against holdout, or continuous-time claim.
All payload hashes use canonical JSON, including exact original source identities.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
from math import isfinite, sqrt

import numpy as np


def digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _hash(value: str) -> None:
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("expected lowercase sha256")


def _vector(value, *, nonnegative=False):
    if (not isinstance(value, tuple) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v)
                   or nonnegative and v < 0 for v in value)):
        raise ValueError("expected finite NED tuple3")


@dataclass(frozen=True)
class Identity:
    manifest_sha256: str
    sequence: int
    source_ns: int
    serialized_sha256: str | None

    def __post_init__(self):
        _hash(self.manifest_sha256)
        if self.serialized_sha256 is not None:
            _hash(self.serialized_sha256)
        if type(self.sequence) is not int or self.sequence < 1 or type(self.source_ns) is not int or self.source_ns < 0:
            raise ValueError("invalid source identity")


@dataclass(frozen=True)
class Binding:
    manifest_sha256: str
    clock_epoch: int
    reset_counter: int
    alignment_sha256: str
    body_point: str
    frame: str
    clock_source: str
    source_domains: tuple[str, ...]

    def __post_init__(self):
        _hash(self.manifest_sha256)
        _hash(self.alignment_sha256)
        if (type(self.clock_epoch) is not int or self.clock_epoch < 0
                or type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255
                or not all(isinstance(v, str) and v for v in
                           (self.body_point, self.frame, self.clock_source, *self.source_domains))
                or not isinstance(self.source_domains, tuple) or not self.source_domains):
            raise ValueError("invalid source binding")

    @classmethod
    def from_dict(cls, value):
        return cls(**{**value, "source_domains": tuple(value["source_domains"])})


@dataclass(frozen=True)
class Sample:
    identity: Identity
    binding: Binding
    position_error_ned: tuple[float, float, float] | None
    velocity_error_ned: tuple[float, float, float] | None
    position_variance: tuple[float, float, float] | None
    velocity_variance: tuple[float, float, float] | None
    velocity_scope: str
    rejection_reasons: tuple[str, ...]
    truth_bracket_ns: tuple[int, int] | None

    def __post_init__(self):
        for value in (self.position_error_ned, self.velocity_error_ned):
            if value is not None:
                _vector(value)
        for value in (self.position_variance, self.velocity_variance):
            if value is not None:
                _vector(value, nonnegative=True)
        if self.velocity_scope not in ("instantaneous", "interval_average", "missing"):
            raise ValueError("invalid velocity scope")
        if not isinstance(self.rejection_reasons, tuple):
            raise ValueError("reasons must be immutable")
        if self.truth_bracket_ns is not None and (len(self.truth_bracket_ns) != 2
                or any(type(v) is not int or v < 0 for v in self.truth_bracket_ns)
                or self.truth_bracket_ns[1] < self.truth_bracket_ns[0]):
            raise ValueError("invalid truth brackets")

    @classmethod
    def from_dict(cls, raw):
        value = dict(raw)
        value["identity"] = Identity(**value["identity"])
        value["binding"] = Binding.from_dict(value["binding"])
        for key in ("position_error_ned", "velocity_error_ned", "position_variance",
                    "velocity_variance", "rejection_reasons", "truth_bracket_ns"):
            if value[key] is not None:
                value[key] = tuple(value[key])
        return cls(**value)


@dataclass(frozen=True)
class Applicability:
    image: str
    firmware: str
    messages: str
    parameter_policy_sha256: str
    clock_source: str
    rtf_min: float
    rtf_max: float
    rtf_window_wall_s: float
    workload: str
    body_point: str
    frame: str

    def __post_init__(self):
        _hash(self.parameter_policy_sha256)
        if (not all(isinstance(v, str) and v for v in
                    (self.image, self.firmware, self.messages, self.clock_source,
                     self.workload, self.body_point, self.frame))
                or not all(isfinite(v) for v in (self.rtf_min, self.rtf_max, self.rtf_window_wall_s))
                or not 0 < self.rtf_min <= self.rtf_max or self.rtf_window_wall_s <= 0):
            raise ValueError("finite explicit applicability required")

    @property
    def sha256(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class ProfileProtocol:
    name: str
    bindings: tuple[Binding, ...]
    derivation: tuple[Identity, ...]
    heldout: tuple[Identity, ...]
    holdout_kind: str
    blinding_evidence_sha256: str
    reference_position_axis_m: tuple[float, float, float]
    reference_velocity_axis_mps: tuple[float, float, float]
    position_variance_ceiling: tuple[float, float, float]
    velocity_variance_ceiling: tuple[float, float, float]
    position_margin_m: tuple[float, float, float]
    velocity_margin_mps: tuple[float, float, float]
    margin_rationale: str
    health_rationale: str
    workload: str
    required_velocity_scope: str
    percentiles: tuple[float, ...]
    tracking_error_m: float | None = None
    tracking_evidence_sha256: str = ""
    reference_position_radius_m: float = .01
    applicability: Applicability | None = None

    def __post_init__(self):
        if (any(not isinstance(v, tuple) for v in
                (self.bindings, self.derivation, self.heldout, self.percentiles))
                or any(not isinstance(b, Binding) for b in self.bindings)
                or any(not isinstance(i, Identity) for i in (*self.derivation, *self.heldout))
                or self.applicability is not None and not isinstance(self.applicability, Applicability)):
            raise ValueError("immutable typed protocol required")
        if not self.name or not self.workload or not self.margin_rationale or not self.health_rationale:
            raise ValueError("name, workload and rationale required")
        if self.holdout_kind not in ("retrospective_derivation_disjoint", "untouched_blinded", "in_sample"):
            raise ValueError("invalid holdout kind")
        if not self.bindings or len({b.manifest_sha256 for b in self.bindings}) != len(self.bindings):
            raise ValueError("unique source bindings required")
        for members in (self.derivation, self.heldout):
            if not members or len(set(members)) != len(members):
                raise ValueError("nonempty unique membership required")
            source_keys = {(i.manifest_sha256, i.source_ns) for i in members}
            if len(source_keys) != len(members):
                raise ValueError("source sample aliases are not unique")
            if any(i.manifest_sha256 not in {b.manifest_sha256 for b in self.bindings} for i in members):
                raise ValueError("membership missing source binding")
        if self.holdout_kind != "in_sample" and (
                {(i.manifest_sha256, i.source_ns) for i in self.derivation}
                & {(i.manifest_sha256, i.source_ns) for i in self.heldout}
                or {i.serialized_sha256 for i in self.derivation if i.serialized_sha256 is not None}
                & {i.serialized_sha256 for i in self.heldout if i.serialized_sha256 is not None}):
            raise ValueError("DERIVATION_HELDOUT_NOT_DISJOINT")
        if self.holdout_kind == "untouched_blinded":
            _hash(self.blinding_evidence_sha256)
        for field in (self.reference_position_axis_m, self.reference_velocity_axis_mps,
                      self.position_variance_ceiling, self.velocity_variance_ceiling,
                      self.position_margin_m, self.velocity_margin_mps):
            _vector(field, nonnegative=True)
        if not isfinite(self.reference_position_radius_m) or self.reference_position_radius_m < 0:
            raise ValueError("finite reference position radius required")
        if self.required_velocity_scope not in ("instantaneous", "interval_average"):
            raise ValueError("invalid required velocity scope")
        if (not self.percentiles or any(not isfinite(v) or not 0 <= v <= 100 for v in self.percentiles)
                or tuple(sorted(set(self.percentiles))) != self.percentiles):
            raise ValueError("invalid percentiles")
        if self.tracking_error_m is not None:
            if not isfinite(self.tracking_error_m) or self.tracking_error_m < 0:
                raise ValueError("invalid tracking error")
            _hash(self.tracking_evidence_sha256)

    def to_dict(self):
        return json.loads(json.dumps(asdict(self), allow_nan=False))

    @property
    def sha256(self):
        return digest(self.to_dict())

    @classmethod
    def from_dict(cls, raw):
        value = dict(raw)
        if value.get("applicability") is not None:
            value["applicability"] = Applicability(**value["applicability"])
        value["bindings"] = tuple(Binding.from_dict(b) for b in value["bindings"])
        for key in ("derivation", "heldout"):
            value[key] = tuple(Identity(**i) for i in value[key])
        for key in ("reference_position_axis_m", "reference_velocity_axis_mps",
                    "position_variance_ceiling", "velocity_variance_ceiling",
                    "position_margin_m", "velocity_margin_mps", "percentiles"):
            value[key] = tuple(value[key])
        return cls(**value)


@dataclass(frozen=True)
class EmpiricalProfile:
    protocol: ProfileProtocol
    derivation_samples: tuple[Sample, ...]

    def __post_init__(self):
        if (not isinstance(self.protocol, ProfileProtocol) or not isinstance(self.derivation_samples, tuple)
                or any(not isinstance(s, Sample) for s in self.derivation_samples)):
            raise ValueError("immutable typed profile required")

    def to_dict(self):
        payload = {"schema": 1, "protocol": self.protocol.to_dict(),
                   "protocol_sha256": self.protocol.sha256,
                   "derivation_samples": [asdict(s) for s in self.derivation_samples]}
        return json.loads(json.dumps({**payload, "profile_sha256": digest(payload)}, allow_nan=False))

    @classmethod
    def from_dict(cls, raw):
        payload = {key: value for key, value in raw.items() if key != "profile_sha256"}
        if raw.get("schema") != 1 or digest(payload) != raw.get("profile_sha256"):
            raise ValueError("PROFILE_HASH_MISMATCH")
        protocol = ProfileProtocol.from_dict(raw["protocol"])
        if protocol.sha256 != raw["protocol_sha256"]:
            raise ValueError("PROTOCOL_HASH_MISMATCH")
        return cls(protocol, tuple(Sample.from_dict(s) for s in raw["derivation_samples"]))


def derive_profile(protocol: ProfileProtocol, samples) -> EmpiricalProfile:
    return EmpiricalProfile(protocol, tuple(samples))


def _select(protocol, membership, samples):
    indexed = {}
    for sample in samples:
        indexed.setdefault(sample.identity, []).append(sample)
    selected, uncovered = [], []
    bindings = {b.manifest_sha256: b for b in protocol.bindings}
    for identity in membership:
        matches = indexed.get(identity, [])
        reasons = []
        if identity.serialized_sha256 is None:
            reasons.append("RAW_SERIALIZED_IDENTITY_MISSING")
        if len(matches) != 1:
            reasons.append("MISSING_SAMPLE" if not matches else "DUPLICATE_SAMPLE")
        else:
            sample = matches[0]
            if sample.binding != bindings[identity.manifest_sha256]:
                reasons.append("SOURCE_BINDING_MISMATCH")
            reasons.extend(sample.rejection_reasons)
            if any(v is None for v in (sample.position_error_ned, sample.velocity_error_ned,
                                       sample.position_variance, sample.velocity_variance,
                                       sample.truth_bracket_ns)):
                reasons.append("MATCHING_TRUTH_OR_HEALTH_MISSING")
            if sample.binding == bindings[identity.manifest_sha256]:
                selected.append(sample)
        if reasons:
            uncovered.append({"identity": asdict(identity), "reasons": reasons})
    unexpected = [asdict(i) for i in indexed if i not in set(membership)]
    return selected, uncovered, unexpected


def _stats(rows, field, percentiles):
    values = np.asarray([getattr(row, field) for row in rows if getattr(row, field) is not None], dtype=float)
    if not len(values):
        return [{"count": 0} for _ in range(3)]
    return [{"count": len(values), "signed_min": float(values[:, axis].min()),
             "signed_max": float(values[:, axis].max()),
             "absolute_max": float(abs(values[:, axis]).max()),
             "absolute_percentiles": {str(p): float(np.percentile(abs(values[:, axis]), p)) for p in percentiles},
             "last_minus_first": float(values[-1, axis]-values[0, axis])} for axis in range(3)]


def _assess(protocol, membership, samples, position_limits, velocity_limits):
    rows, uncovered, unexpected = _select(protocol, membership, samples)
    violations = []
    checks = (("position_error_ned", position_limits, True),
              ("velocity_error_ned", velocity_limits, True),
              ("position_variance", protocol.position_variance_ceiling, False),
              ("velocity_variance", protocol.velocity_variance_ceiling, False))
    for row in rows:
        for field, limits, absolute in checks:
            if getattr(row, field) is None:
                continue
            for axis, (value, limit) in enumerate(zip(getattr(row, field), limits)):
                if (abs(value) if absolute else value) > limit:
                    violations.append({"identity": asdict(row.identity), "field": field,
                                       "axis_ned": axis, "value": value, "ceiling": limit})
    per_source = []
    for binding in protocol.bindings:
        source_rows = sorted((r for r in rows if r.binding == binding), key=lambda r: r.identity.source_ns)
        per_source.append({"binding": asdict(binding),
                           "position_error": _stats(source_rows, "position_error_ned", protocol.percentiles),
                           "velocity_error": _stats(source_rows, "velocity_error_ned", protocol.percentiles)})
    return {"membership": [asdict(i) for i in membership], "membership_sha256": digest([asdict(i) for i in membership]),
            "covered_count": len(membership)-len(uncovered), "uncovered": uncovered, "unexpected": unexpected,
            "violations": violations, "velocity_scopes": sorted({r.velocity_scope for r in rows}),
            "source_domains": sorted({d for r in rows for d in r.binding.source_domains}),
            "per_source_drift_and_statistics": per_source,
            "position_error": _stats(rows, "position_error_ned", protocol.percentiles),
            "velocity_error": _stats(rows, "velocity_error_ned", protocol.percentiles),
            "position_health": _stats(rows, "position_variance", protocol.percentiles),
            "velocity_health": _stats(rows, "velocity_variance", protocol.percentiles)}, rows


def evaluate_profile(profile: EmpiricalProfile, heldout_samples) -> dict:
    protocol = profile.protocol
    heldout_samples = tuple(heldout_samples)
    derivation_rows, _, _ = _select(protocol, protocol.derivation, profile.derivation_samples)
    def candidate(field, margins):
        available = [r for r in derivation_rows if getattr(r, field) is not None]
        if not available:
            return None
        return [max(abs(getattr(r, field)[axis]) for r in available)+margins[axis] for axis in range(3)]
    pos = candidate("position_error_ned", protocol.position_margin_m)
    vel = candidate("velocity_error_ned", protocol.velocity_margin_mps)
    derivation, _ = _assess(protocol, protocol.derivation, profile.derivation_samples,
                           pos or protocol.reference_position_axis_m, vel or protocol.reference_velocity_axis_mps)
    heldout, rows = _assess(protocol, protocol.heldout, heldout_samples,
                            pos or protocol.reference_position_axis_m, vel or protocol.reference_velocity_axis_mps)
    reference_derivation, _ = _assess(protocol, protocol.derivation, profile.derivation_samples,
                                     protocol.reference_position_axis_m, protocol.reference_velocity_axis_mps)
    reference_heldout, _ = _assess(protocol, protocol.heldout, heldout_samples,
                                  protocol.reference_position_axis_m, protocol.reference_velocity_axis_mps)
    for report, selected in ((reference_derivation, derivation_rows), (reference_heldout, rows)):
        report["position_radius_m"] = protocol.reference_position_radius_m
        report["position_radius_violations"] = [
            {"identity": asdict(row.identity),
             "norm_m": sqrt(sum(v*v for v in row.position_error_ned)),
             "ceiling_m": protocol.reference_position_radius_m}
            for row in selected if row.position_error_ned is not None
            and sqrt(sum(v*v for v in row.position_error_ned)) > protocol.reference_position_radius_m]
    refusals = []
    for name, report in (("DERIVATION", derivation), ("HELDOUT", heldout)):
        if report["uncovered"] or report["unexpected"]:
            refusals.append(name+"_COVERAGE_UNRESOLVED")
        if report["violations"]:
            refusals.append(name+"_VIOLATIONS")
    if protocol.required_velocity_scope == "instantaneous" and any(
            r.velocity_scope != "instantaneous" for r in (*derivation_rows, *rows)):
        refusals.append("INSTANTANEOUS_VELOCITY_UNCOVERED")
    criteria = not refusals and pos is not None and vel is not None and protocol.holdout_kind != "in_sample"
    if protocol.holdout_kind != "untouched_blinded":
        refusals.append("NO_UNTOUCHED_BLINDED_HOLDOUT")
    if protocol.required_velocity_scope != "instantaneous":
        refusals.append("INSTANTANEOUS_VELOCITY_REQUIRED_FOR_RUNTIME")
    if protocol.tracking_error_m is None:
        refusals.append("INDEPENDENT_TRACKING_EVIDENCE_MISSING")
    if protocol.holdout_kind == "in_sample":
        refusals.append("IN_SAMPLE_ONLY")
    if any(b.body_point != "base_link_origin" or b.frame != "local_ned"
           or b.clock_source != "gazebo_sim" for b in protocol.bindings):
        refusals.append("UNSUPPORTED_FRAME_CLOCK_OR_BODY_POINT")
    if protocol.applicability is None:
        refusals.append("FROZEN_RUNTIME_APPLICABILITY_MISSING")
    elif (protocol.applicability.body_point != "base_link_origin"
          or protocol.applicability.frame != "local_ned"
          or protocol.applicability.clock_source != "gazebo_sim"):
        refusals.append("UNSUPPORTED_APPLICABILITY_FRAME_CLOCK_OR_BODY_POINT")
    if protocol.applicability is not None and protocol.applicability.workload != protocol.workload:
        refusals.append("APPLICABILITY_WORKLOAD_MISMATCH")
    payload = {"schema": 1, "profile_sha256": profile.to_dict()["profile_sha256"],
        "protocol_sha256": protocol.sha256, "heldout_samples": [asdict(s) for s in heldout_samples],
        "holdout_kind": protocol.holdout_kind, "heldout_trial_criteria_met": criteria,
        "trial_profile_eligible": not refusals, "runtime_use_eligible": False, "runtime_admission": False,
        "deterministic_bound_certified": False,
        "scope": "sampled absolute local-NED base-link-origin errors, not capture-to-query transform error or continuous bound",
        "candidate": {"position_axis_m": pos, "velocity_axis_mps": vel,
                      "position_box_enclosing_radius_m": None if pos is None else sqrt(sum(v*v for v in pos)),
                      "position_variance_ceiling": list(protocol.position_variance_ceiling),
                      "velocity_variance_ceiling": list(protocol.velocity_variance_ceiling),
                      "tracking_error_m": protocol.tracking_error_m},
        "derivation": derivation, "heldout": heldout,
        "unchanged_reference": {"derivation": reference_derivation, "heldout": reference_heldout},
        "refusal_reasons": refusals}
    payload = json.loads(json.dumps(payload, allow_nan=False))
    return {**payload, "evaluation_sha256": digest(payload)}


def empirical_trial_refusals(profile_payload: dict, evaluation_payload: dict, *,
                             position_variance_max, velocity_variance_max,
                             velocity_error_mps, position_error_m, tracking_error_m) -> tuple[str, ...]:
    """Recompute conditional trial consistency, not motion authority or a hard bound.

    Evidence hashes bind supplied observations, not their authenticity or approval.
    An empty result satisfies this empirical prerequisite only; the runtime must
    independently enforce N1/N3, provenance, policy and exact operator approval.
    """
    try:
        profile = EmpiricalProfile.from_dict(profile_payload)
        evaluation = evaluate_profile(profile, [Sample.from_dict(s) for s in evaluation_payload["heldout_samples"]])
        if evaluation != evaluation_payload:
            return ("EVALUATION_CONTENT_OR_HASH_MISMATCH",)
        reasons = list(evaluation["refusal_reasons"])
        candidate = evaluation["candidate"]
        if (tuple(candidate["position_variance_ceiling"]) != position_variance_max
                or tuple(candidate["velocity_variance_ceiling"]) != velocity_variance_max
                or candidate["velocity_axis_mps"] is None
                or tuple(candidate["velocity_axis_mps"]) != velocity_error_mps
                or candidate["position_box_enclosing_radius_m"] != position_error_m
                or candidate["tracking_error_m"] != tracking_error_m):
            reasons.append("EMPIRICAL_ENVELOPE_MISMATCH")
        return tuple(reasons)
    except (ValueError, TypeError, KeyError, OverflowError) as error:
        return ("INVALID_EMPIRICAL_PAYLOAD:"+str(error),)
