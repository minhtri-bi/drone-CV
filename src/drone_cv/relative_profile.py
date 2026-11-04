"""Finite capture/query SE(3) consistency, never calibration or motion authority.

Analysis source membership is distinct from a future trial's epoch/reset scope.
The error transform acts in capture-body FLU. Its whole-point displacement norm
is unchanged by the estimated capture-to-query rotation into query local NED.
"""
from bisect import bisect_left, bisect_right
from dataclasses import asdict, dataclass
from math import atan2, isfinite, pi, sin

import numpy as np

from drone_cv.empirical_profile import Binding, Identity, digest


BASIS = "capture_body_flu"
NED_FROM_ENU = ((0., 1., 0.), (1., 0., 0.), (0., 0., -1.))

# These retained sources have already been examined. A relabelled hash cannot
# turn them into untouched validation data.
SEEN_RETAINED_SOURCES = frozenset((
    "306b529d10016f3b073a2cc85f227b8b2562b1dc0d35efad2effae6cd7adf9cb",
    "b8c4b316f4b1c39b6f24f835253e7ce15c72d97d948194c51291fd69ce65ae98",
))


def _hash(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError("expected lowercase sha256")


def _finite(value, *, positive=False):
    if type(value) not in (int, float) or not isfinite(value) or value < 0 or positive and value == 0:
        raise ValueError("expected finite nonnegative number")


def _rotation(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
            or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-8, rtol=0)
            or abs(float(np.linalg.det(matrix)) - 1.) > 1e-8):
        raise ValueError("expected proper rotation")
    return matrix


def _transform(value):
    if not isinstance(value, tuple) or any(not isinstance(row, tuple) for row in value):
        raise ValueError("immutable transform required")
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.array_equal(matrix[3], [0., 0., 0., 1.])):
        raise ValueError("expected finite rigid transform")
    _rotation(matrix[:3, :3])
    return matrix


def _inverse(matrix):
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return result


def stream_source_sha256(role, model_sha256, provenance_sha256, body_point, clock_source, basis=BASIS):
    """Canonical producer/model/calibration contract; not a bag or message ID."""
    if role not in ("capture", "query"):
        raise ValueError("invalid stream role")
    return digest(dict(role=role, model_sha256=model_sha256,
                       provenance_sha256=provenance_sha256, body_point=body_point,
                       clock_source=clock_source, basis=basis))


@dataclass(frozen=True)
class CaptureQueryBound:
    max_age_s: float
    translation_m: float
    rotation_rad: float
    rho_m: float
    source_sha256: str
    evaluation_sha256: str
    provenance_sha256: str
    model_sha256: str
    body_point: str
    clock_source: str
    clock_epoch: str
    reset_counter: int
    basis: str = BASIS
    conditional_model_scope_sha256: str = ""

    def __post_init__(self):
        for value in (self.max_age_s, self.rho_m):
            _finite(value, positive=True)
        for value in (self.translation_m, self.rotation_rad):
            _finite(value)
        if self.rotation_rad > pi:
            raise ValueError("principal rotation must not exceed pi")
        _finite(self.displacement_m)
        for value in (self.source_sha256, self.evaluation_sha256,
                      self.provenance_sha256, self.model_sha256):
            _hash(value)
        if not isinstance(self.conditional_model_scope_sha256, str) or self.conditional_model_scope_sha256:
            _hash(self.conditional_model_scope_sha256)
        if (not isinstance(self.clock_epoch, str) or not self.clock_epoch
                or type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255
                or not all(isinstance(v, str) and v for v in (self.body_point, self.clock_source, self.basis))):
            raise ValueError("invalid capture/query scope")

    @property
    def displacement_m(self):
        return self.translation_m + 2 * sin(self.rotation_rad / 2) * self.rho_m

    @property
    def sha256(self):
        return digest(asdict(self))

    @property
    def capture_source_sha256(self):
        return stream_source_sha256("capture", self.model_sha256, self.provenance_sha256,
                                    self.body_point, self.clock_source, self.basis)

    @property
    def query_source_sha256(self):
        return stream_source_sha256("query", self.model_sha256, self.provenance_sha256,
                                    self.body_point, self.clock_source, self.basis)

    @classmethod
    def from_dict(cls, value):
        return cls(**value)


@dataclass(frozen=True)
class RelativeProtocol:
    name: str
    bindings: tuple[Binding, ...]
    captures: tuple[Identity, ...]
    queries: tuple[Identity, ...]
    derivation_sources: tuple[str, ...]
    heldout_sources: tuple[str, ...]
    max_age_s: float
    rho_m: float
    translation_limit_m: float
    rotation_limit_rad: float
    holdout_kind: str
    blinding_evidence_sha256: str
    model_sha256: str
    rationale: str
    provenance_sha256: str
    conditional_model_scope_sha256: str = ""

    def __post_init__(self):
        if not self.name or not self.rationale:
            raise ValueError("name and frozen rationale required")
        _hash(self.model_sha256)
        _hash(self.provenance_sha256)
        if not isinstance(self.conditional_model_scope_sha256, str) or self.conditional_model_scope_sha256:
            _hash(self.conditional_model_scope_sha256)
        for value in (self.max_age_s, self.rho_m):
            _finite(value, positive=True)
        if not isfinite(self.max_age_s * 1e9) or self.max_age_s * 1e9 != int(self.max_age_s * 1e9):
            raise ValueError("TTL must have exact nanosecond representation")
        for value in (self.translation_limit_m, self.rotation_limit_rad):
            _finite(value)
        if self.rotation_limit_rad > pi:
            raise ValueError("principal rotation limit exceeds pi")
        _finite(self.translation_limit_m + 2 * sin(self.rotation_limit_rad / 2) * self.rho_m)
        for value in (self.bindings, self.captures, self.queries,
                      self.derivation_sources, self.heldout_sources):
            if not isinstance(value, tuple) or not value:
                raise ValueError("nonempty immutable membership required")
        if any(not isinstance(b, Binding) for b in self.bindings):
            raise ValueError("typed binding required")
        sources = {b.manifest_sha256 for b in self.bindings}
        if len(sources) != len(self.bindings):
            raise ValueError("DUPLICATE_SOURCE_BINDING")
        if (set(self.derivation_sources) & set(self.heldout_sources)
                or set(self.derivation_sources) | set(self.heldout_sources) != sources
                or any(len(set(v)) != len(v) for v in (self.derivation_sources, self.heldout_sources))):
            raise ValueError("SOURCE_SPLIT_MUST_BE_DISJOINT_AND_EXHAUSTIVE")
        for identities in (self.captures, self.queries):
            if (any(not isinstance(i, Identity) or i.manifest_sha256 not in sources for i in identities)
                    or len(set(identities)) != len(identities)
                    or len({(i.manifest_sha256, i.source_ns) for i in identities}) != len(identities)):
                raise ValueError("INVALID_OR_ALIASED_MEMBERSHIP")
        derivation_hashes = {i.serialized_sha256 for i in (*self.captures, *self.queries)
                             if i.manifest_sha256 in self.derivation_sources and i.serialized_sha256}
        heldout_hashes = {i.serialized_sha256 for i in (*self.captures, *self.queries)
                          if i.manifest_sha256 in self.heldout_sources and i.serialized_sha256}
        if derivation_hashes & heldout_hashes:
            raise ValueError("SOURCE_SPLIT_SERIALIZED_ALIAS")
        if self.holdout_kind not in ("retrospective_source_disjoint", "untouched_source_disjoint"):
            raise ValueError("invalid source holdout kind")
        if self.holdout_kind == "untouched_source_disjoint":
            _hash(self.blinding_evidence_sha256)

    @property
    def source_sha256(self):
        return digest(sorted(b.manifest_sha256 for b in self.bindings))

    @property
    def sha256(self):
        return digest(self.to_dict())

    def to_dict(self):
        value = asdict(self)
        if not self.conditional_model_scope_sha256:
            del value["conditional_model_scope_sha256"]
        return value

    @classmethod
    def from_dict(cls, raw):
        value = dict(raw)
        value["bindings"] = tuple(Binding.from_dict(b) for b in value["bindings"])
        for field in ("captures", "queries"):
            value[field] = tuple(Identity(**i) for i in value[field])
        for field in ("derivation_sources", "heldout_sources"):
            value[field] = tuple(value[field])
        return cls(**value)


@dataclass(frozen=True)
class PoseObservation:
    role: str
    identity: Identity
    binding: Binding
    estimated_world_from_body: tuple[tuple[float, ...], ...] | None
    truth_world_from_body: tuple[tuple[float, ...], ...] | None
    rejection_reasons: tuple[str, ...]
    ned_from_world: tuple[tuple[float, ...], ...] = NED_FROM_ENU

    def __post_init__(self):
        if (self.role not in ("capture", "query") or not isinstance(self.identity, Identity)
                or not isinstance(self.binding, Binding) or not isinstance(self.rejection_reasons, tuple)
                or any(not isinstance(r, str) or not r for r in self.rejection_reasons)
                or not isinstance(self.ned_from_world, tuple)
                or any(not isinstance(r, tuple) for r in self.ned_from_world)):
            raise ValueError("immutable typed pose observation required")
        _rotation(self.ned_from_world)
        for value in (self.estimated_world_from_body, self.truth_world_from_body):
            if value is not None:
                _transform(value)

    @classmethod
    def from_dict(cls, raw):
        value = dict(raw)
        value["identity"] = Identity(**value["identity"])
        value["binding"] = Binding.from_dict(value["binding"])
        for field in ("estimated_world_from_body", "truth_world_from_body", "ned_from_world"):
            if field in value and value[field] is not None:
                value[field] = tuple(tuple(row) for row in value[field])
        value["rejection_reasons"] = tuple(value["rejection_reasons"])
        return cls(**value)


def relative_profile_payload(protocol: RelativeProtocol, observations):
    payload = {"schema": 3, "protocol": protocol.to_dict(), "protocol_sha256": protocol.sha256,
               "observations": [asdict(row) for row in observations]}
    # Return JSON-shaped payloads so file roundtrips do not change equality.
    import json
    payload = json.loads(json.dumps(payload, allow_nan=False))
    return {**payload, "profile_sha256": digest(payload)}


def _angle(rotation):
    skew = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]]) / 2
    return atan2(float(np.linalg.norm(skew)), float(np.clip((np.trace(rotation) - 1) / 2, -1., 1.)))


def _absolute_error(row):
    estimate, truth = np.asarray(row.estimated_world_from_body), np.asarray(row.truth_world_from_body)
    return np.asarray(row.ned_from_world) @ (estimate[:3, 3] - truth[:3, 3])


def _metrics(capture, query, rho):
    ec, eq = np.asarray(capture.estimated_world_from_body), np.asarray(query.estimated_world_from_body)
    tc, tq = np.asarray(capture.truth_world_from_body), np.asarray(query.truth_world_from_body)
    estimated = _inverse(eq) @ ec
    error = _inverse(estimated) @ (_inverse(tq) @ tc)
    angle = _angle(error[:3, :3])
    translation = float(np.linalg.norm(error[:3, 3]))
    return dict(translation_m=translation, rotation_rad=angle,
                rotation_displacement_m=2 * sin(angle / 2) * rho,
                displacement_m=translation + 2 * sin(angle / 2) * rho,
                linear_displacement_m=translation + angle * rho), error[:3, 3].tolist()


def _statistics(values):
    return {"count": len(values), "maximum": max(values) if values else None,
            "p50": float(np.percentile(values, 50)) if values else None,
            "p95": float(np.percentile(values, 95)) if values else None,
            "p99": float(np.percentile(values, 99)) if values else None}


def _axes(values):
    return [{"count": len(values), "signed_min": min((v[axis] for v in values), default=None),
             "signed_max": max((v[axis] for v in values), default=None),
             "absolute_max": max((abs(v[axis]) for v in values), default=None)} for axis in range(3)]


def _population(protocol, observations, sources, *, intervals=None):
    bindings = {b.manifest_sha256: b for b in protocol.bindings}
    members = {role: [i for i in identities if i.manifest_sha256 in sources]
               for role, identities in (("capture", protocol.captures), ("query", protocol.queries))}
    if intervals is not None:
        members = {role: [i for i in identities if intervals[0] <= i.source_ns <= intervals[1]]
                   for role, identities in members.items()}
    indexed, unexpected = {}, []
    for row in observations:
        if row.identity.manifest_sha256 not in sources:
            continue
        if intervals is not None and not intervals[0] <= row.identity.source_ns <= intervals[1]:
            continue
        if row.identity not in members[row.role]:
            unexpected.append({"role": row.role, "identity": asdict(row.identity)})
        indexed.setdefault((row.role, row.identity), []).append(row)
    selected, uncovered = {}, []
    for role, identities in members.items():
        for identity in identities:
            matches = indexed.get((role, identity), [])
            reasons = []
            if len(matches) != 1:
                reasons.append("MISSING_OBSERVATION" if not matches else "DUPLICATE_OBSERVATION")
            else:
                row = matches[0]
                reasons.extend(row.rejection_reasons)
                if row.binding != bindings[identity.manifest_sha256]:
                    reasons.append("SOURCE_BINDING_MISMATCH")
                if row.estimated_world_from_body is None and not reasons:
                    reasons.append("CAPTURE_QUERY_ESTIMATE_MISSING")
                if row.truth_world_from_body is None and not reasons:
                    reasons.append("CONTINUOUS_TRUTH_MISSING")
            if reasons:
                uncovered.append({"role": role, "identity": asdict(identity), "reasons": list(dict.fromkeys(reasons))})
            else:
                selected[(role, identity)] = matches[0]
    uncovered_by_key = {(u["role"], Identity(**u["identity"])): u["reasons"] for u in uncovered}
    metric_values = {field: [] for field in ("translation_m", "rotation_rad", "rotation_displacement_m",
                                            "displacement_m", "linear_displacement_m")}
    bins = [{"age_lo_s": lo * protocol.max_age_s, "age_hi_s": hi * protocol.max_age_s,
             "pair_count": 0, "covered_pair_count": 0, "translation_m": [], "rotation_rad": [],
             "displacement_m": [], "rotation_displacement_m": [], "axes": [],
             "absolute_capture": [], "absolute_query": []} for lo, hi in ((0., .25), (.25, .5), (.5, 1.))]
    pair_count = 0
    maximum_age = 0.
    missing_pairs, violations, worst, axes = [], [], {}, []
    queries_without_capture = []
    for source in sorted(sources):
        captures = sorted((i for i in members["capture"] if i.manifest_sha256 == source), key=lambda i: i.source_ns)
        stamps = [i.source_ns for i in captures]
        for query in sorted((i for i in members["query"] if i.manifest_sha256 == source), key=lambda i: i.source_ns):
            first = bisect_left(stamps, query.source_ns - int(protocol.max_age_s * 1e9))
            stop = bisect_right(stamps, query.source_ns)
            if first == stop:
                queries_without_capture.append(asdict(query))
            for capture in captures[first:stop]:
                age = (query.source_ns - capture.source_ns) / 1e9
                pair_count += 1
                maximum_age = max(maximum_age, age)
                bucket = next((b for b in bins if age <= b["age_hi_s"]), bins[-1])
                bucket["pair_count"] += 1
                pair = {"capture": asdict(capture), "query": asdict(query), "age_s": age}
                reasons = [*uncovered_by_key.get(("capture", capture), []),
                           *uncovered_by_key.get(("query", query), [])]
                if reasons:
                    missing_pairs.append({**pair, "reasons": list(dict.fromkeys(reasons))})
                    continue
                measured, translation_axes = _metrics(selected[("capture", capture)], selected[("query", query)], protocol.rho_m)
                axes.append(translation_axes)
                bucket["axes"].append(translation_axes)
                bucket["covered_pair_count"] += 1
                bucket["absolute_capture"].append(_absolute_error(selected[("capture", capture)]))
                bucket["absolute_query"].append(_absolute_error(selected[("query", query)]))
                for field, value in measured.items():
                    metric_values[field].append(value)
                    if field in bucket:
                        bucket[field].append(value)
                    if field not in worst or value >= worst[field][field]:
                        worst[field] = {**pair, **measured, "translation_capture_body_flu_m": translation_axes}
                fields = []
                if measured["translation_m"] > protocol.translation_limit_m:
                    fields.append("translation_m")
                if measured["rotation_rad"] > protocol.rotation_limit_rad:
                    fields.append("rotation_rad")
                limit = protocol.translation_limit_m + 2 * sin(protocol.rotation_limit_rad / 2) * protocol.rho_m
                if measured["displacement_m"] > limit:
                    fields.append("displacement_m")
                if fields:
                    violations.append({**pair, **measured, "fields": fields})
    absolute = {}
    for role in ("capture", "query"):
        role_rows = [row for (kind, _), row in selected.items() if kind == role]
        errors = [_absolute_error(row) for row in role_rows]
        absolute[role] = {"axis_ned": _axes(errors),
                          "radius_m": _statistics([float(np.linalg.norm(e)) for e in errors])}
        absolute[role]["rotation_rad"] = _statistics([
            _angle(np.asarray(row.estimated_world_from_body)[:3, :3].T
                   @ np.asarray(row.truth_world_from_body)[:3, :3]) for row in role_rows])
    for bucket in bins:
        bucket["axis_capture_body_flu_m"] = _axes(bucket.pop("axes"))
        for role in ("capture", "query"):
            errors = bucket.pop("absolute_" + role)
            bucket["absolute_" + role + "_pair_weighted"] = {
                "axis_ned": _axes(errors), "radius_m": _statistics([float(np.linalg.norm(e)) for e in errors])}
        for field in ("translation_m", "rotation_rad", "rotation_displacement_m", "displacement_m"):
            bucket[field] = _statistics(bucket[field])
    return {"capture_count": len(members["capture"]), "query_count": len(members["query"]),
            "pair_count": pair_count, "covered_pair_count": len(axes), "maximum_age_s": maximum_age,
            "metrics": {key: _statistics(values) for key, values in metric_values.items()},
            "axis_capture_body_flu_m": _axes(axes), "absolute": absolute, "age_bins": bins,
            "missing_pairs": missing_pairs, "violations": violations, "worst_pairs": worst,
            "uncovered_observations": uncovered, "unexpected_observations": unexpected,
            "queries_without_capture": queries_without_capture}


def evaluate_relative_profile(profile_payload: dict) -> dict:
    payload = {key: value for key, value in profile_payload.items() if key != "profile_sha256"}
    if payload.get("schema") != 3 or digest(payload) != profile_payload.get("profile_sha256"):
        raise ValueError("RELATIVE_PROFILE_HASH_MISMATCH")
    protocol = RelativeProtocol.from_dict(payload["protocol"])
    if protocol.sha256 != payload["protocol_sha256"]:
        raise ValueError("RELATIVE_PROTOCOL_HASH_MISMATCH")
    observations = tuple(PoseObservation.from_dict(row) for row in payload["observations"])
    sources = {binding.manifest_sha256 for binding in protocol.bindings}
    unbound = [{"role": row.role, "identity": asdict(row.identity), "reasons": ["SOURCE_BINDING_MISSING"]}
               for row in observations if row.identity.manifest_sha256 not in sources]
    reports = {name: _population(protocol, observations, sources) for name, sources in
               (("derivation", protocol.derivation_sources), ("heldout", protocol.heldout_sources))}
    reasons = ["RELATIVE_UNBOUND_SOURCE_OBSERVATIONS"] if unbound else []
    for name, report in reports.items():
        if (not report["pair_count"] or report["missing_pairs"] or report["uncovered_observations"]
                or report["unexpected_observations"] or report["queries_without_capture"]):
            reasons.append("RELATIVE_" + name.upper() + "_COVERAGE_UNRESOLVED")
        if report["violations"]:
            reasons.append("RELATIVE_" + name.upper() + "_CLAIM_VIOLATIONS")
    sampled_claim_met = not reasons
    # Hashes/labels do not establish independent validation, calibration or a hard bound.
    if protocol.holdout_kind != "untouched_source_disjoint":
        reasons.append("RELATIVE_NO_UNTOUCHED_SOURCE_HOLDOUT")
    if (protocol.holdout_kind == "untouched_source_disjoint"
            and set(protocol.heldout_sources) & SEEN_RETAINED_SOURCES):
        reasons.append("RELATIVE_PREVIOUSLY_SEEN_SOURCE_NOT_UNTOUCHED")
    if not protocol.conditional_model_scope_sha256:
        reasons.append("RELATIVE_CONDITIONAL_MODEL_SCOPE_MISSING")
    if any(b.body_point != "base_link_origin" or b.frame != "local_ned" or b.clock_source != "gazebo_sim"
           for b in protocol.bindings):
        reasons.append("RELATIVE_UNSUPPORTED_BODY_FRAME_OR_CLOCK")
    per_source = {source: _population(protocol, observations, (source,))
                  for source in sorted(b.manifest_sha256 for b in protocol.bindings)}
    temporal = {}
    for source in per_source:
        stamps = [i.source_ns for i in (*protocol.captures, *protocol.queries) if i.manifest_sha256 == source]
        if not stamps:
            continue
        midpoint = (min(stamps) + max(stamps)) // 2
        temporal[source] = {"split_ns": midpoint, "kind": "retrospective_same_source_NOT_independent",
            "first": _population(protocol, observations, (source,), intervals=(min(stamps), midpoint)),
            "second": _population(protocol, observations, (source,), intervals=(midpoint + 1, max(stamps)))}
    result = {"schema": 3, "profile_sha256": profile_payload["profile_sha256"],
              "protocol_sha256": protocol.sha256, "source_sha256": protocol.source_sha256,
              "provenance_sha256": protocol.provenance_sha256, "model_sha256": protocol.model_sha256,
              "max_age_s": protocol.max_age_s, "rho_m": protocol.rho_m,
              "translation_limit_m": protocol.translation_limit_m, "rotation_limit_rad": protocol.rotation_limit_rad,
              "displacement_limit_m": protocol.translation_limit_m + 2 * sin(protocol.rotation_limit_rad / 2) * protocol.rho_m,
              "basis": BASIS, "sampled_claim_met": sampled_claim_met,
              "conditional_model_scope_sha256": protocol.conditional_model_scope_sha256,
              "continuous_time_bound_certified": False,
              "limitations": ["Finite retained capture/query samples only; no continuous-time certificate",
                             "Pose-only analysis does not create instantaneous truth twist",
                             "External conditional model scope is a declaration, not calibrated by this evaluator"],
              "holdout_kind": protocol.holdout_kind, "runtime_admission": False,
              "calibration_established": False, "deterministic_bound_certified": False,
              "instantaneous_twist_covered": False, "refusal_reasons": reasons,
              "unbound_observations": unbound,
              "per_source": per_source, "same_source_temporal_diagnostics": temporal, **reports}
    return {**result, "evaluation_sha256": digest(result)}


def verify_capture_query_bound(bound: CaptureQueryBound, profile_payload: dict, evaluation_payload: dict, *,
        permit_age_s, max_rho_m, capture_clock_epoch, query_clock_epoch,
        capture_reset_counter, query_reset_counter, model_sha256, body_point,
        clock_source, provenance_sha256, capture_source_sha256, query_source_sha256) -> tuple[str, ...]:
    """Recompute actual pair coverage and bind a claim; empty refusals are not authority.

    Epoch/reset checks concern this bound's separately frozen trial scope, not old
    analysis recordings. Stream identities bind producer/model/calibration contracts.
    A complete untouched-source comparison can satisfy this prerequisite under an
    explicitly declared external conditional model; it never certifies that model.
    """
    try:
        if not isinstance(bound, CaptureQueryBound):
            raise ValueError("typed CaptureQueryBound required")
        evaluation = evaluate_relative_profile(profile_payload)
        if evaluation != evaluation_payload:
            return ("RELATIVE_EVALUATION_CONTENT_OR_HASH_MISMATCH",)
        reasons = list(evaluation["refusal_reasons"])
        if (bound.evaluation_sha256 != evaluation["evaluation_sha256"]
                or bound.source_sha256 != evaluation["source_sha256"]):
            reasons.append("RELATIVE_ANALYSIS_IDENTITY_MISMATCH")
        if (bound.max_age_s != evaluation["max_age_s"] or bound.rho_m != evaluation["rho_m"]
                or bound.translation_m != evaluation["translation_limit_m"]
                or bound.rotation_rad != evaluation["rotation_limit_rad"]):
            reasons.append("RELATIVE_CLAIM_MISMATCH")
        if bound.conditional_model_scope_sha256 != evaluation["conditional_model_scope_sha256"]:
            reasons.append("RELATIVE_CONDITIONAL_MODEL_SCOPE_MISMATCH")
        if (model_sha256 != bound.model_sha256 or bound.model_sha256 != evaluation["model_sha256"]):
            reasons.append("RELATIVE_MODEL_SCOPE_MISMATCH")
        if provenance_sha256 != bound.provenance_sha256 or bound.provenance_sha256 != evaluation["provenance_sha256"]:
            reasons.append("RELATIVE_PROVENANCE_SCOPE_MISMATCH")
        if (type(permit_age_s) not in (int, float) or not isfinite(permit_age_s)
                or not 0 <= permit_age_s <= bound.max_age_s):
            reasons.append("RELATIVE_PERMIT_AGE_OUT_OF_SCOPE")
        if type(max_rho_m) not in (int, float) or not isfinite(max_rho_m) or not 0 < max_rho_m <= bound.rho_m:
            reasons.append("RELATIVE_RHO_OUT_OF_SCOPE")
        if (any(not isinstance(v, str) or not v for v in (capture_clock_epoch, query_clock_epoch))
                or any(type(v) is not int for v in (capture_reset_counter, query_reset_counter))
                or not capture_clock_epoch == query_clock_epoch == bound.clock_epoch
                or not capture_reset_counter == query_reset_counter == bound.reset_counter):
            reasons.append("RELATIVE_EPOCH_RESET_SCOPE_MISMATCH")
        if (body_point != bound.body_point or bound.body_point != "base_link_origin"
                or clock_source != bound.clock_source or bound.clock_source != "gazebo_sim" or bound.basis != BASIS):
            reasons.append("RELATIVE_BODY_CLOCK_OR_BASIS_MISMATCH")
        if capture_source_sha256 != bound.capture_source_sha256 or query_source_sha256 != bound.query_source_sha256:
            reasons.append("RELATIVE_STREAM_SOURCE_SCOPE_MISMATCH")
        return tuple(dict.fromkeys(reasons))
    except (ValueError, TypeError, KeyError, OverflowError, IndexError) as error:
        return ("INVALID_RELATIVE_PAYLOAD:" + str(error),)
