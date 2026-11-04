"""Bounded offline moving oracle; declared enclosures are not physics contacts.

No transport, publisher or runtime discovery is implemented here. Discovery is a
cooperative observation ledger, not DDS authentication or access control.
"""
from dataclasses import dataclass, replace
from itertools import product
import json
from math import acos, hypot, pi, sin

import numpy as np

from drone_cv.oracle import (
    ObserverState, _fixture, _integer, _mapping, _number, _profile, _rigid, _text, evaluate,
)


_PHASES = ("rest", "takeoff", "navigation", "landing", "rest")
_NAV = ("DISARMED", "TAKEOFF", "OFFBOARD", "LAND", "DISARMED")


def _freeze(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))


def _bounded_snapshot(value):
    payload = _freeze(value)
    if len(payload.encode("utf-8")) > 65_536:
        raise ValueError("OBSERVATION_PAYLOAD_CAPACITY_EXCEEDED")
    return payload


def _positive_limit(value, name, ceiling):
    result = _integer(value, name, 1)
    if result > ceiling:
        raise ValueError(f"{name} exceeds bounded ledger capacity {ceiling}")
    return result


def _endpoint(raw):
    raw = _mapping(raw, "reader endpoint")
    if set(raw) != {"gid", "node", "namespace", "topic"}:
        raise ValueError("READER_ENDPOINT_IDENTITY_INCOMPLETE")
    return {key: _text(raw[key], f"endpoint {key}") for key in sorted(raw)}


def _validate_contract(profile, fixture):
    parts = _profile(profile)
    contract = _mapping(fixture.get("moving_contract"), "moving_contract")
    if type(contract.get("schema")) is not int or contract["schema"] != 1:
        raise ValueError("moving contract schema must be 1")
    sources = [_text(contract.get(key), key) for key in
               ("truth_source", "telemetry_source", "support_source")]
    if len(set(sources)) != len(sources):
        raise ValueError("phase sources must have distinct identities")
    _integer(contract.get("max_source_age_ns"), "max_source_age_ns")
    _integer(contract.get("max_wall_gap_ns"), "max_wall_gap_ns", 1)
    _positive_limit(contract.get("max_observations"), "max_observations", 10_000)
    if len(parts) > 256 or len(fixture.get("surfaces", [])) > 256:
        raise ValueError("moving geometry exceeds bounded part/surface capacity")
    schedule = contract.get("schedule")
    if not isinstance(schedule, list) or len(schedule) != len(_PHASES):
        raise ValueError("schedule must freeze rest/takeoff/navigation/landing/rest")
    previous_end = None
    for entry, phase in zip(schedule, _PHASES):
        entry = _mapping(entry, "schedule entry")
        start = _integer(entry.get("start_sim_ns"), "schedule start")
        end = _integer(entry.get("end_sim_ns"), "schedule end", 1)
        if (entry.get("phase") != phase or end <= start
                or previous_end is not None and start != previous_end):
            raise ValueError("schedule must be ordered, contiguous, nonempty phase windows")
        previous_end = end
        _fixture(fixture, parts, phase)
    # Navigation margin is a fixed project invariant, not a caller-selected exemption.
    if fixture["clearance_by_phase"]["navigation"] < .5:
        raise ValueError("navigation clearance must remain at least 0.5 m")
    for phase in ("takeoff",):
        if fixture["clearance_by_phase"][phase] < .5:
            raise ValueError("takeoff clearance must remain at least 0.5 m")
    for entry in fixture["support_allowlist"]:
        if any(phase not in {"rest", "landing"} for phase in entry["phases"]):
            raise ValueError("moving support exemptions are limited to rest/landing")
    motion = contract.get("motion")
    if motion is not None:
        motion = _mapping(motion, "motion bounds")
        _text(motion.get("provenance"), "motion provenance")
        for key in ("translation_speed_m_s", "angular_speed_rad_s"):
            if _number(motion.get(key), key) < 0:
                raise ValueError("motion bounds must be nonnegative")
    discovery = _mapping(contract.get("discovery"), "discovery contract")
    _text(discovery.get("source"), "discovery source")
    _integer(discovery.get("max_gap_ns"), "discovery max_gap_ns", 1)
    _positive_limit(discovery.get("max_snapshots"), "max_snapshots", 10_000)
    limit = _positive_limit(discovery.get("max_endpoints"), "max_endpoints", 256)
    readers = discovery.get("allowed_readers")
    if not isinstance(readers, list) or not readers or len(readers) > limit:
        raise ValueError("allowed_readers must be a bounded nonempty endpoint list")
    identities = [_freeze(_endpoint(reader)) for reader in readers]
    if len(set(identities)) != len(identities):
        raise ValueError("duplicate allowed reader identity")


def _source(raw, identity, observation, max_age):
    raw = _mapping(raw, "independent source evidence")
    stamp = _integer(raw.get("source_ns"), "source_ns")
    epoch = _integer(raw.get("epoch"), "source epoch")
    if (raw.get("source") != identity or epoch != observation["epoch"]
            or not 0 <= observation["sim_ns"]-stamp <= max_age):
        raise ValueError("PHASE_SOURCE_INVALID_OR_STALE")
    return raw


def _phase(contract, fixture, profile, observation, previous):
    sim_ns = observation["sim_ns"]
    index = next((i for i, window in enumerate(contract["schedule"])
                  if window["start_sim_ns"] <= sim_ns < window["end_sim_ns"]), None)
    reasons = []
    if (observation.get("truth_source") != contract["truth_source"]
            or type(observation.get("source_ns")) is not int
            or observation["source_ns"] != sim_ns):
        reasons.append("TRUTH_SOURCE_INVALID")
    if index is None:
        reasons.append("OUTSIDE_AUTHORIZED_SCHEDULE")
    previous_index = previous.get("schedule_index") if previous else None
    if previous is not None and not previous["valid"]:
        reasons.append("PHASE_AUTHORITY_LOST")
    if previous is not None and previous.get("epoch") != observation["epoch"]:
        reasons.append("PHASE_EPOCH_CHANGED")
    if index is not None:
        if previous is None and index != 0:
            reasons.append("PHASE_TRANSITION_SKIPPED")
        elif previous_index is not None:
            if index < previous_index:
                reasons.append("PHASE_TRANSITION_BACKWARD")
            elif index > previous_index+1:
                reasons.append("PHASE_TRANSITION_SKIPPED")
        try:
            telemetry = _source(observation.get("telemetry"), contract["telemetry_source"],
                                observation, contract["max_source_age_ns"])
            support = _source(observation.get("support"), contract["support_source"],
                              observation, contract["max_source_age_ns"])
            if (type(telemetry.get("armed")) is not bool
                    or type(telemetry.get("landed")) is not bool
                    or telemetry.get("armed") != (index not in (0, 4))
                    or telemetry.get("nav_state") != _NAV[index]
                    or index in (0, 4) and not telemetry["landed"]
                    or index == 2 and telemetry["landed"]):
                raise ValueError("PHASE_TELEMETRY_CONTRADICTION")
            contacts = support.get("contacts")
            if (not isinstance(contacts, list) or len(contacts) > 256
                    or any(not isinstance(pair, list) or len(pair) != 2
                           or any(not isinstance(item, str) or not item for item in pair)
                           for pair in contacts)):
                raise ValueError("SUPPORT_OBSERVATION_INVALID")
            candidate = evaluate(profile, observation["world_from_body"], fixture, _PHASES[index])
            geometric = {(pair["part"], pair["surface"]) for pair in candidate["pairs"]
                         if pair["allowed_support"]}
            reported = {tuple(pair) for pair in contacts}
            if len(reported) != len(contacts) or not reported <= geometric:
                raise ValueError("SUPPORT_GEOMETRY_CONTRADICTION")
            if index in (0, 4) and fixture["support_allowlist"] and not reported:
                raise ValueError("REST_SUPPORT_UNOBSERVED")
            exemptions = (reported if index in (0, 3, 4) and telemetry["landed"] else set())
        except (ValueError, TypeError, KeyError) as error:
            reasons.append(str(error))
            exemptions = set()
    else:
        exemptions = set()
    valid = not reasons
    phase = _PHASES[index] if valid and index is not None else "navigation"
    filtered = dict(fixture)
    filtered["support_allowlist"] = [entry for entry in fixture["support_allowlist"]
        if valid and (entry["part"], entry["surface"]) in exemptions]
    # Before actual landed/support observation, landing has navigation clearance.
    if valid and phase == "landing" and not exemptions:
        filtered["clearance_by_phase"] = {**fixture["clearance_by_phase"], "landing":
                                          fixture["clearance_by_phase"]["navigation"]}
    return {"sim_ns": sim_ns, "epoch": observation["epoch"], "wall_ns": observation["wall_ns"],
            "schedule_index": index, "phase": phase, "valid": valid, "reasons": reasons,
            "telemetry": observation.get("telemetry"), "support": observation.get("support"),
            "truth_source": observation.get("truth_source")}, filtered


def _known_violation(profile, fixture, observation, phase, evaluation):
    """Do not promote an unknown phase/exemption into a demonstrated violation."""
    contract = fixture["moving_contract"]
    if (observation.get("truth_source") != contract["truth_source"]
            or type(observation.get("source_ns")) is not int
            or observation["source_ns"] != observation["sim_ns"]):
        return False
    if phase["valid"]:
        return evaluation["unsafe"]
    rest = evaluate(profile, observation["world_from_body"], fixture, "rest")
    landing = evaluate(profile, observation["world_from_body"], fixture, "landing")
    possibly_allowed = {(pair["part"], pair["surface"])
                        for result in (rest, landing) for pair in result["pairs"]
                        if pair["allowed_support"]}
    minimum_margin = min(fixture["clearance_by_phase"].values())
    return any((pair["part"], pair["surface"]) not in possibly_allowed
               and (pair["clearance_m"] <= 0 or pair["clearance_m"] < minimum_margin)
               for pair in evaluation["pairs"])


def _motion_scope(contract):
    motion = contract.get("motion") if isinstance(contract, dict) else None
    return {
        "motion_bounds_provenance": motion.get("provenance") if isinstance(motion, dict) else None,
        "motion_bounds_calibrated": False,
        "independent_motion_bound_evidence_present": False,
        "motion_bounds_scope": "supplied_offline_intervals_only_no_independent_bound_evidence",
    }


def _interval(profile, fixture, before, after, first_phase, last_phase, sampled_unsafe):
    contract = fixture["moving_contract"]
    reasons = []
    dt_ns = after["sim_ns"]-before["sim_ns"]
    if before["epoch"] != after["epoch"]:
        reasons.append("EPOCH_CHANGED")
    if dt_ns <= 0:
        reasons.append("SIM_STAMP_NOT_ADVANCING")
    elif dt_ns > fixture["max_truth_gap_ns"]:
        reasons.append("SIM_SAMPLE_GAP")
    wall_delta = after["wall_ns"]-before["wall_ns"]
    if wall_delta <= 0 or wall_delta > contract["max_wall_gap_ns"]:
        reasons.append("WALL_SAMPLE_GAP")
    for sample in (before, after):
        if (sample.get("truth_source") != contract["truth_source"]
                or type(sample.get("source_ns")) is not int
                or sample["source_ns"] != sample["sim_ns"]):
            reasons.append("TRUTH_SOURCE_INVALID")
    if not first_phase["valid"] or not last_phase["valid"]:
        reasons.append("PHASE_UNOBSERVED")
    if first_phase["schedule_index"] != last_phase["schedule_index"]:
        reasons.append("PHASE_BOUNDARY_UNOBSERVED")
    motion = contract.get("motion")
    if motion is None:
        reasons.append("MOTION_BOUNDS_MISSING")
    pairs = []
    if not reasons and motion is not None:
        first = _rigid(before["world_from_body"])
        last = _rigid(after["world_from_body"])
        dt = dt_ns/1_000_000_000
        distance = hypot(*(last[:3, 3]-first[:3, 3]))
        angle = acos(float(np.clip((np.trace(first[:3, :3].T @ last[:3, :3])-1)/2, -1, 1)))
        if distance > motion["translation_speed_m_s"]*dt:
            reasons.append("TRANSLATION_BOUND_EXCEEDED")
        if angle > motion["angular_speed_rad_s"]*dt:
            reasons.append("ANGULAR_BOUND_EXCEEDED")
        parts = _profile(profile)
        surfaces, supports, margin, _ = _fixture(fixture, parts, first_phase["phase"])
        corroborated = ({tuple(pair) for pair in first_phase["support"]["contacts"]}
                        & {tuple(pair) for pair in last_phase["support"]["contacts"]})
        support_enabled = (first_phase["phase"] in {"rest", "landing"}
                           and first_phase["telemetry"]["landed"]
                           and last_phase["telemetry"]["landed"])
        if first_phase["phase"] == "landing" and not (support_enabled and corroborated):
            margin = max(margin, fixture["clearance_by_phase"]["navigation"])
        # Bound arbitrary intermediate excursions, not just endpoint differences.
        for part in parts:
            corners = np.array(list(product(*zip(part.lo, part.hi))), dtype=np.float64)
            radius = max(hypot(*corner) for corner in corners)
            displacement = (motion["translation_speed_m_s"]*dt
                + 2*radius*sin(min(motion["angular_speed_rad_s"]*dt, pi)/2))
            world = corners @ first[:3, :3].T + first[:3, 3]
            lo, hi = world.min(axis=0)-displacement, world.max(axis=0)+displacement
            if not np.isfinite([displacement, *lo, *hi]).all():
                reasons.append("MOTION_ENCLOSURE_NONFINITE")
                break
            for surface in surfaces:
                separation = hypot(*np.maximum(np.maximum(surface.lo-hi, lo-surface.hi), 0))
                overlap = bool(np.all(hi >= surface.lo) and np.all(lo <= surface.hi))
                intersection_lo, intersection_hi = np.maximum(lo, surface.lo), np.minimum(hi, surface.hi)
                allowed = bool(overlap and support_enabled and part.role == "gear"
                    and (part.identity, surface.identity) in corroborated and any(
                        entry.part == part.identity and entry.surface == surface.identity
                        and first_phase["phase"] in entry.phases
                        and np.all(intersection_lo >= entry.lo)
                        and np.all(intersection_hi <= entry.hi) for entry in supports))
                pairs.append({"part": part.identity, "surface": surface.identity,
                              "part_radius_m": radius, "displacement_bound_m": displacement,
                              "enclosure_separation_m": separation,
                              "allowed_support_enclosure": allowed,
                              "potential_violation": not allowed and (overlap or separation < margin)})
    outcome = ("SAMPLED_VIOLATION" if sampled_unsafe else "UNKNOWN" if reasons else
               "POTENTIAL_VIOLATION" if any(pair["potential_violation"] for pair in pairs) else "CLEAR")
    return {"start_sim_ns": before["sim_ns"], "end_sim_ns": after["sim_ns"],
            "start_epoch": before["epoch"], "end_epoch": after["epoch"],
            "outcome": outcome, "reasons": list(dict.fromkeys(reasons)), "pairs": pairs,
            "scope": "declared_conservative_enclosure_not_physics_contact",
            "clearance_conditional_on_declared_motion_bounds": outcome == "CLEAR",
            **_motion_scope(contract)}


@dataclass(frozen=True)
class MovingObserverState:
    """Immutable bounded ledger, frozen before observations; failures are sticky."""
    profile_json: str
    fixture_json: str
    sampled: ObserverState = ObserverState()
    records: tuple[str, ...] = ()
    phases: tuple[str, ...] = ()
    intervals: tuple[str, ...] = ()
    discovery_records: tuple[str, ...] = ()
    errors: tuple[str, ...] = ()

    @classmethod
    def create(cls, profile: dict, fixture: dict) -> "MovingObserverState":
        profile_json, fixture_json = _freeze(profile), _freeze(fixture)
        _validate_contract(json.loads(profile_json), json.loads(fixture_json))
        return cls(profile_json, fixture_json)

    def fail(self, reason: str) -> "MovingObserverState":
        reason = _text(reason, "failure reason")[:512]
        # Bound both the number and retained size of external failure reasons.
        return replace(self, errors=self.errors+((reason,) if reason not in self.errors
                       and len(self.errors) < 64 else ()))

    def observe(self, observation: dict) -> "MovingObserverState":
        profile, fixture = json.loads(self.profile_json), json.loads(self.fixture_json)
        contract = fixture["moving_contract"]
        try:
            frozen = _bounded_snapshot(observation)
            observation = _mapping(json.loads(frozen), "observation")
            for key in ("sim_ns", "wall_ns", "epoch"):
                _integer(observation.get(key), key)
            _rigid(observation["world_from_body"])
            previous_phase = json.loads(self.phases[-1]) if self.phases else None
            authority = ({**(previous_phase or {}), "valid": False}
                         if self.errors else previous_phase)
            phase, filtered = _phase(contract, fixture, profile, observation, authority)
            evaluation = evaluate(profile, observation["world_from_body"], filtered, phase["phase"])
            known_unsafe = _known_violation(profile, fixture, observation, phase, evaluation)
            phase["known_sampled_violation"] = known_unsafe
            if len(self.records) >= contract["max_observations"]:
                return replace(self.fail("OBSERVATION_CAPACITY_EXCEEDED"), sampled=replace(
                    self.sampled, observed_unsafe=self.sampled.observed_unsafe or known_unsafe))
            sampled = self.sampled.observe(evaluation, sim_ns=observation["sim_ns"],
                epoch=observation["epoch"], wall_ns=observation["wall_ns"],
                max_gap_ns=fixture["max_truth_gap_ns"])
            intervals = self.intervals
            sampled = replace(sampled, observed_unsafe=self.sampled.observed_unsafe or known_unsafe)
            if self.records:
                interval = _interval(profile, fixture, json.loads(self.records[-1]), observation,
                    previous_phase, phase, known_unsafe or
                    previous_phase["known_sampled_violation"])
                if self.errors:
                    interval["reasons"].append("PRIOR_OBSERVATION_FAILURE")
                    if interval["outcome"] != "SAMPLED_VIOLATION":
                        interval["outcome"] = "UNKNOWN"
                        interval["clearance_conditional_on_declared_motion_bounds"] = False
                intervals += (_freeze(interval),)
            state = replace(self, sampled=sampled, records=self.records+(frozen,),
                            phases=self.phases+(_freeze(phase),), intervals=intervals)
            for reason in phase["reasons"]:
                state = state.fail(reason)
            if (observation.get("truth_source") != contract["truth_source"]
                    or type(observation.get("source_ns")) is not int
                    or observation["source_ns"] != observation["sim_ns"]):
                state = state.fail("TRUTH_SOURCE_INVALID")
            return state
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            return self.fail(f"INVALID_MOVING_OBSERVATION:{error}")

    def discover(self, snapshot: dict) -> "MovingObserverState":
        contract = json.loads(self.fixture_json)["moving_contract"]["discovery"]
        try:
            snapshot = _mapping(json.loads(_bounded_snapshot(snapshot)), "discovery snapshot")
            _integer(snapshot.get("wall_ns"), "discovery wall_ns")
            if snapshot.get("source") != contract["source"]:
                raise ValueError("DISCOVERY_SOURCE_INVALID")
            if len(self.discovery_records) >= contract["max_snapshots"]:
                return self.fail("DISCOVERY_CAPACITY_EXCEEDED")
            readers = snapshot.get("readers")
            if not isinstance(readers, list) or len(readers) > contract["max_endpoints"]:
                raise ValueError("DISCOVERY_ENDPOINT_CAPACITY_OR_FORMAT")
            identities = [_freeze(_endpoint(reader)) for reader in readers]
            if len(set(identities)) != len(identities):
                raise ValueError("DISCOVERY_DUPLICATE_ENDPOINT")
            state = replace(self, discovery_records=self.discovery_records+(_freeze(snapshot),))
            if snapshot.get("error") is not None:
                state = state.fail("DISCOVERY_GRAPH_FAILURE")
            if self.discovery_records:
                delta = snapshot["wall_ns"]-json.loads(self.discovery_records[-1])["wall_ns"]
                if not 0 < delta <= contract["max_gap_ns"]:
                    state = state.fail("DISCOVERY_COVERAGE_GAP")
            allowed = {_freeze(_endpoint(reader)) for reader in contract["allowed_readers"]}
            if any(identity not in allowed for identity in identities):
                state = state.fail("UNEXPECTED_TRUTH_READER")
            return state
        except (ValueError, TypeError, KeyError) as error:
            return self.fail(f"INVALID_DISCOVERY_OBSERVATION:{error}")

    def finish(self, complete: bool) -> dict:
        result = self.sampled.finish(complete)
        # The sampled observer's static-only flag is not a moving-path alias.
        result.pop("continuous_clearance_certified")
        for interval in result["coverage_intervals"]:
            interval.pop("continuous_clearance_certified")
        contract = json.loads(self.fixture_json)["moving_contract"]
        records = [json.loads(item) for item in self.records]
        intervals = [json.loads(item) for item in self.intervals]
        discovery = [json.loads(item) for item in self.discovery_records]
        allowed = {_freeze(_endpoint(reader)) for reader in contract["discovery"]["allowed_readers"]}
        unexpected = {_freeze(_endpoint(reader)) for snapshot in discovery
                      for reader in snapshot["readers"] if _freeze(_endpoint(reader)) not in allowed}
        discovery_reasons = [reason for reason in self.errors if
                             "DISCOVERY" in reason or reason == "UNEXPECTED_TRUTH_READER"]
        covered = bool(records and discovery and discovery[0]["wall_ns"] <= records[0]["wall_ns"]
                       and discovery[-1]["wall_ns"] >= records[-1]["wall_ns"])
        if not covered:
            discovery_reasons.append("DISCOVERY_REQUIRED_ENDPOINTS_MISSING")
        discovery_complete = bool(complete and covered and not discovery_reasons)
        reasons = list(result["reasons"])+list(self.errors)+discovery_reasons
        for interval in intervals:
            reasons.extend(interval["reasons"])
            if interval["outcome"] == "POTENTIAL_VIOLATION":
                reasons.append("POTENTIAL_INTERVAL_VIOLATION")
        interval_clear = bool(intervals and all(
            item["clearance_conditional_on_declared_motion_bounds"] for item in intervals))
        coverage = bool(result["coverage_complete"] and interval_clear and discovery_complete
                        and not self.errors)
        if not intervals:
            reasons.append("NO_MOVING_INTERVALS")
        result.update(safety_outcome="UNSAFE_OBSERVED" if result["observed_unsafe"] else
                      "NO_VIOLATION_OBSERVED" if coverage else "UNKNOWN",
                      execution_outcome="COMPLETED" if coverage else
                      "INFRA_ERROR" if self.errors or result["execution_outcome"] == "INFRA_ERROR" else "INCOMPLETE",
                      coverage_complete=coverage,
                      clearance_conditional_on_declared_motion_bounds=coverage,
                      reasons=list(dict.fromkeys(reasons)), moving_intervals=intervals,
                      phase_ledger=[json.loads(item) for item in self.phases],
                      discovery={"ledger": discovery, "unexpected_readers": [json.loads(item)
                                 for item in sorted(unexpected)], "reasons": discovery_reasons,
                                 "cooperative_coverage_complete": discovery_complete,
                                 "dds_security_certified": False,
                                 "scope": "bounded_cooperative_detection_not_access_control"},
                      motion_assumptions=contract.get("motion"),
                      **_motion_scope(contract),
                      scope="OFFLINE_DECLARED_MOVING_CONTRACT", flight_admission=False,
                      live_acceptance_certified=False)
        return result
