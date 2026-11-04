"""Conservative static-fixture geometry and an immutable sampled-observation ledger.

World AABB intersections are not physics-engine contact events. Neither clear
samples nor bounded sample spacing certify inter-sample swept-volume clearance.
"""

from dataclasses import asdict, dataclass, replace
from itertools import product
from math import hypot, isfinite
from numbers import Real
from pathlib import PurePosixPath, PureWindowsPath

import numpy as np
from numpy.typing import ArrayLike, NDArray


_GEOMETRY_SCOPE = "conservative_world_aabb_not_physics_contact"
_SAMPLE_SCOPE = "sampled_static_fixture_only_no_inter_sample_swept_volume_certificate"
_RIGID_TOLERANCE = 1e-8


def _mapping(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a dictionary")
    return value


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _integer(value: object, name: str, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _array(value: ArrayLike | None, shape: tuple[int, ...], name: str) -> NDArray[np.float64]:
    if value is None:
        raise ValueError(f"{name} must be a finite array of shape {shape}")
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite array of shape {shape}") from error
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite array of shape {shape}")
    return result


def _box(value: dict, name: str, prefix: str = "") -> tuple[NDArray, NDArray]:
    lo = _array(value.get(f"{prefix}lo"), (3,), f"{name} lower bound")
    hi = _array(value.get(f"{prefix}hi"), (3,), f"{name} upper bound")
    # Planes and other degenerate physical bounds remain valid without inflation.
    if np.any(lo > hi):
        raise ValueError(f"{name} lower bound exceeds upper bound")
    return lo, hi


def _items(value: object, name: str, *, nonempty: bool = True) -> list:
    if not isinstance(value, list) or (nonempty and not value):
        raise ValueError(f"{name} must be {'a nonempty list' if nonempty else 'a list'}")
    return value


@dataclass(frozen=True)
class _Part:
    identity: str
    role: str
    lo: NDArray
    hi: NDArray


@dataclass(frozen=True)
class _Surface:
    identity: str
    lo: NDArray
    hi: NDArray


@dataclass(frozen=True)
class _Support:
    part: str
    surface: str
    phases: tuple[str, ...]
    lo: NDArray
    hi: NDArray


def _profile(value: dict) -> tuple[_Part, ...]:
    value = _mapping(value, "profile")
    if type(value.get("schema")) is not int or value["schema"] != 1:
        raise ValueError("profile schema must be 1")
    if value.get("frame") != "base_link FLU":
        raise ValueError("profile frame must be base_link FLU")
    hashes = _mapping(value.get("source_sha256"), "source_sha256")
    if not hashes:
        raise ValueError("source_sha256 must not be empty")
    for path, digest in hashes.items():
        path = _text(path, "source path")
        normalized = PurePosixPath(path.replace("\\", "/"))
        if (normalized.is_absolute() or PureWindowsPath(path).drive
                or ".." in normalized.parts or not normalized.name):
            raise ValueError("source paths must be relative without parent traversal")
        if (not isinstance(digest, str) or len(digest) != 64
                or any(character not in "0123456789abcdefABCDEF" for character in digest)):
            raise ValueError("source digest must be a SHA-256 hexadecimal string")
    parts = []
    identities = set()
    for raw in _items(value.get("parts"), "parts"):
        raw = _mapping(raw, "part")
        identity = _text(raw.get("id"), "part id")
        if identity in identities:
            raise ValueError(f"duplicate part id: {identity}")
        identities.add(identity)
        role = _text(raw.get("role"), "part role")
        if role not in {"body", "rotor", "gear"}:
            raise ValueError(f"unknown part role: {role}")
        _text(raw.get("source"), "part source")
        lo, hi = _box(raw, f"part {identity}")
        parts.append(_Part(identity, role, lo, hi))
    return tuple(parts)


def _fixture(value: dict, parts: tuple[_Part, ...], phase: str
             ) -> tuple[tuple[_Surface, ...], tuple[_Support, ...], float, int]:
    value = _mapping(value, "fixture")
    margins = _mapping(value.get("clearance_by_phase"), "clearance_by_phase")
    for name, margin in margins.items():
        _text(name, "phase name")
        if _number(margin, f"phase {name} clearance") < 0:
            raise ValueError("phase clearances must be nonnegative")
    if phase not in margins:
        raise ValueError(f"unknown phase: {phase}")
    max_gap = _integer(value.get("max_truth_gap_ns"), "max_truth_gap_ns", 1)
    surfaces = []
    surface_ids = set()
    for raw in _items(value.get("surfaces"), "surfaces"):
        raw = _mapping(raw, "surface")
        identity = _text(raw.get("id"), "surface id")
        if identity in surface_ids:
            raise ValueError(f"duplicate surface id: {identity}")
        surface_ids.add(identity)
        lo, hi = _box(raw, f"surface {identity}")
        surfaces.append(_Surface(identity, lo, hi))
    part_ids = {part.identity for part in parts}
    supports = []
    for raw in _items(value.get("support_allowlist"), "support_allowlist", nonempty=False):
        raw = _mapping(raw, "support entry")
        part = _text(raw.get("part"), "support part")
        surface = _text(raw.get("surface"), "support surface")
        if part not in part_ids or surface not in surface_ids:
            raise ValueError("support entry references an unknown part or surface")
        phases = tuple(_text(item, "support phase")
                       for item in _items(raw.get("phases"), "support phases"))
        if len(set(phases)) != len(phases) or any(item not in margins for item in phases):
            raise ValueError("support phases must be unique known phases")
        lo, hi = _box(raw, "support zone", "zone_")
        supports.append(_Support(part, surface, phases, lo, hi))
    return tuple(surfaces), tuple(supports), float(margins[phase]), max_gap


def _rigid(value: ArrayLike) -> NDArray[np.float64]:
    transform = _array(value, (4, 4), "world_from_body")
    if not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-12):
        raise ValueError("world_from_body must have homogeneous bottom row [0,0,0,1]")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0, atol=_RIGID_TOLERANCE)
            or abs(float(np.linalg.det(rotation))-1) > _RIGID_TOLERANCE):
        raise ValueError("world_from_body rotation must be proper orthonormal within 1e-8")
    return transform


@dataclass(frozen=True)
class _Pair:
    part: str
    surface: str
    clearance_m: float
    allowed_support: bool
    unsafe: bool
    reason: str


@dataclass(frozen=True)
class _Evaluation:
    phase: str
    pairs: tuple[_Pair, ...]
    minimum_clearance_m: float
    unsafe: bool
    required_clearance_m: float
    max_truth_gap_ns: int

    def as_dict(self) -> dict:
        return {
            "phase": self.phase,
            "pairs": [asdict(pair) for pair in self.pairs],
            "minimum_clearance_m": self.minimum_clearance_m,
            "unsafe": self.unsafe,
            "required_clearance_m": self.required_clearance_m,
            "max_truth_gap_ns": self.max_truth_gap_ns,
            "geometry_scope": _GEOMETRY_SCOPE,
        }


def evaluate(profile: dict, world_from_body: ArrayLike, fixture: dict, phase: str) -> dict:
    """Evaluate every named component/surface pair using enclosing world AABBs.

    Separation is Euclidean for disjoint boxes, zero for touching boxes, and
    minus the smallest intersection extent otherwise. Support is allowed only
    for an exact gear/surface/phase entry whose single frozen zone contains the
    entire intersection, including a zero-thickness touching footprint. All
    other intersections are unsafe; disjoint pairs must meet the phase margin.
    Malformed or incomplete inputs raise ValueError for the caller to record.
    """
    phase = _text(phase, "phase")
    parts = _profile(profile)
    surfaces, supports, margin, max_gap = _fixture(fixture, parts, phase)
    transform = _rigid(world_from_body)
    pairs = []
    for part in parts:
        corners = np.array(list(product(*zip(part.lo, part.hi))), dtype=np.float64)
        with np.errstate(over="ignore", invalid="ignore"):
            world = corners @ transform[:3, :3].T + transform[:3, 3]
        if not np.isfinite(world).all():
            raise ValueError("transformed part geometry is not finite")
        lo, hi = world.min(axis=0), world.max(axis=0)
        for surface in surfaces:
            intersection_lo = np.maximum(lo, surface.lo)
            intersection_hi = np.minimum(hi, surface.hi)
            with np.errstate(over="ignore", invalid="ignore"):
                overlap = intersection_hi-intersection_lo
            if not np.isfinite(overlap).all():
                raise ValueError("box separation is not finite")
            intersects = bool(np.all(overlap >= 0))
            clearance = (min(0.0, -float(overlap.min())) if intersects
                         else hypot(*(np.maximum(-overlap, 0))))
            if not isfinite(clearance):
                raise ValueError("box separation is not finite")
            allowed = intersects and part.role == "gear" and any(
                entry.part == part.identity and entry.surface == surface.identity
                and phase in entry.phases
                and np.all(intersection_lo >= entry.lo)
                and np.all(intersection_hi <= entry.hi)
                for entry in supports
            )
            unsafe = not allowed and (intersects or clearance < margin)
            reason = ("ALLOWED_SUPPORT" if allowed else "GEOMETRIC_INTERSECTION" if intersects
                      else "CLEARANCE_BELOW_PHASE_MINIMUM" if unsafe else "CLEARANCE_SATISFIED")
            pairs.append(_Pair(part.identity, surface.identity, clearance,
                               bool(allowed), bool(unsafe), reason))
    return _Evaluation(phase, tuple(pairs), min(pair.clearance_m for pair in pairs),
                       any(pair.unsafe for pair in pairs), margin, max_gap).as_dict()


def _snapshot(value: dict) -> _Evaluation:
    """Detach the ledger from mutable caller dictionaries and reject partial results."""
    value = _mapping(value, "evaluation")
    phase = _text(value.get("phase"), "evaluation phase")
    margin = _number(value.get("required_clearance_m"), "evaluation required clearance")
    if margin < 0 or value.get("geometry_scope") != _GEOMETRY_SCOPE:
        raise ValueError("evaluation must declare nonnegative clearance and conservative geometry scope")
    max_gap = _integer(value.get("max_truth_gap_ns"), "evaluation max_truth_gap_ns", 1)
    pairs = []
    identities = set()
    for raw in _items(value.get("pairs"), "evaluation pairs"):
        raw = _mapping(raw, "evaluation pair")
        part = _text(raw.get("part"), "evaluation part")
        surface = _text(raw.get("surface"), "evaluation surface")
        if (part, surface) in identities:
            raise ValueError("evaluation contains duplicate pairs")
        identities.add((part, surface))
        clearance = _number(raw.get("clearance_m"), "pair clearance")
        allowed, unsafe = raw.get("allowed_support"), raw.get("unsafe")
        if type(allowed) is not bool or type(unsafe) is not bool:
            raise ValueError("pair allowed_support and unsafe must be booleans")
        if (allowed and clearance > 0
                or unsafe != (not allowed and (clearance <= 0 or clearance < margin))):
            raise ValueError("pair safety flags contradict clearance")
        pairs.append(_Pair(part, surface, clearance, allowed, unsafe,
                           _text(raw.get("reason"), "pair reason")))
    minimum = _number(value.get("minimum_clearance_m"), "evaluation minimum clearance")
    unsafe = value.get("unsafe")
    if (type(unsafe) is not bool or unsafe != any(pair.unsafe for pair in pairs)
            or minimum != min(pair.clearance_m for pair in pairs)):
        raise ValueError("evaluation summary contradicts its pairs")
    return _Evaluation(phase, tuple(pairs), minimum, unsafe, margin, max_gap)


@dataclass(frozen=True)
class _Observation:
    sim_ns: int
    epoch: int
    wall_ns: int
    evaluation: _Evaluation

    def as_dict(self) -> dict:
        return {"sim_ns": self.sim_ns, "epoch": self.epoch, "wall_ns": self.wall_ns,
                "evaluation": self.evaluation.as_dict()}


@dataclass(frozen=True)
class _Interval:
    start_sim_ns: int
    end_sim_ns: int
    epoch: int
    start_wall_ns: int
    end_wall_ns: int

    def as_dict(self) -> dict:
        return {**asdict(self), "scope": "sampled_endpoints_only",
                "continuous_clearance_certified": False}


@dataclass(frozen=True)
class _Gap:
    reason: str
    start_sim_ns: int | None
    end_sim_ns: int | None
    start_epoch: int | None
    end_epoch: int | None
    start_wall_ns: int | None
    end_wall_ns: int | None


def _gap(reason: str, before: _Observation | None, after: _Observation | None) -> _Gap:
    return _Gap(reason, before.sim_ns if before else None, after.sim_ns if after else None,
                before.epoch if before else None, after.epoch if after else None,
                before.wall_ns if before else None, after.wall_ns if after else None)


@dataclass(frozen=True)
class ObserverState:
    """Functional observer; retain each returned state, never mutate the old one.

    max_gap_ns is frozen on the first observation and must match the fixture.
    It bounds adjacent simulation deltas only; wall stamps must advance.
    The recorder's separate wall-clock watchdog owns actual clock-source loss.
    Timing gaps, resets and failures are sticky even after healthy recovery.
    Validation errors propagate: the caller must explicitly call fail(reason).
    """

    ledger: tuple[_Observation, ...] = ()
    coverage_intervals: tuple[_Interval, ...] = ()
    coverage_gaps: tuple[_Gap, ...] = ()
    execution_reasons: tuple[str, ...] = ()
    observed_unsafe: bool = False
    max_gap_ns: int | None = None

    def observe(self, evaluation: dict, *, sim_ns: int, epoch: int, wall_ns: int,
                max_gap_ns: int) -> "ObserverState":
        sim_ns = _integer(sim_ns, "sim_ns")
        epoch = _integer(epoch, "epoch")
        wall_ns = _integer(wall_ns, "wall_ns")
        max_gap_ns = _integer(max_gap_ns, "max_gap_ns", 1)
        snapshot = _snapshot(evaluation)
        if (snapshot.max_truth_gap_ns != max_gap_ns
                or self.max_gap_ns is not None and max_gap_ns != self.max_gap_ns):
            raise ValueError("max_gap_ns must match the frozen fixture and observer policy")
        current = _Observation(sim_ns, epoch, wall_ns, snapshot)
        gaps = tuple(replace(gap, end_sim_ns=sim_ns, end_epoch=epoch, end_wall_ns=wall_ns)
                     if gap.end_wall_ns is None else gap for gap in self.coverage_gaps)
        intervals = self.coverage_intervals
        reasons = []
        if self.ledger:
            previous = self.ledger[-1]
            if epoch != previous.epoch:
                reasons.append("EPOCH_CHANGED")
            elif sim_ns <= previous.sim_ns:
                reasons.append("SIM_STAMP_NOT_ADVANCING")
            elif sim_ns-previous.sim_ns > max_gap_ns:
                reasons.append("SIM_SAMPLE_GAP")
            if wall_ns <= previous.wall_ns:
                reasons.append("WALL_STAMP_NOT_ADVANCING")
            gaps += tuple(_gap(reason, previous, current) for reason in reasons)
            if not reasons and not any(gap.end_wall_ns is None for gap in self.coverage_gaps):
                intervals += (_Interval(previous.sim_ns, sim_ns, epoch,
                                        previous.wall_ns, wall_ns),)
        return replace(self, ledger=self.ledger+(current,), coverage_intervals=intervals,
                       coverage_gaps=gaps, observed_unsafe=self.observed_unsafe or snapshot.unsafe,
                       max_gap_ns=max_gap_ns)

    def fail(self, reason: str) -> "ObserverState":
        """Record an execution/collection failure without clearing the unsafe latch."""
        reason = _text(reason, "failure reason")
        before = self.ledger[-1] if self.ledger else None
        return replace(self, execution_reasons=self.execution_reasons+(reason,),
                       coverage_gaps=self.coverage_gaps+(_gap(reason, before, None),))

    def finish(self, complete: bool) -> dict:
        """Summarize safety separately from execution; completion cannot erase gaps.

        complete is the caller's assertion that the declared collection has
        ended, including its first/last required samples. Missing endpoints
        must instead be reported with fail() or complete=False.
        """
        if type(complete) is not bool:
            raise ValueError("complete must be a boolean")
        gaps = self.coverage_gaps
        reasons = list(self.execution_reasons)+[gap.reason for gap in gaps]
        if not self.ledger:
            reasons.append("NO_OBSERVATIONS")
        if not complete:
            reasons.append("COLLECTION_INCOMPLETE")
            gaps += (_gap("COLLECTION_INCOMPLETE", self.ledger[-1] if self.ledger else None,
                          None),)
        coverage_complete = bool(complete and self.ledger and not gaps
                                 and not self.execution_reasons)
        safety = ("UNSAFE_OBSERVED" if self.observed_unsafe else
                  "NO_VIOLATION_OBSERVED" if coverage_complete else "UNKNOWN")
        execution = ("INFRA_ERROR" if self.execution_reasons or self.coverage_gaps else
                     "COMPLETED" if coverage_complete else "INCOMPLETE")
        return {
            "safety_outcome": safety,
            "execution_outcome": execution,
            "observed_unsafe": self.observed_unsafe,
            "reasons": list(dict.fromkeys(reasons)),
            "execution_reasons": list(self.execution_reasons),
            "ledger": [observation.as_dict() for observation in self.ledger],
            "sample_count": len(self.ledger),
            "minimum_clearance_m": min((item.evaluation.minimum_clearance_m
                                         for item in self.ledger), default=None),
            "coverage_complete": coverage_complete,
            "coverage_intervals": [interval.as_dict() for interval in self.coverage_intervals],
            "coverage_gaps": [asdict(gap) for gap in gaps],
            "max_gap_ns": self.max_gap_ns,
            "scope": _SAMPLE_SCOPE,
            "continuous_clearance_certified": False,
        }
