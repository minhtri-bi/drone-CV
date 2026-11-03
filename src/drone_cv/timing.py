"""Immutable sample identity; never subtract timestamps from different clocks."""

from dataclasses import dataclass
from math import isfinite


def finite_nonnegative(value: float, name: str) -> None:
    if not isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class Stamp:
    epoch: str
    sequence: int
    sim_s: float
    received_wall_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.epoch, str) or not self.epoch:
            raise ValueError("epoch must be a nonempty string")
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("sequence must be a nonnegative integer")
        finite_nonnegative(self.sim_s, "sim_s")
        finite_nonnegative(self.received_wall_s, "received_wall_s")


@dataclass(frozen=True)
class Clock:
    epoch: str
    sim_s: float
    wall_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.epoch, str) or not self.epoch:
            raise ValueError("epoch must be a nonempty string")
        finite_nonnegative(self.sim_s, "sim_s")
        finite_nonnegative(self.wall_s, "wall_s")


@dataclass(frozen=True)
class FreshnessPolicy:
    sim_ttl_s: float
    wall_ttl_s: float

    def __post_init__(self) -> None:
        for name in ("sim_ttl_s", "wall_ttl_s"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


def freshness_error(stamp: Stamp, now: Clock, policy: FreshnessPolicy) -> str | None:
    """A replay/new epoch and either future clock are always invalid."""
    if stamp.epoch != now.epoch:
        return "epoch"
    sim_age = now.sim_s - stamp.sim_s
    wall_age = now.wall_s - stamp.received_wall_s
    if sim_age < 0:
        return "future_sim"
    if wall_age < 0:
        return "future_wall"
    if sim_age > policy.sim_ttl_s:
        return "stale_sim"
    if wall_age > policy.wall_ttl_s:
        return "stale_wall"
    return None


def require_fresh(stamp: Stamp, now: Clock, policy: FreshnessPolicy) -> None:
    reason = freshness_error(stamp, now, policy)
    if reason is not None:
        raise ValueError(f"unusable sample: {reason}")
