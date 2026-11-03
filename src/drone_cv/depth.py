"""Normalize verified depth encodings without inventing free space."""

from dataclasses import dataclass
from enum import IntEnum
from math import isfinite

import numpy as np
from numpy.typing import NDArray

from drone_cv.timing import Stamp


class Validity(IntEnum):
    UNKNOWN = 0
    VALID = 1
    TOO_CLOSE = 2
    NO_RETURN = 3


@dataclass(frozen=True)
class DepthLimits:
    minimum_m: float
    maximum_m: float

    def __post_init__(self) -> None:
        if not (isfinite(self.minimum_m) and isfinite(self.maximum_m)
                and 0 < self.minimum_m < self.maximum_m):
            raise ValueError("depth limits must be finite, 0 < minimum < maximum")


@dataclass(frozen=True)
class DepthFrame:
    metres: NDArray[np.float64]
    validity: NDArray[np.uint8]
    stamp: Stamp

    def __post_init__(self) -> None:
        depth = np.array(self.metres, dtype=np.float64, copy=True)
        states = np.array(self.validity, copy=True)
        if depth.ndim != 2 or depth.size == 0 or states.shape != depth.shape:
            raise ValueError("depth and validity must be matching nonempty 2D arrays")
        if not np.isin(states, list(Validity)).all():
            raise ValueError("invalid depth validity state")
        known = (states == Validity.VALID) | (states == Validity.NO_RETURN)
        if np.any(known & (~np.isfinite(depth) | (depth <= 0))):
            raise ValueError("known depth must be finite and positive")
        depth[states == Validity.UNKNOWN] = np.nan
        depth[states == Validity.TOO_CLOSE] = 0.0
        depth.setflags(write=False)
        states = states.astype(np.uint8)
        states.setflags(write=False)
        object.__setattr__(self, "metres", depth)
        object.__setattr__(self, "validity", states)


def normalize_depth(
    image: NDArray, encoding: str, limits: DepthLimits, stamp: Stamp,
    *, verified_rep117: bool = False,
) -> DepthFrame:
    """Only verified +Inf is NO_RETURN; finite out-of-range values stay unknown.

    16UC1 zero is invalid. 32FC1 zero/negative finite values are invalid.
    Sensor adapters must verify REP-117 semantics before opting in.
    """
    expected = {"32FC1": np.dtype("float32"), "16UC1": np.dtype("uint16")}
    if encoding not in expected:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    raw = np.asarray(image)
    if raw.dtype != expected[encoding]:
        raise ValueError(f"dtype does not match {encoding}")
    if raw.ndim != 2 or raw.size == 0:
        raise ValueError("depth image must be nonempty and two-dimensional")
    metres = raw.astype(np.float64)
    if encoding == "16UC1":
        metres = metres * 0.001
    states = np.full(raw.shape, Validity.UNKNOWN, dtype=np.uint8)
    valid = np.isfinite(metres) & (metres >= limits.minimum_m)
    valid &= metres <= limits.maximum_m
    states[valid] = Validity.VALID
    near = np.isfinite(metres) & (metres > 0) & (metres < limits.minimum_m)
    states[near] = Validity.TOO_CLOSE
    if encoding == "32FC1" and verified_rep117:
        states[np.isneginf(metres)] = Validity.TOO_CLOSE
        far = np.isposinf(metres)
        states[far] = Validity.NO_RETURN
        metres[far] = limits.maximum_m
    return DepthFrame(metres, states, stamp)
