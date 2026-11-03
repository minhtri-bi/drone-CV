"""Static-obstacle, constant-deceleration bounds; NOT PX4 braking calibration."""

from dataclasses import dataclass
from math import atan2, floor, hypot, isfinite, pi, sqrt

import numpy as np

from drone_cv.frames import vector3, wrap_angle
from drone_cv.histogram import BINS, Histogram
from drone_cv.timing import Clock, FreshnessPolicy, Stamp, freshness_error


@dataclass(frozen=True)
class BrakingLimits:
    deceleration_mps2: float
    latency_s: float
    extra_margin_m: float
    maximum_speed_mps: float

    def __post_init__(self) -> None:
        values = (self.deceleration_mps2, self.latency_s,
                  self.extra_margin_m, self.maximum_speed_mps)
        if not all(isfinite(v) for v in values):
            raise ValueError("braking configuration must be finite")
        if (self.deceleration_mps2 <= 0 or self.latency_s < 0
                or self.extra_margin_m < 0 or self.maximum_speed_mps <= 0):
            raise ValueError("invalid braking configuration domain")


def speed_limit(free_distance_m: float, limits: BrakingLimits) -> float:
    """Distance is already swept-footprint free path; margin is EXTRA clearance."""
    if not isfinite(free_distance_m) or free_distance_m < 0:
        raise ValueError("free distance must be finite and nonnegative")
    available = max(0.0, free_distance_m-limits.extra_margin_m)
    if available == 0:
        return 0.0
    # Rationalized root avoids cancellation for large latency/small distance.
    time = hypot(limits.latency_s, sqrt(2*available/limits.deceleration_mps2))
    speed = 2*available / (time+limits.latency_s)
    if not isfinite(speed):
        raise ValueError("braking bound exceeds representable numeric range")
    return min(speed, limits.maximum_speed_mps)


@dataclass(frozen=True)
class MotionState:
    position_enu: tuple[float, float, float]
    velocity_enu: tuple[float, float, float]
    stamp: Stamp

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_enu", tuple(vector3(self.position_enu, "position")))
        object.__setattr__(self, "velocity_enu", tuple(vector3(self.velocity_enu, "velocity")))


@dataclass(frozen=True)
class Proposal:
    velocity_enu: tuple[float, float, float]
    yaw_rad: float
    stamp: Stamp

    def __post_init__(self) -> None:
        object.__setattr__(self, "velocity_enu", tuple(vector3(self.velocity_enu, "proposal")))
        object.__setattr__(self, "yaw_rad", wrap_angle(self.yaw_rad))


@dataclass(frozen=True)
class Decision:
    velocity_enu: tuple[float, float, float]
    yaw_rad: float
    speed_limit_mps: float
    brake_required: bool
    reason: str


def _direction_speed(vector: np.ndarray, histogram: Histogram,
                     limits: BrakingLimits) -> float:
    angle = atan2(vector[1], vector[0])
    sector = floor((angle+pi/BINS) % (2*pi) / (2*pi/BINS)) % BINS
    return speed_limit(float(histogram.free_distance_m[sector].min()), limits)


def decide_velocity(proposal: Proposal, actual: MotionState, histogram: Histogram,
                    now: Clock, policy: FreshnessPolicy, limits: BrakingLimits) -> Decision:
    """Fail closed in G0. A zero velocity request is NOT proof a drone can hover.

    Changing direction while moving is deliberately stop-then-turn. A moving
    origin needs reprojected coverage, not scalar distance subtraction. Yaw is
    returned as intent only: rotation-envelope safety belongs to the SITL layer.
    """
    def stop(reason: str) -> Decision:
        return Decision((0.0, 0.0, 0.0), proposal.yaw_rad, 0.0, True, reason)

    for name, stamp in (("PROPOSAL", proposal.stamp), ("ODOMETRY", actual.stamp),
                        ("DEPTH", histogram.stamp)):
        error = freshness_error(stamp, now, policy)
        if error is not None:
            return stop(f"{name}_{error}")
    if np.linalg.norm(np.subtract(actual.position_enu, histogram.origin_enu)) > 1e-6:
        return stop("ORIGIN_CHANGED")
    desired = np.asarray(proposal.velocity_enu)
    velocity = np.asarray(actual.velocity_enu)
    if desired[2] != 0 or velocity[2] != 0:
        return stop("VERTICAL_MOTION")
    current_speed = float(np.linalg.norm(velocity))
    requested_speed = float(np.linalg.norm(desired))
    if not isfinite(current_speed) or not isfinite(requested_speed):
        return stop("NUMERIC_RANGE")
    if current_speed > 0 and current_speed > _direction_speed(velocity, histogram, limits):
        return stop("STOPPING_ROOM")
    if requested_speed == 0:
        return Decision((0.0, 0.0, 0.0), proposal.yaw_rad, 0.0,
                        current_speed > 0, "STOP" if current_speed > 0 else "HOLD")
    if current_speed > 0:
        alignment = float((velocity/current_speed) @ (desired/requested_speed))
        if alignment < 1-1e-9:
            return stop("STOP_BEFORE_TURN")
    bound = _direction_speed(desired, histogram, limits)
    if bound == 0:
        return stop("NO_CORRIDOR")
    allowed_speed = min(bound, requested_speed)
    safe = tuple(float(v) for v in desired/requested_speed*allowed_speed)
    return Decision(safe, proposal.yaw_rad, bound, False,
                    "CLAMPED" if allowed_speed < requested_speed else "ALLOWED")
