"""Independent lease/telemetry supervisor, with no ROS, planner or truth imports.

All distances and velocities are local NED. Bounds are supplied by the operator,
not certified here. A bootstrap lease is exploratory, never navigation evidence.
Revocation stops all Offboard output; the configured PX4 failsafe owns recovery.
It is NOT proof of a safe stop. There is no automatic arm, retry, land or reclaim.
"""
from dataclasses import dataclass, replace
from enum import Enum
from math import hypot, isfinite, pi

from drone_cv.timing import Clock, FreshnessPolicy, Stamp, freshness_error
from drone_cv.supervision_contract import (
    ClockContract, LossResponse, RateWindow, REQUIRED_SOURCES, SourceBudget,
)
from drone_cv.relative_profile import CaptureQueryBound


def _positive(value: float, name: str) -> None:
    if not isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def _vector(value, name):
    result = tuple(value)
    if len(result) != 3 or not all(isfinite(v) for v in result):
        raise ValueError(f"{name} must be a finite 3-vector")
    return result


def _identity(value):
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("identity must be a nonempty string of at most 128 characters")


@dataclass(frozen=True)
class Envelope:
    minimum_ned: tuple[float, float, float]
    maximum_ned: tuple[float, float, float]
    maximum_speed_mps: float
    deceleration_mps2: float
    extra_margin_m: float
    yaw_error_rad: float
    position_variance_max: tuple[float, float, float]
    velocity_variance_max: tuple[float, float, float]
    bounds_provenance: str
    velocity_error_mps: tuple[float, float, float]
    tracking_error_m: float
    position_error_m: float
    target: str
    capture_query_bound: CaptureQueryBound | None = None

    def __post_init__(self):
        for name in ("minimum_ned", "maximum_ned"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        if any(a >= b for a, b in zip(self.minimum_ned, self.maximum_ned)):
            raise ValueError("envelope minimum must be below maximum")
        for name in ("maximum_speed_mps", "deceleration_mps2",
                     "extra_margin_m", "yaw_error_rad"):
            _positive(getattr(self, name), name)
        for name in ("position_variance_max", "velocity_variance_max"):
            value = _vector(getattr(self, name), name)
            if any(v <= 0 for v in value):
                raise ValueError("per-axis variance health ceilings must be positive")
            object.__setattr__(self, name, value)
        _identity(self.bounds_provenance)
        object.__setattr__(self, "velocity_error_mps",
                           _vector(self.velocity_error_mps, "velocity error"))
        if min(*self.velocity_error_mps, self.tracking_error_m, self.position_error_m) < 0:
            raise ValueError("motion uncertainty bounds must be nonnegative")
        if not all(isfinite(v) for v in (self.tracking_error_m, self.position_error_m)):
            raise ValueError("motion uncertainty bounds must be finite")
        if self.target not in ("synthetic_offline", "exploratory_simulation"):
            raise ValueError("unsupported motion target; hardware is not authorized")
        if self.yaw_error_rad >= pi / 2:
            raise ValueError("yaw error must be below pi/2")
        if self.capture_query_bound is not None and not isinstance(self.capture_query_bound, CaptureQueryBound):
            raise ValueError("typed capture/query uncertainty required")


@dataclass(frozen=True)
class Config:
    owner: str
    planner: str
    safety: str
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    request_freshness: FreshnessPolicy
    clock_contract: ClockContract
    source_budgets: tuple[SourceBudget, ...]
    loss_response: LossResponse
    clock_stall_wall_s: float
    max_clock_step_s: float
    max_tick_gap_wall_s: float
    prime_sim_s: float
    prime_wall_s: float
    request_timeout_s: float
    session_timeout_s: float
    max_command_ttl_s: float
    envelope: Envelope

    def __post_init__(self):
        for name in ("owner", "planner", "safety"):
            _identity(getattr(self, name))
        if len({self.owner, self.planner, self.safety}) != 3:
            raise ValueError("operator, planner and independent safety identities must differ")
        for name in ("source_system", "target_system", "target_component", "source_component"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 255:
                raise ValueError(f"{name} must be a non-broadcast uint8; no mode-executor identity")
        if (self.source_system, self.source_component) == (self.target_system, self.target_component):
            raise ValueError("external command identity must differ from the PX4 target")
        for name in ("clock_stall_wall_s", "max_clock_step_s",
                     "max_tick_gap_wall_s", "prime_sim_s", "prime_wall_s",
                     "request_timeout_s", "session_timeout_s", "max_command_ttl_s"):
            _positive(getattr(self, name), name)
        object.__setattr__(self, "source_budgets", tuple(self.source_budgets))
        if (len(self.source_budgets) != len(REQUIRED_SOURCES)
                or {value.name for value in self.source_budgets} != REQUIRED_SOURCES):
            raise ValueError("one explicit budget per required dynamic source is mandatory")
        for budget in self.source_budgets:
            budget.wall_deadline(self.clock_contract)
        if self.clock_stall_wall_s < self.clock_contract.progress_deadline_wall_s:
            raise ValueError("clock wall watchdog is below supported cadence/RTF plus jitter")
        if any(self.max_command_ttl_s > self.source(name).sim_ttl_s
               for name in ("COMMAND", "PERMIT")):
            raise ValueError("command TTL may not exceed source freshness policy")
        self.loss_response.required_delay(self.max_tick_gap_wall_s, self.clock_contract)
        expected_scope = ("synthetic_offline" if self.envelope.target == "synthetic_offline"
                          else "exploratory_conditional")
        if self.loss_response.scope != expected_scope:
            raise ValueError("response scope does not bind the motion target")

    def source(self, name: str) -> SourceBudget:
        return next(value for value in self.source_budgets if value.name == name)

    @property
    def response_delay_s(self) -> float:
        return self.loss_response.required_delay(self.max_tick_gap_wall_s, self.clock_contract)


@dataclass(frozen=True)
class Odometry:
    stamp: Stamp
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    yaw_ned: float
    reset_counter: int
    position_variance: tuple[float, float, float]
    velocity_variance: tuple[float, float, float]
    quaternion_squared_norm_error: float = 0.0

    def __post_init__(self):
        for name in ("position_ned", "velocity_ned", "position_variance", "velocity_variance"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        if not isfinite(self.yaw_ned) or not -pi <= self.yaw_ned <= pi:
            raise ValueError("yaw must be finite radians in [-pi,pi]")
        if type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255:
            raise ValueError("reset counter must be uint8")
        if min(*self.position_variance, *self.velocity_variance) < 0:
            raise ValueError("variances must be nonnegative")


@dataclass(frozen=True)
class Status:
    stamp: Stamp
    nav_state: int
    arming_state: int
    failsafe: bool
    executor_in_charge: int
    preflight_checks_pass: bool
    system_id: int
    component_id: int
    nav_state_since_s: float
    armed_since_s: float

    def __post_init__(self):
        for name in ("nav_state_since_s", "armed_since_s"):
            value = getattr(self, name)
            if not isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if max(self.nav_state_since_s, self.armed_since_s) > self.stamp.sim_s:
            raise ValueError("status transition timestamp cannot follow its source timestamp")
        if self.arming_state not in (1, 2) or not 0 <= self.nav_state < 31:
            raise ValueError("unsupported pinned PX4 status enum")


@dataclass(frozen=True)
class Command:
    source: str
    stamp: Stamp
    ttl_s: float
    velocity_ned: tuple[float, float, float]
    yaw_ned: float

    def __post_init__(self):
        _identity(self.source)
        _positive(self.ttl_s, "ttl_s")
        object.__setattr__(self, "velocity_ned", _vector(self.velocity_ned, "velocity"))
        if not isfinite(self.yaw_ned) or not -pi <= self.yaw_ned <= pi:
            raise ValueError("command yaw must be finite radians in [-pi,pi]")


@dataclass(frozen=True)
class Permit:
    """Certified NED reference-center box, eroded by footprint and phase clearance.

    The producer covers the COMPLETE box including near-field support. Additional
    state/tracking error and stopping travel are checked here exactly once.
    Bounds are absolute in the reset-bound local NED frame, not body coordinates.
    Original capture identity is never replaced by decision time.
    """
    source: str
    stamp: Stamp
    ttl_s: float
    command_source: str
    command_sequence: int
    velocity_ned: tuple[float, float, float]
    capture_stamp: Stamp
    minimum_ned: tuple[float, float, float]
    maximum_ned: tuple[float, float, float]
    reference_ned: tuple[float, float, float]
    reset_counter: int
    frame: str
    evidence_kind: str
    bounds_provenance: str
    capture_position_ned: tuple[float, float, float] | None = None
    evidence_rho_m: float | None = None
    relative_bound_sha256: str | None = None
    capture_source_sha256: str | None = None
    query_source_sha256: str | None = None

    def __post_init__(self):
        _identity(self.source)
        _identity(self.command_source)
        if not isinstance(self.capture_stamp, Stamp):
            raise ValueError("permit requires an original observation capture stamp")
        _positive(self.ttl_s, "ttl_s")
        if type(self.command_sequence) is not int or self.command_sequence < 0:
            raise ValueError("command sequence must be a nonnegative integer")
        object.__setattr__(self, "velocity_ned", _vector(self.velocity_ned, "velocity"))
        for name in ("minimum_ned", "maximum_ned", "reference_ned"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        if any(a >= b for a, b in zip(self.minimum_ned, self.maximum_ned)):
            raise ValueError("certificate minimum must be below maximum")
        if any(not lo <= p <= hi for p, lo, hi in
               zip(self.reference_ned, self.minimum_ned, self.maximum_ned)):
            raise ValueError("certificate reference must lie within certified region")
        if self.frame != "local_ned_reference_centers":
            raise ValueError("unsupported certificate frame")
        if self.evidence_kind not in ("bootstrap_known_clear", "perception_supported"):
            raise ValueError("unsupported certificate evidence kind")
        if type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255:
            raise ValueError("certificate reset counter must be uint8")
        _identity(self.bounds_provenance)
        if self.evidence_kind == "perception_supported":
            if self.capture_position_ned is None:
                raise ValueError("perception requires original capture position")
            position = _vector(self.capture_position_ned, "capture position")
            object.__setattr__(self, "capture_position_ned", position)
            if self.evidence_rho_m is None:
                raise ValueError("perception requires a complete supported-point radius")
            _positive(self.evidence_rho_m, "supported-point radius")
            # Producer must include all supported rays, complete geometry and
            # optical/body lever arms; this independent lower bound covers the
            # complete certified reference-center box, not only its center.
            minimum_radius = hypot(*(max(abs(lo-p), abs(hi-p)) for p, lo, hi in
                zip(position, self.minimum_ned, self.maximum_ned)))
            if self.evidence_rho_m < minimum_radius:
                raise ValueError("supported radius does not enclose the whole certified box")
            for value in (self.relative_bound_sha256, self.capture_source_sha256, self.query_source_sha256):
                if (not isinstance(value, str) or len(value) != 64
                        or any(c not in "0123456789abcdef" for c in value)):
                    raise ValueError("perception requires bound and canonical stream identities")
        elif any(value is not None for value in (
                self.capture_position_ned, self.evidence_rho_m, self.relative_bound_sha256,
                self.capture_source_sha256, self.query_source_sha256)):
            raise ValueError("bootstrap must not masquerade as a relative perception certificate")


@dataclass(frozen=True)
class Lease:
    owner: str
    stamp: Stamp
    sim_ttl_s: float
    wall_ttl_s: float
    kind: str

    def __post_init__(self):
        _identity(self.owner)
        _positive(self.sim_ttl_s, "sim_ttl_s")
        _positive(self.wall_ttl_s, "wall_ttl_s")
        if self.kind not in ("bootstrap", "mission"):
            raise ValueError("authorization kind must be bootstrap or mission")


@dataclass(frozen=True)
class OperatorRequest:
    owner: str
    stamp: Stamp
    action: str

    def __post_init__(self):
        _identity(self.owner)
        if self.action not in ("prime", "enter", "arm", "handoff", "stop"):
            raise ValueError("unsupported operator action")


@dataclass(frozen=True)
class Ack:
    stamp: Stamp
    command: int
    result: int
    target_system: int
    target_component: int
    from_external: bool


class Stage(str, Enum):
    IDLE = "idle"
    PRIMING = "priming"
    MODE_WAIT = "mode_wait"
    OFFBOARD_DISARMED = "offboard_disarmed"
    ARM_WAIT = "arm_wait"
    ACTIVE = "active"
    HANDOFF_WAIT = "handoff_wait"
    RELEASED = "released"
    REVOKED = "revoked"


@dataclass(frozen=True)
class Inputs:
    now: Clock
    odometry: Odometry | None
    status: Status | None
    command: Command | None
    permit: Permit | None
    lease: Lease | None
    graph_exclusive: bool
    request: OperatorRequest | None = None
    ack: Ack | None = None
    rate: RateWindow | None = None


@dataclass(frozen=True)
class State:
    stage: Stage = Stage.IDLE
    reason: str = "NOT_AUTHORIZED"
    started: Clock | None = None
    previous: Clock | None = None
    clock_progress_wall_s: float = 0.
    streams: tuple = ()
    hold_ned: tuple[float, float, float] | None = None
    hold_yaw: float = 0.
    reset_counter: int | None = None
    pending_since: Clock | None = None
    ack_accepted: bool = False
    issued_speed_bound_mps: float = 0.


@dataclass(frozen=True)
class Output:
    heartbeat: bool = False
    position_ned: tuple[float, float, float] | None = None
    velocity_ned: tuple[float, float, float] | None = None
    yaw_ned: float | None = None
    vehicle_command: str | None = None


def revoke(state: State, reason: str) -> tuple[State, Output]:
    if state.stage in (Stage.REVOKED, Stage.RELEASED):
        return state, Output()
    return replace(state, stage=Stage.REVOKED, reason=reason), Output()


def stopping_distance(cfg: Config, velocity_ned: tuple[float, float, float],
                      issued_speed_bound_mps: float = 0., *, absolute: bool = True) -> float:
    """Whole-direction conditional crash model; not measured stopping evidence."""
    env = cfg.envelope
    uncertain = hypot(*(abs(v)+error for v, error in zip(velocity_ned, env.velocity_error_mps)))
    speed = max(env.maximum_speed_mps, uncertain, issued_speed_bound_mps)
    return (speed*cfg.response_delay_s + speed**2/(2*env.deceleration_mps2)
            + env.extra_margin_m + env.tracking_error_m + (env.position_error_m if absolute else 0.))


def _motion_error(data: Inputs, cfg: Config, state: State) -> str | None:
    command, permit, actual = data.command, data.permit, data.odometry
    if command is None or permit is None:
        return "MISSING_COMMAND" if command is None else "MISSING_PERMIT"
    if actual is None:
        return "MISSING_ODOMETRY"
    if (permit.command_source != command.source or permit.command_sequence != command.stamp.sequence
            or permit.velocity_ned != command.velocity_ned):
        return "PERMIT_BINDING"
    if permit.capture_stamp.sim_s > permit.stamp.sim_s:
        return "PERCEPTION_FUTURE_DECISION"
    env = cfg.envelope
    speed = hypot(*command.velocity_ned)
    if speed > env.maximum_speed_mps:
        return "REQUESTED_SPEED"
    # Rotation/blind-direction planning is intentionally not implemented here.
    if abs((command.yaw_ned - state.hold_yaw + pi) % (2*pi) - pi) > 1e-6:
        return "YAW_COMMAND_UNSUPPORTED"
    if (permit.reset_counter != actual.reset_counter
            or permit.capture_stamp.epoch != actual.stamp.epoch
            or permit.stamp.epoch != actual.stamp.epoch
            or permit.bounds_provenance != env.bounds_provenance):
        return "PERMIT_FRAME_BINDING"
    if data.lease is None or (data.lease.kind == "mission"
                             and permit.evidence_kind != "perception_supported"):
        return "PERMIT_EVIDENCE_KIND"
    distance = stopping_distance(cfg, actual.velocity_ned, state.issued_speed_bound_mps, absolute=False)
    region_error = env.position_error_m
    if permit.evidence_kind == "perception_supported":
        bound = env.capture_query_bound
        if bound is None:
            return "RELATIVE_UNCERTAINTY_MISSING"
        if (bound.clock_epoch != actual.stamp.epoch or bound.reset_counter != actual.reset_counter
                or bound.body_point != "base_link_origin" or bound.clock_source != "gazebo_sim"
                or bound.basis != "capture_body_flu"
                or permit.relative_bound_sha256 != bound.sha256
                or permit.capture_source_sha256 != bound.capture_source_sha256
                or permit.query_source_sha256 != bound.query_source_sha256):
            return "RELATIVE_UNCERTAINTY_BINDING"
        if data.now.sim_s-permit.capture_stamp.sim_s > bound.max_age_s:
            return "RELATIVE_UNCERTAINTY_AGE"
        if permit.evidence_rho_m is None or permit.evidence_rho_m > bound.rho_m:
            return "RELATIVE_UNCERTAINTY_RADIUS"
        region_error = bound.displacement_m
    if not isfinite(distance+region_error):
        return "STOPPING_ROOM"
    # Relative displacement belongs to perception evidence. Independent absolute
    # frame error still protects the geofence; neither is counted twice.
    for p, low, high in zip(actual.position_ned, permit.minimum_ned, permit.maximum_ned):
        if p-distance-region_error < low or p+distance+region_error > high:
            return "STOPPING_ROOM"
    for p, low, high in zip(actual.position_ned, env.minimum_ned, env.maximum_ned):
        if p-distance-env.position_error_m < low or p+distance+env.position_error_m > high:
            return "PREDICTED_ENVELOPE"
    return None


def transition(state: State, data: Inputs, cfg: Config) -> tuple[State, Output]:
    """Pure immutable tick. All deadlines run even if the sim clock stops.

    Cached identical input is allowed only until its ORIGINAL TTL/progress limit.
    Restamped/resequenced cached data cannot refresh command authority. A new
    process/session with fresh explicit authorization is required after a latch.
    """
    if state.stage in (Stage.REVOKED, Stage.RELEASED):
        return state, Output()
    now, request = data.now, data.request
    if state.stage == Stage.IDLE and request is None:
        return state, Output()
    if state.previous:
        old = state.previous
        if now.epoch != old.epoch or now.sim_s < old.sim_s or now.wall_s < old.wall_s:
            return revoke(state, "CLOCK_DISCONTINUITY")
        if now.sim_s - old.sim_s > cfg.max_clock_step_s:
            return revoke(state, "CLOCK_JUMP")
        if now.wall_s - old.wall_s > cfg.max_tick_gap_wall_s:
            return revoke(state, "TICK_DEADLINE")
    rate_error = cfg.clock_contract.error(data.rate, now)
    if rate_error:
        return revoke(state, rate_error)
    progress = (now.wall_s if state.previous is None or now.sim_s > state.previous.sim_s
                else state.clock_progress_wall_s)
    if now.wall_s - progress > cfg.clock_stall_wall_s:
        return revoke(state, "CLOCK_STALLED")
    if state.started and (now.wall_s - state.started.wall_s > cfg.session_timeout_s
                          or now.sim_s - state.started.sim_s > cfg.session_timeout_s):
        return revoke(state, "SESSION_TIMEOUT")
    if not data.graph_exclusive:
        return revoke(state, "OWNER_CONFLICT")
    if data.lease is None:
        return revoke(state, "MISSING_AUTHORIZATION")
    if data.lease.owner != cfg.owner:
        return revoke(state, "AUTHORIZATION_OWNER")
    lease_error = freshness_error(data.lease.stamp, now,
        FreshnessPolicy(data.lease.sim_ttl_s, data.lease.wall_ttl_s))
    if lease_error:
        return revoke(state, "AUTHORIZATION_" + lease_error)
    if (now.sim_s >= data.lease.stamp.sim_s + data.lease.sim_ttl_s
            or now.wall_s >= data.lease.stamp.received_wall_s + data.lease.wall_ttl_s):
        return revoke(state, "AUTHORIZATION_EXPIRED")
    previous = {name: (value, wall) for name, value, wall in state.streams}
    updated = []
    action = None
    streams = (("ODOMETRY", data.odometry), ("STATUS", data.status),
               ("COMMAND", data.command), ("PERMIT", data.permit),
               ("AUTHORIZATION", data.lease), ("REQUEST", request),
               ("PERCEPTION", data.permit.capture_stamp if data.permit else None))
    for name, value in streams:
        if value is None:
            if name in ("ODOMETRY", "STATUS"):
                return revoke(state, "MISSING_" + name)
            if name in previous:
                updated.append((name, *previous[name]))
            continue
        old, wall = previous.get(name, (None, now.wall_s))
        # Requests are consumed once; their original freshness was checked then.
        # Keeping the cached request must neither renew authority nor abort later.
        if name == "REQUEST" and value == old:
            updated.append((name, old, wall))
            continue
        stamp = value if isinstance(value, Stamp) else value.stamp
        if isinstance(value, Stamp) and old is not None:
            if (stamp.epoch, stamp.sequence, stamp.sim_s) == (old.epoch, old.sequence, old.sim_s):
                # A fresh verdict cannot refresh the old capture's wall age.
                value = stamp = old
        budget = cfg.source(name) if name in REQUIRED_SOURCES else None
        policy = budget.freshness(cfg.clock_contract) if budget else cfg.request_freshness
        if isinstance(value, Lease):
            policy = FreshnessPolicy(value.sim_ttl_s, value.wall_ttl_s)
        error = freshness_error(stamp, now, policy)
        if error:
            return revoke(state, name + "_" + error)
        if isinstance(value, (Command, Permit)):
            if value.source != (cfg.planner if name == "COMMAND" else cfg.safety):
                return revoke(state, name + "_SOURCE")
            if value.ttl_s > cfg.max_command_ttl_s or now.sim_s >= value.stamp.sim_s + value.ttl_s:
                return revoke(state, name + "_EXPIRED")
        if old is not None and value != old:
            old_stamp = old if isinstance(old, Stamp) else old.stamp
            if (stamp.sequence <= old_stamp.sequence
                    or stamp.sim_s <= old_stamp.sim_s
                    or stamp.received_wall_s < old_stamp.received_wall_s):
                return revoke(state, name + "_SEQUENCE")
            wall = now.wall_s
        if budget is not None and now.wall_s - wall > budget.wall_deadline(cfg.clock_contract):
            return revoke(state, name + "_PROGRESS_STALLED")
        if isinstance(value, OperatorRequest) and value != old:
            if value.owner != cfg.owner:
                return revoke(state, "REQUEST_OWNER")
            action = value.action
        updated.append((name, value, wall))
    state = replace(state, previous=now, clock_progress_wall_s=progress, streams=tuple(updated))
    if action == "stop":
        return revoke(state, "OPERATOR_STOP")
    odom, status, env = data.odometry, data.status, cfg.envelope
    if odom is None or status is None:
        return revoke(state, "MISSING_TELEMETRY")
    if status.system_id != cfg.target_system or status.component_id != cfg.target_component:
        return revoke(state, "PX4_IDENTITY")
    if status.failsafe:
        return revoke(state, "PX4_FAILSAFE")
    if status.executor_in_charge != 0:
        return revoke(state, "PX4_OWNER_CONFLICT")
    if state.reset_counter is not None and odom.reset_counter != state.reset_counter:
        return revoke(state, "ODOMETRY_RESET")
    if hypot(*odom.velocity_ned) > env.maximum_speed_mps:
        return revoke(state, "MEASURED_SPEED")
    if any(not low <= p <= high for p, low, high in zip(
            odom.position_ned, env.minimum_ned, env.maximum_ned)):
        return revoke(state, "MEASURED_ENVELOPE")
    if (any(v > bound for v, bound in zip(odom.position_variance, env.position_variance_max))
            or any(v > bound for v, bound in zip(odom.velocity_variance, env.velocity_variance_max))):
        return revoke(state, "ODOMETRY_UNCERTAINTY")
    if state.started and abs((odom.yaw_ned-state.hold_yaw+pi) % (2*pi)-pi) > env.yaw_error_rad:
        return revoke(state, "MEASURED_YAW")
    if state.stage == Stage.IDLE:
        if action != "prime" or status.arming_state != 1 or status.nav_state == 14:
            return revoke(state, "STARTUP_REFUSED")
        state = replace(state, stage=Stage.PRIMING, reason="PRIMING", started=now,
                        hold_ned=odom.position_ned, hold_yaw=odom.yaw_ned,
                        reset_counter=odom.reset_counter)
    if state.stage == Stage.PRIMING and (status.arming_state != 1 or status.nav_state == 14):
        return revoke(state, "UNREQUESTED_AUTHORITY")
    if state.stage in (Stage.OFFBOARD_DISARMED, Stage.ARM_WAIT, Stage.ACTIVE):
        if status.nav_state != 14:
            return revoke(state, "OFFBOARD_LOST")
    prior_status = previous.get("STATUS", (None, 0.))[0]
    if prior_status is not None:
        if state.stage == Stage.MODE_WAIT and prior_status.nav_state == 14 and status.nav_state != 14:
            return revoke(state, "OFFBOARD_LOST")
        if state.stage == Stage.ARM_WAIT and prior_status.arming_state == 2 and status.arming_state != 2:
            return revoke(state, "ARMING_LOST")
    if state.stage == Stage.OFFBOARD_DISARMED and status.arming_state != 1:
        return revoke(state, "UNREQUESTED_ARM")
    if state.stage == Stage.ACTIVE and status.arming_state != 2:
        return revoke(state, "ARMING_LOST")
    if state.stage not in (Stage.PRIMING, Stage.HANDOFF_WAIT) or action == "enter":
        error = _motion_error(data, cfg, state)
        if error:
            return revoke(state, error)
    command_out = None
    if action in ("enter", "arm", "handoff"):
        allowed = {"enter": Stage.PRIMING, "arm": Stage.OFFBOARD_DISARMED,
                   "handoff": Stage.ACTIVE}
        if state.stage != allowed[action]:
            return revoke(state, "REQUEST_ORDER")
        if state.started is None:
            return revoke(state, "INVALID_STATE")
        if action == "enter" and (now.sim_s-state.started.sim_s < cfg.prime_sim_s
                                  or now.wall_s-state.started.wall_s < cfg.prime_wall_s):
            return revoke(state, "PRIME_INCOMPLETE")
        if action == "arm" and not status.preflight_checks_pass:
            return revoke(state, "PREFLIGHT_REFUSED")
        stage = {"enter": Stage.MODE_WAIT, "arm": Stage.ARM_WAIT,
                 "handoff": Stage.HANDOFF_WAIT}[action]
        command_out = {"enter": "offboard", "arm": "arm", "handoff": "loiter"}[action]
        state = replace(state, stage=stage, pending_since=now, ack_accepted=False)
    if state.stage in (Stage.MODE_WAIT, Stage.ARM_WAIT, Stage.HANDOFF_WAIT):
        pending, ack = state.pending_since, data.ack
        if pending is None:
            return revoke(state, "INVALID_STATE")
        if max(now.wall_s-pending.wall_s, now.sim_s-pending.sim_s) > cfg.request_timeout_s:
            return revoke(state, "REQUEST_TIMEOUT")
        command_id = 400 if state.stage == Stage.ARM_WAIT else 176
        if (command_out is None and ack is not None and ack.command == command_id
                and ack.target_system == cfg.source_system and ack.target_component == cfg.source_component
                and not ack.from_external and ack.stamp.sim_s > pending.sim_s
                and ack.stamp.received_wall_s > pending.wall_s
                and freshness_error(ack.stamp, now, cfg.request_freshness) is None):
            if ack.result != 0:
                return revoke(state, f"COMMAND_REJECTED_{ack.result}")
            state = replace(state, ack_accepted=True)
        fresh_status = (status.stamp.sim_s > pending.sim_s
                        and status.stamp.received_wall_s > pending.wall_s)
        confirmed = (status.nav_state == (4 if state.stage == Stage.HANDOFF_WAIT else 14)
                     and status.nav_state_since_s > (0. if state.stage == Stage.ARM_WAIT else pending.sim_s))
        if state.stage == Stage.ARM_WAIT:
            confirmed = confirmed and status.arming_state == 2 and status.armed_since_s > pending.sim_s
        elif state.stage == Stage.MODE_WAIT and status.arming_state != 1:
            return revoke(state, "UNREQUESTED_ARM")
        if state.ack_accepted and fresh_status and confirmed:
            stage = {Stage.MODE_WAIT: Stage.OFFBOARD_DISARMED, Stage.ARM_WAIT: Stage.ACTIVE,
                     Stage.HANDOFF_WAIT: Stage.RELEASED}[state.stage]
            state = replace(state, stage=stage, reason=stage.value)
    if state.stage in (Stage.HANDOFF_WAIT, Stage.RELEASED):
        return state, Output(vehicle_command=command_out)
    if status.nav_state != 14:
        # Priming signals availability only. PX4 warns against injecting ROS
        # trajectory setpoints into another mode's internal control pipeline.
        return state, Output(heartbeat=True, vehicle_command=command_out)
    state = replace(state, issued_speed_bound_mps=max(
        state.issued_speed_bound_mps, env.maximum_speed_mps,
        hypot(*data.command.velocity_ned) if data.command else 0.))
    if state.stage == Stage.ACTIVE:
        if data.command is None:
            return revoke(state, "MISSING_COMMAND")
        return state, Output(True, velocity_ned=data.command.velocity_ned,
                             yaw_ned=data.command.yaw_ned, vehicle_command=command_out)
    return state, Output(True, position_ned=state.hold_ned, yaw_ned=state.hold_yaw,
                         vehicle_command=command_out)
