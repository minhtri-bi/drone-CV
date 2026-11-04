"""Typed read-only pinned PX4 context; no ROS construction or control output.

Home is frozen static context. Local reference, capabilities and battery health
are dynamic observations with independent service budgets in the adapter.
"""
from dataclasses import dataclass
from math import isfinite

from drone_cv.px4_policy import HomeSnapshot, LocalReference
from drone_cv.timing import Stamp


def _integer(value, name: str, bits: int, *, positive: bool = False) -> int:
    if type(value) is not int or not (1 if positive else 0) <= value < 2**bits:
        raise ValueError(f"{name} must be {'positive ' if positive else ''}uint{bits}")
    return value


def _number(value, name: str) -> float:
    if type(value) not in (int, float) or not isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _boolean(value, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be a boolean")
    return value


def _stamp(value, epoch: str, wall_s: float) -> Stamp:
    stamp = _integer(value, "PX4 source timestamp", 64, positive=True)
    return Stamp(epoch, stamp, stamp/1e6, wall_s)


@dataclass(frozen=True)
class LocalObservation:
    stamp: Stamp
    reference: LocalReference
    reset_counters: tuple[int, int, int, int]
    heading_reset_counter: int
    invalid: tuple[str, ...]

    @property
    def frame_identity(self) -> tuple:
        return self.reference, self.reset_counters, self.heading_reset_counter


@dataclass(frozen=True)
class HomeObservation:
    stamp: Stamp
    position_ned: tuple[float, float, float]
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    update_count: int

    @property
    def identity(self) -> tuple:
        return (self.position_ned, self.latitude_deg, self.longitude_deg,
                self.altitude_m, self.update_count)

    def bind(self, local: LocalObservation) -> HomeSnapshot:
        if self.stamp.epoch != local.stamp.epoch or local.invalid:
            raise ValueError("Home/local reference scope is invalid")
        return HomeSnapshot(self.position_ned, self.latitude_deg, self.longitude_deg,
                            self.altitude_m, self.update_count, local.reference,
                            self.stamp.epoch, local.reset_counters)


@dataclass(frozen=True)
class FallbackObservation:
    stamp: Stamp
    hold_unavailable: tuple[str, ...]
    faults: tuple[str, ...]
    offboard_lost: bool
    gcs_lost: bool
    battery_warning: int


@dataclass(frozen=True)
class BatteryObservation:
    stamp: Stamp
    identity: int
    remaining: float
    connected: bool
    warning: int
    faults: int
    powering_off: bool


# Exact FailsafeBase::modeCanRun predicate for AUTO_LOITER (4), plus explicit
# headless/manual and workload checks below. A mode label alone is insufficient.
HOLD_REQUIREMENTS = (
    ("mode_req_angular_velocity", "angular_velocity_invalid"),
    ("mode_req_attitude", "attitude_invalid"),
    ("mode_req_local_alt", "local_altitude_invalid"),
    ("mode_req_local_position", "local_position_invalid"),
    ("mode_req_local_position_relaxed", "local_position_invalid_relaxed"),
    ("mode_req_global_position", "global_position_invalid"),
    ("mode_req_global_position_relaxed", "global_position_invalid_relaxed"),
    ("mode_req_mission", "auto_mission_missing"),
    ("mode_req_offboard_signal", "offboard_control_signal_lost"),
    ("mode_req_home_position", "home_position_invalid"),
)
HARD_FAILURES = (
    "local_velocity_invalid", "battery_low_remaining_time", "battery_unhealthy",
    "fd_critical_failure", "fd_esc_arming_failure", "fd_imbalanced_prop",
    "fd_motor_failure", "fd_alt_loss", "geofence_breached", "mission_failure",
    "vtol_fixed_wing_system_failure", "wind_limit_exceeded", "flight_time_limit_exceeded",
    "position_accuracy_low", "navigator_failure", "parachute_unhealthy",
    "traffic_avoidance_unhealthy", "remote_id_unhealthy", "gnss_lost",
)


def decode_local_position(msg, epoch: str, wall_s: float) -> LocalObservation:
    stamp = _stamp(msg.timestamp_sample, epoch, wall_s)
    reference = LocalReference(_number(msg.ref_lat, "reference latitude"),
        _number(msg.ref_lon, "reference longitude"), _number(msg.ref_alt, "reference altitude"),
        _integer(msg.ref_timestamp, "reference timestamp", 64, positive=True))
    resets = (
        _integer(msg.xy_reset_counter, "xy reset", 8),
        _integer(msg.z_reset_counter, "z reset", 8),
        _integer(msg.vxy_reset_counter, "velocity xy reset", 8),
        _integer(msg.vz_reset_counter, "velocity z reset", 8),
    )
    required = ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "xy_global", "z_global",
                "heading_good_for_control")
    invalid = tuple(name for name in required if not _boolean(getattr(msg, name), name))
    if _boolean(msg.dead_reckoning, "dead_reckoning"):
        invalid = (*invalid, "dead_reckoning")
    return LocalObservation(stamp, reference, resets,
                            _integer(msg.heading_reset_counter, "heading reset", 8), invalid)


def decode_home(msg, epoch: str, wall_s: float) -> HomeObservation:
    if not all(_boolean(getattr(msg, name), name) for name in ("valid_lpos", "valid_hpos", "valid_alt")):
        raise ValueError("Home requires valid local/global position and altitude")
    position = (_number(msg.x, "x"), _number(msg.y, "y"), _number(msg.z, "z"))
    return HomeObservation(_stamp(msg.timestamp, epoch, wall_s), position,
        _number(msg.lat, "Home latitude"), _number(msg.lon, "Home longitude"),
        _number(msg.alt, "Home altitude"), _integer(msg.update_count, "Home update count", 32))


def decode_fallback_flags(msg, epoch: str, wall_s: float) -> FallbackObservation:
    bit = 1 << 4  # Pinned NAVIGATION_STATE_AUTO_LOITER, not a configurable enum.
    unavailable = tuple(failure for requirement, failure in HOLD_REQUIREMENTS
        if _integer(getattr(msg, requirement), requirement, 32) & bit
        and _boolean(getattr(msg, failure), failure))
    if _integer(msg.mode_req_other, "mode_req_other", 32) & bit:
        unavailable = (*unavailable, "unknown_hold_requirement")
    if _integer(msg.mode_req_manual_control, "mode_req_manual_control", 32) & bit:
        unavailable = (*unavailable, "manual_input_required_by_hold")
    faults = tuple(name for name in HARD_FAILURES if _boolean(getattr(msg, name), name))
    return FallbackObservation(_stamp(msg.timestamp, epoch, wall_s), unavailable, faults,
        _boolean(msg.offboard_control_signal_lost, "offboard_control_signal_lost"),
        _boolean(msg.gcs_connection_lost, "gcs_connection_lost"),
        _integer(msg.battery_warning, "battery warning", 8))


def decode_battery(msg, epoch: str, wall_s: float) -> BatteryObservation:
    remaining = _number(msg.remaining, "battery remaining")
    if not 0 < remaining <= 1:
        raise ValueError("unknown/invalid battery remaining capacity")
    return BatteryObservation(_stamp(msg.timestamp, epoch, wall_s),
        _integer(msg.id, "battery identity", 8, positive=True), remaining,
        _boolean(msg.connected, "battery connected"), _integer(msg.warning, "battery warning", 8),
        _integer(msg.faults, "battery faults", 16), _boolean(msg.is_powering_off, "powering off"))


def observation_failure(value, *, offboard_required: bool) -> str | None:
    """Keep transient capability loss latched even if a healthy row follows."""
    if isinstance(value, LocalObservation) and value.invalid:
        return "LOCAL_REFERENCE_INVALID:" + ",".join(value.invalid)
    if isinstance(value, FallbackObservation):
        if value.hold_unavailable:
            return "HOLD_UNAVAILABLE:" + ",".join(value.hold_unavailable)
        if value.faults:
            return "UNSUPPORTED_FALLBACK_HEALTH:" + ",".join(value.faults)
        if value.gcs_lost:
            return "GCS_LINK_UNHEALTHY"
        if value.battery_warning:
            return "BATTERY_WARNING"
        if offboard_required and value.offboard_lost:
            return "OFFBOARD_SIGNAL_LOST"
    if isinstance(value, BatteryObservation) and (
            not value.connected or value.warning or value.faults or value.powering_off):
        return "BATTERY_UNHEALTHY"
    return None
