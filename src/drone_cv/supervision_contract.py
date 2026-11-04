"""Explicit conditional simulation response and finite per-source service contracts.

Declarations are assumptions, not calibrated bounds. This module has no ROS,
parameter setter, authority publisher or hardware target.
"""
from dataclasses import dataclass
from math import isfinite

from drone_cv.timing import Clock, FreshnessPolicy

REQUIRED_SOURCES = frozenset({
    "ODOMETRY", "STATUS", "COMMAND", "PERMIT", "PERCEPTION",
    "LOCAL_POSITION", "FALLBACK_FLAGS", "BATTERY",
})


def finite(value: float, name: str, *, positive: bool = False) -> None:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not isfinite(value) or (value <= 0 if positive else value < 0)):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")


def text(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip() or len(value) > 512:
        raise ValueError(f"{name} requires bounded nonempty provenance/identity")


@dataclass(frozen=True)
class RateWindow:
    start: Clock
    end: Clock

    def __post_init__(self) -> None:
        if (self.start.epoch != self.end.epoch or self.end.sim_s < self.start.sim_s
                or self.end.wall_s <= self.start.wall_s):
            raise ValueError("invalid monotonic simulation-rate window")

    @property
    def rtf(self) -> float:
        return (self.end.sim_s-self.start.sim_s)/(self.end.wall_s-self.start.wall_s)


@dataclass(frozen=True)
class ClockContract:
    epoch: str
    minimum_rtf: float
    maximum_rtf: float
    window_wall_s: float
    maximum_observation_age_wall_s: float
    provenance: str
    source: str
    nominal_period_sim_s: float
    jitter_wall_s: float

    def __post_init__(self) -> None:
        for name in ("minimum_rtf", "maximum_rtf", "window_wall_s",
                     "maximum_observation_age_wall_s", "nominal_period_sim_s"):
            finite(getattr(self, name), name, positive=True)
        finite(self.jitter_wall_s, "clock source jitter")
        if self.minimum_rtf > self.maximum_rtf:
            raise ValueError("supported RTF minimum exceeds maximum")
        if self.source != "gazebo_sim":
            raise ValueError("only explicit Gazebo simulation clock is supported")
        text(self.epoch, "clock epoch")
        text(self.provenance, "clock contract provenance")

    @property
    def progress_deadline_wall_s(self) -> float:
        result = self.nominal_period_sim_s/self.minimum_rtf + self.jitter_wall_s
        if not isfinite(result):
            raise ValueError("clock service deadline overflow")
        return result

    def error(self, window: RateWindow | None, now: Clock) -> str | None:
        if window is None:
            return "RTF_OBSERVATION_MISSING"
        if self.epoch != now.epoch or window.end.epoch != now.epoch:
            return "RTF_EPOCH_MISMATCH"
        if window.end.sim_s > now.sim_s or window.end.wall_s > now.wall_s:
            return "RTF_OBSERVATION_FUTURE"
        duration = window.end.wall_s-window.start.wall_s
        if duration < self.window_wall_s:
            return "RTF_OBSERVATION_WINDOW_SHORT"
        if now.wall_s-window.end.wall_s > self.maximum_observation_age_wall_s:
            return "RTF_OBSERVATION_STALE"
        if not self.minimum_rtf <= window.rtf <= self.maximum_rtf:
            return "RTF_OUTSIDE_SUPPORTED_RANGE"
        return None


@dataclass(frozen=True)
class SourceBudget:
    name: str
    sim_ttl_s: float
    period_s: float
    jitter_wall_s: float
    cadence: str
    provenance: str

    def __post_init__(self) -> None:
        if self.name not in REQUIRED_SOURCES:
            raise ValueError("unknown required source budget")
        finite(self.sim_ttl_s, "source simulation TTL", positive=True)
        finite(self.period_s, "nominal source period", positive=True)
        finite(self.jitter_wall_s, "bounded source/transport/scheduler jitter")
        if self.cadence not in ("simulation", "wall"):
            raise ValueError("source cadence domain must be explicit")
        text(self.provenance, "source service provenance")

    def wall_deadline(self, clock: ClockContract) -> float:
        period_wall = (self.period_s / clock.minimum_rtf
                       if self.cadence == "simulation" else self.period_s)
        deadline = period_wall+self.jitter_wall_s
        if not isfinite(deadline):
            raise ValueError("source wall deadline overflow")
        return deadline

    def freshness(self, clock: ClockContract) -> FreshnessPolicy:
        return FreshnessPolicy(self.sim_ttl_s, self.wall_deadline(clock))


@dataclass(frozen=True)
class LossResponse:
    """Nonoverlapping decomposition in PX4/Gazebo simulation-motion seconds.

    mode_transition_s EXCLUDES failsafe_delay_s (COM_FAIL_ACT_T). Additional
    transport_clock_s excludes both and covers publish/receipt offset/lead and
    tracking-onset reserve. No second end-to-end interval is also added.
    """
    epoch: str
    firmware: str
    offboard_loss_s: float
    failsafe_delay_s: float
    mode_transition_s: float
    transport_clock_s: float
    offboard_loss_action: int
    provenance: str
    scope: str

    def __post_init__(self) -> None:
        text(self.epoch, "response epoch")
        text(self.provenance, "response provenance")
        if (not isinstance(self.firmware, str) or len(self.firmware) != 40
                or any(char not in "0123456789abcdef" for char in self.firmware)):
            raise ValueError("response requires exact firmware source identity")
        finite(self.offboard_loss_s, "Offboard loss timeout", positive=True)
        for name in ("failsafe_delay_s", "mode_transition_s", "transport_clock_s"):
            finite(getattr(self, name), name)
        if type(self.offboard_loss_action) is not int or not 0 <= self.offboard_loss_action <= 7:
            raise ValueError("unsupported pinned Offboard-loss action")
        if self.scope not in ("synthetic_offline", "exploratory_conditional"):
            raise ValueError("response must state its conditional, non-certified scope")

    def required_delay(self, tick_wall_s: float, clock: ClockContract) -> float:
        finite(tick_wall_s, "tick wall gap", positive=True)
        if self.epoch != clock.epoch:
            raise ValueError("response/clock epoch mismatch")
        # Upper RTF bounds simulated travel during wall scheduling delay. Runtime
        # readback attests float32 comparison intervals, not exact quiet text.
        loss, delay = self.offboard_loss_s, self.failsafe_delay_s
        if self.scope == "exploratory_conditional":
            from drone_cv.px4_policy import conservative_upper_float_parameter_bound
            loss = conservative_upper_float_parameter_bound("COM_OF_LOSS_T", loss)
            delay = conservative_upper_float_parameter_bound("COM_FAIL_ACT_T", delay)
        result = (tick_wall_s*clock.maximum_rtf + loss
                  + delay + self.mode_transition_s + self.transport_clock_s)
        if not isfinite(result):
            raise ValueError("loss-response budget overflow")
        return result

    def bind_policy(self, observed: dict, firmware: str) -> None:
        if firmware != self.firmware:
            raise ValueError("response firmware binding mismatch")
        expected = {"COM_OF_LOSS_T": self.offboard_loss_s,
                    "COM_FAIL_ACT_T": self.failsafe_delay_s,
                    "COM_OBL_RC_ACT": self.offboard_loss_action}
        if any(name not in observed or observed[name] != value for name, value in expected.items()):
            raise ValueError("RESPONSE_POLICY_MISMATCH")


def rate_window(samples: tuple[Clock, ...], contract: ClockContract) -> RateWindow | None:
    """Choose actual delivered clock observations spanning the frozen wall window."""
    if not samples:
        return None
    end = samples[-1]
    start = next((sample for sample in reversed(samples[:-1])
                  if sample.epoch == end.epoch
                  and end.wall_s-sample.wall_s >= contract.window_wall_s), None)
    return RateWindow(start, end) if start is not None else None
