"""Read-only local pinned-SITL admission and post-revocation observation.

No PX4 parameter setter or control command exists here. A frozen expected policy
is required per trial; this module supplies no deployment/default policy.
"""
from dataclasses import dataclass, replace
from math import isfinite
from pathlib import Path
import re
import subprocess
import time

from drone_cv.supervisor import Status

PARAMETER_COMMAND = "/opt/px4-gazebo/bin/px4-param"
VERSION_COMMAND = "/opt/px4-gazebo/bin/px4-ver"
REQUIRED_PARAMETERS = frozenset({
    "COM_OF_LOSS_T", "COM_OBL_RC_ACT", "COM_FAIL_ACT_T", "COM_RC_LOSS_T",
    "COM_RCL_EXCEPT", "NAV_RCL_ACT", "NAV_DLL_ACT", "COM_DL_LOSS_T",
    "COM_LOW_BAT_ACT", "BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR",
    "GF_ACTION", "GF_MAX_HOR_DIST", "GF_MAX_VER_DIST", "MPC_XY_VEL_MAX",
    "UXRCE_DDS_SYNCT",
})


def validate_policy(expected: dict, rationale: str, timeout_s: float) -> None:
    if not isinstance(expected, dict) or set(expected) != REQUIRED_PARAMETERS:
        raise ValueError("failsafe policy requires the complete pinned parameter inventory")
    if not isinstance(rationale, str) or not rationale.strip():
        raise ValueError("frozen trial policy rationale is required")
    if not isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("readback deadline must be finite and positive")
    for name, value in expected.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
            raise ValueError(f"nonfinite parameter policy: {name}")
    if expected["UXRCE_DDS_SYNCT"] != 0:
        raise ValueError("simulation requires Gazebo clock without DDS wall synchronization")
    if not 0 <= expected["COM_OBL_RC_ACT"] <= 7 or int(expected["COM_OBL_RC_ACT"]) != expected["COM_OBL_RC_ACT"]:
        raise ValueError("unsupported pinned Offboard-loss enum")
    if expected["COM_OF_LOSS_T"] <= 0 or expected["MPC_XY_VEL_MAX"] <= 0:
        raise ValueError("loss timeout and speed cap must be positive")


def read_parameters(expected: dict, rationale: str, timeout_s: float, firmware: str) -> dict:
    """Read current instance zero via pinned local shell client, never a saved file.

    Called immediately before publisher construction. Missing daemon, command,
    exact firmware identity, values or a bounded read all prevent admission.
    The trial must prohibit concurrent parameter writers; this is not security
    against a malicious local operator or proof of the configured flight action.
    """
    validate_policy(expected, rationale, timeout_s)
    if not Path(PARAMETER_COMMAND).is_file() or not Path(VERSION_COMMAND).is_file():
        raise ValueError("pinned local PX4 readback commands unavailable")
    deadline = time.monotonic() + timeout_s

    def query(argv: list[str]) -> str:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ValueError("parameter readback deadline expired")
        result = subprocess.run(argv, capture_output=True, text=True, timeout=remaining, check=True)
        if result.stderr.strip() or len(result.stdout) > 8192:
            raise ValueError("ambiguous PX4 readback output")
        return result.stdout.strip()

    version = query([VERSION_COMMAND, "git"])
    if firmware not in re.findall(r"\b[0-9a-f]{40}\b", version):
        raise ValueError("live PX4 firmware identity mismatch")
    observed = {}
    for name in sorted(expected):
        raw = query([PARAMETER_COMMAND, "show", "-q", name])
        if not re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", raw):
            raise ValueError(f"missing or ambiguous live parameter: {name}")
        value = float(raw)
        # The pinned quiet path prints four decimals. Additionally use PX4's
        # read-only comparison: integers exact, floats abs(error) < 1e-7.
        # Report that precision explicitly; this is not bit-exact float readback.
        if not isfinite(value) or value != float(f"{expected[name]:.4f}"):
            raise ValueError(f"live parameter mismatch: {name}")
        query([PARAMETER_COMMAND, "compare", "-s", name, str(expected[name])])
        observed[name] = value
    if query([VERSION_COMMAND, "git"]) != version:
        raise ValueError("PX4 identity changed during readback")
    return {"instance": 0, "firmware": firmware, "expected": dict(expected),
            "observed": observed, "rationale": rationale, "read_wall_s": time.monotonic(),
            "float_comparison_absolute_limit": 1e-7,
            "scope": "live_parameter_readback_not_fallback_execution_proof"}


@dataclass(frozen=True)
class TerminalObservation:
    started_wall_s: float
    deadline_wall_s: float
    epoch: str
    target_system: int
    target_component: int
    reason: str
    statuses: tuple[tuple[float, float, int, int, bool], ...] = ()
    failure: str | None = None

    def observe(self, status: Status) -> "TerminalObservation":
        stamp = status.stamp
        if self.failure:
            return self
        if (stamp.epoch != self.epoch or (status.system_id, status.component_id)
                != (self.target_system, self.target_component)):
            return replace(self, failure="TERMINAL_SOURCE_MISMATCH")
        if stamp.received_wall_s <= self.started_wall_s:
            return self
        if stamp.received_wall_s > self.deadline_wall_s:
            return self
        if self.statuses and stamp.sim_s <= self.statuses[-1][0]:
            return replace(self, failure="TERMINAL_SOURCE_NOT_ADVANCING")
        if len(self.statuses) >= 256:
            return replace(self, failure="TERMINAL_OBSERVATION_CAPACITY")
        row = (stamp.sim_s, stamp.received_wall_s, status.nav_state,
               status.arming_state, status.failsafe)
        return replace(self, statuses=(*self.statuses, row))

    def report(self) -> dict:
        return {"reason": self.reason, "statuses": self.statuses,
                "started_wall_s": self.started_wall_s, "deadline_wall_s": self.deadline_wall_s,
                "failure": self.failure, "status_observed": bool(self.statuses),
                "safe_abort_certified": False,
                "scope": "bounded_terminal_status_observation_not_stopping_or_fallback_proof"}
