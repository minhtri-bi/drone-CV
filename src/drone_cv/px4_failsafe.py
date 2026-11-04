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
from drone_cv.px4_policy import (
    PARAMETER_DOMAINS, PINNED_FIRMWARE, REQUIRED_PARAMETERS,
    parameter_precision, validate_parameter_policy,
)

PARAMETER_COMMAND = "/opt/px4-gazebo/bin/px4-param"
VERSION_COMMAND = "/opt/px4-gazebo/bin/px4-ver"


def validate_policy(expected: dict, rationale: str, timeout_s: float) -> None:
    validate_parameter_policy(expected, rationale)
    if type(timeout_s) not in (int, float) or not isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("readback deadline must be finite and positive")


def read_parameters(expected: dict, rationale: str, timeout_s: float, firmware: str) -> dict:
    """Read current instance zero via pinned local shell client, never a saved file.

    Called immediately before publisher construction. Missing daemon, command,
    exact firmware identity, values or a bounded read all prevent admission.
    The trial must prohibit concurrent parameter writers; this is not security
    against a malicious local operator or proof of the configured flight action.
    """
    validate_policy(expected, rationale, timeout_s)
    expected = dict(expected)
    if firmware != PINNED_FIRMWARE:
        raise ValueError("readback requires the exact pinned firmware identity")
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
    observed, quiet_text, precision = {}, {}, {}
    for name in sorted(expected):
        raw = query([PARAMETER_COMMAND, "show", "-q", name])
        metadata = parameter_precision(name, expected[name])
        observed[name] = _quiet_value(name, raw, metadata)
        # Successful comparison attests the frozen expected policy within the
        # exact binary32 interval; four-decimal quiet text is only diagnostic.
        query([PARAMETER_COMMAND, "compare", "-s", name, str(expected[name])])
        quiet_text[name], precision[name] = raw, metadata
    if query([VERSION_COMMAND, "git"]) != version:
        raise ValueError("PX4 identity changed during readback")
    return {"instance": 0, "firmware": firmware, "expected": dict(expected),
            "observed": observed, "quiet_observed_text": quiet_text,
            "attested_policy": dict(expected), "precision": precision,
            "attestation": "pinned_param_compare_s",
            "rationale": rationale, "read_wall_s": time.monotonic(),
            "scope": "live_parameter_readback_not_fallback_execution_proof"}


def _quiet_value(name: str, raw: str, precision: dict) -> int | float:
    if not isinstance(raw, str):
        raise ValueError(f"missing live quiet parameter text: {name}")
    if PARAMETER_DOMAINS[name].kind == "int32":
        if not re.fullmatch(r"[+-]?\d+", raw) or int(raw) != precision["lower"]:
            raise ValueError(f"live integer parameter mismatch: {name}")
        return int(raw)
    if not re.fullmatch(r"[+-]?\d+\.\d{4}", raw):
        raise ValueError(f"missing or ambiguous live float parameter: {name}")
    # Preserve even a negative sign rounded to "-0.0000". Every float in this
    # pinned inventory has a nonnegative source domain; comparison to zero
    # alone would otherwise admit a tiny negative delay.
    if raw.startswith("-") and PARAMETER_DOMAINS[name].minimum >= 0:
        raise ValueError(f"negative live float outside pinned domain: {name}")
    value = float(raw)
    # Endpoints are actual accepted binary32 values; their formatted range is
    # a quiet-text consistency check, never a precision upgrade or attestation.
    if not (float(f'{precision["lower"]:.4f}') <= value
            <= float(f'{precision["upper"]:.4f}')):
        raise ValueError(f"live float quiet text contradicts comparison: {name}")
    return value


def attested_parameter_policy(report: dict) -> dict:
    """Recover the declared policy only from a complete readback attestation.

    This validates structure/precision consistency, not report authenticity.
    Runtime must use the report returned by its own bounded read_parameters call
    and prohibit concurrent parameter writers; loading a JSON claim is no proof.
    """
    if (not isinstance(report, dict) or report.get("attestation") != "pinned_param_compare_s"
            or report.get("firmware") != PINNED_FIRMWARE
            or type(report.get("instance")) is not int or report["instance"] != 0):
        raise ValueError("missing pinned parameter comparison attestation")
    policy = report.get("attested_policy")
    rationale = report.get("rationale")
    if not isinstance(policy, dict) or not isinstance(rationale, str):
        raise ValueError("attestation requires a complete typed policy and rationale")
    validate_parameter_policy(policy, rationale)
    expected_policy = report.get("expected")
    if not isinstance(expected_policy, dict):
        raise ValueError("missing complete expected parameter inventory")
    validate_parameter_policy(expected_policy, rationale)
    if report.get("expected") != policy:
        raise ValueError("attested policy differs from frozen expected policy")
    for key in ("precision", "observed", "quiet_observed_text"):
        if not isinstance(report.get(key), dict) or set(report[key]) != REQUIRED_PARAMETERS:
            raise ValueError("readback attestation requires complete precision inventory")
    for name, value in policy.items():
        metadata = parameter_precision(name, value)
        if report["precision"][name] != metadata:
            raise ValueError(f"readback precision metadata mismatch: {name}")
        quiet = _quiet_value(name, report["quiet_observed_text"][name], metadata)
        if type(report["observed"][name]) is not type(quiet) or report["observed"][name] != quiet:
            raise ValueError(f"quiet observed value mismatch: {name}")
    return dict(policy)


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
