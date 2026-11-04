from dataclasses import replace
from types import SimpleNamespace

import pytest

from drone_cv import px4_failsafe as failsafe
from drone_cv.supervisor import Status
from drone_cv.timing import Stamp


def policy():
    return {**{name: 1 for name in failsafe.REQUIRED_PARAMETERS},
            "UXRCE_DDS_SYNCT": 0, "COM_OBL_RC_ACT": 5, "COM_OF_LOSS_T": .5}


def test_missing_and_wrong_clock_policy_fail_before_readback():
    values = policy()
    del values["COM_LOW_BAT_ACT"]
    with pytest.raises(ValueError, match="complete"):
        failsafe.validate_policy(values, "synthetic test", 1.)
    with pytest.raises(ValueError, match="Gazebo"):
        failsafe.validate_policy({**policy(), "UXRCE_DDS_SYNCT": 1}, "synthetic test", 1.)


@pytest.mark.parametrize("fault", ["missing", "changed", "wrong_firmware", "comparison_failed"])
def test_live_readback_rejects_unknown_or_mismatching_daemon(monkeypatch, fault):
    expected = policy()
    monkeypatch.setattr(failsafe.Path, "is_file", lambda self: True)
    def run(argv, **kwargs):
        import subprocess
        if argv[0] == failsafe.VERSION_COMMAND:
            return SimpleNamespace(stdout=("b" if fault == "wrong_firmware" else "a")*40, stderr="")
        assert argv[1] in ("show", "compare")
        if argv[1] == "compare":
            if fault == "comparison_failed":
                raise subprocess.CalledProcessError(1, argv)
            return SimpleNamespace(stdout="", stderr="")
        name = argv[-1]
        value = expected[name]
        return SimpleNamespace(stdout=("" if fault == "missing" else str(value+1)
                                       if fault == "changed" else str(value)), stderr="")
    monkeypatch.setattr(failsafe.subprocess, "run", run)
    with pytest.raises((ValueError, failsafe.subprocess.CalledProcessError)):
        failsafe.read_parameters(expected, "synthetic readback negative", 1., "a"*40)


def status(sim=2., wall=11., nav=4):
    return Status(Stamp("epoch", int(sim*1e6), sim, wall), nav, 2, False, 0, True,
                  1, 1, 1., 1.)


def test_terminal_observation_does_not_turn_mode_into_safe_abort():
    observer = failsafe.TerminalObservation(10., 12., "epoch", 1, 1, "COMMAND_EXPIRED")
    assert observer.observe(status(wall=9.)).statuses == ()
    observed = observer.observe(status())
    assert observed.report()["status_observed"]
    assert not observed.report()["safe_abort_certified"]
    assert observed.reason == "COMMAND_EXPIRED"
    assert observed.observe(status(sim=3., wall=13.)) == observed
    invalid = observed.observe(replace(status(sim=3.), system_id=2))
    assert invalid.failure == "TERMINAL_SOURCE_MISMATCH"
    assert invalid.observe(status(sim=4.)) == invalid


def test_terminal_repeated_source_is_not_fresh_observation():
    observer = failsafe.TerminalObservation(10., 12., "epoch", 1, 1, "OWNER_CONFLICT")
    observed = observer.observe(status()).observe(status(wall=11.1))
    assert observed.failure == "TERMINAL_SOURCE_NOT_ADVANCING"
