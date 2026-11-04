from dataclasses import replace
from types import SimpleNamespace

import pytest

from drone_cv import px4_failsafe as failsafe
from drone_cv.supervisor import Status
from drone_cv.timing import Stamp


def policy():
    return {
        "COM_OF_LOSS_T": .5, "COM_OBL_RC_ACT": 5, "COM_FAIL_ACT_T": 0.,
        "COM_RC_IN_MODE": 4, "COM_RC_LOSS_T": .5, "COM_RCL_EXCEPT": 0,
        "NAV_RCL_ACT": 1, "NAV_DLL_ACT": 1, "COM_DL_LOSS_T": 10,
        "COM_DLL_EXCEPT": 0, "COM_LOW_BAT_ACT": 2, "BAT_LOW_THR": .15,
        "BAT_CRIT_THR": .07, "BAT_EMERGEN_THR": .05,
        "GF_ACTION": 2, "GF_SOURCE": 0, "GF_MAX_HOR_DIST": 20., "GF_MAX_VER_DIST": 10.,
        "MPC_XY_VEL_MAX": 1., "MPC_Z_VEL_MAX_UP": .5, "MPC_Z_VEL_MAX_DN": .5,
        "MPC_POS_MODE": 4, "MPC_ACC_HOR": 3., "MPC_ACC_HOR_MAX": 5.,
        "MPC_ACC_UP_MAX": 4., "MPC_ACC_DOWN_MAX": 3.,
        "MPC_JERK_MAX": 8., "MPC_JERK_AUTO": 4., "UXRCE_DDS_SYNCT": 0,
    }


def test_missing_and_wrong_clock_policy_fail_before_readback():
    values = policy()
    del values["COM_LOW_BAT_ACT"]
    with pytest.raises(ValueError, match="complete"):
        failsafe.validate_policy(values, "synthetic test", 1.)
    with pytest.raises(ValueError, match="Gazebo"):
        failsafe.validate_policy({**policy(), "UXRCE_DDS_SYNCT": 1}, "synthetic test", 1.)


@pytest.mark.parametrize("fault", ["missing", "changed", "wrong_firmware",
                                 "comparison_failed", "negative_zero"])
def test_live_readback_rejects_unknown_or_mismatching_daemon(monkeypatch, fault):
    expected = policy()
    monkeypatch.setattr(failsafe.Path, "is_file", lambda self: True)
    def run(argv, **kwargs):
        import subprocess
        if argv[0] == failsafe.VERSION_COMMAND:
            return SimpleNamespace(stdout=("b"*40 if fault == "wrong_firmware"
                                           else failsafe.PINNED_FIRMWARE), stderr="")
        assert argv[1] in ("show", "compare")
        if argv[1] == "compare":
            if fault == "comparison_failed":
                raise subprocess.CalledProcessError(1, argv)
            return SimpleNamespace(stdout="", stderr="")
        name = argv[-1]
        value = expected[name]
        output = str(value) if type(value) is int else f"{value:.4f}"
        if fault == "negative_zero" and name == "COM_FAIL_ACT_T":
            output = "-0.0000"
        return SimpleNamespace(stdout=("" if fault == "missing" else str(value+1)
                                       if fault == "changed" else output), stderr="")
    monkeypatch.setattr(failsafe.subprocess, "run", run)
    with pytest.raises((ValueError, failsafe.subprocess.CalledProcessError)):
        failsafe.read_parameters(expected, "synthetic readback negative", 1.,
                                 failsafe.PINNED_FIRMWARE)


@pytest.mark.parametrize("declared", [.333, .33333])
def test_quiet_text_is_not_the_comparison_attested_live_float(monkeypatch, declared):
    from drone_cv.px4_policy import parameter_precision
    expected = policy() | {"COM_OF_LOSS_T": declared}
    monkeypatch.setattr(failsafe.Path, "is_file", lambda self: True)
    calls = []
    def run(argv, **kwargs):
        calls.append(tuple(argv))
        if argv[0] == failsafe.VERSION_COMMAND:
            return SimpleNamespace(stdout=failsafe.PINNED_FIRMWARE, stderr="")
        if argv[1] == "compare":
            return SimpleNamespace(stdout="", stderr="")
        value = expected[argv[-1]]
        return SimpleNamespace(stdout=str(value) if type(value) is int else f"{value:.4f}",
                               stderr="")
    monkeypatch.setattr(failsafe.subprocess, "run", run)
    report = failsafe.read_parameters(expected, "synthetic precision", 1.,
                                      failsafe.PINNED_FIRMWARE)
    assert report["quiet_observed_text"]["COM_OF_LOSS_T"] == f"{declared:.4f}"
    assert report["observed"]["COM_OF_LOSS_T"] == float(f"{declared:.4f}")
    assert report["attested_policy"]["COM_OF_LOSS_T"] == declared
    assert report["precision"]["COM_OF_LOSS_T"] == parameter_precision("COM_OF_LOSS_T", declared)
    assert failsafe.attested_parameter_policy(report) == expected
    assert (failsafe.PARAMETER_COMMAND, "compare", "-s", "COM_OF_LOSS_T", str(declared)) in calls
    broken = dict(report, precision=dict(report["precision"]))
    broken["precision"]["COM_OF_LOSS_T"] = {"type": "float32", "upper": declared}
    with pytest.raises(ValueError, match="precision"):
        failsafe.attested_parameter_policy(broken)
    with pytest.raises(ValueError, match="attestation"):
        failsafe.attested_parameter_policy({"observed": report["observed"]})


def test_full_inventory_and_canonical_types_precede_any_readback(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("must not query an unvalidated policy")
    monkeypatch.setattr(failsafe.subprocess, "run", forbidden)
    with pytest.raises(ValueError, match="int32"):
        failsafe.read_parameters(policy() | {"COM_DL_LOSS_T": 10.}, "synthetic", 1.,
                                 failsafe.PINNED_FIRMWARE)


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
