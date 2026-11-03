import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


OBSERVER = load("g1_ros_observer", "scripts/g1_ros_observer.py")
INSTALLER = load("g1_install_locked", "docker/g1/install_locked.py")


def test_ros_big_endian_depth_summary_does_not_reverse_bytes():
    pixels = np.array([.5, 4.0, np.nan, np.inf], dtype=">f4")
    msg = SimpleNamespace(encoding="32FC1", step=16, width=4, height=1,
                          is_bigendian=True, data=pixels.tobytes(),
                          header=SimpleNamespace(frame_id="camera_optical"))
    result = OBSERVER.image_record(msg)
    assert result["finite_min_m"] == .5 and result["finite_max_m"] == 4
    assert result["nan_pixels"] == 1 and result["positive_inf_pixels"] == 1


def test_ros_truncated_depth_is_rejected_before_decode():
    msg = SimpleNamespace(encoding="32FC1", step=16, width=4, height=1,
                          is_bigendian=False, data=b"abcd")
    with pytest.raises(ValueError, match="byte count"):
        OBSERVER.image_record(msg)


@pytest.mark.parametrize("changed", [{"name": "foo;curl"}, {"version": "1 --bad-option"},
                                     {"sha256": "not-a-hash"}, {"architecture": "arm64"}])
def test_lock_rejects_untrusted_apt_arguments_and_wrong_platform(changed):
    entry = {"name": "ros-jazzy-bridge", "version": "1.2.3-1noble",
             "sha256": "a"*64, "architecture": "amd64", **changed}
    with pytest.raises(ValueError):
        INSTALLER.validate_packages([entry])


def test_duplicate_package_lock_cannot_choose_ambiguous_versions():
    entry = {"name": "ros-jazzy-bridge", "version": "1.2.3-1noble",
             "sha256": "a"*64, "architecture": "amd64"}
    with pytest.raises(ValueError, match="duplicate"):
        INSTALLER.validate_packages([entry, {**entry, "version": "2.0.0"}])
