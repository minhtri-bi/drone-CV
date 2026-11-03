from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Pose
from model_profile import diagnostic_envelope, model_profile

SOURCES = Path(__file__).parents[3] / "docs/replay/model-profile-r51"


def test_pinned_collision_proxy_covers_gear_and_rotating_propeller_extremes():
    profile = model_profile(SOURCES)
    assert profile["z_lo_m"] == pytest.approx(-.227)
    assert profile["z_hi_m"] == pytest.approx(.06042307692307692)
    assert profile["radius_m"] == pytest.approx(.3859447197013662)
    physical = diagnostic_envelope(profile, Pose(0, 0, 0), 0, clearance_m=0, pose_xy_m=0, pose_z_m=0)
    points = [[0, -.132, -.227], [0, .132, -.227], [.174, -.174, .06042307692307692]]
    for rotor in profile["rotors"]:
        hub = np.array(rotor["hub_body"])
        direction = hub[:2]/np.linalg.norm(hub[:2])
        points.append([*(hub[:2]+direction*rotor["sweep_radius_m"]), hub[2]])
    assert physical.contains(points, inflate_m=1e-12).all()
    assert not physical.contains([[0, 0, -.228], [.387, 0, 0]]).any()
    assert not profile["flight_admission"]
    assert profile["visual_meshes_not_bounded"]


def test_new_default_optical_origin_agrees_with_independently_derived_model_mount():
    profile = model_profile(SOURCES)
    camera = CameraModel(Intrinsics.pinned_x500_depth())
    np.testing.assert_allclose(camera.mount.translation_flu, [.13233, 0, .02078], atol=1e-12)
    _, origin = camera.world_from_optical(Pose(1, 2, 3))
    np.testing.assert_allclose(origin, np.array(profile["optical_translation_flu"])+[1, 2, 3], atol=1e-12)


def test_physical_geometry_is_uninflated_and_clearance_is_added_once():
    profile = model_profile(SOURCES)
    env = diagnostic_envelope(profile, Pose(0, 0, 1), 1, clearance_m=.5, pose_xy_m=.1, pose_z_m=.02)
    assert env.radius_m == pytest.approx(profile["radius_m"]+.6)
    assert env.z_lo == pytest.approx(1-.227-.5-.02)
    assert env.z_hi == pytest.approx(1+.06042307692307692+.5+.02)
    assert profile["radius_m"] == pytest.approx(.3859447197013662)
    with pytest.raises(ValueError, match="TILTED"):
        diagnostic_envelope(profile, Pose(0, 0, 1, rotation_enu_flu=[[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
                            1, clearance_m=.5, pose_xy_m=.1, pose_z_m=.02)


def test_unknown_collision_geometry_is_not_silently_omitted(tmp_path):
    shutil.copytree(SOURCES, tmp_path / "models")
    path = tmp_path / "models/x500_base.sdf"
    root = ET.parse(path)
    geometry = root.find("model/link/collision/geometry")
    geometry.remove(geometry.find("box"))
    ET.SubElement(geometry, "mesh")
    root.write(path)
    with pytest.raises(ValueError, match="UNSUPPORTED_COLLISION"):
        model_profile(tmp_path / "models")
