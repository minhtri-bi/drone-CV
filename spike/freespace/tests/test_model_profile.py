from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from camera import CameraModel, Intrinsics, Pose
from model_profile import diagnostic_envelope, model_profile
from experiments import strategies
from model_liveness import physical_fixture, run
import model_liveness
from evidence import Decision

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


def complete_profile():
    """Typed full-profile fixture; not a replacement for pinned-image evidence."""
    model_from_body = np.eye(4)
    model_from_body[2, 3] = .24
    mount = np.eye(4)
    mount[:3, 3] = [.13233, 0., .02078]
    return {"schema": 1, "frame": "base_link FLU",
            "parts": [
                {"id": "gear", "role": "gear", "source": "typed fixture",
                 "lo": [-.1, -.1, -.2275995], "hi": [.1, .1, -.1]},
                {"id": "body", "role": "body", "source": "typed fixture",
                 "lo": [-.2, -.3, -.1], "hi": [.2, .3, .06938461481722502]},
                {"id": "rotor", "role": "rotor", "source": "typed fixture",
                 "lo": [-.4, -.3, .03], "hi": [.4, .3, .06]}],
            "source_sha256": {"model.sdf": "a"*64},
            "model_from_body": model_from_body.tolist(),
            "optical_mount_body": mount.tolist(),
            "z_lo_m": -.2275995, "z_hi_m": .06938461481722502,
            "flight_admission": False}


def test_complete_profile_derives_support_rest_and_encloses_every_part_corner():
    profile = complete_profile()
    fixture = physical_fixture(profile)
    assert fixture["rest_height_m"] == pytest.approx(.2275995)
    assert fixture["model_rest_height_m"] == pytest.approx(-.0124005)
    assert fixture["radius_m"] == pytest.approx(.5)
    assert fixture["support_part_ids"] == ["gear"]
    assert fixture["source_sha256"] == profile["source_sha256"]
    assert len(fixture["profile_canonical_sha256"]) == 64
    for name, plan in strategies(rest_height_m=fixture["rest_height_m"]).items():
        assert min(pose.z for pose in plan) >= fixture["rest_height_m"]
        if not name.startswith("level_"):
            assert plan[0].z == pytest.approx(fixture["rest_height_m"])


def test_fixture_rejects_rest_below_complete_support_geometry():
    with pytest.raises(ValueError, match="REST_HEIGHT_BELOW_SUPPORT"):
        physical_fixture(complete_profile(), rest_height_m=.15)


def test_fixture_rejects_collision_only_and_non_gear_floor_support():
    with pytest.raises(ValueError, match="COMPLETE_ORACLE_PROFILE_REQUIRED"):
        physical_fixture(model_profile(SOURCES))
    profile = complete_profile()
    profile["parts"][0]["role"] = "body"
    with pytest.raises(ValueError, match="LOWEST_PROFILE_PART_NOT_GEAR"):
        physical_fixture(profile)


def test_fixture_rejects_stale_bounds_and_profile_binding_changes_with_geometry():
    profile = complete_profile()
    before = physical_fixture(profile)["profile_canonical_sha256"]
    profile["parts"][2]["hi"][0] += .01
    assert physical_fixture(profile)["profile_canonical_sha256"] != before
    profile["z_lo_m"] = -.227
    with pytest.raises(ValueError, match="PROFILE_VERTICAL_BOUNDS_MISMATCH"):
        physical_fixture(profile)


def test_liveness_rejects_old_odometry_span_as_truth_error():
    observed = {"diagnostics": {"odometry": {"position_ned_peak_to_peak_m": [.1, .1, .1]}}}
    with pytest.raises(ValueError, match="STATIONARY_TRUTH_DIAGNOSTIC_REQUIRED"):
        run(complete_profile(), observed)


@pytest.mark.parametrize("height", [.0, -.1, np.inf, np.nan, 1.3])
def test_strategies_require_explicit_finite_rest_below_target(height):
    with pytest.raises(ValueError, match="INVALID_REST_HEIGHT"):
        strategies(rest_height_m=height)


def truth_diagnostic():
    return {"scope": "STATIONARY_TRUTH_DIAGNOSTIC_NOT_CALIBRATION", "motion_authority": False,
            "relative_error": {"pair_count": 3, "maximum_translation_m": .1,
                               "maximum_rotation_rad": .0, "maximum_bound_m": .12, "rho_m": 15.},
            "source": {"recording_manifest_sha256": "b"*64}, "coverage": {"pairs_evaluated": 3},
            "sensing_caps": {"near_trusted_m": .3, "far_trusted_m": 10.}}


def test_liveness_binds_rest_and_observed_scalar_inflation_without_running_fixture(monkeypatch):
    """Plumbing proof only; the frozen real synthetic workloads remain separate."""
    captures = []

    def record_capture(scene, camera, sensor, poses):
        captures.append(poses)
        return None, 0.

    def no_motion(store, envelope, **kwargs):
        return Decision(False, "NO_FRESH_FRAMES"), None, None

    monkeypatch.setattr(model_liveness, "capture", record_capture)
    monkeypatch.setattr(model_liveness, "decide", no_motion)
    observed = truth_diagnostic()
    result = run(complete_profile(), observed)
    assert captures
    assert min(pose.z for poses in captures for pose in poses) == pytest.approx(.2275995)
    active = [row for row in result["rows"] if row["profile"] == "model_navigation_diagnostic"]
    assert active
    for row in active:
        assert row["radius_m"] == pytest.approx(.5+.5+.12)
        assert row["z_lo_m"] == pytest.approx(1.2-.2275995-.5-.12)
        assert row["z_hi_m"] == pytest.approx(1.2+.06938461481722502+.5+.12)
    assert result["uncertainty_status"]["source"] == observed["source"]
    assert result["uncertainty_status"]["calibrated_pose_bound_m"] is None
    assert result["sensing_caps"] == {**observed["sensing_caps"], "rho_m": 15.}
    assert result["motion_authority"] is False
    diagnostic = result["profiles"]["model_navigation_diagnostic"]["sensor_profile"]
    assert diagnostic["source"] == "stationary_measured_diagnostic"
    assert diagnostic["near_translation_uncertainty_m"] == .1
    assert diagnostic["near_rotation_uncertainty_rad"] == 0.
    assert diagnostic["provenance"] and not diagnostic["no_return_certified"]
    historical = result["profiles"]["historical_synthetic"]["sensor_profile"]
    assert historical["source"] == "analytic_synthetic"
    assert historical["near_translation_uncertainty_m"] == 0.


def test_liveness_rejects_diagnostic_sensor_cap_mismatch_before_capture():
    observed = truth_diagnostic()
    observed["sensing_caps"]["far_trusted_m"] = 19.1
    with pytest.raises(ValueError, match="SENSOR_CAP_MISMATCH"):
        run(complete_profile(), observed)


def test_full_model_replay_envelope_uses_tilted_physical_corners():
    from replay_semantics import full_profile_envelope

    pose = Pose(0., 0., 1.2, rotation_enu_flu=((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)))
    envelope = full_profile_envelope(complete_profile(), pose, .12)
    assert envelope.z_lo == pytest.approx(.28)
    assert envelope.z_hi == pytest.approx(2.12)
    assert envelope.radius_m == pytest.approx(np.hypot(.4, .06)+.62)
    assert envelope.end_xy == pytest.approx((1., 0.))


def test_frozen_sweep_retains_skipped_geometry_and_unchanged_seed(monkeypatch):
    import experiments
    from scene import Box, Scene

    # An independently intersecting approach must not disappear from the ledger.
    obstruction = Scene(boxes=(Box((.5, -.1, 1.), (.6, .1, 1.4)),))
    monkeypatch.setattr(experiments, "hard_scene", lambda rng, envelope: obstruction)
    report = experiments.safety_sweep(2, 11)
    assert report["seed"] == 11 and report["requested_trials"] == 2
    assert report["all_cases_retained"]
    assert report["counts"]["skipped_unsafe_approach"] == 2
    assert [row["classification"] for row in report["cases"]] == ["SKIPPED_UNSAFE_APPROACH"]*2
    assert all(row["scene"]["boxes"][0]["lo"] == (.5, -.1, 1.) for row in report["cases"])
