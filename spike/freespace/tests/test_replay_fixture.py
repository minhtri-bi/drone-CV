import json
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np

from camera import CameraModel, Intrinsics, Mount, Pose
from replay_bag import collect, fault_passes, replay_exit_code
from replay_fixture import build_world, grade_frame

FIXTURE = json.loads((Path(__file__).parents[1] / "replay_fixture.json").read_text())


def analytic_setup():
    camera = CameraModel(Intrinsics(32, 24, 30, 30, 16, 12), Mount((0, 0, 0)))
    fixture = {**FIXTURE, "edge_pixels": 1, "targets": [
        {"name": "plane", "axis": 0, "coordinate": 3.,
         "lo": [3, -.8, -.7], "hi": [3, .8, .7]}]}
    # Looking along world +x, a fronto-parallel plane has optical Z=3 everywhere.
    return camera, fixture, np.full((24, 32), 3., dtype=np.float32)


def test_independent_plane_accepts_geometry_and_rejects_wrong_alignment():
    camera, fixture, depth = analytic_setup()
    result = grade_frame(depth, camera, Pose(0, 0, 0), np.eye(4), fixture)
    assert result["all_pass"] and result["targets"]["plane"]["maximum_m"] == 0
    wrong = np.eye(4)
    wrong[0, 3] = .5
    bad = grade_frame(depth, camera, Pose(0, 0, 0), wrong, fixture)
    assert not bad["all_pass"] and bad["targets"]["plane"]["maximum_m"] >= .5


def test_missing_support_and_single_large_outlier_do_not_hide_in_average():
    camera, fixture, depth = analytic_setup()
    holes = depth.copy()
    holes[:, 12:20] = np.nan
    missing = grade_frame(holes, camera, Pose(0, 0, 0), np.eye(4), fixture)
    assert not missing["all_pass"] and missing["targets"]["plane"]["support"] < .95
    outlier = depth.copy()
    outlier[12, 16] = 4.
    result = grade_frame(outlier, camera, Pose(0, 0, 0), np.eye(4), fixture)
    assert not result["all_pass"] and result["targets"]["plane"]["maximum_m"] == 1.


def test_ground_roi_is_not_mistaken_for_the_physical_floor_edge():
    camera = CameraModel(Intrinsics(320, 240, 200, 200, 160, 120), Mount((0, 0, 0)))
    v, _ = np.mgrid[:240, :320]
    depth = np.full((240, 320), np.nan)
    np.divide(.25 * 200, v-120, out=depth, where=v > 120)
    ground = {"name": "ground", "axis": 2, "coordinate": 0.,
              "lo": [2., -.4, 0.], "hi": [3., .4, 0.],
              "surface_lo": [-250., -250., 0.], "surface_hi": [250., 250., 0.]}
    fixture = {**FIXTURE, "edge_pixels": 1, "targets": [ground]}
    # A 5 cm height error moves a grazing ray outside the ROI, not off the physical floor.
    good = grade_frame(depth, camera, Pose(0, 0, .30), np.eye(4), fixture)
    assert good["all_pass"]
    assert np.isclose(good["targets"]["ground"]["maximum_m"], .05)
    wrong_height = grade_frame(depth, camera, Pose(0, 0, .50), np.eye(4), fixture)
    assert not wrong_height["all_pass"]


def test_fixture_generation_preserves_original_physics_and_ground():
    base = '<sdf version="1.9"><world name="default"><physics><max_step_size>0.004</max_step_size></physics><model name="ground_plane"/></world></sdf>'
    root = ET.fromstring(build_world(base, FIXTURE))
    assert root.findtext("world/physics/max_step_size") == "0.004"
    assert root.find("world/model[@name='ground_plane']") is not None
    assert root.find("world/model[@name='front_target']/static").text == "true"


def odometry(t, reset=0):
    return NS(timestamp_sample=t, reset_counter=reset, pose_frame=1,
              position=[0, 0, 0], q=[1, 0, 0, 0])


def test_reset_counter_splits_pose_history_instead_of_cross_epoch_interpolation():
    records = [("/fmu/out/vehicle_odometry", odometry(10), 0),
               ("/clock", NS(clock=NS(sec=0, nanosec=10_000)), 0),
               ("/fmu/out/vehicle_odometry", odometry(20, 1), 1),
               ("/clock", NS(clock=NS(sec=0, nanosec=20_000)), 1)]
    epochs, _, rejected = collect(records, FIXTURE)
    assert [s.reset_counter for s in epochs[0]["odom"]] == [0]
    assert [s.reset_counter for s in epochs[1]["odom"]] == [1]
    assert not rejected


def test_backward_clock_and_duplicate_odometry_do_not_extend_old_epoch():
    records = [("/clock", NS(clock=NS(sec=1, nanosec=0)), 0),
               ("/fmu/out/vehicle_odometry", odometry(10), 0),
               ("/fmu/out/vehicle_odometry", odometry(10), 1),
               ("/clock", NS(clock=NS(sec=0, nanosec=0)), 2),
               ("/fmu/out/vehicle_odometry", odometry(1), 3)]
    epochs, _, rejected = collect(records, FIXTURE)
    assert rejected["NONMONOTONIC_ODOMETRY"] == 1
    assert epochs[2]["odom"][0].timestamp_ns == 1000
    assert len(epochs[0]["odom"]) == 1


def test_unsupported_pose_breaks_epoch_instead_of_bridging_around_it():
    invalid = odometry(20)
    invalid.pose_frame = 0
    records = [("/fmu/out/vehicle_odometry", odometry(10), 0),
               ("/fmu/out/vehicle_odometry", invalid, 1),
               ("/fmu/out/vehicle_odometry", odometry(30), 2)]
    epochs, _, rejected = collect(records, FIXTURE)
    assert sum(rejected.values()) == 1
    assert [sample.timestamp_ns for sample in epochs[0]["odom"]] == [10_000]
    assert [sample.timestamp_ns for sample in epochs[1]["odom"]] == [30_000]


def test_fault_success_requires_rejection_and_never_allows_a_query():
    empty = {"accepted_frames": 0, "gates": [], "rejected": {}, "checks": {}}
    assert not fault_passes("missing_tf", empty)
    rejected = {**empty, "rejected": {"OPTICAL_TF_MISSING": 3}}
    assert fault_passes("missing_tf", rejected)
    assert not fault_passes("missing_tf", {**rejected, "gates": [{"allowed": True}]})


def test_reset_counter_returning_to_original_value_cannot_restore_alignment():
    records = [("/fmu/out/vehicle_odometry", odometry(10, 7), 0),
               ("/fmu/out/vehicle_odometry", odometry(20, 8), 1),
               ("/fmu/out/vehicle_odometry", odometry(30, 7), 2)]
    epochs, _, _ = collect(records, FIXTURE)
    assert not epochs[0]["alignment_invalid"]
    assert epochs[1]["alignment_invalid"]
    assert epochs[2]["alignment_invalid"]


def test_legacy_analysis_exempts_only_unavailable_provenance_not_failed_geometry():
    report = {"checks": {"verified_alignment_provenance": False, "all_target_frames_pass": True,
                         "fault_variants_pass": True}}
    assert replay_exit_code(report, True) == 0
    assert replay_exit_code(report, False) == 2
    assert replay_exit_code({"checks": {**report["checks"], "all_target_frames_pass": False}}, True) == 2
    assert replay_exit_code({"checks": {**report["checks"], "fault_variants_pass": False}}, True) == 2
