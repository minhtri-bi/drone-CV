# drone-CV

Computer-vision obstacle avoidance for drones, **simulation first**, using PX4 SITL, Gazebo and ROS 2.

## Status

The repository has a **working offline G0 implementation**: metric depth/validity, capture-time projection, frame/quaternion transforms, a conservative 72×3 histogram, braking and command freshness. **The G1 environment passed an unarmed smoke run** with ROS camera/clock/TF and PX4 DDS on a pinned derived image. Pytest and a CI workflow are available; the oracle, supervisor, flight runtime and SITL evaluator are not implemented.

Core target: reach the goal in ≥45/50 frozen navigation worlds, with zero unsafe events and navigation clearance ≥0.5 m under the phase contract.
This target has not been achieved; it is not a benchmark result.

## Reading order

1. [Revised implementation plan](docs/PLAN.md): scope, architecture, contracts and gates G0–G5.
2. [Safety contracts S01–S08](docs/SAFETY.md): phase-aware contact/clearance, unsafe latch, coverage and corrected dependencies.
3. [Design review and official sources](docs/REVIEW.md): risks in the original plan and subsequent contract corrections.
4. [65 scenarios and their oracles](docs/SCENARIOS.md): G0 geometry fixtures exist; flight scenarios have not been run.
5. [Development and evaluation harness](docs/HARNESS.md): agent workflow, runner/evaluator contract and Git gates.
6. [OMP skills and roadmap](docs/OMP.md): 10 trigger-loaded skills, without installing another bundle.
7. [Repository agent instructions](.omp/AGENTS.md): invariants and automation scope.
8. [Offline seen-free spike](spike/freespace/README.md): synthetic perception liveness, near-hazard retention and sampled-coverage limits; not flight acceptance.

[Original PDF, preserved byte for byte](docs/source/drone-obstacle-avoidance-plan.pdf).
The plan records its source, checksum and relationship to the revised documents.

## Roadmap in OMP

The [canonical state](.omp/roadmap/roadmap.json) contains 27 tasks with dependencies and acceptance criteria.
Use `/roadmap status` or `/roadmap view` in OMP; the offline canvas is generated automatically. Do not edit its HTML.
Current status and acceptance criteria live in JSON; the [English portable view](docs/ROADMAP.md) uses the same state with a hash-bound [display translation](docs/roadmap.en.json). Do not edit generated Markdown manually.
This roadmap covers the simulation scope; it is neither an estimate of effort percentages nor flight certification.

## Run G0 now

Python 3.12, NumPy 2.0.0, pytest 8.2.2. From the repository root:

```sh
python -m pip install numpy==2.0.0 pytest==8.2.2
python -m pytest -q
python scripts/check_project.py
python scripts/g0_demo.py --output artifacts/g0-demo
```

Skip installation if the dependencies are already available. ROS, a GPU, matplotlib and a global Python package installation are not required.
The demo refuses to overwrite an existing directory; choose a new output name for each rerun.

Open `artifacts/g0-demo/preview.svg`; `summary.json` contains decisions, and the NPZ files contain the ENU point cloud, depth and histogram.
Three scenes exercise the actual API: a wall at 6 m, a one-pixel-row horizontal wire at 2 m and unknown observations at boot. A stale proposal is rejected separately.
These are **synthetic fixtures**, not flight data or a real-obstacle avoidance benchmark.

### G0 limitations

- Pixel-cone support must be explicitly verified; one ray does not prove that an entire bin is free.
- Swept boxes enclose the circular footprint across the full 5° sector. This conservative method may block a clear path.
- Space near or even behind the camera is not automatically free. The demo supplies a 1 m sphere proven clear by fixture geometry; a future runtime needs independent, fresh evidence that does not contradict depth.
- No temporal median or percentile filtering is used; old history cannot erase new near-obstacle evidence.
- Braking assumes constant acceleration with exploratory parameters. Jerk, PX4 tracking and actual braking have not been measured.
- Changing direction while moving requires stopping first; an origin offset greater than 1 µm or motion along Z blocks the command. Motion compensation, yaw-envelope control and hover guarantees are not implemented.

### Learn alongside the code

1. Read the [NumPy quickstart](https://numpy.org/doc/2.0/user/quickstart.html) and run `tests/test_depth.py`.
2. Read [REP-103](https://www.ros.org/reps/rep-0103.html) and [REP-118](https://www.ros.org/reps/rep-0118.html), then compare them with `tests/test_geometry.py`.
3. Read the braking formula in PLAN, then run `tests/test_safety.py` and the demo; compare the wall/wire/unknown scene outputs.

## G1: environment passed, flight gate not passed

After Docker/image and bridge approval, the [derived image](docker/g1/Dockerfile) was built from a fixed digest. The [dependency lock](docker/g1/packages.lock.json) pins versions and SHA-256 hashes for 13 added packages; the installer does not upgrade base packages and rejects archives that differ from the lock.

- PX4 package `1.18.0~beta1-noble`, firmware `61907962c9220437ec01a98fa26bfbf5cbb254b5`; Gazebo Sim `8.15.0`; ROS 2 Jazzy; bridge `1.0.24-1noble.20260920.234644`.
- Actual X500 simulated depth, ROS `32FC1` at 640×480, 35 Image/CameraInfo pairs with matching stamps in an 8-second window; optical TF matches the SDF.
- Gazebo `/clock` is the only clock: `UXRCE_DDS_SYNCT=0`, observer/bridge use `use_sim_time=true`, and wall time does not replace sensor stamps. Raw PX4 odometry falls within the same sim-time window and increases strictly.
- DDS status remained **Disarmed**; no control publisher was created and no arm command was sent. The container used network none, 4 CPUs/6 GiB, software rendering and no host mounts.
- The camera reached approximately 4.5 Hz in wall time during this window, not a real 30 Hz rate or performance/calibration acceptance.

[New evidence](docs/environments/g1-bridge-observation.json) preserves pins, QoS, encoding, intrinsics, TF, clock and two failed windows before the clock-policy correction. The [base-image probe](docs/environments/g1-observation.json) preserves the earlier missing-bridge history.

`scripts/g1_probe.py --ros` and `scripts/g1_ros_observer.py` are read-only observers inside the container running Agent/PX4/Gazebo/bridge/TF; they exit with code 2 when observations are missing. See the [executed recipe and its limits](docs/HARNESS.md#derived-smoke-recipe-and-observed-results).

Run-owned containers/processes were stopped and removed; Docker Desktop and the images were retained. No host packages, drivers, WSL configuration or permissions were changed.
Only `g1-environment` passed: oracle/logger and supervisor are still required before bootstrap flight; calibration remains pending. This does not certify `+Inf` as free space, capture-time pose interpolation or successful obstacle avoidance.

## Principles

- Unknown, stale and out-of-FOV observations are blocked; NaN does not imply free space.
- Safety is independent of the planner; heartbeat does not replace command freshness.
- The evaluator uses independent ground truth; the planner must not read the oracle.
- No LLM in the control loop and no real flights within the core scope.
- Commit/push/merge require user authority and actual gates; never bypass checks or reviews.
