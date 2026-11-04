# drone-CV

Computer-vision obstacle avoidance for drones, **simulation first**, using PX4 SITL, Gazebo and ROS 2.

## Status

The repository has a **working offline G0 implementation**: metric depth/validity, capture-time projection, frame/quaternion transforms, a conservative 72×3 histogram, braking and command freshness. **The G1 environment passed an unarmed smoke run** with ROS camera/clock/TF and PX4 DDS on a pinned derived image. The independent static oracle/logger is implemented and exercised. Supervisor code remains offline-only; no flight, calibration or navigation gate has passed.

The [unarmed sensor replay](docs/SENSOR_REPLAY_RESULTS.md) also passed stationary integration: 152 real Gazebo frames paired at capture time, independent target checks and six injected fault variants. Its movement query remains STOP; there is no real-data navigation result.

The [integrity repair](docs/SENSOR_REPLAY_INTEGRITY.md) adds a fresh 136-frame replay with snapshot-time/reset-bound alignment, receive-wall health evidence and a real camera-stream loss refusal. Historical unbound bags remain diagnostic-only under the new profile.

The [model-profile and rolling-TTL preparation](docs/PREPARATION_RESULTS.md) also ran: pinned collision/rotor bounds, 96 explicit synthetic coverage cases, and a 90 s Disarmed recording spanning 19.328 s sim. Old evidence expires while fresh frames remain. These diagnostics do not authorize movement or complete G1/G2.

The [static oracle](docs/G1_ORACLE_RESULTS.md) resolves complete visual/collision/rotor geometry and records lossless evaluator-only truth in the sensor bag. Frozen body/rotor intersections and truth loss exercise unsafe-latch and incomplete-evidence classification. These are sampled geometric observations, not armed flight or continuous swept-volume proof.

The [evidence-semantics results](docs/EVIDENCE_SEMANTICS_RESULTS.md) add localized finite near-hazard cones, 315,712 truth-relative capture/query pairs over a 10 s TTL and independent dropout/expiry identities. Original geometry grading failures from the added occluding side target remain reported, not suppressed. The [supervisor results](docs/G1_SUPERVISOR_RESULTS.md) exercise actual pinned-message serialization under guards forbidding ROS initialization/nodes/control publishers; its runtime gate remains incomplete.

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
9. [Unarmed sensor replay results](docs/SENSOR_REPLAY_RESULTS.md): immutable bags, capture-time adapter, independent geometry, fault outcomes and replay commands.

[Original PDF, preserved byte for byte](docs/source/drone-obstacle-avoidance-plan.pdf).
The plan records its source, checksum and relationship to the revised documents.

## Roadmap in OMP

The [canonical state](.omp/roadmap/roadmap.json) contains 32 tasks with dependencies and acceptance criteria.
Use `/roadmap status` or `/roadmap view` in OMP; the offline canvas is generated automatically. Do not edit its HTML.
Current status and acceptance criteria live in JSON; the [English portable view](docs/ROADMAP.md) uses the same state with a hash-bound [display translation](docs/roadmap.en.json). Do not edit generated Markdown manually.
This roadmap covers the simulation scope; it is neither an estimate of effort percentages nor flight certification.

The [unarmed sensor-recording/replay plan](docs/SENSOR_REPLAY_PLAN.md) is an implemented preparation branch after the environment and G0 demo, not permission to start flight or claim G2 acceptance.
The approved [review repair plan](docs/SENSOR_REPLAY_REPAIR_PLAN.md) adds integrity, model-profile and rolling-TTL preparation branches without changing existing flight dependencies.

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
- Historical environment check: X500 simulated depth, ROS `32FC1` at 640×480, and 35 Image/CameraInfo pairs with matching stamps in an 8-second window. Its TF used the outer-model origin; the [unarmed replay](docs/SENSOR_REPLAY_RESULTS.md#failures-preserved-not-relabeled) subsequently corrected the body-relative z offset by 0.24 m. The old observation remains preserved, not a body-frame calibration certificate.
- Gazebo `/clock` is the only clock: `UXRCE_DDS_SYNCT=0`, observer/bridge use `use_sim_time=true`, and wall time does not replace sensor stamps. Raw PX4 odometry falls within the same sim-time window and increases strictly.
- DDS status remained **Disarmed**; no control publisher was created and no arm command was sent. The container used network none, 4 CPUs/6 GiB, software rendering and no host mounts.
- The camera reached approximately 4.5 Hz in wall time during this window, not a real 30 Hz rate or performance/calibration acceptance.

[New evidence](docs/environments/g1-bridge-observation.json) preserves pins, QoS, encoding, intrinsics, TF, clock and two failed windows before the clock-policy correction. The [base-image probe](docs/environments/g1-observation.json) preserves the earlier missing-bridge history.

`scripts/g1_probe.py --ros` and `scripts/g1_ros_observer.py` are read-only observers inside the container running Agent/PX4/Gazebo/bridge/TF; they exit with code 2 when observations are missing. See the [executed recipe and its limits](docs/HARNESS.md#derived-smoke-recipe-and-observed-results).

Run-owned containers/processes were stopped and removed; Docker Desktop and the images were retained. No host packages, drivers, WSL configuration or permissions were changed.
The G1 flight gate remains pending: the static oracle/logger is complete, but supervisor runtime verification is still required before bootstrap flight. Calibration remains pending. Unarmed replay validates capture-time interpolation and stationary geometry; no experiment certifies `+Inf` as free space or successful obstacle avoidance.

## Principles

- Unknown, stale and out-of-FOV observations are blocked; NaN does not imply free space.
- Safety is independent of the planner; heartbeat does not replace command freshness.
- The evaluator uses independent ground truth; the planner must not read the oracle.
- No LLM in the control loop and no real flights within the core scope.
- Commit/push/merge require user authority and actual gates; never bypass checks or reviews.
