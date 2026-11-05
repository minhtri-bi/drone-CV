# drone-CV

Computer-vision obstacle avoidance for drones, **simulation first**, using PX4 SITL, Gazebo and ROS 2.

## Milestones

**Current milestone: M0 — completed; M1 execution awaits approval.**

This table is the v2 status source until native roadmap migration. Historical G0–G5 tasks and prep-* gates do not select or block v2 work.

| Milestone | Demo and acceptance | Status |
| --- | --- | --- |
| M0 | Pinned Gazebo/PX4/ROS bridge runs; depth and odometry rates are observed; a script opens a depth image. | Complete: 2026-11-05 unarmed smoke; depth 4.526 Hz, odometry 19.473 Hz (wall-time receive rates); 640×480 depth opened |
| M1 | A new minimal Offboard node takes off to 1.2 m, flies A-to-B in an empty world and lands; evaluator records goal, time and collisions; 5/5 successful trials. | Pending; M0 complete, M1 execution not authorized |
| M2 | Stop before a wall using forward-corridor depth and stopping distance; 10/10 trials without contact, with minimum clearance recorded. | Pending M1 |
| M3 | Simple seen-free detours reach the goal across 20 fixed-seed worlds with zero collisions. | Pending M2 |
| M4 | Add depth noise, dropout and delay with evidence TTL; report success and collision rates against M3. | Pending M3 |
| M5 | Optional stereo or monocular image depth; compare results against M4. | Optional; pending M4 |

### M0 run: 2026-11-05

Used the existing `docker/g1` image `sha256:061a39905d2d84d40859bb46b67048241b6dacf2ed3983889da7f5fd3e718285`, without builds, pulls or installs. The run-owned container was `drone-cv-m0-20261105-01`, with network disabled, 4 CPUs, 6 GiB RAM and software rendering.

Two concurrent `ros2 topic hz --wall-time --window 10000` observers ran for 25 wall seconds. Their final printed cumulative estimates were:

| Topic | Receive rate | CLI window | Observed receive-interval min/max |
| --- | --- | --- | --- |
| `/drone_cv_g1/camera/depth/image_raw` | 4.526 Hz | 100 messages | 0.107 / 0.768 s |
| `/drone_cv_g1/fmu/out/vehicle_odometry` | 19.473 Hz | 434 messages | 0.003 / 0.269 s |

These are wall-time subscriber measurements, not simulation-time rates or calibration. Discovery and the CLI's periodic printing mean the final windows do not cover every instant of the 25-second observer lifetime.

The new [single-frame viewer](scripts/m0_depth_view.py) subscribes inside the existing ROS container, prints valid positive finite optical-Z statistics, copies a PNG preview to Windows and opens it in the default image viewer. NumPy/Pillow/ROS were already available in the image; host mode uses only Python's standard library and Docker.

```sh
python scripts/m0_depth_view.py --container drone-cv-m0-20261105-01 --output artifacts/m0-20261105-01/depth-final.png
```

Observed final-script output:

```text
size=640x480 encoding=32FC1 min_m=0.44744744896888733 max_m=16.487102508544922 invalid=157574 total=307200
```

The PNG was visibly opened in Windows Photos. Invalid pixels (nonfinite or nonpositive) are magenta; positive finite depths are grayscale scaled for display, not a metric PNG. Two invocations, before and after a type-narrowing correction, returned identical statistics. Local previews and raw rate output remain under `artifacts/m0-20261105-01/`, outside tracked source.

PX4 reported **Disarmed** before and after observation. Final publisher counts for `vehicle_command`, `offboard_control_mode` and `trajectory_setpoint` under `/drone_cv_g1/fmu/in/` were all zero. No arm commands, control publishers or manual parameter-write commands were issued. The unchanged image's PX4 startup itself applied its baked-in defaults, including `UXRCE_DDS_SYNCT=0` and DDS domain 83; this is not a claim that boot performs zero internal parameter assignments.

The owned container and its processes were stopped and removed after observation. The image ID was unchanged. No M1 flight was attempted.

M1 will use a **new minimal Offboard node**, not the v1 supervisor/bridge admission chain. V1 code is reference material, not a dependency on Home, variance, RTF, permits or prep-* gates. This M0 slice adds only read-only observation; it authorizes no flight/control implementation or execution.

## Backlog

- Archive v1/PR #12 and simplify layout in a separately scoped change; preserve history and source PDFs.
- Migrate the native roadmap to M0–M5 through supported tooling; until then, use the table above.
- After separate M1 approval, implement the node and evaluator, then run five explicitly authorized simulation trials.

## Historical v1 status

The repository has a **working offline G0 implementation**: metric depth/validity, capture-time projection, frame/quaternion transforms, a conservative 72×3 histogram, braking and command freshness. **The G1 environment passed an unarmed smoke run** with ROS camera/clock/TF and PX4 DDS on a pinned derived image. Pytest and a CI workflow are available; the oracle, supervisor, flight runtime and SITL evaluator are not implemented.

The [unarmed sensor replay](docs/SENSOR_REPLAY_RESULTS.md) also passed stationary integration: 152 real Gazebo frames paired at capture time, independent target checks and six injected fault variants. Its movement query remains STOP; there is no real-data navigation result.

The [integrity repair](docs/SENSOR_REPLAY_INTEGRITY.md) adds a fresh 136-frame replay with snapshot-time/reset-bound alignment, receive-wall health evidence and a real camera-stream loss refusal. Historical unbound bags remain diagnostic-only under the new profile.

The [model-profile and rolling-TTL preparation](docs/PREPARATION_RESULTS.md) also ran: pinned collision/rotor bounds, 96 explicit synthetic coverage cases, and a 90 s Disarmed recording spanning 19.328 s sim. Old evidence expires while fresh frames remain. These diagnostics do not authorize movement or complete G1/G2.

Historical v1 target: reach the goal in ≥45/50 frozen navigation worlds, with zero unsafe events and navigation clearance ≥0.5 m under the phase contract.
This target was not achieved; it is not a benchmark result or an additional v2 milestone gate.

## Current harness work

Project instructions now live in [AGENTS.md](AGENTS.md): one demo at a time, inline bounded changes, failure-driven guards and review triage before fixes. [OMP guidance](docs/OMP.md) is a short compatibility guide, not a mandatory skill pipeline.

The harness change preserved v1 code, evidence and native roadmap. M0 has now passed the read-only smoke above; archive/reset, native roadmap migration and M1 flight remain separate work.

## Historical v1 references

For new work, start with [project instructions](AGENTS.md) and the milestone table above. The documents below explain retained v1 code and evidence; they are not a mandatory reading sequence or v2 gate list.

1. [Revised implementation plan](docs/PLAN.md): scope, architecture, contracts and gates G0–G5.
2. [Safety contracts S01–S08](docs/SAFETY.md): phase-aware contact/clearance, unsafe latch, coverage and corrected dependencies.
3. [Design review and official sources](docs/REVIEW.md): risks in the original plan and subsequent contract corrections.
4. [65 scenarios and their oracles](docs/SCENARIOS.md): G0 geometry fixtures exist; flight scenarios have not been run.
5. [Development and evaluation harness](docs/HARNESS.md): agent workflow, runner/evaluator contract and Git gates.
6. [OMP harness compatibility](docs/OMP.md): short workflow, existing commands and unchanged v1 roadmap.
7. [Repository agent instructions](AGENTS.md): canonical project policy and operational scope.
8. [Offline seen-free spike](spike/freespace/README.md): synthetic perception liveness, near-hazard retention and sampled-coverage limits; not flight acceptance.
9. [Unarmed sensor replay results](docs/SENSOR_REPLAY_RESULTS.md): immutable bags, capture-time adapter, independent geometry, fault outcomes and replay commands.

[Original PDF, preserved byte for byte](docs/source/drone-obstacle-avoidance-plan.pdf).
The plan records its source, checksum and relationship to the revised documents.

## Historical native roadmap

The retained [native state](.omp/roadmap/roadmap.json) contains 31 v1 tasks. It is not the status source for M0–M5 until migration.
If `/roadmap status` or a startup hook shows g1-oracle, g1-supervisor or prep-* work, treat it as historical; do not select that work for the v2 milestone.
The [portable view](docs/ROADMAP.md) and [display translation](docs/roadmap.en.json) remain unchanged for v1 integrity. Do not hand-edit native state or generated views.
Use the milestone table above for current work; existing v1 completion records neither accept nor block M0–M5.

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
Under the historical v1 plan, the G1 flight gate remained pending on oracle/logger, supervisor and calibration. These are not prerequisites for the new minimal M1 path. The unarmed replay established stationary geometry only, not successful obstacle avoidance.

## Principles

- Unknown, stale and out-of-FOV observations are blocked; NaN does not imply free space.
- Keep watchdog checks independent of planner decisions; one M1 node may own both checks and setpoint publishing. Heartbeat does not replace command freshness.
- The evaluator uses independent ground truth; the planner must not read the oracle.
- No LLM in the control loop and no real flights within the core scope.
- Commit/push/merge require user authority and actual gates; never bypass checks or reviews.
