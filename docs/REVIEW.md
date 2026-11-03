# Plan review and source verification

Cross-check date: 2026-11-03. Scope: reading the complete 15-page PDF, including the architecture/pipeline/roadmap diagrams, and cross-checking official documentation. This is a design analysis; the flight stack has not been tested experimentally.

## Conclusion

Retain simulation-first, depth-first, no training, a separate supervisor, and seed-based evaluation. Correct sensor semantics, geometry/filtering, recovery, Offboard liveness, and grading before writing flight nodes.

The risks below are **technical analysis**, not bugs reproduced in this repo's runtime. The repo has no runtime yet.

The next preparation round found 8 additional gaps in the revised contract itself; the design was corrected in [SAFETY.md](SAFETY.md), without falsely calling that a runtime fix:

- Whole-flight contact/clearance contradicts landing; phase contracts and a pair allowlist are needed.
- The oracle/supervisor arrived too late relative to G1/G2; dependencies were changed to provide them before flight.
- Infrastructure errors could overwrite collisions; an independent safety latch is needed alongside execution outcome.
- Unavoidable stress was mixed with zero-collision navigation; frozen suite membership is needed.
- Clocks/deadlines and delays may span different domains or be counted twice; trace endpoints and calibration scope are needed.
- One free ray could clear an entire bin; swept-corridor coverage is needed.
- Valid negative position/velocity values risk rejection as invalid distances.
- An A* grid route does not prove continuous clearance or provide a dynamics-feasible time reference.

The corresponding runtime tasks are in the native roadmap and remain pending.

## Issues to address first

| Severity | Issue in the PDF | Why it can fail | Decision | Scenario |
| --- | --- | --- | --- | --- |
| P0 | Treat all invalid readings alike; assume 10 m is clear when no near obstacle has been seen | NaN/dropouts provide no free-space evidence; there is no history at boot | Adapter-verified validity; unknown blocks, and only explicitly defined `NO_RETURN` permits free space up to its range | A01, A02, E01 |
| P0 | 5% percentile per column and 3-frame median | Horizontal wires/small obstacles occupying <5% of a column can disappear; two old frames can erase a new obstacle | Near-evidence fast path and few-pixel obstacle tests; temporal filtering must compensate for pose and not delay braking | A04, A09, E02–E03 |
| P0 | Clamping the velocity proposal is enough for safety | A side-slipping/braking drone has momentum; lowering a setpoint does not stop it instantly | Check the swept stopping envelope using measured velocity, latency, and actual braking | C07, E14–E15 |
| P0 | Forward camera, RECOVER turns 360° then reverses | Rear regions can remain unknown/stale; the rotating footprint can also hit a wall | Stop, check the rotation envelope, turn to face the travel direction, obtain a fresh frame, then move | A08, B01, E16–E17 |
| P0 | Bridge continues heartbeat even when the proposal is stalled | A live ROS 2 `OffboardControlMode` does not prove that `TrajectorySetpoint` is fresh | Separate freshness/sequence/epoch for proposal, safety result, and observation; do not replay expired commands | C01, E12 |
| P0 | “Every fault → ABORT, hover then land/stop sending” | Hover requires pose; landing requires a safe region below; stopping the stream merely triggers the configured failsafe | Classify faults, bound hold, confirm handoff, check mode/ack; do not re-arm/reclaim authority | C02–C04, E21–E23 |
| P1 | Only NED/ENU mentioned; optical/Z and pose synchronization unclear | Optical frame differs from body FLU; Z-depth differs from range; pose timestamp mismatch creates false/missing obstacles | Contract for intrinsics/extrinsics, clock epoch, and capture-time pose; convert frames at boundaries | A03, A07, C08, E04–E08 |
| P1 | `atan(r/d)` and fixed ±0.5 m filtering | Angle may underinflate; margin omits propellers/pose/tilt; obstacles outside the band can still hit the body | Inflate the Cartesian footprint/swept volume; derive height bands from the envelope rather than hard-coding them | A05, B05, E13–E14 |
| P1 | 90%/50 worlds, excluding infrastructure crashes from the rate | 45/50 has substantial uncertainty; arbitrary crash exclusion beautifies results | Frozen seed set, transparent denominator, statistical CI, and a separate collision gate | D01–D03, F01–F05 |
| P1 | All timeouts use sim time, with only one wall watchdog | Sim resets/pauses can make data age negative; slow machines hide inference latency | Sim time for mission/physics, monotonic wall time for compute/liveness, epoch for resets | A07, D03–D04, E08–E09 |
| P1 | DA2 Small and DA3 both described as generally metric/Apache | Standard DA2 Small is relative; DA3 licenses differ by checkpoint; metric DA3 requires focal scaling | Choose the exact checkpoint, pin weights/license/preprocessing; verify scale/focal against ground truth offline | E18–E19 |
| P1 | Correct stack names taken as proof of compatibility | Firmware/messages/Agent/QoS/namespace can mismatch despite “latest” versions | Lock versions after a smoke test; match message definitions and check actual topics/QoS | E10–E11, F09 |

P0 blocks every navigation experiment. P1 blocks claims of benchmark/G3 acceptance. Collision/clearance evaluation is meaningful only when the evaluator actually observes ground truth.

### Easily overlooked geometry details

For a point obstacle at range `d` and a circular footprint of radius `R < d`, the ideal tangent cone has half-angle `asin(R/d)`, not `atan(R/d)`. The angles are close at long range but diverge near obstacles. Do not blindly replace the formula: if `d` is optical Z, distance to an obstacle surface, or already inflated, that model no longer has the same meaning. Use Cartesian Minkowski/swept-footprint checks to avoid ambiguity; `d <= R` must block immediately.

A vertical pole may survive a per-column percentile; a **horizontal wire** occupying a few image rows may not. “Uses a per-column percentile” is insufficient to conclude that all thin obstacles are handled. Obstacles smaller than the angular/pixel resolution at long range may not be observed by the sensor; the harness must declare its limits, not assign itself a 100% detection rate.

A pixel median does not track the same spatial point as the drone rotates/translates. Each observation needs a pose/timestamp, or history must not be used for the immediate braking path.

## Cross-checked PDF claims

### 1. Stack and simulator

- **Correct according to current documentation:** PX4 recommends ROS 2 Jazzy + Ubuntu 24.04. The Gazebo model `gz_x500_depth` has a forward-facing simulated OAK-D camera. [S1, S2]
- **Correct, but requires pinning:** the Docker image named in the PDF exists; the provider describes it as packaging PX4 and matching messages. There is no need to reject the image because its name is unfamiliar. It has not yet been pulled or run in the repo. [S3]
- The candidate digest in PLAN comes from the Docker Hub tag API, not an invented hash. The actual firmware/package versions and depth assets still need checking in G1. [S3]
- DDS firmware and `px4_msgs` need matching definitions. PX4 has a translation mechanism for different versions from v1.16 onward, but keeping them matched is the least complex choice here. [S1]

### 2. Offboard and Collision Prevention

- **Correct:** Collision Prevention targets acceleration-based Position mode; it is not an automatic protection layer for Offboard. [S4]
- **Needs more precise wording:** ROS 2 uses `OffboardControlMode` as proof-of-life; simply observing 10 Hz setpoints is not enough. PX4 v1.16 requires streaming for over 1 s before entry and a rate >2 Hz. Choose a 20 Hz target for margin, then measure jitter. [S5]
- `OffboardControlMode.velocity=true` does not automatically disable populated position setpoints. Under the PX4 contract, velocity control requires position fields to be NaN; this is **protocol-correct** NaN, unlike NaN in input depth/velocity. [S5]
- When PX4 changes mode on its own, do not replay setpoints to seize authority. The current `main` documentation also warns about conflicts between external/internal publishers; always check the pinned firmware's contract rather than treating `main` as a fixed release. [S6]
- `ObstacleDistance` v1.16 has 72 elements, in cm: `0` means too close, `max_distance+1` means no obstacle, and `65535` means unknown. It does not contain 3 height bands/data-age metadata as required by the internal contract. [S7]
- `/fmu/in/obstacle_distance` is present in v1.16.0 DDS topics; do not assume a custom build is needed merely to send this topic. Recheck the exact image before drawing a conclusion. [S8]

### 3. Sensors, clocks, and middleware

- REP-118 defines float depth as optical Z in meters; the common `16UC1` format uses mm with 0 invalid. Do not apply the same scale to every encoding. [S9]
- REP-117 defines `-Inf` as too close, `NaN` as an error, and `+Inf` as out of range. This is a reference standard, **not proof that a particular driver/model complies**; the adapter needs actual testing. [S10]
- Optical axes, FLU/ENU, and NED differ under REP-103. Quaternion field order also differs between interfaces; check the actual messages. [S11]
- A ROS 2 best-effort publisher cannot connect to a reliable subscriber. The sensor profile prioritizes fresh data and small queues; health checks must detect incompatible QoS, not just dead processes. [S12]

### 4. Pretrained depth and licensing

- **Standard DA2 Small 24.8M is relative depth.** The specific indoor metric version is `depth-anything/Depth-Anything-V2-Metric-Hypersim-Small`; its model card lists Apache-2.0. Use the correct architecture/weights/preprocessing, not a relative model compared against meter thresholds. [S13, S14]
- DA3 exists, with `DA3METRIC-LARGE` at approximately 0.35B parameters. The README requires `metric_depth = focal * net_output / 300.` for this metric model; do not assume raw network output is already in meters. Camera resizing/cropping that changes focal length requires updating it. [S15]
- **Correct the statement “DA3 is Apache 2.0”**: the model table includes both CC BY-NC 4.0 and Apache 2.0. DA3Metric-Large is Apache 2.0 according to the table, but this must not be generalized to every model/nested weight set. [S15]
- Do not choose DA3 as the initial default: the project has no latency/hardware benchmark yet, and a larger model is not automatically suitable for real-time control.
- Do not copy GPL planner/VIO code or redistribute weights before checking the license of the exact revision and intended use. Linking research does not imply code integration.

## Improve the workflow without unnecessary scope growth

1. G0 runs offline on CPU; do not introduce ROS, models, or simulators into the projection exercise yet.
2. Each feature has a scenario ID and a specific oracle; do not wait until the end of the project to design the judge.
3. Measure perception on bags first, then close the actual control loop in SITL.
4. Accept safe abort as a separate outcome, without disguising it as navigation success.
5. Do not force the local planner to solve every maze. If navigation completeness is needed, open a global map/planner branch with its own gates.
6. Retain a metric-depth adapter with uncertainty/validity; changing sources does not remove the need to reevaluate safety/planning.
7. Stage 4 remains in Gazebo: use simulated RGB/stereo and replace the depth adapter. Do not “replace Gazebo with an RGB camera” and lose physics/the oracle.
8. Start the harness with a bounded workflow + independent evidence; do not build an autonomous multi-agent daemon or auto-tune the test set yet.

## Official sources

Web sources on `main` may change; pin revisions during implementation. Release-tagged sources are used to check specific contracts, not to claim that the release has been run.

| ID | Source | Used to cross-check |
| --- | --- | --- |
| S1 | [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html) | Jazzy, Ubuntu, DDS/messages |
| S2 | [PX4 v1.16 Gazebo vehicles](https://docs.px4.io/v1.16/en/sim_gazebo_gz/vehicles.html) | Depth/lidar models |
| S3 | [Docker image API](https://hub.docker.com/v2/repositories/px4io/px4-sitl-gazebo-ros2/) and [tag/digest API](https://hub.docker.com/v2/repositories/px4io/px4-sitl-gazebo-ros2/tags/latest/) | Provider-described image contents, digest |
| S4 | [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention.html) | Position mode, sensing coverage, tracking delay |
| S5 | [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard.html) | Entry, heartbeat, setpoint, failsafe |
| S6 | [PX4 main Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) | Current publisher/control conflict warning |
| S7 | [ObstacleDistance v1.16.0](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/msg/ObstacleDistance.msg) | cm, unknown, no-return, angles |
| S8 | [DDS topics v1.16.0](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/uxrce_dds_client/dds_topics.yaml) | Actual release input/output topics |
| S9 | [REP-118](https://www.ros.org/reps/rep-0118.html) | Depth image encoding/Z |
| S10 | [REP-117](https://www.ros.org/reps/rep-0117.html) | Invalid/near/far semantics |
| S11 | [REP-103](https://www.ros.org/reps/rep-0103.html) | Frames and SI units |
| S12 | [ROS 2 Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | Sensor freshness and compatibility |
| S13 | [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) | Relative model, size |
| S14 | [DA2 metric implementation](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth) and [Small indoor model card](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small) | Metric weights, preprocessing, license |
| S15 | [Depth Anything 3 README/model cards](https://github.com/ByteDance-Seed/Depth-Anything-3) | Metric scaling, parameters, per-model license |

Context7 was attempted, but the server was not connected; research used official documentation/repos/APIs directly. No community survey or latency experiment supports a “best stack” conclusion.
