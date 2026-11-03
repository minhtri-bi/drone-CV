# Scenario and oracle catalog

**65 scenarios are specified; G0 geometry/validity/braking fixtures have offline tests. No SITL flight scenarios have been run.** A–D are the PDF's 30 cases; E–F add 35 cases. G0 fixture/test mappings are in the test names and native roadmap acceptance criteria; do not consider an entire integration scenario passed just because a geometry fixture passes.

## General rules

- `PASS` is determined by each case's oracle, not a “success” log printed by the planner itself.
- Each scenario must pin geometry, footprint, seed, injection time, configuration, clock, and deadline before running.
- Unknown space does not permit translation into that corridor; in-place rotation is permitted only when the swept rotation envelope has sufficient clearance.
- “Brake/hold” means requesting a stop within a measured, **feasible** stopping envelope; it does not promise collision avoidance for objects spawned inside the braking distance.
- “Fail closed” means rejecting invalid data/commands and following the supervisor contract; it does not necessarily mean shutting down motors or stopping the heartbeat immediately.
- A negative test passes when the fault is detected and handled correctly; it does not count toward navigation success.
- Mission timing uses sim time; health/compute timing uses monotonic wall time. A clock reset starts a new epoch.
- Glass/wire/low-light cases need an appropriate sensor model/noise. Merely changing a material in the simulator does not prove that real camera failures are modeled correctly.
- Apply [SAFETY.md](SAFETY.md): a contact ledger for the entire trial, phase-specific navigation clearance, a support-contact allowlist, an unsafe latch, and suite membership set before the attempt.

## A. Perception — 10 original cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| A01 | Wall 0.2 m in front of the camera, below minimum range | Too-near/unknown bins are blocked; NaN is not interpreted as free; do not arm at an unsafe spawn |
| A02 | Empty world beyond maximum range | Clear only up to the range for which the adapter proves `NO_RETURN`; unknown does not become clear |
| A03 | Acceleration causing pitch/roll, flat floor | Floor outside the envelope does not create false obstacles; real objects within the envelope are retained |
| A04 | Ø5 cm pole at 3 m | With a sensor configuration capable of observing the pole, evidence survives reduction and the collision direction is blocked |
| A05 | Two walls forming a 0.6 m gap | Do not enter if the gap is smaller than the envelope + margin; check a wider boundary case to avoid blocking everything |
| A06 | Camera stops/drops frames in flight | Data age exceeding the deadline triggers braking/bounded hold; stale frames do not extend control authority |
| A07 | Backward timestamp, future stamp, one node with incorrect `use_sim_time` | Reject different clocks/epochs; negative data age is never considered fresh |
| A08 | Goal 90° to the side or behind | Stop/rotate to observe first; do not fly into an unverified FOV |
| A09 | Depth flicker through bag replay | Retain a near-immediate response; history-based filtering must not create free space through an object |
| A10 | Low-texture wall, glass, darkness with an appropriate sensor fault | Dropout/low confidence blocks or reduces movement authority; do not claim glass visibility just because simulation depth is clean |

## B. Planner — 7 original cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| B01 | Dead end/U-shape | Enter RECOVER after 5 s without progress; at most 3 attempts; no blind reverse; safe abort if no exit is found |
| B02 | Pole between two symmetric routes | Hysteresis reduces left/right switching; >3 switches/2 s must be detected, not hide an oscillator |
| B03 | Goal inside a sealed box | Validator rejection or bounded safe abort in the negative suite; no success/fake goal |
| B04 | Reach the goal, then apply small pose noise | Goal hold follows distance/speed/dwell criteria; latch state, do not orbit |
| B05 | Crossbar/low object intersecting the envelope | Every band intersecting the swept volume blocks the direction; do not read only the middle band |
| B06 | Spawn a box during flight | If outside the stopping envelope, brake before the planner reacts; if avoidance is infeasible, record the unsafe outcome, do not claim a guarantee |
| B07 | Goal across yaw ±π | Correct shortest-path angular error, no near-2π rotation due to wrap; consistent ±π boundaries |

## C. Safety and control — 8 original cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| C01 | Kill perception or planner | Independent supervisor detects loss of progress/observation; no replay of old proposals |
| C02 | Kill supervisor or Offboard bridge | PX4 loses proof-of-life and executes the configured failsafe; logs measure timing, mode, and outcome |
| C03 | PX4 leaves Offboard due to battery/geofence/RC | Mission yields authority; no automatic re-entry/arming or continued interfering setpoints |
| C04 | Arm/takeoff rejected | Acknowledgment/status provides a reason; finite retries; no unbounded resend or false airborne state |
| C05 | EKF reset/odometry jump >1 m | Reset planner/history/goal transform as appropriate; no velocity spike or stale-map use |
| C06 | Spawn 0.3 m from a wall | PREFLIGHT blocks if the takeoff envelope is unsafe; do not check only the drone center |
| C07 | Distant goal, dense obstacles, high-speed command | Bound speed by latency/stopping distance; monitor actual velocity and side-slip as well |
| C08 | Mix NED/ENU and FRD/FLU | Vector/quaternion fixtures have independent geometric answers; positive-up/forward/yaw are correct through PX4 output as well |

## D. Evaluation — 5 original cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| D01 | Body/propeller collision, including after reaching the goal/during landing, unreported by the planner | Latch unsafe for the entire trial; allow only gear/support pairs in the correct phase/zone; success or infrastructure errors do not overwrite a collision |
| D02 | Random world with an A* grid path, but segments cut corners/gaps lack footprint clearance | Validator checks the continuous swept route/start/goal before the trial; record rejection/seed, do not exclude based on planner outcome |
| D03 | Gazebo/PX4 hangs/crashes, including immediately after a collision | Wall watchdog performs bounded collection/cleanup of owned processes; retain both the unsafe latch and infrastructure reason; a batch with missing trials does not pass |
| D04 | RTF drops under machine load | Report sim age and monotonic wall progress separately; workload beyond the calibrated bound must not continue motion using old data |
| D05 | Drone leaves bounds or tilts >60° | Evaluator fails the trial and performs a controlled stop; do not wait for timeout and then label it an infrastructure error |

## E. Additional runtime gaps — 25 cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| E01 | Boot with all NaN/0 and no frame history | Unknown blocks motion; “no near object has ever been seen” must not imply free space |
| E02 | Horizontal wire/obstacle occupies <5% of pixels in each column | If observable by the sensor, near evidence is not removed by a percentile; disclose resolution limits |
| E03 | New near object in the third frame after two empty frames | Fast path blocks in the first frame containing evidence, without waiting for a median majority |
| E04 | Switch from `32FC1` meters to `16UC1` mm; unfamiliar encoding | Normalize units/invalid values correctly; reject unsupported encodings, no silent scaling |
| E05 | Resize/crop an image while retaining old CameraInfo | Contract detects the mismatch; projection uses updated intrinsics, without creating false depth/gaps |
| E06 | Missing/stale TF, changed extrinsics or `frame_id` | Do not process with an identity transform by default; use a fresh epoch or an invalid observation |
| E07 | Camera/odom time offset during fast yaw | Interpolate at the correct capture time or reject; do not arbitrarily use the latest pose |
| E08 | Sim resets to t=0, rosbag seek/loop | Flush history, commands, sequences, and TF from the old epoch; mission must not continue unintentionally |
| E09 | Pause simulation while processes/heartbeat remain alive | Do not use wall ticks to fabricate fresh frames; watchdog distinguishes intentional pause from hang, and resumption requires revalidation |
| E10 | Reliable QoS subscriber with a best-effort publisher | Health reports no connection/incompatibility; PREFLIGHT does not pass merely because a topic name is visible |
| E11 | XRCE Agent restart, packet reordering, namespace/domain collision | Controlled rediscovery; reject old run/vehicle data; no automatic arming on reconnect |
| E12 | Planner freezes while bridge/safety thread still sends a heartbeat | Proposal expires; do not refresh timestamps on old commands; safe behavior has command-source traces |
| E13 | Bin boundary/wrap, one free ray with the remainder unknown, cm saturation | Do not clear an entire corridor from one ray; sampling support/inflation must not create gaps, and a sentinel must not overflow into free space |
| E14 | Large roll, off-center camera, propellers wider than the body | Swept/rotation envelope and clearance use actual geometry, not just a sphere at the camera |
| E15 | Side-slip, jerk limit, tracking slower than configured | Brake according to actual motion and measured bounds; report envelope violations, do not merely clamp proposals |
| E16 | Yaw scan near a wall, front is clear but propeller contact occurs during rotation | Do not scan with an insufficient rotation envelope; abort instead of assuming “rotation is always safe” |
| E17 | After a 360° scan, an object enters the direction observed at the start | Expired per-bin timestamps block motion; do not treat an old scan as fresh coverage |
| E18 | Relative-depth model or metric model with scale/focal wrong by 2× | Offline gate fails; do not enable navigation using meter thresholds even if the depth image looks plausible |
| E19 | Model has good full-image AbsRel but overestimates distance for near/OOD objects | Evaluate corridor/near-field overestimation and uncertainty; averages must not hide dangerous errors |
| E20 | NaN/Inf in pose/velocity/command, negative distance, `a_brake<=0`; control case with valid negative vector components | Reject invalid physical inputs/configuration, accept signed-axis values in the correct frame; NaN in unused PX4 fields belongs only in the protocol adapter |
| E21 | Land command rejected or no acknowledgment | Do not enter DONE; bounded retry/timeout, record the outcome; do not stop control before appropriate handoff |
| E22 | Camera lost and odometry also invalid | Do not assume zero velocity means hover; execute the tested PX4 fallback and record loss of position-hold capability |
| E23 | Two setpoint publishers, bridge restarts while still airborne | Detect ownership conflict/restart; do not automatically take authority/re-arm or publish default position=(0,0,0) |
| E24 | Sloped floor, tall steps, doorway threshold, low ceiling | Filters must not remove objects intersecting the envelope; fixed altitude must not be treated as floor clearance |
| E25 | Sensor faults in correlated patches, motion blur, saturation | Unknown/uncertainty does not become free; independent Gaussian noise must not be the only test |

## F. Evaluator and integrity — 10 cases

| ID | Stimulus | Required oracle |
| --- | --- | --- |
| F01 | Same seed but changed model/world/plugin/PX4/configuration | Manifest distinguishes inputs; do not combine them as the same experiment or baseline |
| F02 | Trial fails, then is retried multiple times | Retain every attempt; report first attempts and retries separately; do not select only successful runs |
| F03 | Missing seed, duplicate trial, telemetry lost at the end of a run | Do not aggregate to a pass; an incomplete batch has a nonzero exit and a reason, not fabricated results filling the gaps |
| F04 | Contact sensor lacks output/propeller mesh; simultaneous landing support and body-wall contact | Oracle health fails if coverage is missing; gear/support in the correct phase is allowed, body/propeller-wall contact remains unsafe; clearance does not double-count the planning margin |
| F05 | Nearby goal but long detour route or dynamically infeasible time reference | Verify continuous route/constraints; report only detour if a valid time reference is unavailable, without silently dropping the time-efficiency gate |
| F06 | Slow inference, low RTF, but apparently good sim latency | Report wall compute/transport, sim age, and RTF separately; apply actual timeouts/frame drops under the contract |
| F07 | Ground-truth obstacle topic/map leaks into the planner | Topic/interface audit fails the benchmark; only the evaluator may read the oracle, do not “fix” it by hiding it in the report |
| F08 | Agent changes seed/threshold/oracle after seeing a failure | Stop acceptance, require a user decision and a new baseline; do not weaken the gate yourself |
| F09 | Machine retains old Gazebo/Agent processes, namespace collision, incomplete asset download | Preflight identifies the conflict/asset; do not kill unowned processes or silently download a new model |
| F10 | Dynamic obstacle spawns within unavoidable braking distance, suite already frozen | Stress report retains the unsafe outcome; classifier test may pass but does not count toward navigation success; do not reclassify a navigation failure as stress after the run |

## Coverage by stage

Gate mappings are in [PLAN.md](PLAN.md). Cases E18–E19 are required only for learned depth sources; F06 is needed for latency benchmarks, especially at G4. The remaining cases are enabled incrementally but must be completed before claiming G3 within the static scope.

Passing A–C fault-handling tests does not increase the number of successful navigation worlds. Groups D/F test the evaluator itself; a broken evaluator invalidates the entire batch for acceptance.
