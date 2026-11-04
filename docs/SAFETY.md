# Revised safety and acceptance contracts

**These are safety and acceptance contracts, not flight evidence.** The repository has an offline core, unarmed static oracle/recording path and supervisor code exercised only offline. All flight verification remains pending in the native roadmap. Merging code or documentation must not be used to mark unexercised gates as passed.

This document supplements the contracts in [PLAN](PLAN.md), [HARNESS](HARNESS.md), and the [scenarios](SCENARIOS.md). Deadlines/bounds that require measurement are stored with the effective config during implementation; missing bounds mean no authorization for autonomous motion.

## S01. Phase-specific contact and clearance

The rule “clearance ≥0.5 m throughout the flight, every contact fails” cannot coexist with taking off from/landing on a floor.

- **Maintain a contact ledger for the entire run**, from preflight through disarming and log flushing. Do not omit collisions after the goal or during shutdown.
- Landing support contact is allowed only between **designated landing gear** and a **declared support surface**. The pair allowlist, surface bounds, and phase must be in the manifest before the trial.
- Allow support pairs only while resting on the support, during the takeoff transition, or during intentional landing. Contact involving the body, propeller envelope, walls, ceilings, objects, or surfaces outside the landing zone is always unsafe.
- Rollover or clearance violations are not exempt merely because a valid support pair exists simultaneously.
- **Navigation clearance ≥0.5 m** is measured from the physical body/propeller envelope to every relevant surface in `NAVIGATE`, `RECOVER`, and `GOAL_HOLD`. Include floor/ceiling when near the envelope.
- Takeoff/landing have separate frozen envelopes, known-clear volumes, and limits; do not apply 0.5 m to the support surface touched by the landing gear. This does not exempt other obstacles.
- Record minimum clearance separately by phase. G3 cannot pass without phase/contact telemetry.

The oracle uses uninflated physical geometry to measure surface clearance. The planner may add tracking/localization margins through inflation; **do not subtract the same margin twice in the evaluator**. If propeller collision meshes are incomplete, add an oracle geometry envelope; do not assume the contact sensor will detect propellers.

**Proof required during implementation:** D01/F04 accept valid support contact while also detecting body-wall contact; C06/E14 check the takeoff envelope. Tasks `g1-oracle`, `g1-flight`.

## S02. No circular dependency

Required order before navigation:

1. G0: offline geometry, validity, and braking/freshness core.
2. G1 environment: simulator/camera/odom/clock/QoS, **not armed yet**.
3. G1 minimal oracle/logger and supervisor/bridge. Check static contact/ownership/freshness before flight.
4. A low-speed exploratory mission in a known-clear test volume, with supervisor, observer, and operator stop.
5. Braking/delay/tracking calibration, deadline/bound configuration, and fault handoff.
6. G2 perception replay, then wall-stop **through the existing supervisor**.
7. G3 planner/recovery/runner/batch; do not build the oracle for the first time after already using it to grade G2.

Prior knowledge that a volume is clear is only an **exploratory harness fixture**, recorded as `bootstrap` in the manifest; do not give the planner the true map or count that run toward navigation acceptance. Before calibration, allow only exploratory scenarios constrained by their own volume/low-speed/operator-stop limits, without claiming a stopping guarantee.

**Proof required during implementation:** H1 has an oracle/supervisor before the mission; G2 depends on `g1-calibration`. The roadmap DAG expresses these dependencies.

## S03. Infrastructure errors do not overwrite safety failures

Results have two independent axes:

- `safety_outcome`: `NO_VIOLATION_OBSERVED`, `UNSAFE_OBSERVED`, or `UNKNOWN` when evidence is insufficient.
- `execution_outcome`: `COMPLETED`, `SAFE_ABORT`, `TASK_FAIL`, `INFRA_ERROR`, or `INVALID_WORLD` before the trial.

`UNSAFE_OBSERVED` is permanently latched for the attempt. For example, if the drone hits a wall and Gazebo then crashes, retain both the collision and infrastructure error; do not convert it into an infrastructure run that can be excluded from the denominator.

- SUCCESS requires completed execution, correct goal/termination, sufficient observer coverage, and no unsafe event; “not seen” is insufficient if the observer died.
- `SAFE_ABORT` requires sufficient evidence of no violation and handoff/stopping according to the contract. Losing the oracle does not justify assigning safe status.
- One event may produce multiple reason codes; the primary report does not erase other reasons.
- Every terminal branch, including start/health/infrastructure failures, goes through **best-effort bounded collection**, followed by cleanup of the run's processes. Collection failures are recorded separately, without swallowing the original error.
- Exit `1` when a safety/task failure has been observed; `2` when no known failure exists but evidence is invalid/incomplete; `0` only when the full gate passes.

**Proof required during implementation:** D01/D03/F02/F03/F04 check collision followed by crash, telemetry loss, and flush failure. Tasks `g1-oracle`, `g3-runner`, `g3-batch`.

## S04. Separate navigation, safety-negative, and stress suites

Suite membership, expected outcome, geometry, fault timing, and deadlines must be frozen **before** an attempt.

| Suite | Purpose | Acceptance |
| --- | --- | --- |
| `navigation` | Static obstacles, valid route/start/goal, no injection of impossible-to-handle faults | 50 frozen test worlds; G3 ≥45 successes, zero unsafe events, complete telemetry, and the navigation clearance gate |
| `safety-negative` | Arm refusal, stale input, crash, unreachable goal, loss of authority | PASS according to the fault/handoff oracle; do not add to the navigation numerator, and do not call unsafe outcomes safe aborts |
| `stress-diagnostic` | Moving obstacles spawned too close to brake, glass sensor limitations/subpixel wires | Record unsafe/unknown outcomes; grade correct detector/classifier behavior, without requiring avoidance of physically impossible situations |

F10 may pass the **classification test** when the evaluator correctly records an unavoidable collision. That collision remains visible in the stress report, without falsely implying that the navigation suite crashed or that the obstacle was avoided. Do not move a failed navigation run into stress after seeing the result.

**Proof required during implementation:** B03/B06/F08/F10; task `g3-faults` and the frozen manifest.

## S05. Freshness and latency with explicit clock domains

- Keep capture stamp/epoch on the sim clock; receive/processing/progress use the monotonic wall clock. Do not subtract timestamps from different domains.
- `sim_age` checks how long the simulated world has had to change; `wall_age`/the progress watchdog detect stalled compute/transport. Report the two values separately.
- A sim pause is valid only after the runner has confirmed it. Do not fabricate fresh frames with a timer heartbeat; resuming requires revalidation of epoch/pose/coverage and command ownership.
- Deadlines for observation, pose, proposal, supervisor result, handoff, and logger must be finite and explicit in the effective config. Do not default to “infinite”.
- Use measured conservative delay/braking/jerk bounds for safety; latency p95 is a metric, not a bound.
- Do not count the same delay interval twice: capture-to-safe-publish and publish-to-tracking traces have separate endpoints. If adding a margin, identify it explicitly as additional to the measured interval.
- Calibrate under a specified workload/RTF/config. Load/latency beyond the bound reduces motion authority or triggers abort according to policy; do not continue relying on old calibration.
- Command freshness depends on the source sequence/epoch and original expiry. The bridge must not extend a command's lifetime simply by stamping a new publish time.

**Proof required during implementation:** A06/A07/E08/E09/E12/E15/F06; tasks `g0-safety`, `g1-calibration`.

## S06. Free-space evidence with actual coverage

One `NO_RETURN` ray does not prove that an entire 5° bin or volumetric corridor is free.

- Coverage must reflect pixel/ray support, FOV, near/far bounds, timestamps, and occluded regions, not merely a “bin has data” boolean.
- A corridor may be authorized only up to the range where **the entire footprint/swept volume requiring checks** has support under the agreed sensor sampling model. Unsupported cells/bins remain unknown.
- Compute inflation/support width before reduction. A near pixel must not be averaged/percentiled away among distant rays in the same bin.
- `NO_RETURN` clears only the segment within a verified ray's trusted range; it does not clear the shadow behind a near obstacle or arbitrary gaps between rays.
- Declare the minimum resolvable obstacle size by range/FOV/resolution. Safety bounds do not cover obstacles the sensor cannot observe; subpixel wires belong to the stress suite.

**Proof required during implementation:** A04/A05/A08/E02/E13/E17; tasks `g0-geometry`, `g2-adapter`.

## S07. Valid input does not require every number to be nonnegative

- Position, velocity, yaw, and quaternion components may legitimately be negative in ENU/NED/FLU/FRD.
- Reject NaN/Inf in physical pose/velocity/commands; check quaternion norm/order, units, and frame against the contract.
- Distance, latency, and margins must be within their domains; `a_brake` must be positive. Negative distance is not reverse velocity.
- Only the PX4 adapter may write NaN to **unused setpoint fields as required by the protocol**; do not let NaN sensor/actual-velocity values pass through.
- Geometric/numerical tolerances need units, rationale, and independent fixtures; do not default to clipping quaternion/velocity values to hide invalid input.

**Proof required during implementation:** C08/E20 must include both valid negative-axis fixtures and invalid physical values. Tasks `g0-contracts`, `g0-geometry`.

## S08. Route validation and efficiency are not an all-powerful oracle

- A* grid-centre connectivity does not prove that a drone can pass through a gap or cut a corner. Continuously validate segments against footprint, margin, altitude, and geofence.
- Validate start/goal and takeoff/landing volumes separately, rather than only finding a route between two cell centers.
- The oracle route needs the same motion constraints for a valid comparison: speed bound, stopping/turn assumptions, and the NAVIGATE-to-goal-hold protocol.
- Without a dynamics-feasible reference time, report only path length/detour ratio; **do not grade time-to-goal ≤2× an arbitrarily claimed optimal number**. The time-efficiency gate remains unmet without a reference; do not drop the gate.
- Freeze the validator before test-seed generation; retain every rejection and cap generator retries. Do not exclude a world because the local planner fails.

**Proof required during implementation:** D02/F01/F05/F08 and A05; task `g3-worlds`, then `g3-batch`.

## Gap closure status

S01–S08 are closed at the contract level. The G0 core has verified units/frames, synthetic swept-volume coverage, clock/freshness, and constant-deceleration bounds through code/tests/a demo; this does not prove that real sensors or PX4 conform to the model. The unarmed G1 environment and [static oracle](G1_ORACLE_RESULTS.md) paths have their own bounded evidence. Supervisor runtime, flight proofs, actual numerical deadlines/bounds and stopping calibration remain pending. The separate offline freespace and stationary truth diagnostics do not supply motion authority.
