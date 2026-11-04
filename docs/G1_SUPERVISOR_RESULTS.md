# G1 supervisor: code and offline pinned-message results

## Authority and gate status

The operator approved complete code and offline pinned-message verification only. No control publisher, ROS node, rclpy initialization or arming command was permitted during verification. `g1-supervisor` remains blocked on its actual stationary runtime fault-injection/ownership/handoff evidence. `g1-flight`, `g1-calibration` and every G2/navigation gate remain unmet.

Historical protocol evidence: [`replay/evidence-semantics-01/supervisor-offline.json`](replay/evidence-semantics-01/supervisor-offline.json). Its lifecycle is synthetic, not a vehicle mode change. The current [review repair results](SIM_REPAIR_RESULTS.md) replace the old directional-permit contract and add new guarded pinned-message/real-bag diagnostics. Synthetic envelope/deadline parameters remain protocol fixtures, not measured stopping or tracking bounds.

## Implementation

- `src/drone_cv/supervisor.py`: immutable deterministic transition state, independent command/permit/authorization inputs, explicit priming/mode/arm/handoff states and latched revocation.
- `src/drone_cv/px4_bridge.py`: pinned generated-message encoding, strict bounded JSON decoding and lazily constructed optional ROS adapter. Importing this module does not initialize ROS or create publishers.
- `scripts/g1_supervisor.py`: separate `offline-smoke`, `sources` and consequential `run` subcommands.
- `tests/test_supervisor.py`: behavioral freshness, epoch, capture-source progress, complete stopping-region containment, request/ACK attribution, failsafe/owner conflict and TTL-boundary regressions.

A safety permit binds the command identity/velocity and the original required observation `capture_stamp`. This stamp is the oldest observation still needed to justify that permit; producing another verdict cannot refresh it. Wire stamps contain `epoch`, `sequence` and simulation `sim_s`; receive monotonic time is assigned locally. Lease, command, permit, capture, status, odometry and clock progress have separate finite checks. A live bridge heartbeat does not renew an expired proposal.

Priming emits only `OffboardControlMode`, never `TrajectorySetpoint` before fresh PX4 status observes Offboard. Mode entry and arming require separate explicit one-shot requests. ACKs must be addressed correctly and occur strictly after the request; successful ACK alone does not replace fresh status evidence. Zero command and noisy measured drift now require complete all-direction stopping-room containment, not angular coincidence. Transient observed failsafe, authority loss, disarm, rejected ACK and ownership conflict latch revocation; no automatic re-entry follows.

Unused setpoint fields are NaN only at the PX4 encoding boundary. Mode/arm fields consumed by pinned Commander remain finite where required. Handoff output is not proof that PX4 accepted the mode or that the vehicle stopped. When valid odometry is absent, the code does not synthesize a hold setpoint.

The current bridge requires explicit exploratory-simulation configuration, a recomputed conditionally eligible held-out empirical profile with exact workload/environment/policy applicability, finite per-source clock contracts, and loss-response budgets bound to read-only pinned live parameters before publisher construction. None grants calibrated motion authority. Revocation stops publication while bounded terminal status observation continues. These mechanisms have offline verification only; actual healthy matched-baseline/fault trials, live readback and fallback remain blocked. See [the repair contract and current limitations](SIM_REPAIR_RESULTS.md).

## Exercised pinned path

The smoke constructed, serialized and decoded the installed generated PX4 messages in the existing isolated image:

- Image: `sha256:061a39905d2d84d40859bb46b67048241b6dacf2ed3983889da7f5fd3e718285`.
- Installed `px4_msgs` source HEAD: `8fb8277cf95c4ba87094146bc8922e76c4ac0799`.
- Installed firmware package: `px4-gazebo 1.18.0~beta1-noble`.
- Declared pinned firmware source: `61907962c9220437ec01a98fa26bfbf5cbb254b5`, as bound by the existing image/environment evidence. A package version or message constant alone is not an independent firmware rebuild proof.

The verification replaced `rclpy.init`, `rclpy.create_node` and `Node.create_publisher` with raising guards. All remained uncalled. Actual CDR serialization/roundtrip covered `OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`, `VehicleOdometry`, `VehicleStatus` and `VehicleCommandAck`. The synthetic lifecycle reached priming, mode wait, Offboard-disarmed, arm wait, active, handoff wait and released; missing odometry then exercised revocation without fake hold output.

The first smoke attempt found incorrectly nested Docker-copied sources and stopped with a `Permit` constructor mismatch. The copy destination was corrected, then the same guarded smoke passed. This was source staging, not a protocol workaround or control execution.

## Reproduction and remaining proof

Copy the current `src/.` into the exact existing container `src` directory; copying a directory without `/.` can create an unintended nested directory. Use the existing ROS/PX4 setup and run:

```sh
python3 scripts/g1_supervisor.py offline-smoke
```

Do not run the `run` subcommand under the current approval. Its explicit CLI opt-in is an additional guard, not a substitute for direct operator authorization. Startup files/configuration/native task state cannot grant that permission.

Remaining runtime evidence includes stationary stale-command/permit/owner-conflict fault injection, real DDS exclusivity and PX4 ACK/status/mode/handoff behavior. DDS graph/lock checks are cooperative ownership in an operator-controlled isolated domain, not authentication against hostile participants. Actual timing, flight, stopping and calibration require their own approved trials. Suppressing publication alone cannot be called safe abort, hover or collision avoidance.
