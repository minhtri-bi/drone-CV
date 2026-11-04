# Simulation review repairs: code and offline results

## Scope and roadmap

The user approved [the repair plan](SIM_REPAIR_PLAN.md), native roadmap updates and code/offline implementation. The native API registered `prep-evidence-review`, `prep-flight-oracle`, `prep-depth-path-audit` and `prep-geometry-crosscheck`. There are now 36 tasks. Their real acquisition/runtime criteria are not replaced by offline results.

No simulator was started, no new sensor recording was collected, no control publisher/node/ROS initialization was constructed by verification, no arming command was sent, and no packages, PX4 parameters, permissions or hardware settings changed. The existing pinned image ran an isolated **offline verifier** with network none, four CPUs, 6 GiB memory, dropped capabilities and no host mounts. Raw historical evidence and the source PDF remain unchanged.

The supplemental checker enforces `prep-evidence-review` before G2 completion, `prep-flight-oracle` before G1 flight starts, and `prep-depth-path-audit` before calibration completes. These are not native DAG edges. Unrelated G1 work is not gated by the y-plane branch.

## R1: stopping region and actual retained odometry

`Permit` now carries a complete reset/epoch-bound local-NED reference-center box instead of scalar forward distance. It binds command identity/velocity, original capture identity, reference position, evidence kind and motion-model provenance. The producer must erode the region by the complete physical footprint and the applicable phase clearance. Bootstrap known-clear evidence remains separate from perception-supported mission evidence. There is no live G2 producer to claim integrated.

The supervisor encloses arbitrary direction changes by a stopping ball. Let `s` be the larger of commanded speed and the norm of componentwise `abs(measured_velocity)+velocity_error`. Required radius is:

```text
s * response_delay + s**2 / (2 * deceleration)
+ extra_margin + position_error + tracking_error
```

The ball must fit inside both the certified box and the local geofence. This deliberately overblocks; it does not assume two thin nominal paths cover a direction change. Its model assumes speed remains bounded through response and then decreases at the declared deceleration, with residual tracking/position error explicitly bounded. Estimator variance does not establish those hard bounds. Missing/invalid declarations remain errors. Synthetic and exploratory-simulation targets are explicit; hardware is unsupported. Motion/uncertainty budgets remain uncalibrated.

Behavioral boundaries exercise noisy zero-command drift, lateral/diagonal/reverse drift, inadequate room, geofence faces, reset/frame/provenance, mission evidence kind and seeded independently integrated changing-direction paths. A forward-limited certificate still refuses reverse drift without room.

The [guarded pinned smoke and retained replay](replay/sim-repair-offline-01/supervisor.json) decoded actual installed messages and retained original source/wall identities:

| Input | Odometry samples | Conditional geometry | Actual transition refusals |
| --- | ---: | --- | --- |
| r39 | 630 | 630 contained in explicitly synthetic region | 630 `ODOMETRY_UNCERTAINTY` |
| support-06 | 1,893 | 1,873 contained; 20 lack causal status | 1,733 `ODOMETRY_UNCERTAINTY`, 140 `STATUS_stale_wall`, 20 missing causal status |

**No stationary admission is demonstrated by these real traces.** Position variance reached 0.03260526433587074 and 0.030841292813420296, respectively, above the unchanged synthetic health ceiling 0.01. No ceiling, stale-status deadline or source-lead allowance was increased. Geometry containment is conditional on synthetic free space/dynamics, not a claim that the recorded world or actual stopping behavior satisfied them. No PX4 ACK/Offboard telemetry was invented. Independent conditional ticks do not prove continuous runtime watchdog behavior.

## R8: shared quaternion ingress

`src/drone_cv/px4_quaternion.py` is the pure shared wxyz ingress for bridge decoding and capture-time replay. It rejects malformed/nonfinite/zero/oversized norms before bounded normalization and retains squared-norm residuals. The stricter existing replay ceiling `1e-6` is preserved; the old bridge-only `1e-3` allowance is removed. Four rounded float32 components of a unit vector have representation error bounded by approximately `2*u+u*u`, with `u=2**-24`; this is not an angular-error bound.

Observed maximum raw squared-norm residuals were `1.8410179669636761e-7` for r39 and `1.842088146464249e-7` for support-06. Tests cover both norm-boundary sides, malformed values, q/-q equivalence and near-pi yaw. No epsilon was enlarged to fit these data.

## R4: policy readback and terminal observation

`src/drone_cv/px4_failsafe.py` supplies read-only local pinned-instance admission and bounded post-revocation status observation. Startup requires an explicit exploratory-simulation target, instance zero, complete expected parameter inventory/rationale and finite readback/terminal deadlines. The bridge reads the live firmware identity and parameter values **before** constructing publishers, then rechecks cooperative DDS ownership. Missing daemon, wrong identity, unavailable parameter, mismatch or timeout prevents admission. It never writes parameters.

Pinned sources: [Commander parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/commander/commander_params.yaml) and [readback/comparison implementation](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/systemcmds/param/param.cpp). `COM_OBL_RC_ACT=5` means Hold in this pin; it is not a default chosen here. Quiet float readback prints four decimals, so the code also invokes read-only `param compare -s`: integers compare exactly, floats require absolute difference below `1e-7`. That finite precision is reported, not called bit-exact readback.

Cooperative handoff remains the existing explicit one-shot request/ACK/fresh-status sequence while authority is valid. Ownership/failsafe/source loss suppresses output immediately; no Hold request follows another owner's takeover or PX4's exit. The CLI keeps subscriptions alive through a bounded terminal window instead of immediately discarding later status. Last-256 status/ACK/publication traces and the first failure are retained. Zero output, a Hold label or an accepted ACK never certifies safe abort/stopping.

Offline policy/state/encoding negatives passed. **Live readback, parameter policy values/rationale for a particular trial, actual fallback, owner conflict and process-death observation remain unverified.** Process death requires an independent observer; an in-process terminal ledger cannot observe its own death. No control-publisher trial or armed fallback is authorized here.

## R5 and R2: provenance, visibility and dy

See [the evidence contract](SIM_EVIDENCE_REPAIRS.md). All SensorProfile consumers require explicit uncertainty/source/provenance/no-return inputs; none grants operational motion authority. Historical/diagnostic data cannot borrow synthetic positive-infinity certification.

The actual [offline fixture preflight](replay/sim-repair-offline-01/fixture-preflight.json) passed its visible positive control, failed the old occluding side fixture as expected, and passed all **90 physically admissible poses** of the frozen 243-pose grid. All 153 below-support poses are listed as excluded, not passed. Primary geometry/ROIs, 3-pixel erosion, support 0.95, p95 0.10 m and maximum 0.20 m remain unchanged. The auxiliary target is finite and y-sensitive; the result is sampled analytical visibility, not a rendered Gazebo acquisition or continuous perturbation proof.

Frozen fixture ID: `nonoccluding-973805435c60600f62694086c01c4e0048ee991f6e4ef4b20f03bc8cfe83ec0a`.

The [new historical plane summary](replay/sim-repair-offline-01/planes-summary.json) retains all 433 independent dy diagnostic rows, original primary pass/fail and missing-budget decisions. Every frame lacks a justified numerical dy budget; zero original-primary frames pass. No alignment was fitted and no old geometry failure was relabelled. The original replay process exceeded the client's 120 s wait, then its complete JSON artifact was observed/read back. Its process exit status was not observed. The artifact reports diagnostic coverage only, **not primary geometry or G2 acceptance**.

Remaining: freeze justified residual/rotation/depth/extrinsic/conditioning budgets, obtain separate acquisition approval, build a matching finite physical fixture without extra occluders, and collect fresh unarmed primary-plus-dy evidence. This branch gates G2 only.

## R3: actual offline moving-oracle path

See [the moving contract](SIM_ORACLE_REPAIRS.md). The actual `g1_oracle.py evaluate` path now uses complete-component translation/angular enclosures, evaluator-owned phase/support evidence, and a bounded cooperative endpoint-discovery ledger when a frozen moving contract is supplied. Static fixture/replay behavior remains static; old bags are not upgraded.

The [clear CLI smoke](replay/sim-repair-offline-01/oracle-clear.json) exited 0 with two analytic samples and complete declared interval coverage. The [thin-crossing CLI negative](replay/sim-repair-offline-01/oracle-thin.json) exited 2: both endpoints are clear but the interval is `POTENTIAL_VIOLATION`, safety UNKNOWN, not a demonstrated physics contact. Frozen analytic inputs are included in the evidence bundle. Regression cases also cover rotation-only/curved excursions, rate/gap/epoch loss, phase order/contact exemptions, unsafe latching and transient unexpected readers.

These offline semantic labels are not pinned wire-enum mapping or live truth isolation. Real evaluator-only transport, independently bound telemetry/support, justified motion constraints, bounded graph observation and separately approved unarmed faults remain missing before G1 flight.

## E7, R6, R7 and R10

See [depth-path guarantees and missing taps](SIM_DEPTH_PATH_REPAIRS.md). The actual [historical audit](replay/sim-repair-offline-01/depth-audit.json) keeps the source/bridge loss location UNKNOWN. Exact new enqueue/storage identity accounting was exercised through the actual separate-process rosbag writer under the pinned offline guard; both generated messages were read back. This does not certify any source/bridge production stage.

The [unchanged-profile geometry audit](replay/sim-repair-offline-01/geometry-audit.json) reports 42 parts, outer-AABB radial bound 0.45363579247019004 m and conditional level analytic rotor reach 0.39284210489752325 m. The current profile and every navigation consumer retain the outer bound. Vertical extent stays `[-0.2275995, 0.06938461481722502]` m and navigation clearance stays 0.5 m. No smaller envelope, floor exemption or deleted near evidence was introduced. Independent Gazebo mesh-loading comparison and tilted/new-primitive containment remain unverified; R10 is not passed.

## Verification and preserved failed attempts

- `python -m pytest -q`: **648 passed in 109.04 s** after integration.
- Actual offline fixture CLI: passed; all sampled/excluded poses retained.
- Actual oracle CLI: clear exit 0; thin interval expected exit 2.
- Actual pinned generated-message serialization/decoding and bounded storage roundtrip: passed with zero initialization/nodes/publishers/arming effects.
- Actual retained-bag replay: geometry diagnostic completed; actual admission refusals preserved above.
- Historical plane artifact: 433 rows read back, process exit unobserved after client timeout; no acceptance claim.
- `python scripts/check_project.py --export-roadmap` followed by `python scripts/check_project.py`: both passed at native revision 90, with 36 acyclic tasks, 65 scenario IDs and 83 evidence hashes; PDF, links and portable view remain consistent.
- Actual migrated synthetic CLI `python spike/freespace/experiments.py --profile docs/replay/evidence-semantics-01/profile.json --trials 0 --output artifacts/sim-repair-freespace-smoke`: passed. This exercises deterministic paths only, not a new Monte Carlo or navigation acceptance run.

The first focused regression found two incorrectly sized synthetic test certificates; those fixtures were corrected rather than weakening containment. The first integrated suite found an old manually constructed writer fixture missing the new counter state; it was migrated and the full suite passed. The pinned helper initially overwrote the sourced ROS import path, then exposed legitimate variance refusals, then exposed numpy-float JSON serialization. Import handling and serialization were corrected; variance/stale checks remained unchanged, and results now distinguish geometry from admission rather than requiring a false no-revoke result. Prior local partial output is not overwritten. A standalone oracle attempt without its documented PYTHONPATH failed before evaluation and remains a failed staging attempt, not geometry evidence.

No flight, calibration, G2 wall-stop, hardware operation, live failsafe/readback or new acquisition gate passed. PR #12 stays draft. Future operational approvals must name their exact target, scope and trial conditions.
