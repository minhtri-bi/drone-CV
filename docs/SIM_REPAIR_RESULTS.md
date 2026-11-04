# Simulation review repairs: code and offline results

Sections R1–R10 below retain first-round evidence at `46e1771`. The [second-round section](#second-round-n1n7-code-and-offline-results) records the current N1–N7 cutover; it does not upgrade historical results.

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

## Second round N1–N7: code and offline results

### Authority and unchanged gates

After the requested plan amendments, the user approved code/offline work. Existing native IDs are reused; no new task, `Stage.BRAKING`, operational execution, parameter write or calibration-before-bootstrap dependency is introduced. The complete outer geometry, 0.5 m navigation clearance and independent safety boundaries remain unchanged.

Actual verification used the existing isolated pinned image for decoding, serialization, file-based replay and bounded storage only. ROS initialization, nodes and publishers were forbidden in those paths. No simulator, new recording, control trial, live parameter readback, arming or hardware operation was performed.

### N1: one explicit crash-after-tick response horizon

`supervision_contract.LossResponse` replaces the free `Envelope.response_delay_s`. `Config.response_delay_s` is derived, not independently configurable:

```text
T_loss = max_tick_gap_wall_s * maximum_rtf
       + COM_OF_LOSS_T + COM_FAIL_ACT_T
       + mode_transition_s + transport_clock_s
s_loss = max(declared maximum speed, requested speed,
             norm(abs(measured velocity) + per-axis velocity error),
             retained possibly issued speed bound)
d_loss = s_loss * T_loss + s_loss**2 / (2 * minimum_deceleration)
       + position_error + tracking_error + extra_margin
```

Every admitted setpoint tick requires the whole all-direction ball to fit both the permit's absolute reference-center box and the geofence. A new zero request cannot erase an older potentially effective setpoint. The complete declared tracking-speed envelope is included even during a position hold; this is deliberately more conservative than assuming an instantaneous commanded velocity is the maximum future speed.

The delay legs are explicitly nonoverlapping simulation-motion seconds; only the wall tick gap uses the declared upper RTF conversion. The mode-transition leg excludes `COM_FAIL_ACT_T`. Frozen expected policy and actual readback must match the response's firmware, `COM_OF_LOSS_T`, `COM_FAIL_ACT_T` and `COM_OBL_RC_ACT`. Missing/invalid legs or mismatched readback reject. A lower positive frozen loss timeout tightens the horizon; its heartbeat/jitter rationale and actual fallback proof remain required. No parameter setter was added.

These are conditional model assumptions, not measured dynamics or a safe-abort certificate. The synthetic message-lifecycle fixture uses explicitly synthetic 0.2 m/s maximum speed, 0.25 m/s² deceleration and a 0.4 s response horizon to exercise the protocol. It is not a tuned real-flight profile. Revocation still suppresses output immediately, with no automatic Hold command, retry, re-entry or new braking stage.

### N2: exact empirical split fails, without retuning

See [the empirical and paired-fault contract](SIM_PROFILE_CONTRACT.md) and [compact results](replay/sim-repair-round2-01/empirical-summary.json). The [v2 protocol](replay/sim-repair-round2-01/empirical-protocol-v2.json) was frozen before this evaluation: 1,216 derivation identities and 1,212 distinct heldout identities over the entire declared post-warmup windows. Historical traces were already reviewed, so the heldout set is retrospective and derivation-disjoint, **not untouched/blinded**. The unexecuted v1 remains preserved; v2 corrects schema-two source-domain metadata before outcomes.

| Split | Declared identities | Position + interval-average velocity available | Missing continuous truth/CDR identity | Position-health axis exceedances | Candidate position-error axis exceedances |
| --- | ---: | ---: | ---: | ---: | ---: |
| Derivation | 1,216 | 921 | 295 | 3,648 | 0 |
| Heldout | 1,212 | 919 | 293 | 3,636 | 166 |

The descriptive derivation position maxima are `(0.0439757239073515, 0.02743459306657314, 0.0602348148822784)` m in local NED. Heldout absolute Z error reaches `0.06402713060379024` m, exceeding that unchanged candidate. The enclosing position-box radius is `0.07946542716053577` m; it is not an empirically certified hard bound. Additional margins are frozen at zero to avoid inventing a passing reserve. Health ceilings remain the original 0.01, now represented as explicit NED three-vectors and compared per axis.

Support-06 has Pose_V positions, not independent instantaneous twist. Finite differences remain interval-average diagnostics; instantaneous velocity coverage is zero. r39 retains decoded estimator-health values but has no continuous truth and no synthesized CDR hashes. Missing truth, real held-out failures, health violations, absent untouched/blinded evidence, tracking justification and runtime applicability all remain explicit. `trial_profile_eligible`, runtime admission and deterministic-bound certification are false. No runtime profile is generated or enabled.

The bridge recomputes the supplied empirical payloads and binds exact Envelope values plus image, firmware, message pin, frozen parameter-policy digest, clock/rate window, workload, body origin and NED frame before any ROS construction. This checks a conditional prerequisite only; content hashes and labels do not authenticate provenance, calibrate bounds or authorize a trial.

### Runtime fault validity, not vacuous suppression

The actual [classifier CLI smoke](replay/sim-repair-round2-01/fault-classifier-execution.json) uses **analytical synthetic records only**, not runtime trials. Its four cases retain one valid causal PASS, one valid wrong-reason FAIL, two INVALID_TEST cases, and one unsafe latch surviving a missing baseline file. Expected aggregate exit is 1. Exact smoke inputs are retained in the accompanying ZIP with CRC/byte readback.

`FaultProtocol` freezes distinct trial IDs, known profile/evaluation identities, target stage, actual PX4 mode/arming requirements, matched windows, source/workload/policy bindings, injection identity, expected first reasons and finite deadlines. Unknown future trace hashes are not preregistration requirements: the post-run manifest binds the actual resulting trace hashes. A baseline that previously revoked cannot be resurrected by choosing a later healthy window. A fault must reach the healthy target independently, show actual injection, and suppress output immediately after the first observed revoke. Wrong/absent causal response in a valid trial is FAIL; unrelated masking or an invalid baseline is never PASS. Missing/unattempted trials stay in the planned denominator. Unsafe evidence and S03 execution outcomes remain separate.

No real healthy matched baseline or causal injection is demonstrated. Existing Disarmed recordings cannot substitute for an armed ACTIVE target; typed ACK/observer booleans do not replace raw addressed-message and independence evidence.

### N3: finite per-source cadence and measured clock basis

Each dynamic source declares its cadence domain, nominal period, bounded wall jitter and unchanged simulation freshness. For a simulation-cadenced source, wall freshness and progress use `period_sim / RTF_min + jitter_wall`; a wall-cadenced source uses its own wall period. Explicit clock progress has the same minimum service-budget check. Human leases, command TTL, request, handoff, session and tick deadlines are not stretched by slow simulation.

The bridge measures rate from the node's delivered Gazebo ROS-clock snapshots at monotonic receive time, not an older queued explicit `/clock` message. The separate explicit-source progress watchdog remains. Missing, short, future, stale, wrong-epoch or outside-range observations revoke with explicit reasons. Startup waits for actual observations before publisher construction.

The retained replay uses 0.196–1.0 as a **retrospective diagnostic hypothesis**, not a validated workload range. The prior 0.196 long-window observation is not a guarantee for every frozen one-second window. [Final node-clock replay](replay/sim-repair-round2-01/supervisor-node-clock.json) observed minima 0.17462875456963206 for r39 and 0.053682691941849776 for support-06. Below-floor windows remain refused; the floor was not lowered to admit them.

| Recording | Actual refusal counts with final node-clock basis |
| --- | --- |
| r39, 630 odometry rows | 20 missing RTF windows; 177 outside supported range; 433 odometry uncertainty |
| support-06, 1,893 odometry rows | 20 missing causal status; 1,870 outside supported range; 2 odometry uncertainty; 1 stale RTF observation |

Conditional synthetic geometry still contains 630 and 1,873 eligible diagnostic samples respectively. Neither trace establishes stationary admission. The earlier [explicit-callback diagnostic](replay/sim-repair-round2-01/supervisor.json) is retained unchanged and identified as superseded in [execution provenance](replay/sim-repair-round2-01/offline-execution.json); callback-source throughput is not silently relabelled node-clock rate.

### N5, N6 and N7: conditional outputs, one-source gates, witnessed exit

The actual moving-oracle clear CLI exits 0 with `clearance_conditional_on_declared_motion_bounds: true`; the thin-crossing negative exits 2 with UNKNOWN/INCOMPLETE and the conditional flag false. Both report calibration/flight admission false. Moving emitters and consumers remove the legacy certificate key atomically, including inherited sampled intervals and malformed-initialization output. Static reports and historical evidence remain unchanged.

All four supplemental checker policies now live in one immutable structured declaration. Enforcement and the generated roadmap table derive from the same prerequisite, exact/prefix target selector and guarded statuses. Future G2-prefix tasks fail closed; unrelated G1 work is not blocked. These remain checker policies, not invented native dependency edges.

[N7 provenance](replay/sim-repair-round2-01/provenance.json) witnesses actual pinned replay exit **0**, no 600 s deadline expiry, and an owned child that was reaped. [Readback comparison](replay/sim-repair-round2-01/planes-comparison.json) has the identical uncompressed report SHA-256 `04f69883f90a870a866ac6e9173ef09119819bcb9faf6d5fbbfb9b87628a3a7d`. All 433 rows retain zero passing original-primary frames and missing justified dy budgets. Diagnostic completion is not dy/primary/G2 acceptance. The old bundle's `original_process_exit_observed: false` remains unchanged.

Full plane and empirical JSON reports are retained losslessly as `planes.json.gz` and `empirical-consistency.json.gz`. [Compression identities](replay/sim-repair-round2-01/compression.json) bind compressed/uncompressed hashes, sizes and exact decompression readback. No historical bytes were overwritten.

### Exercised second-round verification and remaining blockers

- `python -m pytest -q`: **724 passed in 83.13 s** after integration.
- Focused response/source/bridge/failsafe regressions: **113 passed** before the final positive applicability boundary and integrated run.
- Actual pinned message lifecycle, retained r39/support-06 decode and bounded storage roundtrip: completed with zero ROS initialization/nodes/publishers/arming effects.
- Actual frozen empirical CLI: computation exited 0; the candidate **failed** heldout/health/coverage prerequisites and remains non-admitting. Computation exit is not profile acceptance.
- Actual conditional oracle CLI: clear exit 0; thin-crossing expected exit 2.
- Actual synthetic paired-fault classifier CLI: expected exit 1, four analytical cases retained; zero real runtime trials.
- Actual N7 decoder: exit 0 observed, exact hash equality, owned child reaped. Exact gzip decompression and synthetic-input ZIP CRC/byte readback passed.
- Final `python scripts/check_project.py --export-roadmap` and `python scripts/check_project.py` both passed: original PDF identity, 36 acyclic tasks, 65 scenario IDs, local links and **103 evidence hashes**. Native revision **101** and its portable view retain 14 done, 5 blocked and 17 pending tasks.
- The owned offline-only container was stopped after execution; its existing image and previous evidence were retained.

The first focused check found two old TTL fixtures hitting the newly independent finite receive-wall deadline before simulation expiry. Their receive times were separated from their original simulation acquisition times to isolate the intended exact-TTL boundary; no source TTL, wall deadline or safety threshold was widened. The first retained rate diagnostic used queued explicit callback source stamps; its artifact remains, and the final adapter/replay use the node ROS-clock receive snapshot with a separate source-progress watchdog.
An intervening integrity check encountered a transient unlisted duplicate check ledger during evidence publication. The redundant generated file was removed; both final checks passed against the unchanged 20-file evidence inventory.


`g1-supervisor`, `prep-evidence-review`, `prep-flight-oracle`, `prep-depth-path-audit` and `prep-geometry-crosscheck` retain their unmet operational/acquisition criteria. In particular, no eligible untouched profile, justified tracking/stopping/RTF workload range, actual healthy baseline/causal faults, live parameter/fallback/ownership observation, new dy acquisition, evaluator isolation/phase/support proof, authoritative source/bridge taps or independent mesh-loader observation was supplied. No flight, calibration, G2 or hardware gate passes. PR #12 remains draft.

