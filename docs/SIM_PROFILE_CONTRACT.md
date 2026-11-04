# Sampled empirical consistency and paired fault validity

This is an offline evaluator contract, not runtime, flight, calibration, a deterministic uncertainty bound, or proof of observer authenticity. It implements N2 and the mandatory fault-validity gate in section 6 of SIM_REPAIR_PLAN.md. No runtime collector or control publisher is added.

## Frozen empirical inputs

`ProfileProtocol` is immutable and content-hashed. It binds exact derivation and evaluation identities (manifest hash, topic sequence, original source nanoseconds, original serialized CDR hash), recording epoch/reset/alignment, body reference point, NED axes, source clock domains, percentiles, margins and health rationale. Duplicate/aliased source stamps or overlapping derivation/holdout CDR identities reject. Missing original CDR hashes remain null and nonaccepting; schema-two provenance is never upgraded.

`artifacts/sim-repair-round2-work/empirical-protocol-v2.json` was frozen before this evaluation. It selects every stored post-warmup odometry identity within each recording end, split deterministically at the temporal midpoint: 1216 derivation and 1212 heldout members. Both retained traces had already been reviewed historically: this is **retrospective derivation-disjoint** membership, not genuinely untouched/blinded holdout. The prior unexecuted v1 protocol remains unchanged; v2 corrects the schema-two source-domain description before outcomes, without changing membership or input bytes.

The frozen zero additional margins produce descriptive derivation maxima only. The unchanged 0.01 synthetic per-axis health references are evaluated, not fitted to a passing outcome or justified for runtime. The original position reference is also checked as a Euclidean 0.01 m radius; three component checks do not establish that radius. Every exceedance and missing/rejected identity is retained, along with signed extrema, absolute percentiles, per-source drift, coverage and actual source domains. Health values are retained even when independent position/velocity truth is missing.

The actual CLI uses existing `load_recording`, generated PX4 decoding, `body_truth`/`decode_wire`, alignment validation and truth interpolation. It never creates fake messages. `world_from_odom` is inverted exactly once to express independent world-ENU base-link-origin truth in odometry-local ENU, then ENU is converted to NED. Position is the **absolute base_link origin** error in the PX4 local-NED basis, not capture-to-query relative transform error. No optical lever-arm point is substituted. Reset/clock invalidation remains sticky across the complete stored recording.

The pinned schema-four Pose_V source contains no twist. Its position brackets support only finite-difference **interval-average** velocity comparisons, never an instantaneous error certificate, even when sampled positions are stationary. Existing independent truth velocity must be preferred when such an actual matching source is available; none is available in these retained inputs. The r39 schema-two trace has no continuous independent truth and remains explicitly uncovered/nonaccepting; its real decoded estimator variance is still reported. No constant position or older alignment snapshot is converted into continuous truth.

## Conditional empirical prerequisite, never authority

`derive_profile(protocol, derivation_samples)` returns an immutable `EmpiricalProfile`. `evaluate_profile(profile, heldout_samples)` returns hash-bound candidate and evaluation payloads. `empirical_trial_refusals(profile_payload, evaluation_payload, *, position_variance_max, velocity_variance_max, velocity_error_mps, position_error_m, tracking_error_m)` recomputes the evaluation from the exact embedded observed rows, validates hashes, and compares the exact Envelope values. Position uses the enclosing radius of the per-axis error box. No trusted PASS label, calibrated boolean, k-sigma conversion, or source-classification string grants eligibility.

Empty conditional prerequisite refusals are not operational permission or hard bounds. Genuine untouched/blinded evidence, instantaneous velocity coverage, justified tracking evidence, finite rationale and immutable applicability must all be represented before that prerequisite can be satisfied. Applicability binds `image`, `firmware`, `messages`, `parameter_policy_sha256`, `clock_source`, `rtf_min`, `rtf_max`, `rtf_window_wall_s`, `workload`, `body_point` and `frame`. Canonical scope is `gazebo_sim`, `base_link_origin`, `local_ned`. Runtime independently compares the context to actual readback/auth/config and enforces response, source-health, authorization and exploratory limitations. Recording epochs are not forced equal across independent recordings or future runs.

All reports keep `runtime_use_eligible: false`, `runtime_admission: false` and `deterministic_bound_certified: false`. `trial_profile_eligible` is only conditional sampled empirical consistency. The retained protocol deliberately has no runtime applicability/tracking justification, no untouched holdout and no instantaneous truth velocity. It cannot satisfy runtime prerequisites by changing a target label.

## Runnable commands

In the existing pinned ROS/generated-message decoder environment, with no ROS initialization:

```text
python scripts/sim_profile_consistency.py evaluate --protocol artifacts/sim-repair-round2-work/empirical-protocol-v2.json --output artifacts/sim-repair-round2-work/empirical-consistency.json
```

To freeze a new authorized offline input selection **before** evaluating it, use `sim_profile_consistency.py freeze --recording <path> [--recording <path>] --output <new-protocol.json>`. This freezes identities and hashes only and uses the same complete post-warmup midpoint rule. It never declares a historical trace untouched. All outputs use exclusive creation; do not overwrite earlier protocols/results.

## Paired fault observation normalization

`FaultProtocol` freezes distinct `baseline_trial_id`/`fault_trial_id`, known candidate/evaluation hashes, complete applicability/source/fixture/seed/authorization/scenario bindings, target stage plus actual PX4 nav/arming state, pre/matched windows, injection identity, expected first reasons, finite response and observation-gap deadlines. **Future trace hashes are not pre-trial inputs.** Windows use the explicitly declared observer monotonic-wall origin and must be frozen after observer startup but before the relevant baseline/fault execution/injection; no outcome-dependent trimming is allowed. The post-run CLI manifest binds resulting `Trace.sha256` values after execution.

`ObservedRecord` is a typed normalization of actual source records, with original artifact hash, observer ID, source sequence and monotonic wall timestamp. A normalization process must independently establish actual stage, PX4 mode/arm state, addressed ACK acceptance, observed control eligibility/suppression, injection activation, safety coverage and source/workload/RTF envelope coverage. In particular, `ack_accepted` alone does **not** decode or prove raw ACK command/target-system/target-component/result addressing: original pinned source evidence and correct normalization are mandatory. Hashes and observer labels alone do not authenticate independence. No live collector is implemented here.

Continuous observed channels are `stage`, `px4`, `ack`, `output`, `safety`, and `envelope`, with exact frozen endpoints and no gaps above the declared finite bound. Stage observers must be disjoint from independent observers. The baseline must reach and remain at the real requested target with matching PX4 telemetry and output eligibility. Any previous/current baseline revocation before the matched end remains invalid even if a later window looks healthy. Healthy diagnostic stage reasons matching the stage (case-insensitive, e.g. active/PRIMING/offboard_disarmed) are not themselves revocation faults. A disarmed baseline never substitutes for an active armed target.

The fault run must independently reach the same healthy stage before injection. Missing injection proof, unrelated masking faults, coverage/envelope/source failures, mismatched context or invalid baseline yield INVALID_TEST/NOT_EVALUATED, never PASS. For a valid controlled trial, wrong/absent first reason, late revoke, output after the first observed revoke, authority re-entry, or missed/lost suppression yields FAIL. A response deadline is not permission to continue output after observed revocation. Unsafe records are permanently latched independently of validity and execution errors. Execution and safety axes remain separate; the classifier never manufactures SAFE_ABORT.

The fault CLI is:

```text
python scripts/classify_fault_trial.py --manifest <paired-observed-manifest.json> --output <new-classification.json>
```

Manifest schema 1 contains `profile` and `evaluation` file paths, `protocols` (serialized FaultProtocol list), `trials` (each with protocol_sha256, baseline/fault trace paths and post-run baseline_sha256/fault_sha256), and `source_artifacts` mapping original SHA-256 to retained artifact path. Source artifacts include original observations, blinding/tracking evidence where claimed. Unattempted planned trials and missing/malformed attempted trace files remain counted INVALID_TEST/INFRA_ERROR; an independently decoded side's unsafe evidence survives the other side's failure. Missing/hash-mismatched artifacts invalidate instead of disappearing from the denominator. Counts include planned, valid, passed, failed, invalid and unsafe attempts. Exit 1 preserves known unsafe/task failure, 2 represents invalid/incomplete evidence, and 0 requires the complete classification gate.

No healthy runtime baseline, causal injection, addressed ACK/output observation, justified runtime applicability, instantaneous truth-velocity, independently justified tracking/stopping bounds, or operational approval is supplied by these retained static disarmed recordings. Those are explicit missing evidence, not implementation success claims.
