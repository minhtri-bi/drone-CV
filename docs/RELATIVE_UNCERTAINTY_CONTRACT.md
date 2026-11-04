# Capture-to-query relative uncertainty (F5)

This is a CODE/OFFLINE evidence contract. It creates no publisher, motion authority,
calibration certificate, instantaneous truth twist, or continuous-time error bound.
Historical absolute-profile v2 artifacts are unchanged.

## Two different error questions

`Envelope.position_error_m` remains an **absolute** local-NED base-link-origin
position-error assumption for bootstrap known-clearance and geofence containment.
An absolute bound at capture and another at query are not automatically a bound
of the same size on their relative transform. A common fixed world bias cancels;
opposite 0.04 m errors can each satisfy 0.05 m while producing 0.08 m relative error.

For estimated poses `That_c`, `That_q` and independent truth poses `T_c`, `T_q`,
all represented as world-from-base-link-FLU transforms, the evaluator uses:

```text
Ahat = inverse(That_q) * That_c
A    = inverse(T_q) * T_c
E    = inverse(Ahat) * A
reserve = translation_bound + 2 * sin(rotation_bound / 2) * rho
```

`E` acts on capture-body-FLU points. This is an explicitly identified basis, not
an unlabelled local-NED component error. Since `A = Ahat * E`, the norm of the
actual point displacement in query-body coordinates equals `||E*p - p||`.
The subsequent proper rotation into query-local-NED preserves that norm. Thus
`rho` must enclose **every relevant capture-body point**, including lever arms and
any producer-supported physical footprint; optical Z alone is not Euclidean range.
For a principal angle in `[0, pi]`, the reserve bounds the entire radius-`rho`
point ball. A larger `translation + rotation*rho` reserve is also reported.

Relative reserve must be applied once to the perception-derived geometric claim.
Absolute uncertainty may still be needed independently for geofence/known-world
containment. Do not sum two absolute bounds for the same interval and then add a
relative bound for that identical interval without a separate uncertainty rationale.

## Frozen value and verifier

`src/drone_cv/relative_profile.py` defines the frozen value:

```python
CaptureQueryBound(
    max_age_s, translation_m, rotation_rad, rho_m,
    source_sha256, evaluation_sha256, provenance_sha256, model_sha256,
    body_point, clock_source, clock_epoch, reset_counter,
    basis="capture_body_flu", conditional_model_scope_sha256="",
)
```

Numerical values are finite, age/radius positive, errors nonnegative, and the
rotation is a principal angle. `clock_epoch` is the runtime's exact nonempty
**string**, while historical `Binding.clock_epoch` remains an analysis-only integer.
A future trial freezes its own string epoch/reset: it need not equal an old bag's
integer epoch or reset counter. Capture, query, and bound must match each other
within that new trial. A reset cannot revive an old bound merely by wrapping its
8-bit counter; the runtime separately owns sticky epoch invalidation.

- `bound.displacement_m` computes the whole-point reserve above.
- `bound.sha256` is canonical `digest(asdict(bound))`.
- `source_sha256` binds the sorted **analysis manifest source set**.
- `evaluation_sha256` binds the recomputed complete finite pair evaluation.
- `model_sha256` and `provenance_sha256` identify the model and declared
  producer/calibration/frame/clock contract, excluding per-run world alignment.
- `conditional_model_scope_sha256`, when supplied, binds an **externally declared**
  conditional timing/intersample/calibration model. The evaluator does not certify
  that declaration. Its absence remains a refusal for conditional trial use.
- `capture_source_sha256` and `query_source_sha256` are computed properties, not
  bag-manifest or individual CDR hashes. They are `stream_source_sha256(role,
  model_sha256, provenance_sha256, body_point, clock_source, basis)`, using canonical
  JSON keys with exactly those names and role `capture` or `query`.

The authoritative conditional prerequisite is:

```python
verify_capture_query_bound(
    bound, profile_payload, evaluation_payload,
    permit_age_s=..., max_rho_m=...,
    capture_clock_epoch=..., query_clock_epoch=...,
    capture_reset_counter=..., query_reset_counter=...,
    model_sha256=..., body_point=..., clock_source=..., provenance_sha256=...,
    capture_source_sha256=..., query_source_sha256=...,
) -> tuple[str, ...]
```

It reconstructs the typed protocol/observations, recomputes all pairs and coverage,
and compares the **entire** evaluation, not a claimed `passed`/`calibrated` flag.
It checks the frozen claim, evidence identities, age/rho, model, provenance, body,
clock, stream contract identities, and separately frozen runtime epoch/reset.
An empty refusal tuple can satisfy only this conditional evidence-consistency
prerequisite when actual complete independent heldout data and an explicit external
model scope are present. It never grants operational motion or certifies calibration.
The output fields `runtime_admission`, `calibration_established`,
`deterministic_bound_certified`, and `continuous_time_bound_certified` remain false.

## Frozen retrospective protocol and populations

`artifacts/sim-repair-round3-work/relative-protocol.json` was frozen from the same
input aliases and exact source files as the v2 absolute analysis, **before relative
evaluation**. Freeze hashes metadata, file inventories, source identities, model,
calibration declaration, alignment/body point, resets, and clock scope. It reads no
estimator/truth outcome values. All post-warmup depth capture and PX4 query identities
are included, not only previously usable rows:

| Source | Captures | Frozen query identities | Role |
| --- | ---: | ---: | --- |
| schema-4 support06 | 433 | 1840 | Entire derivation source |
| schema-2 r39 | 136 | 588 | Entire heldout source, continuous truth unavailable |

The previous 1831-query usable support06 population must not silently replace the
1840 frozen identities. Decode/truth/pose rejections and missing coverage remain
in the evaluated population. No schema-two truth is synthesized or upgraded.
The source-disjoint heldout is intentionally reported honestly as uncovered, not
as a successful independent relative comparison. Separate deterministic midpoint
**same-source temporal diagnostics** are included for descriptive support06
comparison; they exclude cross-boundary pairs and are explicitly not independent
source validation. Whole-source and whole-source-split reports still retain every
within-TTL pair, including temporal-boundary pairs in their whole-source population.

The frozen TTL is **0.5 s**, from the unchanged synthetic command/observation
maximum, not the old 10 s pair result. The 30 m `rho` is a declared diagnostic
capture-body point ball covering the pinned 19.1 m optical-Z outer cone plus mount;
it is not a calibrated operational sensor/airframe bound. Translation 0.05 m and
rotation 0.01 rad are frozen hypothetical diagnostic claims, not accepted runtime
limits and not fitted to heldout outcomes. Current sources were already examined;
no retrospective hash can make them untouched or prospectively blinded.

Every actual capture/query identity pair satisfying `0 <= query-capture <= TTL`
is evaluated within its source and source split; there is no adjacent-frame shortcut.
Reports preserve every violation, missing/rejected pair, uncovered/unexpected
observation, query without a retained capture, exact pair/population counts, worst
pair identities, signed axes, absolute NED errors, rotation/lever-arm displacement,
and fixed age-bin summaries. Absolute age-bin metrics are explicitly pair-weighted.
Missing metrics are JSON null rather than infinity or fictitious zero coverage.
Strict pinned decoders, sticky alignment/reset validation, capture interpolation,
independent body truth, and no-extrapolation truth brackets are reused. Existing
`truth_error.all_pairs`, `relative_error`, and `transform_error` independently
cross-check all usable whole-source pairs and worst-pair values.

## Commands and evidence limits

The metadata-only freeze already executed; do not overwrite or refreeze this file
in response to evaluation outcomes. The original freeze command was:

```console
python scripts/sim_relative_consistency.py freeze --input-aliases artifacts/sim-repair-round2-work/empirical-protocol-v2.json --output artifacts/sim-repair-round3-work/relative-protocol.json --max-age-s 0.5 --rho-m 30 --translation-limit-m 0.05 --rotation-limit-rad 0.01
```

The main agent runs the real pinned-decoder evaluation separately:

```console
python scripts/sim_relative_consistency.py evaluate --protocol artifacts/sim-repair-round3-work/relative-protocol.json --output artifacts/sim-repair-round3-work/relative-consistency.json
```

Evaluation requires the already pinned offline decoder environment. ROS init,
node construction and publisher construction are guarded out; there is no new
recording, parameter write, arming, simulator launch, or control publisher.
Output creation is exclusive. The evaluation has not been executed by this worker.
Tests, builds, lint and formatters are deliberately left to integrated verification.

Future metadata freezes can specify both `--derivation-source` and
`--heldout-source` manifest hashes, `--holdout-kind untouched_source_disjoint`,
`--blinding-evidence-sha256`, and `--conditional-model-scope-sha256`. Exact source
partitions must be nonempty, disjoint and exhaustive; aliases cannot cross splits.
The known already examined retained hashes cannot be relabelled untouched. Caller
history/evidence hashes do not authenticate themselves or grant independent approval.
The CLI refuses TTLs above the pinned synthetic 0.5 s contract maximum.

Current missing evidence includes independent untouched source-disjoint heldout,
r39 continuous truth, timing/calibration/intersample bounds and an explicit external
conditional model scope, instantaneous independent truth twist, and operational
tracking/dynamics evidence. Pose finite differences are not instantaneous twist.
This module neither relaxes freshness/variance limits nor grants runtime admission.

## Exercised retained result

The [third-round results](SIM_REPAIR_RESULTS.md#third-round-f1-f9-restricted-policy-and-relative-uncertainty) retain the actual frozen protocol, complete compressed report and witnessed execution. Support06 has 21,546 covered pairs over the unchanged 0.5 s age domain, but three frozen queries precede any retained capture. All 6,668 r39 pairs lack continuous truth. Neither previously seen source is untouched validation; no conditional timing/calibration/intersample scope is supplied. The computation exits successfully while sampled coverage and admission remain false.

