# Offline sensing evidence repairs (R5 and R2)

## Scope and authority

This implements the code/offline portion of [the approved repair plan](SIM_REPAIR_PLAN.md), sections R5 and R2. It does not launch a simulator, create a recording, publish control, arm, install software, write parameters, or modify published evidence. It does not depend on completion of G1. The conservative complete physical profile and 0.5 m navigation-clearance contract are unchanged.

The integrated suite passed 648 tests. The actual preflight CLI passed 90 physically admissible sampled poses and retained 153 below-support exclusions from its 243-pose grid. See [current results and exact evidence](SIM_REPAIR_RESULTS.md). These checks do not certify fresh acquisition, calibration or G2. Historical reports under `replay/` retain their original bytes and original meaning.

## Required uncertainty and provenance

`spike/freespace/evidence.py` now requires all five constructor arguments: near translation uncertainty in metres, near rotation uncertainty in radians, `EvidenceSource`, nonempty provenance, and explicit no-return certification. Missing arguments do not produce implicit zeros. Invalid, nonfinite, negative or unsupported inputs reject. The immutable source classifications are:

- `analytic_synthetic`: explicitly generated scene/capture geometry. An exact zero uncertainty is permitted. Only this source may declare the synthetic renderer's positive-infinity no-return semantics.
- `historical_nonadmitting_analysis`: an explicitly assumed comparison, including zero assumptions in old bag/TTL/temporal replays. It cannot certify positive infinity as free space.
- `stationary_measured_diagnostic`: observed stationary truth-relative uncertainty, not a calibrated future-motion bound. It cannot borrow synthetic no-return certification.

There is no operational/calibrated classification or calibration boolean. Every profile's `motion_authority` is false. An offline `decide()` result is a sampled geometry result, not a planner permit. Actual G2 calibration evidence and its binding remain absent.

Bag, TTL, temporal derivatives, truth-relative envelope diagnostics, synthetic experiments, complete-profile liveness, plotting and relevant tests are migrated together. Liveness historical *rendered* comparisons remain explicitly analytic. Its stationary diagnostic uses measured translation/rotation maxima and leaves positive-infinity pixels unknown. Therefore a newly generated analysis may differ from an old report; the published report is not regenerated or relabelled.

## Nonoccluding fixture preflight

`spike/freespace/fixture_preflight.py` keeps the original front, off-axis and floor boxes/ROIs and invokes the actual unchanged `grade_frame()` API. It requires the original 3-pixel erosion, 0.95 support, 0.10 m p95 and 0.20 m maximum limits. Changed original target geometry or thresholds reject.

The additional adapter is evaluator-only: it combines the existing `scene.render_depth()` result with a finite, two-sided vertical rectangle. It does **not** add a partially supported primitive to `Scene` or the planner's collision grader. The declared candidate has:

- World ENU center `(2.5, -1.2, 0.55)` m.
- Unit normal `(1/sqrt(2), 1/sqrt(2), 0)` and horizontal tangent `(-1/sqrt(2), 1/sqrt(2), 0)`.
- Physical half-width 0.50 m along the tangent and half-height 0.45 m vertically.
- Separate interior ROI half-width 0.25 m and half-height 0.25 m, before unchanged 3-pixel image erosion.

These are candidate geometry values, not an acquisition approval or a claim that 45 degrees is already a successful physical fixture. The mathematical finite surface is the preflight representation; a later simulator target must match its finite visible surface and must not introduce un-preflighted thickness/occluders.

Independent identity checks intersect convex face polygons by oriented edge half spaces, separately from the renderer's box slab tests and auxiliary normal/local-coordinate intersection. They determine the first physical target on every included ROI ray. Finite depth alone is insufficient: every primary and auxiliary ROI needs at least 0.95 **correct-target** support as well as the unchanged geometric limits. Original primary pixel counts are cross-checked against the real grader.

The original two-box/floor fixture is the independently visible positive control. The retained historical side box `(1.8, 0.9, 0.2)` to `(3.0, 1.1, 0.9)` is an expected failing control, including its own original side ROI. The independent off-axis-center ray check has the known y=0.9 crossing before the off-axis target. No positive control borrows a widened ROI or weaker threshold.

The nominal pose uses complete-profile gear support height and the profile's optical mount, at original 640x480 intrinsics. The deterministic sampled grid is the Cartesian product of:

- All 27 XYZ translations with each component in `{-0.07, 0, +0.07}` m.
- All eight simultaneous RPY corners with each component in `{-0.1, +0.1}` degrees, plus zero RPY.

Thus 243 poses are proposed. Every outer AABB corner of every complete physical-profile part is transformed at each pose. Minimum physical z below -1e-12 m excludes that pose as `PHYSICAL_PROFILE_BELOW_SUPPORT`; such exclusions are recorded and never counted as passing acquisitions. The tolerance handles floating arithmetic at exact support only. The report retains every included result and every exclusion. This finite grid is **not** proof for every intermediate translation/rotation.

The CLI emits non-null fixture/world/source identities only after the positive control passes, the old occluding control fails, and every physically included candidate pose passes. IDs bind geometry, original contract, full profile and implementation hashes. A failed preflight retains the candidate and all failures with null frozen IDs; it must not trigger collection. Output uses exclusive creation and does not overwrite previous evidence.

Run once after integration with a fresh output filename:

```sh
python spike/freespace/fixture_preflight.py --profile docs/replay/evidence-semantics-01/profile.json --output artifacts/sim-repair-fixture-preflight.json
```

Exit 0 means the declared **offline sampled preflight** passed; exit 2 preserves a failed preflight. Neither exit authorizes capture or completes the fresh primary-baseline requirement.

## Independent dy equation and explicit blockers

`spike/freespace/dy_crosscheck.py` is a pure evaluator-only calculation. For each plane, measured signed point residual `e = n dot (p_est - p_plane)` obeys:

```text
delta = truth translation - estimated translation
nu = signed rotation + depth + extrinsic contribution
e = -n dot delta + nu
r = -e + nu
r_aux = n_x * dx + n_y * dy + n_z * dz
dx = r_front                  # original +X front plane
dz = r_floor                  # original +Z floor plane
dy = (r_aux - n_x*r_front - n_z*r_floor) / n_y
```

Rotation is about the body origin and includes the camera lever arm. Optical-Z depth perturbation maps through the estimated body/camera ray rotation. Extrinsic origin/orientation errors also contribute to the signed point residual; they cannot be silently set to zero as a physical claim. Corrections must be supplied explicitly, ordered auxiliary/front/floor. Explicit zero correction means an **uncorrected diagnostic**, not zero physical nuisance error. Same-bag truth affects only the final comparison, never dy estimation, correction fitting or alignment.

`DyErrorBudget` requires a numerical minimum absolute y normal, three plane-residual bounds and separately derived combined-numerator rotation, depth and extrinsic bounds with nonempty provenance. The propagated dy bound weights front/floor residuals by absolute normal components, sums the combined-numerator nuisance bounds, then divides by `abs(n_y)`. Exact zero bounds occur only in analytic equation tests; no numerical physical budget is proposed here. A declared budget is not a calibration certificate, and `g2_acceptance` remains false even when a diagnostic comparison is within it.

Exactly zero y normal returns `Y_TRANSLATION_UNIDENTIFIABLE`; insufficient declared conditioning, missing same-bag truth, or an exceeded bound produce distinct failures. Missing numerical justification returns `MISSING_JUSTIFIED_NUMERICAL_DY_BUDGET`, retaining a finite diagnostic estimate when calculable without claiming acceptance.

The existing `replay_semantics.py --mode planes` path now includes per-frame independent dy diagnostics for the historical axis-aligned side target when front/floor/side residuals exist. It supplies explicit zero nuisance corrections and **no physical budget**, and preserves failed original/auxiliary grading as ineligible geometry. A missing required residual is a separate explicit failure. The old occluded side fixture cannot become a valid y crosscheck because a number was calculated.

Inside the already available pinned ROS decoder environment, with `PYTHONPATH=src:spike/freespace`, extract the immutable `docs/replay/evidence-semantics-01/support-06.tar.gz` into a fresh location and set `R` to its `run/recording` directory. Then use a new output:

```sh
python3 spike/freespace/replay_semantics.py --recording "$R" --mode planes --output planes-sim-repair-new.json
```

This is offline decoding only. Complete diagnostic coverage is not a primary-geometry or dy-budget pass; retain each row's failures. No installation or new simulator execution is authorized by this command.

## Integration checks and remaining prerequisites

Focused offline regressions (not executed by the delegated implementation worker):

```sh
python -m pytest -q spike/freespace/tests/test_near_evidence.py spike/freespace/tests/test_freespace.py spike/freespace/tests/test_truth_error.py spike/freespace/tests/test_model_profile.py spike/freespace/tests/test_fixture_preflight.py spike/freespace/tests/test_dy_crosscheck.py
```

The tests cover explicit provenance/zero/missing/invalid inputs, nonanalytic no-return rejection, immutable profiles, original near-cone/capture/lever-arm/epoch/TTL retention, visible and occluded target identities, unchanged ROIs/thresholds, finite extent versus interior ROI, deterministic support exclusions, signed dx/dz elimination, nuisance signs, y identifiability, amplification and missing budgets. The CLI exercises the complete declared candidate grid separately.

Offline preflight is now exercised and passed its included sampled poses. G2 still requires a justified residual/conditioning/rotation/depth/extrinsic numerical budget frozen **before** collection; explicit separate execution approval; a fresh unarmed pinned acquisition of the matching target geometry; unchanged original primary target checks passing on that recording; and per-frame independent auxiliary dy versus same-bag truth with adequate identity/support and error coverage. There is no fresh recording, frozen physical error budget, primary-baseline pass, flight acceptance or calibrated permit in this implementation.
