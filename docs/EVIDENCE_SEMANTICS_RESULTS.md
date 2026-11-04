# Evidence semantics and stationary truth-relative TTL results

## Scope

The approved `prep-evidence-semantics` branch follows `prep-replay-integrity`, `prep-model-profile` and `prep-ttl-replay`. Its output is conservative evidence handling and stationary diagnostics, not calibration or navigation acceptance. The supplemental project checker prevents completed G2 tasks until this preparation task is done. The native API did not edit existing dependency edges; no G1 task is blocked by this preparation branch.

Evidence is under [`replay/evidence-semantics-01/`](replay/evidence-semantics-01/). Original bags, failed attempts, grading thresholds, original ROIs, source PDF and earlier published bundles remain unchanged. The same pinned image, simulation clock, corrected optical mount and complete 42-component physical profile were used. No control publisher or arming command was used.

## Finite near evidence

Finite positive depth below the 0.3 m trusted near bound retains the entire possible occupied pixel cone from optical Z zero through 0.3 m. Grouped cells enclose every contributing pixel's corner rays. The capture-time full body rotation and optical lever arm transform the cone; declared translation/rotation uncertainty and fixed margin inflate it before query intersection. Conservative outer boxes may block clear space.

A later clear/unknown frame cannot erase retained intersecting near evidence. A cone outside the query does not block an independently supported query elsewhere. `-Inf` remains globally fail closed because its sensor-specific meaning is unverified. Epoch/TTL expiry removes evidence, never establishes free space. Source-time nanosecond comparisons preserve the inclusive TTL boundary without floating-point subtraction artifacts.

Synthetic rest height is derived from the complete support profile: body origin **0.2275995 m** above the floor. The former **0.15 m** posed-camera fixture is retained only as a historical reference. Near-floor observations are not exempted or deleted to obtain an ALLOWED result.

## Long acquisition and retained failure

The interrupted `support-05-ttl` attempt was recovered from its surviving container. It was incomplete at **59.750299286 s wall**: `SOURCE_AHEAD_OF_CLOCK` rejected odometry at 24.532 s while delivered node `/clock` remained at 24.424 s. This is **108 ms**, beyond the unchanged **100 ms** acquisition lead allowance. Truth and odometry continued during the clock-delivery interruption. Its safety/execution classification is `UNKNOWN` / `INFRA_ERROR`; orchestration `complete=true` does not turn recorder/replay exit 2 into a pass.

The follow-up `support-06` isolated clock bridging from image/CameraInfo bridging in separate processes, retaining the exact one-way topic mappings and all health limits. This addresses shared-process coupling; the raw failed trace alone does not identify serialization versus transport as the precise internal stall. The runner also terminates run-owned process groups because ROS launch wrappers can leave bridge children alive. After the new run, only the container's sleep process remained.

The frozen follow-up collected **120.012917737 s wall**, spanning **18.412 s sim after warmup**. It contains 452 depth images, 581 CameraInfo, 4,797 clock, 1,893 odometry, 1,305 truth and 38 status messages, plus static TF. Every observed status was Disarmed. Recorder and static-oracle replay both exited 0: `NO_VIOLATION_OBSERVED` / `COMPLETED`, with complete sampled truth coverage. This is not an inter-sample swept-volume certificate.

## Absolute and all-pairs TTL error

The actual pinned ROS decoder accepted **433/433** post-warmup image/pose pairs. Every one of **1,831 recorded PX4 query timestamps** in the frozen image-span population had truth brackets. The diagnostic evaluated **315,712 capture/query pairs** with age between zero and **10 s**, including an observed pair at exactly 10 s. It records epoch/sequence/stamps, worst-pair transforms, individual bracket endpoints/gaps/fractions and rejected coverage. Continuous query times are not certified.

For column-vector transforms:

```text
A_hat = inverse(T_hat_q) @ T_hat_c
A     = inverse(T_q) @ T_c
E     = inverse(A_hat) @ A
point displacement <= norm(delta_t) + 2*sin(delta_theta/2)*rho
```

Observed maxima:

| Quantity | Maximum |
| --- | ---: |
| Absolute translation | 0.068006654 m |
| Absolute rotation | 0.001554186 rad |
| TTL relative translation | 0.072242374 m |
| TTL full-quaternion rotation | 0.001856750 rad |
| Pairwise propagated point-displacement expression | 0.086990396 m |

These are maxima over this stationary sampled population, **not certified future-motion bounds**. No uncertainty parameter was fitted to admit a command. The 10 m trusted optical-Z cap corresponds to corner ray range **13.632203839 m**, optical lever-arm length **0.133951623 m** and body-origin radius **13.737999458 m**. The renderer's 19.1 m far clip gives corner range **26.037509333 m**; it does not extend the profile's trusted sensing range. Intrinsics, cone boundaries, rotation and lever-arm inputs are retained in `truth.json`.

## Independent temporal derivatives

`temporal.json` preserves the source recording identity. It does not rewrite or claim new sensor captures.

- The image-only dropout covers **16.052–27.052 s sim**, an 11 s gap exceeding the 10 s TTL. Unmodified clock and odometry have 2,750/1,100 samples progressing through the gap.
- An independent integer event-sweep oracle, keyed by stamp/epoch/sequence, matches every retained identity. Queries retain 26 frames before the gap, zero frames after all old evidence expires in the gap (`NO_FRESH_FRAMES`), and exactly one valid recovery frame (`UNSEEN_VOLUME`).
- The explicit early-only 3×3 pixel obstacle derivative blocks its tiny probe before expiry: contributor sequence **19**, with 243 retained frames. After expiry, its contributor set is empty while 242 fresh frames remain; the probe becomes ALLOWED. This probe is not a vehicle envelope, and the result is not movement authority.

Historical and complete-model real-replay envelope comparisons both remain STOP (`OBSTACLE_IN_ENVELOPE`). The complete-model diagnostic includes the 0.5 m navigation margin once and observed stationary uncertainty, while calibrated pose/tracking bounds remain null. Reasons, envelopes, sensor uncertainty and FREE/OBSTACLE/OCCLUDED/UNSEEN counts are retained.

## Primary geometry failure is retained

Strict geometry replay exited **2** despite valid provenance, 433 paired frames and all six fault variants passing. `all_target_frames_pass=false` remains visible. The added y-normal side box occludes the old off-axis target; primary ROIs, 3-pixel erosion, minimum support 0.95, p95 limit 0.10 m and maximum limit 0.20 m were not changed.

An independent segment-plane check uses the first true optical origin and the off-axis target center. The line crosses y=0.9 at **(2.940117858, 0.9, 0.442064219) m**, inside the frozen side box before reaching the original target. This explains why off-axis depth does not measure the original plane. `occlusion-crosscheck.json` retains that calculation.

| Original or additional target | Minimum estimator-ROI support | Worst p95/max geometry error |
| --- | ---: | ---: |
| Front | 0.971831 | 0.028972 / 0.029066 m |
| Off-axis | 1.0 | 2.088866 / 2.128266 m |
| Floor | 1.0 | 0.058994 / 0.059004 m |
| Additional side | 0.931520 | 0.041767 / 1.559039 m |

`planes.json` uses the untouched real grader and separately reports estimator-selected versus truth-projected ROI coverage/exclusions. Direct truth-plane residuals remain tiny for the front but remain about 2.08 m for the occluded off-axis plane. Truth correction cannot remove occlusion. Complete truth coverage means the diagnostic ran, **not that the primary geometry grader passed**. No auxiliary ROI replaces the primary result.

## Frozen synthetic checks

The unchanged seed **11** was exercised for **400 attempted worlds in each of four declared configurations**. Every attempted world, geometry, approach rejection, capture-noise seed and decision is retained in `frozen-sweep.json`; no failure or seed was replaced.

| XYZ/yaw noise sigma | Envelope pose allowance | Eligible / rejected approach | False admissions | Conservative blocks |
| --- | ---: | ---: | ---: | ---: |
| None | 0.05 m | 266 / 134 | 0 | 21 |
| 0.02 m / 0.5 deg | 0.05 m | 266 / 134 | 0 | 43 |
| 0.05 m / 1.5 deg | 0.05 m | 266 / 134 | 0 | 70 |
| 0.05 m / 1.5 deg | 0.15 m | 250 / 150 | 0 | 61 |

The 0.15 m configuration was declared before execution, not selected after a failure. It changes the evaluated approach population, so its denominator is reported separately. Zero observed false admissions in this finite synthetic population is not a safety proof.

Complete-profile liveness exercised **96 cases** with profile-derived rest height and the actual stationary truth-error diagnostic. For a straight camera, the complete-model diagnostic first supports an approach at tested **2.5 m** and a level-only history at tested **3.0 m**. At 20-degree downward pitch neither passes on the tested grid. Historical-envelope/corrected-mount comparison passes at tested 1.0/1.5 m for straight pitch and 2.0/2.0 m for downward pitch. These are first tested passing points, not continuous minimum distances. `liveness.json` retains every reason and unsupported assumption.

The final offline suite passed **502 tests in 121.50 s**. The exact-deadline authorization regression asserts revocation/output suppression rather than incidental error wording; the epoch-isolation fixture uses a sampling spacing appropriate to its tiny probe. Source ZIP CRC/full hash readback and both complete/incomplete raw bag archive readback also passed. These checks do not change the primary geometry failure or complete supervisor runtime, flight or calibration.

## Reproduction

Inside the existing pinned decoder image, preserve the repository layout and use `PYTHONPATH=src:spike/freespace`. Extract `support-06.tar.gz` into a fresh directory. With `R` pointing to its `run/recording`, and fresh output filenames:

```sh
python3 spike/freespace/replay_semantics.py --recording "$R" --profile "$R/oracle_profile.json" --mode truth --ttl-s 10 --output truth-new.json
python3 spike/freespace/replay_semantics.py --recording "$R" --mode temporal --ttl-s 10 --output temporal-new.json
python3 spike/freespace/replay_semantics.py --recording "$R" --mode planes --output planes-new.json
python3 spike/freespace/replay_bag.py --recording "$R" --output geometry-new
```

The final command is expected to retain the documented primary geometry failure, not return a geometry pass. Temporal mode also supports a strict schema-three long bag; truth/plane modes require schema four. None permits legacy provenance upgrades or wall-clock substitution.
