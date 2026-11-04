# Pinned PX4 conditional policy contract

## Scope and authority

`src/drone_cv/px4_policy.py` contains pure, frozen geometry and parameter compatibility helpers. `src/drone_cv/px4_failsafe.py` performs bounded **read-only** local parameter/version queries and retains terminal observations. Neither module starts ROS, constructs a publisher, sets parameters, changes mode, arms, or certifies a safe abort. This contract is restricted to the pinned exploratory simulation; it is not hardware authorization.

A successful compatibility check is not a measured response bound. Source labels, provenance strings, a configured action, or a numeric box cannot establish physical clearance or calibration. The runtime consumer must also require its independent empirical applicability/response evidence and current valid telemetry. No runtime profile, landing survey, or flight result is supplied by this change.

All source links below are fixed at firmware [`61907962c9220437ec01a98fa26bfbf5cbb254b5`](https://github.com/PX4/PX4-Autopilot/tree/61907962c9220437ec01a98fa26bfbf5cbb254b5). A different firmware requires a new source review, not an alias to this contract.

## Complete parameter inventory

The authoritative code inventory is `PARAMETER_DOMAINS` / `REQUIRED_PARAMETERS`. Policy keys must match it exactly. The JSON/Python policy must use integer values for native `int32`, enum, bitmask and boolean-backed parameters, and float values for native float parameters. Booleans are not accepted as integers. No silent coercion, extra key, omitted parameter, or arbitrary numeric enum between sparse values is accepted. Source metadata domains are distinct from the narrower admitted action policy.

### Commander source domains

Source: [`commander_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/commander/commander_params.yaml).

| Parameter | Native representation / complete domain | Restricted policy |
| --- | --- | --- |
| `COM_OF_LOSS_T` | float, [0, 60] s | positive; frozen per trial |
| `COM_OBL_RC_ACT` | int32 enum: 0 Position, 1 Altitude, 2 Stabilized, 3 Return, 4 Land, 5 Hold, 6 Terminate, 7 Disarm | **5 Hold** |
| `COM_FAIL_ACT_T` | float, [0, 25] s | **0.0**, no intentional delay |
| `COM_RC_IN_MODE` | int32 enum: 0 RC, 1 MAVLink, 2 fallback, 3 keep first, 4 disable manual control, 5 RC then MAVLink low instance, 6 MAVLink low instance then RC, 7 RC then MAVLink high instance, 8 MAVLink high instance then RC | **4**, explicitly headless exploratory profile |
| `COM_RC_LOSS_T` | float, [0, 35] s | domain-valid, frozen |
| `COM_RCL_EXCEPT` | int32 bitmask [0, 31]: Mission, Auto, Offboard, External, Altitude Cruise bits | **0**, no exception mask |
| `COM_DLL_EXCEPT` | same bitmask/domain as above | **0**, no exception mask |
| `NAV_RCL_ACT` | int32 enum {1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm, 7 Hold without failsafe} | **1 Hold** |
| `NAV_DLL_ACT` | int32 enum {0 Disabled, 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm} | **1 Hold** |
| `COM_DL_LOSS_T` | int32 [5, 300] s | domain-valid, frozen |
| `COM_LOW_BAT_ACT` | int32 enum {0 Warning, 2 Land, 3 Return-critical/Land-emergency, 4 Return-critical/Terminate-emergency} | **2 Land**, not disabled |

`COM_LOW_BAT_ACT` has **no Hold enum** in this source. In particular, 1 is not a domain-valid substitute. This restricted profile preserves critical/emergency battery Land protection and therefore requires a nonmissing independently justified known-clear landing context. If that context is unavailable, admission is refused; warning-only is not a workaround.

The [actual loss-selection implementation](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/commander/failsafe/failsafe.cpp) skips manual-control-loss checking when `COM_RC_IN_MODE=4`. This does not create a manual pilot or change the frozen fallback action/exception mask. It does not suppress GCS connection-loss checking during armed Offboard operation: admission needs an actually observed healthy GCS connection. No link is invented from configuration.

### Battery, fence and clock domains

Battery source: [`src/lib/battery/module.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/lib/battery/module.yaml).

| Parameter | Native domain | Additional condition |
| --- | --- | --- |
| `BAT_LOW_THR` | float [.12, .5] | above critical |
| `BAT_CRIT_THR` | float [.05, .5] | between emergency and low |
| `BAT_EMERGEN_THR` | float [.03, .5] | below critical |

Threshold ordering must survive the full comparison uncertainty intervals, not just ordering of the expected decimal values. Current battery connection, warning/fault/remaining state and health are independently checked by the runtime consumer; parameter agreement cannot replace them.

Fence sources: [`geofence_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/navigator/geofence_params.yaml), [`geofence.cpp`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/navigator/geofence.cpp).

| Parameter | Native domain | Restricted policy |
| --- | --- | --- |
| `GF_ACTION` | int32 enum {0 None, 1 Warning, 2 Hold, 3 Return, 4 Terminate, 5 Land} | **2 Hold** |
| `GF_SOURCE` | int32 enum {0 global-position estimate, 1 GPS} | **0** |
| `GF_MAX_HOR_DIST` | float [0, 10000] m | enabled positive Home-centered radius |
| `GF_MAX_VER_DIST` | float [0, 10000] m | enabled positive height **above Home** |

Actual source enabling uses `> FLT_EPSILON`; the entire possible comparison interval must remain enabled. Actual containment is strict (`distance < limit`). The vertical limit is an upper altitude ceiling, **not** a lower floor and not a symmetric absolute vertical-distance fence. Additional uploaded polygon/circle fences and their validity are not certified by these two parameters; unsupported simultaneous fence behavior must not be admitted as if this inventory proves it harmless.

Clock source: [`uxrce_dds_client/module.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/uxrce_dds_client/module.yaml). `UXRCE_DDS_SYNCT` is boolean-backed int32 {0, 1}; this profile requires **0**, preserving Gazebo simulation-time stamps rather than synchronizing to agent OS time. ROS consumers still require `use_sim_time=true` and independent progress/rate/freshness checks.

### Motion and braking-related domains

Sources:

- [`multicopter_position_control_limits_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/mc_pos_control/multicopter_position_control_limits_params.yaml): `MPC_XY_VEL_MAX` float [0, 20] m/s; `MPC_Z_VEL_MAX_UP` float [.5, 8] m/s; `MPC_Z_VEL_MAX_DN` float [.5, 4] m/s.
- [`multicopter_position_mode_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/mc_pos_control/multicopter_position_mode_params.yaml): `MPC_POS_MODE` int32 enum {0 direct velocity, 4 acceleration based}; `MPC_ACC_HOR_MAX` float [2, 15] m/s²; `MPC_JERK_MAX` float [.5, 500] m/s³. The source explicitly says `MPC_ACC_HOR_MAX` limits deceleration for mode 0 and is unused for mode 4.
- [`multicopter_autonomous_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/mc_pos_control/multicopter_autonomous_params.yaml): `MPC_ACC_HOR` float [2, 15] m/s²; `MPC_JERK_AUTO` float [1, 80] m/s³.
- [`multicopter_altitude_mode_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/mc_pos_control/multicopter_altitude_mode_params.yaml): `MPC_ACC_UP_MAX` and `MPC_ACC_DOWN_MAX`, both float [2, 15] m/s².

`validate_motion_policy` requires

`hypot(upper(MPC_XY_VEL_MAX), max(upper(MPC_Z_VEL_MAX_UP), upper(MPC_Z_VEL_MAX_DN))) <= maximum_speed_mps`.

Here `upper` includes float32 conversion and the pinned comparison uncertainty. Checking each axis alone is insufficient. A declaration exactly equal to the nominal combined cap can fail because no quantization/comparison reserve was provided.

The declared all-direction minimum deceleration must not exceed any relevant configured axis/trajectory acceleration cap's conservative lower bound. This is only an impossibility check. These parameters are commanded **upper** limits; they never prove an actual minimum braking acceleration. Jerk limits do not establish onset/delay or stopping performance either. Independent response/workload evidence, uncertainty/error bounds, bootstrap/calibration ordering and the all-direction stopping enclosure remain necessary. No braking stage or safe-stop assertion is introduced.

[`multicopter_takeoff_land_params.yaml`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/mc_pos_control/multicopter_takeoff_land_params.yaml) was also reviewed: landing has altitude-dependent speed limits and a distance-sensor-dependent crawl phase. Those settings are not inferred as a landing clearance or guaranteed measured descent trajectory. The certified conditional corridor must cover every allowed landing phase; unsupported dynamics remain nonadmitting.

## Frozen Home/frame/landing geometry

The public frozen types are:

- `LocalReference(latitude_deg, longitude_deg, altitude_m, ref_timestamp_us)`.
- `HomeSnapshot(position_ned, latitude_deg, longitude_deg, altitude_m, update_count, local_reference, epoch, reset_counters)`, with reset order **xy, z, vxy, vz**.
- `LandingCorridor(minimum_ned, maximum_ned, support_down_m, support_plane_down_m, body_provenance, physical_provenance, clearance_provenance, frame="local_ned_reference_centers", body_point="base_link_origin")`.
- `PolicyGeometry(home, horizontal_radius_m, height_above_home_m, landing, motion_reserve_m, frame_margin_m, frame_rationale)`.

`validate_geometry_policy(expected, context, *, current_home, minimum_ned, maximum_ned, required_motion_reserve_m, body_provenance)` checks:

1. Exact actual/frozen Home, local-reference origin/timestamp, epoch, update count and reset-counter equality. Missing/invalid current snapshots reject. The runtime consumer independently enforces validity flags, current local-position health/freshness and **sticky** invalidation after any observed change, even if a counter later returns to the original value. Home is an on-change frozen reference; it is not required to publish with odometry cadence.
2. Both cylinder dimensions exactly bind the expected parameter policy. Lower live bounds are used for containment. The whole supervisor geofence is expanded by `motion_reserve_m + frame_margin_m`, not compared as individual homeless scalar extents. Farthest horizontal box-corner distance from actual Home and highest altitude above actual Home must be strictly inside the live outer limits.
3. `motion_reserve_m` covers the separately computed required body/motion/response/error/clearance room. `frame_margin_m` is a positive independently justified bound for local/global projection/reference disagreement. Its rationale must bind those coordinates and the applicable local region; a string alone is not proof of such a bound.
4. A nonmissing body-bound, independently physically justified landing corridor covers the **entire expanded admitted horizontal geofence**, from its highest allowed reference center down through support contact. A small landing pad beneath only the initial position is insufficient.

`support_down_m` is the base-origin down coordinate **at contact**, not ground height. `support_plane_down_m` is the physical support plane, below that origin. The corridor is already eroded by complete body/rotor footprint, admitted attitude/yaw envelope, contact geometry and all phase clearances. Bounds cannot infer support truth, exclude overhangs, or guarantee no yaw-induced intersection; the independent provenance must justify these conditions. No invented floor, body offset or clearance is supplied.

Current Hold capability, position/reference/home health, battery health, relevant failsafe flags, timing and no unsupported escalation are additional admission conditions owned by runtime. In the actual [`failsafe/framework.cpp`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/modules/commander/failsafe/framework.cpp), `Action::Hold` is excluded from `COM_FAIL_ACT_T` delayed-action handling. If Hold cannot run, the availability switch can escalate through RTL, Land, Descend and Terminate. Therefore **these Hold configuration values do not guarantee that all failsafes stay in Hold**, nor does initial health guarantee survival of a later structural failure after process death. Unsupported escalation paths remain nonadmitting, not reclassified as safe.

## Precision and readback attestation

The pinned [`systemcmds/param/param.cpp`](https://github.com/PX4/PX4-Autopilot/blob/61907962c9220437ec01a98fa26bfbf5cbb254b5/src/systemcmds/param/param.cpp) quiet display prints floats using `%4.4f`. Its comparison parses with `strtod`, converts the target to binary32 and checks `fabsf(live_float - target_float) < 1e-7f`; integers compare exactly.

Consequently `.333` and `.33333` can both be comparison-attested, although their quiet texts are `0.3330` and `0.3333`. Quiet `0.3333` is **not** exact live `.33333`, and it must not be bound as if it were. Conversely, exact equality with quiet text must not reject a successful `.33333` comparison.

Quiet parsing also preserves the sign of `-0.0000` and refuses negative live text for this inventory's nonnegative float domains. A successful comparison to expected zero alone must not admit a tiny negative delay hidden by numeric conversion of that text. The reported binary32 interval remains a conservative comparison bound, not an assertion of exact zero.

`float32_comparison_bounds(expected)` returns the inclusive first/last finite binary32 values accepted by the actual strict comparison, including subtraction rounding. It uses monotonic ordered-float binary search, not an arbitrary `isclose`, decimal rounding or a bare decimal ±1e-7 interval. `conservative_upper_float_parameter_bound(name, expected)` exposes the upper endpoint for response horizons and resultant speed caps. Such a bound is conditional on successful actual readback; calling the pure helper does not produce attestation.

`read_parameters` validates the complete canonical policy first, checks the exact live firmware before/after the bounded sequence, retains quiet text and issues an actual read-only `param compare -s` for every key. Its report contains:

- `expected`, `attested_policy`: copies of the comparison-attested frozen declaration;
- `observed`: **quiet-only diagnostic** numeric values;
- `quiet_observed_text`: the actual retained quiet strings;
- `attestation="pinned_param_compare_s"`;
- `precision[name]`: exact integer bounds, or `type="float32"`, `expected_float32`, inclusive `lower`/`upper`, binary32 `comparison_absolute_limit`, comparison semantics and `quiet_decimal_places=4`;
- instance, firmware, rationale, read wall time and an explicit non-execution-proof scope.

`attested_parameter_policy(report)` checks the full expected/attested inventory, canonical types, quiet consistency and recomputed precision metadata. It is not an authenticity verifier: runtime must use its own fresh `read_parameters` result, not trust a loaded JSON claim. Parameter writers must be prohibited for the trial; bounded sequential readback is not an atomic snapshot or protection against a malicious concurrent operator. No parameter is silently written.

The consumer binds `LossResponse` to this attested policy plus comparison reserve, never to `observed`. Expected `COM_FAIL_ACT_T=0.0` has a small comparison interval; consumers must retain that uncertainty rather than claiming an exact observed zero. Synthetic offline loss-response fixtures may independently retain their declared 0.02 s delay; they are not this exploratory runtime parameter profile.

## Regression scope

`tests/test_px4_policy.py` covers sparse enums/domain/type/inventory refusal, unsupported loss/battery actions, missing/body-mismatched/insufficient landing corridors, wrong Home/origin/epoch/reset, disabled or insufficient outer fences, 3D combined speed with comparison reserve, acceleration-cap-only compatibility and exact binary32 comparison boundaries on both sides. `tests/test_px4_failsafe.py` covers diagnostic quiet text versus comparison-attested `.333`/`.33333`, complete metadata and missing/failed comparisons with synthetic command responses.

These are authored offline regressions. Their execution belongs to the main integrated verification after all delegated changes land. No simulator, ROS initialization, live parameter query, publisher construction, acquisition, mode/arming operation or flight was performed for this delegated implementation.
