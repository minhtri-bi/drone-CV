# Pinned model profile and real rolling-TTL preparation

## Scope and authority

The user authorized both remaining preparation tasks after the acquisition review repairs, then requested a complete next-tab handoff. This work remains unarmed and simulation-only: no control publisher, setpoint, teleportation, `set_pose`, flight or no-return certification. Existing G1/G2 flight/calibration dependencies are unchanged.

This report complements the [repair plan](SENSOR_REPLAY_REPAIR_PLAN.md), [r39 results and explicit error regression](SENSOR_REPLAY_INTEGRITY.md), and immutable packages under `docs/replay/`.

## Final acquisition repair

Schema-three acquisition uses one **1.5 s monotonic-wall clock-progress watchdog**. Image/CameraInfo/odometry/status health is checked against actual node ROS-time snapshots, not wall time scaled by assumed RTF. Simulation-age limits are 0.5/0.5/0.2/1.1 s respectively. Source timestamps may lead the latest delivered node clock by at most the declared **100 ms acquisition allowance**. These are not calibrated flight deadlines or proof of clock synchronization.

The explicit `/clock` callback and the node's internal ROS time subscription can have different queue lag. Their stamps must not be substituted silently. New reports expose source-minus-received-node-clock residuals; storage-minus-capture contains clock delivery lag and is not independent sensor latency. Unavailable DDS publisher history/depth are null, not UNKNOWN/0 measurements.

The bounded 512-sample startup pose buffer is discarded after alignment. Capture-time interpolation/reset validation remains unchanged. The old duplicate fixture clock deadline is removed from new fixtures. The recorder now writes through a **128-item separate-process queue**: backpressure, writer death, missing result, shutdown timeout or count mismatch makes the run incomplete. Writer shutdown has its own 10 s collection limit, not movement authority.

### Failed long attempts preserved

| Attempt | Observed result | Causal correction |
| --- | --- | --- |
| `recording-01-incomplete.tar.gz` | At 62.925 s wall, callback clock lag falsely triggered odometry lead rejection; 13.56 s sim captured | Compare sim ages with the newer node ROS-clock snapshot, not the queued explicit clock callback; keep the 100 ms limit |
| `recording-02-incomplete.tar.gz` | At 65.945 s wall, the synchronous image serialization/storage hot path stopped callback progress long enough to trip the 1.5 s clock watchdog | Isolate disk storage in a bounded process; keep all health limits |
| `recording-03.tar.gz` | Complete: 90.003 s wall, 19.328 s sim, all statuses Disarmed | Final accepted health/TTL input |

The second failed trace's final image callback was followed by approximately 1.65 s without callback progress. It did not instrument serialization and disk separately, so that interval is not claimed as pure disk latency. The isolated final writer reports a maximum individual storage-write time of 0.06051 s. Both failures and their partial raw data remain archived; no timeout was increased to obtain success.

The final recording contains 445 depth, 587 CameraInfo, 4,832 clock, 1,933 odometry, 39 vehicle-status and one static TF message. Mean RTF is 0.21470. Maximum depth gap is **396 ms sim**, larger than the 132 ms in r35/r39. Image loss location is still unknown; no publisher-side count was collected, so no renderer-versus-transport attribution is claimed.

Measured N/E/D odometry spans over the longer trace are **0.061841 / 0.062281 / 0.048546 m**. They further invalidate treating the historical 0.05 m allowance as a demonstrated navigation bound. The observed spans remain diagnostics, not relative-error calibration.

## `prep-model-profile`

### Geometry derived from pinned sources

`model_profile.py` reads the pinned merged X500/OakD model chain, transforms collision box corners into body FLU, and encloses each rotor's complete collision sweep. Source SHA-256 identities are retained alongside the exact SDF inputs.

| Physical collision proxy | Derived value |
| --- | ---: |
| Body-relative lower bound | -0.227000 m |
| Body-relative upper bound | +0.060423 m |
| Radial bound including rotating collision boxes | 0.385945 m |
| Optical translation FLU | (0.132330, 0, 0.020780) m |

This is **collision-derived proxy geometry**, not a claim that visual meshes completely match physical propellers. Visual mesh URIs are enumerated but their bounds remain unresolved. Unknown collision shapes/include chains are rejected; tilted-body diagnostic sweeps are rejected. Contact ledger, landing-pair allowlist, continuous coverage and PX4 tracking remain future G1 work.

The default `Mount` now uses body-relative z=0.02078. The original synthetic regression fixture passes its old 0.26078 mount explicitly. Historical images/results are unchanged. New experiments therefore do not silently inherit the wrong mount.

### Liveness with the mount and envelope together

`model_liveness.py` executed 96 cases: three explicit profiles, straight or 20-degree-down cameras, and climb/scan/approach/level strategies. Tested approach lengths are 1, 1.5, 2, 2.5, 3, 3.5 and 4 m. Results below are **first allowed tested points, not continuous minimum-distance estimates**.

The model navigation diagnostic adds the contract's 0.5 m navigation clearance once to the collision proxy, plus illustrative r39 observed displacement spans: XY=0.053985 m and Z=0.031003 m. Its radius is 0.939930 m and its z band at body height 1.2 m is [0.441997, 1.791426] m. Calibrated pose, angular and tracking bounds remain null; no configuration receives flight admission.

| Profile | Camera pitch | First tested approach coverage | First tested level-only coverage |
| --- | ---: | --- | --- |
| Historical mount/envelope | 0 deg | 1 m | 1.5 m |
| Historical mount/envelope | 20 deg | 1 m | 1.5 m |
| Corrected mount, historical envelope | 0 deg | 1 m | 1.5 m |
| Corrected mount, historical envelope | 20 deg | None on tested grid | 2 m |
| Model/navigation diagnostic | 0 deg | 2.5 m | 2.5 m |
| Model/navigation diagnostic | 20 deg | None on tested grid | None on tested grid |

The low synthetic climb starts the body at 0.15 m; it is a historical posed-camera fixture, not a physically validated takeoff. With the corrected low optical mount and down pitch, early near-floor TOO_CLOSE returns remain retained. The current near-hazard guard therefore blocks those approach cases even when later frames look clear. No floor exception, early-frame deletion or weakened hazard retention was added to make them ALLOWED. The original supplied spike did not retain that hazard in the same way; its old pitch-distance result is not the current corrected implementation's result.

Every sampled decision, reason, effective envelope and unsupported assumption is retained in [`replay/model-profile-r51/`](replay/model-profile-r51/). Synthetic no-return is renderer-certified only; real no-return remains uncertified. ALLOWED here means sampled synthetic coverage, not motion authority.

Reproduce in the existing offline environment with fresh output names:

```sh
python spike/freespace/model_profile.py --models docs/replay/model-profile-r51 --output artifacts/model-profile-new.json
python spike/freespace/model_liveness.py --profile artifacts/model-profile-new.json --observed-report docs/replay/integrity-r39/report.json --output artifacts/model-liveness-new.json
```

## `prep-ttl-replay`

`ttl_fixture.json` fixes 90 s wall, a minimum 15 s sim span, a 10 s simulation TTL and the existing recording size limit before acquisition. World geometry and the simulation were not moved or retuned. The successful trace spans 19.328 s sim; 429 post-warmup images have valid capture-time pairing.

`replay_ttl.py` uses the same strict bag/hash/policy/receive/alignment reader as geometry replay. It processes only the observed prefix at each query, never future depth frames, and records retained sequence IDs and capture timestamps. Its envelope is explicitly the historical stationary diagnostic query: it tests temporal retention, not the model-profile navigation margin or a movement gate.

| Query sim time | Observed frames so far | Expired frames | Retained frames | Result |
| --- | ---: | ---: | ---: | --- |
| 531.584 s | 127 | 0 | 127 | STOP |
| 537.584 s | 260 | 24 | 236 | STOP |
| 545.228 s | 429 | 206 | 223 | STOP |
| 556.228 s, no new capture | 429 | All | 0 | `NO_FRESH_FRAMES`, STOP |

The final in-trace query excludes the first frame while retaining 223 fresh frames. The stale query is an offline query-time advance, not an additional sensor capture. All six temporal checks pass; real `no_return_certified=False` is retained. This proves rolling expiry on real rendered data, not positive A-to-B navigation.

The immutable complete/incomplete bags, actual receive traces, alignment samples and per-query frame identities are under [`replay/ttl-r51/`](replay/ttl-r51/). See the package for the separate long-trace geometry diagnostic; temporal success must not hide a geometry or calibration limit.

Use the pinned ROS decoder recipe from [the prior results](SENSOR_REPLAY_RESULTS.md#replay-the-immutable-bag), extract the final archive to a fresh directory and run **without** legacy mode:

```sh
python3 /tmp/freespace/replay_ttl.py --recording /tmp/ttl-recording-03 --output /tmp/new-ttl-report.json
```

The new mixed-clock 30 s acquisition and separate image-loss failure are also preserved under [`replay/acquisition-r43/`](replay/acquisition-r43/). They are earlier acquisition diagnostics, not replacements for the accepted final long trace.

The separate strict geometry replay accepted 429/429 frames and all six fault variants passed. Worst frame p95/max errors were front **0.052492/0.052552 m**, off-axis **0.053017/0.053080 m**, and floor **0.031943/0.031987 m**. Minimum support was **97.1659%** on the front and 100% on the other targets. These measured errors still fit the unchanged integration limits but are not calibrated localization or clearance bounds.

The final offline suite passed **143 tests in 75.64 s**. Actual pinned-schema health validation, full geometry/fault replay and rolling-TTL replay were executed separately from hosted offline tests. Raw incomplete runs were preserved and the original r35/r39 checksums remained unchanged.

## Integrity and next boundary

Every immediate bundle directory must have a checksum inventory; deleting an entire `SHA256SUMS` is now a failure rather than silently excluding the bundle. Inner bag/fixture/policy/receive identities remain checked. Legacy mode exempts only the unavailable alignment-provenance check from its result; decode, geometry, pairing and fault failures still fail.

No thresholds were relaxed and no original r35/r39 evidence bytes were changed. Native task completion is separate from final publication checks. The next implementation boundary remains `g1-oracle` / `g1-supervisor`, with source/mesh geometry and estimator/clock assumptions explicitly revalidated there. No arming authority or calibrated flight guarantee follows from these preparation results.
