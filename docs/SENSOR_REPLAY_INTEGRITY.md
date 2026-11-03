# Capture-time alignment and acquisition evidence integrity

## Scope

This implements `prep-replay-integrity` from the approved [repair plan](SENSOR_REPLAY_REPAIR_PLAN.md). It hardens the unarmed recording and offline replay boundary, not the flight controller or navigation envelope. No arming, control publisher, vehicle relocation, host installation or safety-threshold relaxation occurred.

At the r39 snapshot, `prep-model-profile` and `prep-ttl-replay` were pending. Their later [exercised preparation results](PREPARATION_RESULTS.md) do not complete physical contact/oracle, supervisor, flight or calibration acceptance. The historical capsule remains diagnostic, not a movement certificate.

## Exercised results

A fresh acquisition ran for **30.0174 s wall / 6.344 s sim** in the retained pinned image. It captured 145 depth images, 192 CameraInfo, 1,587 clocks, 630 odometry samples, 13 vehicle-status messages and one latched TF. Every recorded status was Disarmed.

- **136/136 post-warmup images** had matching calibration and capture-time poses; all declared target/frame checks passed the unchanged geometry limits.
- The independent Gazebo body snapshot is at **20.664 s sim**. Its PX4 pose sample is at exactly that timestamp, reset counter 12. No cached-pose time substitution occurred. Analytic regressions additionally exercise a non-exact two-sided bracket, excessive gaps and a reset within the bracket.
- Strict replay validates the alignment reconstruction, policy hash, receive-trace hash, per-topic event sequence, source/storage stamps, counts, required-stream progress and actual Disarmed messages before grading.
- A changed initial reset counter plus a translated odometry origin rejects every evaluated frame: **136 `ALIGNMENT_RESET_CHANGED`, no gate or geometry accepted**.
- A reset injected halfway preserves the earlier valid epoch but invalidates all later world grading: 61 accepted earlier frames, 74 `ALIGNMENT_EPOCH_INVALIDATED` rejections and one boundary clock rejection. It never reuses the old transform in the new epoch. Counter wrap/return cannot restore it.
- Missing CameraInfo/TF, excessive pose gaps and backward/future image stamps remain rejected. All six explicit variants pass their negative checks. Their event streams are derived in memory; they are not new sensor recordings.
- The stationary query remains **STOP / `OBSTACLE_IN_ENVELOPE`**, with 3,432 UNSEEN samples. Stale-all returns `NO_FRESH_FRAMES`. This is not positive real-data navigation.

Passing the original integration limits did **not** mean equal geometry accuracy:

| Target | r35 worst frame p95 | r39 first frame p95 | r39 worst frame p95 | r39 maximum error |
| --- | ---: | ---: | ---: | ---: |
| Front | 0.016582 m | 0.014709 m | 0.039870 m | 0.039893 m |
| Off-axis | 0.016493 m | 0.014874 m | 0.039851 m | 0.039862 m |
| Horizontal floor | 0.022381 m | 0.001994 m | 0.019794 m | 0.019801 m |

Front p95 worsened by about 2.4 times between these independent recordings. Recorded odometry peak-to-peak E displacement reached 0.049471 m during this 6.344 s stationary acquisition; N/D spans were 0.021610/0.031003 m. This nearly consumes the historical illustrative 0.05 m horizontal pose allowance, while the evidence TTL is 10 s. A short stationary span is neither a worst-case relative-pose bound nor permission to keep that allowance for navigation. Model-profile experiments must expose this observed floor and leave the calibrated pose/tracking/angular budgets unresolved.

### Real stream-loss fault

A second actual recorder run started with every stream present. The owned depth-only bridge was then stopped, while a separate bridge kept clock and CameraInfo live and PX4 continued publishing.

The recorder exited **2** after 2.1705 s of recording with:

```text
ValueError: STREAM_PROGRESS_STALLED:/drone_cv_g1/camera/depth/image_raw
```

Its manifest is explicitly incomplete and preserves 2 images, 15 CameraInfo, 126 clock, 45 odometry and 2 Disarmed status messages. Raw messages and the receive trace remain archived. A live clock can no longer conceal a missing camera stream.

## Alignment and epoch contract

`recording_integrity.py` uses the simulator snapshot timestamp and validated PX4 `timestamp_sample` poses. Exact-time samples or two-sided interpolation are permitted; extrapolation, brackets over 50 ms, reset-crossing brackets and malformed rigid transforms are rejected.

The manifest stores the snapshot body transform, actual bracket samples, capture stamp and reset identity. Replay reconstructs the alignment and compares it with the declared transform. Clock/reset changes invalidate alignment permanently for the remaining acquisition unless a separately implemented, independently registered alignment exists; this slice does not add automatic re-registration.

The recorder aborts a reset after alignment rather than publishing a deceptively complete recording. The replay can still retain pre-reset valid data in an explicitly injected negative variant. World target geometry remains grader-only, outside the evidence gate.

## Historical r39 acquisition progress policy

The policy archived inside `replay/integrity-r39/recording-01.tar.gz` used these finite monotonic-wall limits. It is preserved as historical input, not the current RTF-independent policy:

| Dynamic stream | Maximum source-progress silence |
| --- | ---: |
| Depth image | 2.0 s wall |
| CameraInfo | 2.0 s wall |
| PX4 odometry | 2.0 s wall |
| Vehicle status | 3.0 s wall |
| Clock | 1.5 s wall |

These limits detect acquisition failure under the observed slow software-rendered fixture. They are **not calibrated flight-control freshness or braking deadlines**. A duplicate source stamp cannot refresh health. Backward stamps fail explicitly. Static TF is latched and is not required to republish periodically.

Follow-up review identified the 3 s status wall deadline as RTF-sensitive: a roughly 0.504 s simulation period consumes most of it near RTF 0.21. Current schema-three acquisition uses one wall watchdog for clock progress and simulation-age checks for the other streams. The old 3 s fixture clock deadline was redundant and has been removed from new fixtures. Old bags retain their exact declared policy.

The receive sidecar stores actual source stamps, monotonic receive times, ROS storage times, topic sequences and seeded-cache labels. The manifest records requested subscriber QoS and discovered publisher QoS. No fictitious QoS is inserted into legacy rosbag metadata.

DDS discovery's `history=UNKNOWN` and `depth=0` are unavailable values, not measured publisher queue behavior. New manifests encode those fields as null and distinguish discovery from requested subscriber configuration. Storage-minus-capture also includes delivery lag of the recorder's `/clock`; it is not independent sensor latency or a measured PX4/Gazebo clock offset.

## Diagnostics, not safety bounds

The new report includes image gaps, unmatched calibration stamps, clock/wall span, mean RTF, observed one-second wall-window clock progress, odometry peak-to-peak position and estimator variance/sigma with frame labels.

For this recording:

- Mean RTF: **0.21134**. Minimum received clock progress across 29 complete, non-overlapping one-second wall windows: **0.19644**. This is an observed received-clock statistic, not a physics or compute guarantee.
- Depth gaps: median **32 ms sim**, maximum **132 ms sim**. There are 47 CameraInfo stamps without an image. Pairing every received image does not mean receiving every rendered image; loss location is unknown.
- Storage-minus-capture: depth p50/p95/max **36/44/68 ms sim**; odometry **8/12/32 ms sim**. These are not monotonic receive-wall latencies or conservative motion bounds.
- Last position sigma: approximately **0.108 / 0.108 / 0.173 m**. Body orientation sigma is recorded in radians. These absolute estimator quantities do not calibrate relative drift within the TTL or justify the old 5 cm pose allowance. Pinned `quality` is unused and zero is not an admission test.

No variance-based movement authority was added. Admission budgets, physical envelope and tracking/angular uncertainty remain work for the model-profile and existing G1 calibration/runtime gates.

## Historical evidence and CI

The original `unarmed-r35` package remains byte-identical. Its alignment was made from a cached pose without epoch-bound provenance or per-message receive-wall times. Current strict replay refuses it with `LEGACY_UNBOUND_ALIGNMENT`.

Explicit `--legacy-analysis` permits historical diagnostics while setting `acceptance_eligible=False` and `verified_alignment_provenance=False`. It cannot earn current-profile acceptance. Missing historical wall-window statistics remain unavailable; no synthetic metadata is added to old manifests.

`python scripts/check_project.py` now verifies every published bundle's complete SHA-256 file inventory, archive member safety, inner bag sizes/hashes, fixture identity and schema-two policy/receive identities. The existing Ubuntu/Windows CI already runs this command. It does not pretend that those hosted jobs run ROS; the actual decoder smoke was exercised locally in the pinned image.

The full offline suite passed **133 tests in 74.86 s**. The integrity command read back both old and new archived payload identities. A portable roadmap check initially detected the expected stale export after native receipt updates; regenerate it from canonical state rather than editing the view manually.

## Evidence and reproduction

The new package is [`replay/integrity-r39/`](replay/integrity-r39/). It retains complete/incomplete acquisitions, the replay report, initial-reset refusal and environment/source provenance. Historical diagnostics are stored separately from the unchanged earlier report.

Use the isolated decoder recipe in [the prior results](SENSOR_REPLAY_RESULTS.md#replay-the-immutable-bag), substituting `recording-01.tar.gz`, its extracted `recording-01` directory and a fresh output name. **Do not pass `--legacy-analysis` for schema-two acceptance.** The entry point is unchanged:

```sh
python3 /tmp/freespace/replay_bag.py --recording /tmp/recording-01 --output /tmp/new-integrity-replay
```

For new acquisition, run the same unarmed fixture/clock/TF/PX4 environment and:

```sh
python3 /tmp/freespace/record_sensor_bag.py --fixture /tmp/freespace/replay_fixture.json --policy /tmp/freespace/recording_policy.json --output /tmp/new-integrity-recording
```

Choose fresh names and preserve incomplete attempts. The split depth/clock-info bridge configurations are included only to reproduce the scoped image-stream loss, not to introduce control services. An engine interruption during the initial setup was observed; no recording was claimed from that setup. The retained owned container was inspected, restarted and its runtime recreated before acquisition. Run-owned simulator processes were stopped after evidence capture; the decoder-only container was then stopped and removed after analysis. No unrelated Docker resources or images were removed.
