# G1 static oracle results

## Scope

The actual pinned PX4/Gazebo path exercised an independent geometric observer while disarmed. No arming command, control publisher, setpoint, model modification, installation or `set_pose` operation was used. A frozen world added a y-normal target and, for negative trials, one explicit box. Static surfaces are evaluator-only inputs; no planner reads the truth topic or oracle ledger.

Evidence: [`replay/oracle-static-01/summary.json`](replay/oracle-static-01/summary.json) and the adjacent checksum inventory, complete/incomplete recording archives and diagnostic ZIP. The image identity, consumed model/mesh hashes, resolved frames, effective fixtures, raw receive traces and source-file hashes are retained. All attempts are preserved. Original preparation bundles and source PDF are unchanged.

## Physical geometry

The existing image supplies `sdformat14` 14.9.0. Native semantic-pose resolution merges the pinned includes and resolves link, collision, visual, joint and optical frames. The complete profile contains **42 components**. Collision boxes, COLLADA active-scene instances and ASCII/binary STL are enclosed separately. COLLADA unit/up-axis and ordered nested transforms precede the SDF mesh scale and visual pose. Propeller bounds include complete rotation about the resolved joint axis, including the visual offset from the hub.

- Physical body-relative lower bound: **-0.2275995 m**.
- Physical body-relative upper bound: **+0.06938461481722502 m**.
- Optical translation: **(0.13233, 0, 0.02078) m**.
- Body origin in the merged model: **z = 0.24 m**.
- Initial model z is derived from the complete profile: **-0.0124005 m**, not the historical 0.15 m body fixture or a drop from 0.25 m model height.

The native resolver agrees with the corrected optical mount without applying `CameraJoint` pose a second time. Named landing foam/rubber/plastic and the four landing collision boxes are distinct gear components. Carbon-fiber body and rotors are never allowlisted. The selected outer AABBs can conservatively intersect without a physical impact; reports explicitly call this geometric intersection, not a Gazebo contact-sensor event.

## Truth and integrity

The installed `ros_gz_bridge` 1.0.24 conversion lost every name/frame and stamp in a real seven-pose transport probe. It retained positions, but emitted zero transform stamps and empty frame names. The operator approved a lossless replacement: native `gz.transport13` receives the generated `gz.msgs.Pose_V` and sends its complete bytes on `/drone_cv_g1/evaluator/truth_pose_v` inside `std_msgs/UInt8MultiArray`. This is a read-only evaluator transport, not a control interface.

Schema four records truth in the same ROS bag as camera, clock and PX4 streams. The pinned generated protobuf decoder preserves outer stamps and exact entity name/ID. `world_from_body` composes world-model and model-relative body poses. Required-stream progress, finite queues, source simulation age, raw CDR hashes, receive ordering and epoch/reset checks apply. Old schemas are not silently upgraded.

A pinned decoder experiment found that deserialize/reserialize changes CDR padding bytes, including `/tf_static` offsets 42, 43 and 75. Integrity now hashes original `reader.read_next()` bytes, never reconstructed CDR. The same unchanged support bag passed strict replay after that correction; no replacement recording was used.

Each alignment stores both PX4 `timestamp_sample` endpoints and their signed difference from the Gazebo truth stamp. These are observed bracket/cadence differences, not independent clock-offset calibration. All trials retain Gazebo clock, `UXRCE_DDS_SYNCT=0`, and simulation-time consumers.

## Exercised outcomes

| Frozen case | Truth samples | Safety outcome | Execution outcome |
| --- | ---: | --- | --- |
| Support, complete same-bag replay | 385 | `NO_VIOLATION_OBSERVED` | `COMPLETED` |
| Body-box intersection plus allowed gear support | 285 | `UNSAFE_OBSERVED` | `COMPLETED` |
| Full propeller-envelope intersection plus allowed gear support | 281 | `UNSAFE_OBSERVED` | `COMPLETED` |
| Stop truth adapter before any unsafe observation | 73 | `UNKNOWN` | `INFRA_ERROR` |
| Stop truth adapter after unsafe observation | 75 | `UNSAFE_OBSERVED` | `INFRA_ERROR` |

All observed PX4 statuses were disarmed. Body negatives identified the carbon-fiber visual component; the rotor negative identified `rotor_2`'s visual sweep. Simultaneous allowed gear/floor pairs did not exempt either hazard. Stopping only the truth adapter caused `STREAM_SIM_AGE_EXCEEDED` and `TRUTH_END_UNCOVERED`, while other streams continued. Exit status remains 1 after unsafe even when infrastructure subsequently fails; incomplete without unsafe exits 2.

An independent typed fixture placed the body origin 1 m above the floor. Analytic minimum clearance `1 - 0.2275995 = 0.7724005 m` matched the actual CLI result. This is explicitly offline typed geometry, not a live floating vehicle or flight test.

## Retained failures and limits

The first discovery attempt consumed a scene packet before vehicle spawn. Discovery now waits within a fixed deadline for exact identities; duplicate identities still fail. A second trial began from a model-height drop and correctly latched floor intersections, including body geometry. Its original unsafe record remains unchanged. Another attempt froze alignment during estimator startup reset and correctly failed. Subsequent fixtures declare a fixed **10 s simulation bootstrap**, with **120 s wall deadline**, before oracle preflight. Bootstrap clearance is not observed or certified; it is not a search for a favorable clearance window. After recording begins, resets remain fatal and unsafe remains sticky.

The observer covers sampled endpoints and records every gap. It does **not** certify the swept volume between samples. The frozen maximum truth gap is 50 ms simulation time. Missing intervals are unobserved, never clear. Static results do not authorize flight, calibrate tracking/localization error, or complete `g1-supervisor`, `g1-flight`, `g1-calibration`, or any G2 gate. Landing allowlist rules are exercised geometrically, not by an armed landing.

The primary grading ROIs and thresholds were not changed. The y-normal target is present in the new recordings; all-pairs TTL pose-error analysis belongs to the separate evidence-semantics branch. That branch must block G2 only, not oracle/supervisor work.

## Reproduction

Inside the existing approved pinned image, copy the repository layout and set `PYTHONPATH` to its `src` directory. Use fresh output paths; never overwrite evidence:

```sh
python3 scripts/g1_oracle.py profile --models /opt/px4-gazebo/share/gz/models --output /tmp/new-profile.json
python3 scripts/g1_oracle.py replay --recording /tmp/extracted-run/run/recording --profile /tmp/extracted-run/profile.json --fixture /tmp/extracted-run/run/oracle_fixture.json --output /tmp/new-replay
```

`scripts/g1_oracle_run.py` starts actual unarmed simulator processes and is an operational command, not an offline checker. Run it only within the explicitly approved pinned-container boundary. Its `run.json.complete` describes orchestration completion; inspect recorder/replay exit codes and both outcome axes for classification.

The offline behavioral suite passed **344 tests in 107.36 s**. After packaging, **13 project-integrity tests** passed, including the three additional schema-four archive-binding regressions. `check_project.py --export-roadmap` verified the unchanged PDF, 31 acyclic tasks, 65 scenarios, links and 53 evidence hashes at oracle completion. Native `g1-oracle` completed at revision 66; later task additions do not change these static results.
