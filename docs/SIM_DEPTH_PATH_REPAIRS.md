# Depth-path audit repair (E7)

## Scope

`prep-depth-path-audit` now has offline accounting and recorder-side instrumentation. No new capture, Gazebo/bridge rebuild, installation or control publisher was executed. Historical bags and manifests are unchanged. Missing source/bridge evidence prevents localization of their image deficits and prevents completion of this task.

## Actual code paths

- `src/drone_cv/depth_path_audit.py`: matched stage/window analyzer and explicitly partial historical summary.
- `scripts/depth_path_audit.py`: exclusive-output CLI for either a declared stage contract plus JSONL events or an existing recording.
- `spike/freespace/record_sensor_bag.py`: exact callback-entry counts, separate from enqueue counts. Warmup/seeded events are not equated with collection callbacks.
- `spike/freespace/bag_writer.py`: exact accepted enqueue counts and ordered identity digests; the separate worker binds the same identities after storage writes. Close compares both counts and digests. Full queues, worker death, missing close results and identity mismatches fail instead of dropping observations silently. No approximate `qsize()` is used.

New receive events use `depth_audit_schema: 1` with `enqueue_wall_ns`, `storage_write_begin_wall_ns` and `storage_write_complete_wall_ns`. The returned storage result separately states `storage_closed`. A write-completion timestamp is not a disk-durability/fsync certificate. Recorder manifests add a versioned `depth_path_audit` section; old schema-two/four recordings do not acquire these fields retroactively.

The guarded pinned-message smoke additionally writes and reads two actual generated `VehicleStatus` CDR messages through the real finite separate-process writer. Its event identities are synthetic protocol inputs, not sensor captures or production-stage observations.

## Matched-stage contract

One analyzer call covers one stream, epoch and half-open acquisition-source interval. Image and CameraInfo are separate streams. Required stage order:

```text
source_publish
bridge_receive
bridge_publish
recorder_callback
enqueue
storage_complete
```

The contract contains `schema: 1`, `stream`, `epoch`, `source_start_ns`, `source_end_ns`, and `coverage` keyed by these stages. Every claimed complete stage must supply:

- `authoritative: true`, `complete: true`, `drained: true`;
- integer `probe_loss_count: 0`;
- the exact common `source_start_ns` and `source_end_ns`;
- nonempty instrumentation `provenance`.

Each event contains `stage`, `stream`, `epoch`, acquisition `source_ns`, monotonic observation `wall_ns`, preserved `source_sequence` or explicit null, and nonempty `payload_identity`. Recorder arrival ordinals must not be substituted for publisher sequences. Where the wire has no sequence, match capture stamp plus canonical semantic payload identity as multisets. Raw Gazebo serialization and ROS CDR bytes are not interchangeable identities; cross-format taps must justify a common semantic representation. The recorder's existing CDR hash only binds its own receive/storage boundary.

Missing, lossy, undrained, nonauthoritative or differently windowed coverage yields `UNKNOWN_COVERAGE`, not a measured production-loss count. Fully observed adjacent boundaries report missing/excess multisets. Their named boundary identifies where an identity disappeared, **not its physical root cause**. Missing identities at enqueue/storage require queue/error evidence before calling them backpressure rather than rejection or instrumentation error.

Optional `planned_source_ns` and `schedule_provenance` permit comparison of requested frame times with actual source publications. Only an authoritative source tap can report unpublished requested frames. CameraInfo cadence alone cannot supply that schedule.

Duplicate identities, source reordering and events outside the frozen window are retained separately. Gap quantiles use chronological distinct instants; duplicate/reordering counters remain visible. Source-simulation and receive-wall gap quantiles are separate. Neither is an end-to-end capture-to-actuation latency bound or a worst-case deadline certificate.

## Exercised historical path

```sh
python scripts/depth_path_audit.py --recording artifacts/g1-oracle-work/drone-cv-oracle-support-06-ttl/run/recording --output artifacts/sim-repair-depth-audit.json
```

The actual support-06 persisted trace contains 452 images and 581 CameraInfo events, with no duplicate/reordered source stamps in those stored streams. Image source-sim gap p95/max are 0.082/0.164 s; receive-wall gap p95/max are 0.534214031/1.07828322 s. CameraInfo source-sim max is 0.036 s. These are stored-trace statistics, not renderer/bridge rates or command latency.

The result explicitly reports unavailable authoritative `source_publish`, `bridge_receive` and `bridge_publish` taps, no historical callback/enqueue/storage identity certificate, `loss_location_certified: false`, and no end-to-end latency bound.

## Remaining runtime prerequisites

The pinned bridge currently has no exercised authoritative ingress/publication counters. A second BEST_EFFORT subscriber does not provide them. Source and bridge instrumentation may require a separately approved rebuild or evaluator fixture; no such installation/build/execution permission is inferred here. A fresh unarmed trial must freeze workload, QoS, source window, bounded queues/drain, instrumentation loss and overhead. Controlled source skip, bridge-path loss and storage backpressure must then be observed at their actual stages. Timing/TTL/watchdog/source-lead limits remain unchanged. This is an input to calibration, not calibration or flight acceptance.
