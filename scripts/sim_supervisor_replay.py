"""Guarded pinned-message smoke and retained-bag conditional stopping replay.

No ROS initialization, node, publisher, mode change or new capture is permitted.
Synthetic certificates and dynamics are hypothetical test inputs, not evidence
that the recorded world was free or that braking/error bounds were satisfied.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"src"), str(ROOT/"spike/freespace"), str(ROOT/"scripts")]

from drone_cv.px4_bridge import decode_odometry, decode_status, message_types
from drone_cv.supervisor import (Command, Config, Envelope, Inputs, Lease, Permit,
                                 Stage, State, _motion_error, transition)
from drone_cv.timing import Clock, FreshnessPolicy, Stamp
from drone_cv.px4_bridge import FIRMWARE_PIN
from drone_cv.supervision_contract import ClockContract, LossResponse, SourceBudget, rate_window


def diagnostic_config() -> Config:
    return Config("diagnostic_owner", "diagnostic_planner", "diagnostic_safety",
        42, 191, 1, 1, FreshnessPolicy(.5, .5),
        ClockContract("conditional_retained_epoch", .196, 1., 1., .2,
                      "Retrospective diagnostic candidate floor equals prior observed minimum; "
                      "no supported workload or untouched validation is claimed", "gazebo_sim", .004, .05),
        tuple(SourceBudget(name, ttl, period, jitter, "simulation",
                           "Declared retrospective service hypothesis, not measured runtime guarantee")
              for name, ttl, period, jitter in (
                  ("ODOMETRY", .5, .02, .05), ("STATUS", 3., .504, .1),
                  ("COMMAND", .5, .02, .05), ("PERMIT", .5, .02, .05),
                  ("PERCEPTION", .5, .064, .1))),
        LossResponse("conditional_retained_epoch", FIRMWARE_PIN, .05, .02, .01, .02, 5,
                     "Synthetic diagnostic response; no actual fallback readback/execution",
                     "synthetic_offline"),
        .5, .5, .3, .2, .2, 1., 20., .5,
        Envelope((-20., -20., -20.), (20., 20., 20.), 1., .1, .1,
                 .2, (.01, .01, .01), (.01, .01, .01),
                 "conditional synthetic diagnostic v2; not measured",
                 (.01, .01, .01), .01, .01, "synthetic_offline"))


def replay(recording: Path) -> dict:
    from replay_bag import read_bag
    manifest = json.loads((recording/"manifest.json").read_text())
    rows = [json.loads(line) for line in (recording/"receive_progress.jsonl").read_text().splitlines()]
    indexed = {}
    for topic, message, _ in read_bag(recording/"bag"):
        if topic.endswith("vehicle_odometry"):
            indexed[(topic, int(message.timestamp_sample)*1000)] = message
        elif topic.endswith("vehicle_status_v4"):
            indexed[(topic, int(message.timestamp)*1000)] = message
    cfg, results, norms = diagnostic_config(), Counter(), []
    geometry_results = Counter()
    maximum_position_variance = maximum_velocity_variance = 0.
    latest_status, reference, reset = None, None, manifest["alignment"]["reset_counter"]
    epoch = "conditional_retained_epoch"
    clock_samples, observed_rates = (), []
    for row in sorted(rows, key=lambda item: item["received_wall_ns"]):
        if row["topic"] == "/clock" and row["source_ns"] is not None:
            # Recorded node ROS-time snapshot, not queued explicit callback age.
            sample = Clock(epoch, row["received_sim_ns"]/1e9, row["received_wall_ns"]/1e9)
            clock_samples = (*clock_samples, sample)
            earlier = [i for i, item in enumerate(clock_samples)
                       if sample.wall_s-item.wall_s >= cfg.clock_contract.window_wall_s]
            if earlier:
                clock_samples = clock_samples[earlier[-1]:]
            continue
        key = (row["topic"], row["source_ns"])
        if key not in indexed:
            continue
        wall = row["received_wall_ns"]/1e9
        msg = indexed[key]
        if row["topic"].endswith("vehicle_status_v4"):
            latest_status = decode_status(msg, epoch, wall)
            continue
        odom = decode_odometry(msg, epoch, wall)
        norms.append(abs(odom.quaternion_squared_norm_error))
        maximum_position_variance = max(maximum_position_variance, *odom.position_variance)
        maximum_velocity_variance = max(maximum_velocity_variance, *odom.velocity_variance)
        if reference is None:
            reference = odom
        if latest_status is None:
            results["MISSING_CAUSAL_STATUS"] += 1
            continue
        sim = row["received_sim_ns"]/1e9
        now = Clock(epoch, sim, wall)
        stamp = Stamp(epoch, int(row["topic_sequence"]), sim, wall)
        command = Command(cfg.planner, stamp, .4, (0., 0., 0.), reference.yaw_ned)
        permit = Permit(cfg.safety, stamp, .4, cfg.planner, stamp.sequence, command.velocity_ned,
                        odom.stamp, (-10., -10., -10.), (10., 10., 10.),
                        reference.position_ned, reset, "local_ned_reference_centers",
                        "bootstrap_known_clear", cfg.envelope.bounds_provenance)
        # MODE_WAIT is synthetic protocol state. Preserve actual observed mode,
        # arm state and timestamps; no fabricated PX4 ACK or Offboard telemetry.
        state = State(stage=Stage.MODE_WAIT, reason="conditional_mode_wait", started=now,
                      hold_ned=reference.position_ned, hold_yaw=reference.yaw_ned,
                      reset_counter=reset, pending_since=now)
        rate = rate_window(clock_samples, cfg.clock_contract)
        if rate is not None:
            observed_rates.append(rate.rtf)
        data = Inputs(now, odom, latest_status, command, permit,
                      Lease(cfg.owner, stamp, 1., 1., "bootstrap"), True, rate=rate)
        result, output = transition(state, data, cfg)
        results[result.reason if result.stage == Stage.REVOKED else result.stage.value] += 1
        geometry_results[_motion_error(data, cfg, state) or "CONTAINED"] += 1
        if output.vehicle_command or output.position_ned or output.velocity_ned:
            raise ValueError("retained disarmed replay generated unexpected control output")
    if (not norms or geometry_results["DIRECTION_UNSUPPORTED"]
            or results["DIRECTION_UNSUPPORTED"]):
        raise ValueError(f"unexpected direction rejection: {dict(results)}")
    return {"recording": recording.name, "manifest_sha256": hashlib.sha256((recording/"manifest.json").read_bytes()).hexdigest(),
            "evaluated_odometry": len(norms), "outcomes": dict(results),
            "conditional_geometry_outcomes": dict(geometry_results),
            "stationary_admission_demonstrated": results["mode_wait"] > 0,
            "observed_position_variance_max": float(maximum_position_variance),
            "observed_velocity_variance_max": float(maximum_velocity_variance),
            "health_limits_unchanged": True,
            "clock_contract": {"minimum_rtf": cfg.clock_contract.minimum_rtf,
                               "maximum_rtf": cfg.clock_contract.maximum_rtf,
                               "window_wall_s": cfg.clock_contract.window_wall_s,
                               "observed_minimum_rtf": min(observed_rates) if observed_rates else None,
                               "observed_maximum_rtf": max(observed_rates) if observed_rates else None,
                               "runtime_supported_range_established": False},
            "source_wall_deadlines_s": {budget.name: budget.wall_deadline(cfg.clock_contract)
                                        for budget in cfg.source_budgets},
            "maximum_raw_squared_norm_error": max(norms),
            "window": "all stored odometry matched to original receive identities; missing causal status reported",
            "scope": "independent conditional ticks, not continuous runtime/clock/watchdog or dynamics validation",
            "bounds": {"velocity_error_axis_mps": .01, "tracking_error_m": .01,
                       "position_error_m": .01, "deceleration_mps2": .1,
                       "response_delay_s": cfg.response_delay_s, "extra_margin_m": .1},
            "physical_bounds_verified": False, "synthetic_clear_region": [-10., 10.]}


def writer_smoke(directory: Path) -> dict:
    """Exercise actual generated CDR through the separate-process finite writer."""
    from bag_writer import BagWriter
    from replay_bag import read_bag
    from rclpy.serialization import serialize_message
    directory.mkdir(exist_ok=False)
    topic = "/offline/vehicle_status_v4"
    writer = BagWriter(directory, {topic: "px4_msgs/msg/VehicleStatus"},
                       {"writer_queue_items": 8, "writer_shutdown_wall_s": 10.})
    types = message_types()
    for sequence in (1, 2):
        message = types.VehicleStatus()
        message.timestamp = sequence*1_000_000
        payload = serialize_message(message)
        writer.write(topic, payload, {"source_ns": sequence*1_000_000_000,
            "topic_sequence": sequence, "received_sim_ns": sequence*1_000_000_000,
            "received_wall_ns": sequence*1_000_000_000,
            "serialized_sha256": hashlib.sha256(payload).hexdigest()})
    result = writer.close()
    messages = read_bag(directory/"bag")
    if [int(message.timestamp) for _,message,_ in messages] != [1_000_000, 2_000_000]:
        raise ValueError("writer storage changed actual pinned message identities")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("refuse to overwrite prior diagnostic evidence")
    import rclpy
    from rclpy.node import Node
    def forbidden(*args, **kwargs):
        raise RuntimeError("OFFLINE_GUARD: runtime initialization/control construction forbidden")
    rclpy.init = rclpy.create_node = Node.create_publisher = forbidden
    from g1_supervisor import offline_smoke
    report = {"scope": "OFFLINE_ONLY", "pinned_message_smoke": offline_smoke(),
              "retained_replays": [replay(path) for path in args.recording],
              "actual_bounded_writer": writer_smoke(args.output.with_suffix(".writer"))}
    payload = json.dumps(report, indent=2, allow_nan=False)+"\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(payload)
    print(payload)


if __name__ == "__main__":
    main()
