"""Offline pinned-message smoke or explicitly authorized ROS control executable.

The offline subcommand constructs/serializes actual installed PX4 messages with
no rclpy.init, node, publisher or process lock. It is NOT runtime gate evidence.
The run subcommand is consequential: operator authorization is required again
on the command line, not inferred from a repository/config file or roadmap.
"""
import argparse
from dataclasses import asdict, replace
from hashlib import sha256
from importlib import import_module
import json
from math import isnan
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))

from drone_cv.px4_bridge import (
    FIRMWARE_PIN, IMAGE_PIN, MESSAGES_PIN, PINNED_SOURCES, RosBridge,
    RuntimeAuthorization, decode_ack, decode_odometry, decode_status, encode, message_types,
)
from drone_cv.supervisor import (
    Ack, Command, Config, Envelope, Inputs, Lease, Odometry, OperatorRequest,
    Permit, Stage, State, Status, transition,
)
from drone_cv.timing import Clock, FreshnessPolicy, Stamp
from drone_cv.supervision_contract import ClockContract, LossResponse, RateWindow, SourceBudget
from drone_cv.px4_observation import (
    decode_battery, decode_fallback_flags, decode_home, decode_local_position,
)


def _context_messages(types):
    local = types.VehicleLocalPosition()
    local.timestamp, local.timestamp_sample, local.ref_timestamp = 1050000, 1000000, 123
    local.ref_lat, local.ref_lon, local.ref_alt = 47., 8., 500.
    local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
    local.xy_global = local.z_global = local.heading_good_for_control = True
    home = types.HomePosition()
    home.timestamp, home.update_count = 1000000, 1
    home.lat, home.lon, home.alt = 47., 8., 500.
    home.valid_lpos = home.valid_hpos = home.valid_alt = True
    flags = types.FailsafeFlags()
    flags.timestamp, flags.mode_req_local_position, flags.mode_req_local_alt = 1000000, 16, 16
    flags.offboard_control_signal_lost, flags.manual_control_signal_lost = True, True
    battery = types.BatteryStatus()
    battery.timestamp, battery.id, battery.remaining, battery.connected = 1000000, 1, .8, True
    return ((local, decode_local_position), (home, decode_home),
            (flags, decode_fallback_flags), (battery, decode_battery))


def _context_smoke(cfg, decoded):
    from drone_cv.px4_policy import LandingCorridor, PolicyGeometry, validate_motion_policy
    from drone_cv.px4_runtime_guard import runtime_context_error
    # Complete synthetic policy is never applied or accepted as a runtime profile.
    policy = {"COM_OF_LOSS_T": .33333, "COM_OBL_RC_ACT": 5, "COM_FAIL_ACT_T": 0.,
        "COM_RC_LOSS_T": .5, "COM_RCL_EXCEPT": 0, "NAV_RCL_ACT": 1, "COM_RC_IN_MODE": 4,
        "NAV_DLL_ACT": 1, "COM_DL_LOSS_T": 10, "COM_DLL_EXCEPT": 0, "COM_LOW_BAT_ACT": 2,
        "BAT_LOW_THR": .15, "BAT_CRIT_THR": .07, "BAT_EMERGEN_THR": .05, "GF_ACTION": 2,
        "GF_SOURCE": 0, "GF_MAX_HOR_DIST": 50., "GF_MAX_VER_DIST": 50., "MPC_XY_VEL_MAX": 1.,
        "MPC_Z_VEL_MAX_UP": .5, "MPC_Z_VEL_MAX_DN": .5, "MPC_POS_MODE": 4,
        "MPC_ACC_HOR": 3., "MPC_ACC_HOR_MAX": 5., "MPC_ACC_UP_MAX": 4.,
        "MPC_ACC_DOWN_MAX": 3., "MPC_JERK_MAX": 8., "MPC_JERK_AUTO": 4., "UXRCE_DDS_SYNCT": 0}
    cfg = replace(cfg, envelope=replace(cfg.envelope, maximum_speed_mps=1.2))
    validate_motion_policy(policy, maximum_speed_mps=1.2, minimum_deceleration_mps2=.25)
    latest = {key: decoded[name] for key, name in (
        ("local", "VehicleLocalPosition"), ("home", "HomePosition"), ("flags", "FailsafeFlags"),
        ("battery", "BatteryStatus"), ("odometry", "VehicleOdometry"), ("status", "VehicleStatus"))}
    context = PolicyGeometry(latest["home"].bind(latest["local"]), 50., 50.,
        LandingCorridor((-11., -11., -11.), (11., 11., 6.), 6., 6.25, cfg.envelope.bounds_provenance,
                        "Synthetic full landing volume; not surveyed", "Synthetic eroded phase clearances"),
        5., .1, "Synthetic local/global frame reserve; not measured")
    now = Clock("offline_epoch", 1., 1.)
    baseline = runtime_context_error(latest, now, cfg, context, policy, State())
    hold = runtime_context_error({**latest, "flags": replace(latest["flags"],
        hold_unavailable=("local_position_invalid",))}, now, cfg, context, policy, State())
    changed = runtime_context_error({**latest, "home": replace(latest["home"], update_count=2)},
                                    now, cfg, context, policy, State())
    if baseline is not None or hold != "HOLD_UNAVAILABLE:local_position_invalid" or not (
            changed and changed.startswith("PX4_GEOMETRY:")):
        raise ValueError("pinned offline context guard did not enforce frozen policy")
    return {"scope": "synthetic declarations only; not live policy/fallback validation",
            "baseline_reason": baseline, "hold_failure_reason": hold, "home_change_reason": changed}


def offline_smoke() -> dict:
    """Synthetic bounds exercise protocol encoding, never calibration or safety."""
    types = message_types()
    serialization = import_module("rclpy.serialization")
    serialize = serialization.serialize_message
    cfg = Config(
        owner="offline_owner", planner="offline_planner", safety="offline_safety",
        source_system=42, source_component=191, target_system=1, target_component=1,
        request_freshness=FreshnessPolicy(.5, .5),
        clock_contract=ClockContract("offline_epoch", .196, 1., .01, .1,
                                    "Synthetic protocol RTF range, not workload evidence", "gazebo_sim", .004, .05),
        source_budgets=tuple(SourceBudget(name, ttl, period, jitter, "simulation",
                                         "Synthetic per-source protocol cadence")
            for name, ttl, period, jitter in (
                ("ODOMETRY", .5, .02, .05), ("STATUS", 3., .5, .1),
                ("COMMAND", .5, .02, .05), ("PERMIT", .5, .02, .05),
                ("PERCEPTION", .5, .02, .05), ("LOCAL_POSITION", .5, .02, .05),
                ("FALLBACK_FLAGS", .5, .2, .1), ("BATTERY", 2., 1., .1))),
        loss_response=LossResponse("offline_epoch", FIRMWARE_PIN, .05, .02, .01, .02, 5,
                                   "Synthetic crash-after-tick fixture", "synthetic_offline"),
        clock_stall_wall_s=.5, max_clock_step_s=.5,
        max_tick_gap_wall_s=.3, prime_sim_s=.2, prime_wall_s=.2,
        request_timeout_s=1., session_timeout_s=20., max_command_ttl_s=.5,
        envelope=Envelope((-5., -5., -5.), (5., 5., 0.), .2, .25, .1,
                          .2, (.01,)*3, (.01,)*3, "offline synthetic fixture; NOT measured or certified",
                          (.01, .01, .01), .01, .01, "synthetic_offline"),
    )
    def data(t, nav=2, armed=False, action=None, ack=None):
        stamp = Stamp("offline_epoch", round(t*1e6), t, t)
        return Inputs(Clock(stamp.epoch, t, t),
            Odometry(stamp, (0., 0., -1.), (0., 0., 0.), 0., 0, (.001,)*3, (.001,)*3),
            Status(stamp, nav, 2 if armed else 1, False, 0, True, 1, 1, t, t if armed else 0.),
            Command(cfg.planner, stamp, .4, (.1, 0., 0.), 0.),
            Permit(cfg.safety, stamp, .4, cfg.planner, stamp.sequence, (.1, 0., 0.), stamp,
                   (-.3, -.5, -1.5), (2., .5, -.5), (0., 0., -1.), 0,
                   "local_ned_reference_centers", "bootstrap_known_clear",
                   cfg.envelope.bounds_provenance),
            Lease(cfg.owner, stamp, 10., 10., "bootstrap"), True,
            OperatorRequest(cfg.owner, stamp, action) if action else None,
            Ack(stamp, ack, 0, 42, 191, False) if ack else None,
            RateWindow(Clock(stamp.epoch, t-.02, t-.02), Clock(stamp.epoch, t, t)))
    # Exercise the installed generated input schemas and acquisition-time decoder,
    # not stand-in classes. Serialization imports alone do not initialize ROS.
    odom = types.VehicleOdometry()
    odom.timestamp, odom.timestamp_sample = 1050000, 1000000
    odom.pose_frame, odom.velocity_frame = 1, 1
    odom.position, odom.velocity, odom.q = [0., 0., -1.], [0., 0., 0.], [1., 0., 0., 0.]
    odom.position_variance, odom.velocity_variance = [.001]*3, [.001]*3
    status = types.VehicleStatus()
    status.timestamp, status.nav_state_timestamp = 1000000, 1000000
    status.nav_state, status.arming_state = 2, 1
    status.system_id, status.component_id = 1, 1
    status.pre_flight_checks_pass = True
    ack = types.VehicleCommandAck()
    ack.timestamp, ack.command, ack.result = 1000000, 176, 0
    ack.target_system, ack.target_component = 42, 191
    input_records, decoded_inputs = {}, {}
    for msg, decoder in ((odom, decode_odometry), (status, decode_status), (ack, decode_ack),
                         *_context_messages(types)):
        wire = serialize(msg)
        decoded = decoder(serialization.deserialize_message(wire, type(msg)), "offline_epoch", 1.)
        if decoded.stamp.sim_s != 1.:
            raise ValueError("decoder did not preserve pinned acquisition/source timestamp")
        input_records[type(msg).__name__] = {
            "cdr_bytes": len(wire), "cdr_sha256": sha256(wire).hexdigest()}
        decoded_inputs[type(msg).__name__] = decoded
    context_smoke = _context_smoke(cfg, decoded_inputs)
    state, records, active_state = State(), [], None
    trajectory_count, heartbeat_count = 0, 0
    steps = ((data(1., action="prime"), Stage.PRIMING),
             (data(1.21, action="enter"), Stage.MODE_WAIT),
             (data(1.22, nav=14, ack=176), Stage.OFFBOARD_DISARMED),
             (data(1.23, nav=14, action="arm"), Stage.ARM_WAIT),
             (data(1.24, nav=14, armed=True, ack=400), Stage.ACTIVE),
             (data(1.25, nav=14, armed=True, action="handoff"), Stage.HANDOFF_WAIT),
             (data(1.26, nav=4, armed=True, ack=176), Stage.RELEASED))
    for inputs, expected in steps:
        state, output = transition(state, inputs, cfg)
        if state.stage != expected:
            raise ValueError(f"offline transition mismatch: {state.stage}: {state.reason}")
        if state.stage == Stage.ACTIVE:
            active_state = state
        encoded = encode(output, round(inputs.now.sim_s*1e6), cfg, types)
        inventory = {}
        heartbeat_count += "offboard_control_mode" in encoded
        if inputs.status.nav_state != 14 and "trajectory_setpoint" in encoded:
            raise ValueError("trajectory injection outside observed Offboard")
        for name, msg in encoded.items():
            wire = serialize(msg)
            inventory[name] = {"type": type(msg).__name__, "cdr_bytes": len(wire),
                               "cdr_sha256": sha256(wire).hexdigest()}
        if "trajectory_setpoint" in encoded:
            trajectory_count += 1
            point, mode = encoded["trajectory_setpoint"], encoded["offboard_control_mode"]
            unused = point.position if mode.velocity else point.velocity
            if not all(isnan(v) for v in (*unused, *point.acceleration, *point.jerk, point.yawspeed)):
                raise ValueError("unused trajectory fields must be NaN")
            if mode.position == mode.velocity or any((mode.acceleration, mode.attitude,
                    mode.body_rate, mode.thrust_and_torque, mode.direct_actuator)):
                raise ValueError("Offboard field priority mismatch")
        if "vehicle_command" in encoded:
            cmd = encoded["vehicle_command"]
            if (cmd.source_system, cmd.source_component, cmd.target_system,
                    cmd.target_component, cmd.confirmation, cmd.from_external) != (42, 191, 1, 1, 0, True):
                raise ValueError("command routing mismatch")
            expected_mode = {"offboard": (176, 1., 6., 0.), "arm": (400, 1., 0., 0.),
                             "loiter": (176, 1., 4., 3.)}[output.vehicle_command]
            if (cmd.command, cmd.param1, cmd.param2, cmd.param3) != expected_mode:
                raise ValueError("pinned mode/arm parameters mismatch")
            if not all(isnan(v) for v in (cmd.param4, cmd.param5, cmd.param6, cmd.param7)):
                raise ValueError("unused vehicle command fields must be NaN")
        records.append({"stage": state.stage.value, "messages": inventory})
    if active_state is None or trajectory_count != 3 or heartbeat_count != 5:
        raise ValueError("incomplete offline transition coverage")
    fault = data(1.25, nav=14, armed=True)
    revoked, output = transition(active_state, replace(fault, odometry=None), cfg)
    if revoked.stage != Stage.REVOKED or encode(output, 1250000, cfg, types):
        raise ValueError("odometry loss emitted a fake hold/control message")
    return {"scope": "OFFLINE_ONLY", "runtime_verified": False, "gate_verified": False,
            "publishers_created": 0, "rclpy_initialized": False, "armed_commands_sent": 0,
            "nodes_created": 0, "ownership_locks_acquired": 0, "arm_messages_encoded": 1,
            "declared_firmware_pin": FIRMWARE_PIN, "declared_messages_pin": MESSAGES_PIN,
            "declared_image_pin": IMAGE_PIN,
            "pin_note": "Generated field contract checked; installed source provenance must be verified separately.",
            "sources": list(PINNED_SOURCES), "decoded_input_messages": input_records,
            "synthetic_readonly_context": context_smoke,
            "transitions": records,
            "fault": revoked.reason, "bounds": "Synthetic protocol fixture, no flight calibration."}


def read_config(path):
    values = json.loads(Path(path).read_text(encoding="utf-8"))
    raw_bound = values["envelope"].get("capture_query_bound")
    if raw_bound is not None:
        from drone_cv.relative_profile import CaptureQueryBound
        values["envelope"]["capture_query_bound"] = CaptureQueryBound.from_dict(raw_bound)
    values["envelope"] = Envelope(**values["envelope"])
    values["request_freshness"] = FreshnessPolicy(**values["request_freshness"])
    values["clock_contract"] = ClockContract(**values["clock_contract"])
    values["loss_response"] = LossResponse(**values["loss_response"])
    values["source_budgets"] = tuple(SourceBudget(**budget) for budget in values["source_budgets"])
    return Config(**values)


def run(config_path, startup_path, confirmation):
    cfg = read_config(config_path)
    startup = json.loads(Path(startup_path).read_text(encoding="utf-8"))
    # An authored file cannot silently supply the opt-in action at point of risk.
    if "confirmation" in startup:
        raise ValueError("startup file must not contain confirmation; pass the explicit CLI authorization")
    from drone_cv.px4_policy import HomeSnapshot, LandingCorridor, LocalReference, PolicyGeometry
    geometry = startup["policy_geometry"]
    raw_home = geometry["home"]
    home = HomeSnapshot(**{**raw_home, "local_reference": LocalReference(**raw_home["local_reference"])})
    startup = {**startup, "policy_geometry": PolicyGeometry(
        **{**geometry, "home": home, "landing": LandingCorridor(**geometry["landing"])})}
    authorization = RuntimeAuthorization(confirmation=confirmation, **startup)
    authorization.validate(cfg)
    rclpy = import_module("rclpy")
    executor_type = import_module("rclpy.executors").SingleThreadedExecutor
    bridge = None
    rclpy.init(args=[])
    executor = executor_type()
    try:
        bridge = RosBridge(cfg, authorization)
        executor.add_node(bridge.node)
        while rclpy.ok() and not bridge.terminal_finished():
            executor.spin_once(timeout_sec=authorization.tick_wall_s)
        print(json.dumps({"stage": bridge.state.stage.value, "reason": bridge.state.reason,
                          "parameter_readback": bridge.parameter_readback,
                          "status_trace": [asdict(value) for value in bridge.status_trace],
                          "ack_trace": [asdict(value) for value in bridge.ack_trace],
                          "publication_trace": bridge.publication_trace,
                          "trace_scope": "last 256 events per stream; not full-session coverage",
                          "terminal_observation": (bridge.terminal_observation.report()
                              if bridge.terminal_observation is not None else None),
                          "scope": "runtime execution is NOT gate acceptance"}))
        return 0 if bridge.state.stage == Stage.RELEASED else 2
    finally:
        executor.shutdown(timeout_sec=cfg.request_timeout_s)
        if bridge is not None:
            bridge.close()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("offline-smoke", help="actual generated PX4 serialization; NO nodes/publishers/init")
    sub.add_parser("sources", help="print immutable firmware/message source URLs")
    runtime = sub.add_parser("run", help="CONSEQUENTIAL opt-in; constructs control publishers")
    runtime.add_argument("--config", required=True, type=Path)
    runtime.add_argument("--startup", required=True, type=Path)
    runtime.add_argument("--allow-control-publishers", required=True,
                         metavar="ALLOW_CONTROL_PUBLISHERS:<namespace>:<owner>")
    args = parser.parse_args(argv)
    if args.mode == "sources":
        print(json.dumps(list(PINNED_SOURCES), indent=2))
        return 0
    if args.mode == "offline-smoke":
        print(json.dumps(offline_smoke(), indent=2, allow_nan=False))
        return 0
    return run(args.config, args.startup, args.allow_control_publishers)


if __name__ == "__main__":
    raise SystemExit(main())
